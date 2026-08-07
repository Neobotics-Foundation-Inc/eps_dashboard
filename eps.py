#!/usr/bin/env python3
"""Neoracer EPS service: episodic policy search over the gap follower's gains.

Same pattern as the wallfollow dashboard (stdlib HTTP + rclpy) on port 8084.
The dynamic gap follower from wallfollow is the plant; its four PID
coefficients (kp, kd, speed_kp, speed_kd) are learned by a (1+1) evolution
strategy with the 1/5th success rule. The student is the reward function:
one Success or Fail press per episode is the entire training signal.
Sustained success raises the speed cap a notch (staircase curriculum);
sustained failure backs it off.

Episode state machine (Fail is live during RUNNING; Success only after Done):
  IDLE -arm-> ARMED -go-> RUNNING -done-> AWAITING_LABEL -label-> IDLE

Search runs in normalized units (each coefficient divided by its nominal
value) so one sigma drives all four. Every label appends a JSONL row to
ledger.jsonl carrying the full learner state, so a service restart resumes
exactly where the session left off and Undo restores exactly one step.

The car only drives during RUNNING; every other phase publishes zeros.
Disable the wallfollow service while EPS runs: two gap followers on /drive
starve each other at the mux. The transmitter SWB switch is the physical
gate. Steering is negated on publish: wire positive turns this car left.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import random
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import yaml

PORT = 8084
BASE = Path(__file__).resolve().parent
YAML_PATH = BASE / 'eps.yaml'
LEDGER = BASE / 'ledger.jsonl'
INIT_MARK = BASE / '.init_set'   # exists once the starting point is committed

GAIN_KEYS = ('kp', 'kd', 'speed_kp', 'speed_kd')
PLANT_INIT_KEYS = ('width', 'window', 'side_weight', 'lookahead')  # set once at Starting point, never learned
NORM_CLAMP = (0.2, 5.0)     # normalized gain bounds: 0.2x .. 5x nominal
SEARCH_STEP = 2             # gap follower constants, as in wallfollow
CENTER_BIAS = 0.006
HALF_CLEAR = 0.21

_lock = threading.Lock()
_cfg: dict = {}


def load_cfg():
    with _lock:
        _cfg.update(yaml.safe_load(YAML_PATH.read_text()))
        _cfg.setdefault('max_mps', 4.0)   # older yaml files lack this key


def save_cfg():
    """Rewrite values in place so the yaml comments survive."""
    import re
    text = YAML_PATH.read_text()
    with _lock:
        for k, v in _cfg.items():
            if isinstance(v, (int, float)):
                text = re.sub(rf'(?m)^{k}:\s*[-\d.eE+]+', f'{k}: {v}', text)
    YAML_PATH.write_text(text)


class Learner:
    """(1+1)-ES with 1/5th-rule sigma adaptation and a speed staircase.

    All mutation happens in normalized units around the nominal gain vector.
    Every method that changes state is called with the config snapshot so a
    mid-session hyperparameter edit applies from the next label onward.
    """

    def __init__(self, cfg):
        self.reset(cfg)

    def reset(self, cfg):
        self.theta = [1.0, 1.0, 1.0, 1.0]   # normalized incumbent
        self.proposal = None                 # normalized, set by arm()
        self.sigma = float(cfg['sigma0'])
        self.speed = float(cfg['speed_floor'])
        self.wins = 0
        self.fails = 0
        self.episode = 0
        self.best = {}                       # speed(str) -> normalized theta
        self.best_dur = {}                   # speed(str) -> fastest passing seconds

    # -- state as a plain dict, for the ledger and for undo --

    def snapshot(self):
        return {'theta': list(self.theta), 'sigma': self.sigma,
                'speed': self.speed, 'wins': self.wins, 'fails': self.fails,
                'episode': self.episode, 'best': dict(self.best),
                'best_dur': dict(self.best_dur)}

    def restore(self, snap):
        self.theta = list(snap['theta'])
        self.sigma = snap['sigma']
        self.speed = snap['speed']
        self.wins = snap['wins']
        self.fails = snap['fails']
        self.episode = snap['episode']
        self.best = dict(snap['best'])
        self.best_dur = dict(snap.get('best_dur', {}))
        self.proposal = None

    def gains(self, norm):
        nominal = _cfg['nominal']
        return {k: round(norm[i] * nominal[k], 5) for i, k in enumerate(GAIN_KEYS)}

    def arm(self, cfg):
        lo, hi = NORM_CLAMP
        self.proposal = [max(lo, min(hi, t + self.sigma * random.gauss(0, 1)))
                         for t in self.theta]
        return self.gains(self.proposal)

    def label(self, outcome, cfg):
        """Apply one Success or Fail. Returns nothing; caller snapshots."""
        self.episode += 1
        if outcome == 'success':
            if self.proposal is not None:
                self.theta = list(self.proposal)
            self.sigma = min(self.sigma * cfg['a_up'], cfg['sigma_max'])
            self.wins += 1
            self.fails = 0
            if self.wins >= cfg['confirm_runs']:
                self.best[f'{self.speed:.3f}'] = list(self.theta)
                self.speed = min(self.speed + cfg['speed_step'], cfg['speed_max'])
                self.wins = 0
                self.sigma = min(self.sigma * cfg['sigma_kick'], cfg['sigma_max'])
        else:
            self.sigma = max(self.sigma * cfg['a_down'], cfg['sigma_min'])
            self.fails += 1
            self.wins = 0
            if self.fails >= cfg['patience']:
                self.speed = max(self.speed - cfg['speed_backoff'], cfg['speed_floor'])
                self.fails = 0
        self.proposal = None

    def revert_best(self, cfg):
        """Restore the checkpoint at the highest speed ever passed, capped
        at the current speed_max so a lowered cap is honored."""
        if not self.best:
            return False
        top = max(self.best, key=float)
        self.theta = list(self.best[top])
        self.speed = min(float(top), cfg['speed_max'])
        self.wins = 0
        self.fails = 0
        self.proposal = None
        return True


class Session:
    """Episode state machine + ledger. Owns the learner."""

    PHASES = ('IDLE', 'ARMED', 'RUNNING', 'AWAITING_LABEL')

    def __init__(self, cfg):
        self.learner = Learner(cfg)
        self.phase = 'IDLE'
        self.armed_gains = None
        self.armed_speed = None
        self.t_go = None
        self.last_duration = None
        self.prev_snapshot = None            # for one-step undo
        self.history = []                    # chart rows, rebuilt from ledger
        self._replay()

    def _replay(self):
        """Resume from the last ledger row's embedded learner state."""
        if not LEDGER.exists():
            return
        rows = [json.loads(x) for x in LEDGER.read_text().splitlines() if x.strip()]
        stateful = [r for r in rows if r.get('state_after')]
        if not stateful:
            return
        self.learner.restore(stateful[-1]['state_after'])
        # Undo is one step only: offer it after a restart only when the last
        # row is a label (an undo row means the step was already taken back).
        if 'ep' in stateful[-1] and len(stateful) >= 2:
            self.prev_snapshot = stateful[-2]['state_after']
        # Walk in order: an undo row cancels the label right before it.
        labels = []
        for r in stateful:
            if 'ep' in r:
                labels.append(r)
            elif r.get('undo') and labels:
                labels.pop()
        self.history = [{'ep': r['ep'], 'speed': r['speed'],
                         'outcome': r['outcome'], 'sigma': r['sigma'],
                         'gains': r['gains'], 't': r.get('t'),
                         'duration': r.get('duration')} for r in labels][-500:]

    def arm(self, cfg):
        if self.phase != 'IDLE':
            return False
        self.armed_gains = self.learner.arm(cfg)
        self.armed_speed = self.learner.speed
        self.phase = 'ARMED'
        return True

    def go(self):
        if self.phase != 'ARMED':
            return False
        self.t_go = time.monotonic()
        self.phase = 'RUNNING'
        return True

    def done(self):
        if self.phase != 'RUNNING':
            return False
        self.last_duration = round(time.monotonic() - self.t_go, 1)
        self.phase = 'AWAITING_LABEL'
        return True

    def label(self, outcome, cfg):
        # Both labels are legal straight from RUNNING: one press ends the
        # run and labels it. Fail stops a crash instantly; Success means the
        # attempt finished to the student's standard.
        if outcome in ('fail', 'success', 'discard') and self.phase == 'RUNNING':
            self.done()
        if self.phase != 'AWAITING_LABEL':
            return False
        if outcome == 'discard':
            self.phase = 'IDLE'
            self.armed_gains = None
            self.learner.proposal = None
            return True
        self.prev_snapshot = self.learner.snapshot()
        gains = dict(self.armed_gains)
        speed = self.armed_speed
        sigma = self.learner.sigma
        # A pass must also keep pace: the fastest passing lap at this speed
        # sets the bar, and a pass slower than bar * duration_slack scores
        # as `slow`, which the learner treats as a rejection.
        shown = outcome
        if outcome == 'success':
            key = f'{speed:.3f}'
            bar = self.learner.best_dur.get(key)
            d = self.last_duration
            if bar is not None and d is not None and d > bar * cfg.get('duration_slack', 1.05):
                shown = 'slow'
                self.learner.label('fail', cfg)
            else:
                if d is not None:
                    self.learner.best_dur[key] = min(bar, d) if bar is not None else d
                self.learner.label('success', cfg)
        else:
            self.learner.label(outcome, cfg)
        outcome = shown
        row = {'ep': self.learner.episode, 't': time.strftime('%H:%M:%S'),
               'speed': round(speed, 3), 'gains': gains, 'sigma': round(sigma, 4),
               'outcome': outcome, 'duration': self.last_duration,
               'state_after': self.learner.snapshot()}
        with open(LEDGER, 'a') as f:
            f.write(json.dumps(row) + '\n')
        self.history.append({'ep': row['ep'], 'speed': row['speed'],
                             'outcome': outcome, 'sigma': row['sigma'],
                             'gains': gains, 't': row['t'],
                             'duration': self.last_duration})
        self.history = self.history[-500:]
        self.phase = 'IDLE'
        self.armed_gains = None
        return True

    def abort(self):
        if self.phase == 'IDLE':
            return False
        self.phase = 'IDLE'
        self.armed_gains = None
        self.learner.proposal = None
        return True

    def undo(self):
        """Revert exactly one update, sigma and counters included."""
        if self.phase != 'IDLE' or self.prev_snapshot is None:
            return False
        self.learner.restore(self.prev_snapshot)
        self.prev_snapshot = None
        if self.history:
            self.history.pop()
        with open(LEDGER, 'a') as f:
            f.write(json.dumps({'undo': True, 't': time.strftime('%H:%M:%S'),
                                'state_after': self.learner.snapshot()}) + '\n')
        return True


class EpsNode(Node):
    """The gap follower, driving only while an episode is RUNNING."""

    def __init__(self, session):
        super().__init__('eps')
        self.session = session
        with _lock:
            scan_topic = _cfg.get('scan_topic', '/scan')
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / 15.0, self._actuate)
        self._steer = 0.0
        self._speed = 0.0
        self._trim = 0.0
        self._v = 0.0
        self._v_stamp = 0.0
        self._last_error = 0.0
        self._last_speed_error = 0.0
        self._telemetry = {'error': None, 'steer': 0.0, 'speed_cmd': 0.0,
                           'scan_seen': False}

    def _odom_cb(self, msg):
        self._v = msg.twist.twist.linear.x
        self._v_stamp = time.monotonic()

    def _mean_at(self, msg, deg, window):
        n = len(msg.ranges)
        center = round((math.radians(-deg) - msg.angle_min) / msg.angle_increment)
        half = max(1, round(math.radians(window / 2) / msg.angle_increment))
        vals = [msg.ranges[(center + k) % n] for k in range(-half, half + 1)]
        vals = [r for r in vals if msg.range_min < r < msg.range_max]
        return sum(vals) / len(vals) if vals else msg.range_max

    def _scan_cb(self, msg):
        self._telemetry['scan_seen'] = True
        self._telemetry['v'] = round(self._v, 2)
        self._telemetry['odom_fresh'] = (time.monotonic() - self._v_stamp) < 0.5
        s = self.session
        if s.phase != 'RUNNING' or s.armed_gains is None:
            self._steer = 0.0
            self._speed = 0.0
            self._trim = 0.0
            self._last_error = 0.0
            self._last_speed_error = 0.0
            return
        with _lock:
            p = dict(_cfg)
        g = s.armed_gains
        la = max(p['lookahead'], 0.1)
        degs = list(range(-int(p['width']), int(p['width']) + 1, SEARCH_STEP))
        dists = [min(self._mean_at(msg, d, p['window']), la) for d in degs]
        corridor = list(dists)
        for j, d in enumerate(dists):
            if d >= la:
                continue
            block = math.degrees(math.atan2(HALF_CLEAR, d))
            span = int(block // SEARCH_STEP) + 1
            for i in range(max(0, j - span), min(len(degs), j + span + 1)):
                if abs(degs[i] - degs[j]) <= block and d < corridor[i]:
                    corridor[i] = d
        best = max(range(len(degs)), key=lambda i: corridor[i]
                   - CENTER_BIAS * abs(degs[i]) + p['side_weight'] * degs[i])
        error = degs[best] / p['width']
        cmd = g['kp'] * error + g['kd'] * (error - self._last_error)
        self._last_error = error
        self._steer = max(-1.0, min(1.0, cmd))

        # Speed, kept simple and identical to wallfollow: the staircase
        # value is the constant throttle when the learned speed gains are
        # zero; nonzero gains bend it toward the road-shaped target using
        # measured speed. Stale odometry falls back to the constant command.
        road = min(corridor[len(degs) // 2], corridor[best])
        target = s.armed_speed * p['max_mps'] * min(road / la, 1.0)
        serr = target - self._v
        gains_off = g['speed_kp'] == 0 and g['speed_kd'] == 0
        odom_stale = time.monotonic() - self._v_stamp > 0.5
        if s.armed_speed <= 1e-3:
            self._trim = 0.0
            self._speed = 0.0
        elif gains_off or odom_stale:
            self._trim = 0.0
            self._speed = s.armed_speed
        else:
            blocked = self._v < 0.05 and self._speed > 0.3
            if serr < 0 or not blocked:
                self._trim += g['speed_kp'] * serr \
                    + g['speed_kd'] * (serr - self._last_speed_error)
            self._trim = max(-1.0, min(0.3, self._trim))
            self._speed = max(0.0, min(1.0, s.armed_speed + self._trim))
        self._last_speed_error = serr
        self._telemetry.update({'error': round(error, 3),
                                'steer': round(self._steer, 3),
                                'speed_cmd': round(self._speed, 3)})

    def _actuate(self):
        out = AckermannDriveStamped()
        if self.session.phase == 'RUNNING':
            out.drive.speed = float(self._speed)
            out.drive.steering_angle = float(-self._steer)  # wire + = left
        self._pub.publish(out)


session: Session = None
node: EpsNode = None


def implied_pstar(cfg):
    try:
        return round(-math.log(cfg['a_down'])
                     / (math.log(cfg['a_up']) - math.log(cfg['a_down'])), 3)
    except (ValueError, ZeroDivisionError):
        return None


HYPER_KEYS = ('sigma0', 'a_up', 'a_down', 'sigma_min', 'sigma_max',
              'speed_step', 'speed_backoff', 'patience', 'confirm_runs',
              'sigma_kick', 'speed_floor', 'speed_max', 'duration_slack')


def init_locked():
    """The starting point locks the moment it is set, and on any training.
    Only Reset unlocks it."""
    return (INIT_MARK.exists() or session.learner.episode > 0
            or session.phase != 'IDLE' or len(session.history) > 0)


def write_init_to_yaml(nominal, start_speed, plant):
    """Persist the starting point so a service restart keeps it."""
    text = YAML_PATH.read_text()
    import re
    for k, v in nominal.items():
        text = re.sub(rf'(?m)^  {k}: [-\d.eE+]+', f'  {k}: {v}', text)
    text = re.sub(r'(?m)^speed_floor: [-\d.eE+]+', f'speed_floor: {start_speed}', text)
    for k, v in plant.items():
        text = re.sub(rf'(?m)^{k}: [-\d.eE+]+', f'{k}: {v}', text)
    YAML_PATH.write_text(text)


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def do_GET(self):
        if self.path == '/':
            self._send((BASE / 'eps.html').read_bytes(), 'text/html; charset=utf-8')
        elif self.path == '/state':
            ln = session.learner
            recent = [h for h in session.history if h['outcome'] != 'discard'][-20:]
            wins = sum(1 for h in recent if h['outcome'] == 'success')
            with _lock:
                cfg = dict(_cfg)
            body = json.dumps({
                'phase': session.phase, 'episode': ln.episode,
                'speed': round(ln.speed, 3), 'sigma': round(ln.sigma, 4),
                'incumbent': ln.gains(ln.theta),
                'armed': session.armed_gains, 'armed_speed': session.armed_speed,
                'best_speed': max((float(s) for s in ln.best), default=None),
                'pstar': implied_pstar(cfg),
                'observed': round(wins / len(recent), 2) if recent else None,
                'history': session.history[-200:],
                'telemetry': node._telemetry if node else {},
                'duration': session.last_duration,
                'best_lap': session.learner.best_dur.get(f'{ln.speed:.3f}'),
                'can_undo': session.prev_snapshot is not None,
                'hyper': {k: cfg[k] for k in HYPER_KEYS},
                'init': {**cfg['nominal'], 'start_speed': cfg['speed_floor'],
                         **{k: cfg[k] for k in PLANT_INIT_KEYS}},
                'plant': {'lookahead': cfg['lookahead'], 'max_mps': cfg['max_mps']},
                'init_locked': init_locked(),
            })
            self._send(body.encode(), 'application/json')
        elif self.path == '/ledger':
            data = LEDGER.read_bytes() if LEDGER.exists() else b''
            self._send(data, 'application/jsonl')
        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get('Content-Length') or 0)
        data = json.loads(self.rfile.read(length)) if length else {}
        with _lock:
            cfg = dict(_cfg)
        ok = False
        if self.path == '/episode/start':
            ok = session.arm(cfg) and session.go()
        elif self.path == '/episode/arm':
            ok = session.arm(cfg)
        elif self.path == '/episode/go':
            ok = session.go()
        elif self.path == '/episode/done':
            ok = session.done()
        elif self.path == '/episode/abort':
            ok = session.abort()
        elif self.path == '/episode/label':
            ok = session.label(data.get('result', ''), cfg)
        elif self.path == '/episode/undo':
            ok = session.undo()
        elif self.path == '/learner':
            if data.get('action') == 'reset':
                # Full clean slate: learner, charts, episode count. The old
                # ledger is archived beside the live one, never destroyed.
                if LEDGER.exists():
                    LEDGER.rename(BASE / time.strftime('ledger_%Y%m%d_%H%M%S.jsonl'))
                INIT_MARK.unlink(missing_ok=True)
                session.learner.reset(cfg)
                session.prev_snapshot = None
                session.history = []
                session.armed_gains = None
                session.last_duration = None
                session.phase = 'IDLE'
                ok = True
            elif data.get('action') == 'revert_best':
                ok = session.learner.revert_best(cfg)
        elif self.path == '/init':
            if init_locked():
                ok = False
            else:
                with _lock:
                    for k in GAIN_KEYS:
                        if k in data:
                            _cfg['nominal'][k] = float(data[k])
                    for k in PLANT_INIT_KEYS:
                        if k in data:
                            _cfg[k] = float(data[k])
                    if 'start_speed' in data:
                        _cfg['speed_floor'] = max(0.0, min(1.0, float(data['start_speed'])))
                    write_init_to_yaml(_cfg['nominal'], _cfg['speed_floor'],
                                       {k: _cfg[k] for k in PLANT_INIT_KEYS})
                    cfg = dict(_cfg)
                session.learner.reset(cfg)
                INIT_MARK.touch()
                ok = True
        elif self.path == '/hyper':
            with _lock:
                for k in HYPER_KEYS:
                    if k in data:
                        _cfg[k] = float(data[k]) if k not in ('patience', 'confirm_runs') \
                            else max(1, int(data[k]))
                # Sanity: floors can never exceed ceilings, and the live
                # sigma is pulled back into the allowed range immediately.
                _cfg['sigma_min'] = min(_cfg['sigma_min'], _cfg['sigma_max'])
                _cfg['speed_floor'] = min(_cfg['speed_floor'], _cfg['speed_max'])
                lo, hi = _cfg['sigma_min'], _cfg['sigma_max']
            session.learner.sigma = max(lo, min(hi, session.learner.sigma))
            with _lock:
                applied = {k: _cfg[k] for k in HYPER_KEYS}
            body = json.dumps({'ok': True, 'hyper': applied}).encode()
            self._send(body, 'application/json')
            return
        elif self.path == '/save':
            save_cfg()
            ok = True
        elif self.path == '/load':
            load_cfg()
            ok = True
        else:
            self.send_error(404)
            return
        body = json.dumps({'ok': ok}).encode()
        self._send(body, 'application/json')

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


def main():
    global session, node
    load_cfg()
    session = Session(dict(_cfg))
    rclpy.init()
    node = EpsNode(session)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f'EPS dashboard on http://0.0.0.0:{PORT} '
          f'(resumed at episode {session.learner.episode})')
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()


if __name__ == '__main__':
    main()
