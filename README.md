# EPS Dashboard

Episodic Policy Search for the Neoracer, on port 8084. The gap follower from the wallfollow dashboard is the plant; a (1+1) evolution strategy learns its four PID coefficients from one Success or Fail press per episode. Sustained success raises the speed a notch; the lab's result is the highest speed the car holds and how many attempts it took.

## Installation

On the car:

```
git clone https://github.com/Neobotics-Foundation-Inc/eps_dashboard.git
bash eps_dashboard/setup.sh
```

The service runs the files where the checkout sits; nothing is copied. A first install leaves it stopped and disabled.

```
bash setup.sh enable     # start now and at every boot
bash setup.sh disable    # stop now and keep off across boots
bash setup.sh restart    # restart, taking port 8084 back first
bash setup.sh remove     # uninstall the unit; the checkout is kept
```

Dashboard, when enabled: `http://<car-ip>:8084`. Disable wallfollow, pursuit, and neoracer-autonomy while EPS runs: only one thing may publish /drive.

## Workflow

1. Students enter starting gains and speed in the Starting point card and press Set. It locks immediately; only Reset unlocks it.
2. START RUN arms a mutated gain proposal and drives it. SUCCESS or FAIL ends the run and is the entire training signal; Discard voids a spoiled run, Undo takes back exactly one label.
3. Success accepts the proposal and grows the mutation size sigma; failure keeps the old gains and shrinks it. confirm_runs successes in a row raise the speed; patience fails in a row back it off.
4. A pass must also keep pace: slower than the best lap at that speed times duration_slack scores as slow and is rejected. Green, gold, red dots on the staircase chart.
5. Reset archives the ledger, clears the charts, and returns gains and speed to the starting point.

## Speed control

The staircase speed is a 0..1 command; times max_mps (the car's top speed, in eps.yaml) it becomes a real m/s target, scaled by the road actually available ahead. speed_kp and speed_kd regulate the measured odometry speed onto that target, so battery and surface changes are corrected. Odometry silent for half a second cuts the throttle.

## Files

eps.yaml holds the learner hyperparameters (hover any panel field for what it does), the plant settings, and the nominal gain vector that scales the search. ledger.jsonl is the session record, one row per label with the full learner state; Reset archives it as ledger_TIMESTAMP.jsonl. Both stay on the car and out of git.

## Safety

The neoracer mux forwards /drive with no software deadman; the transmitter's SWB switch is the physical gate. The car drives only while an episode is RUNNING, speed ships at 0, speed_max caps the staircase, proposals are clamped to 0.2x..5x nominal, and losing odometry stops the car.
