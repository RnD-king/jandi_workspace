# Action 31: single left turn in WALK stance

`vision/msg/ActionCommand.msg` defines `TURN_LEFT_IN_PLACE=31`.
The core emits it as a locomotion action for the near-left BALL/GOAL/HURDLE
approach region. It requires WALK stance, executes once, and does not emit READY.
The existing actions and their numeric IDs are retained.

The motion author's original files from `RnD-king/2026_motion/10.14.3` are used:

```text
31 -> forward_walk/motions/좌회전_최종.json
28 -> forward_walk/motions/우회전_최종.json
```

Both files are copied without modifying their keyframes. Each is one motion
containing four keyframes. Missing or invalid files are rejected before ACK.
Existing ACK/READY/DONE and one-slot reservation rules still apply.

The motion list counts for existing right-turn-forward actions remain unchanged:
12 has four entries; 25 has three; 26 has two.
Action 28 retains the public name `STEP_FORWARD_RIGHT_ONE_HUDDLE` and now plays
the in-place right-turn motion directly, matching the motion author's mapping.
