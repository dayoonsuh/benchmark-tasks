# Raise a heavy machine's edge with a pry bar and crib it up on a block

The whole task (objects, scoring and scripted demo) is one task file for the `yam_arena` embodiment:
`lever_and_crib.py`.

## Offline check (no GPU, a few seconds)

```bash
python tasks/lever-and-crib/check_demo.py
```

## Play the scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lever-and-crib/lever_and_crib.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press down on the pry bar to raise the machine's edge, push the wooden block under it, then release the bar so the machine rests on the block." \
  --scorer task_score \
  --max-steps 500 --store-frames --log-dir ~/yam-eval/crib-demo
```

The left arm presses the bar's free end down with its closed fingers (the edge rises about 11 mm); the right
arm pushes the block under the raised edge and backs off; the left arm eases off the bar and the edge settles on
the block. About 16 s.

Checks: `CRIB_PROBE=lift` plays the bar arm alone (the edge rises, scores 0.3); `CRIB_DEMO_NO_LIFT=1` plays the
block arm alone (the block is stopped by the overhang, scores 0). `CRIB_DEBUG=1` prints the edge height and the
block and bar poses once a second.

## Run a trial (Dream-1)

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/lever-and-crib/lever_and_crib.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press down on the pry bar to raise the machine's edge, push the wooden block under it, then release the bar so the machine rests on the block." \
  --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/crib-dream
```

## Scoring

`task_score` gives 1.0 when the episode ends in success (block upright under the overhang, the machine's edge
resting on it within 2 mm, the machine in place; held 1 s); otherwise the task file's `score()` at the end of the
run: 1.0 cribbed, 0.3 if the edge was raised at least 8 mm at some point, 0 if the machine was shoved more than
1.5 cm or tipped. While the bar still holds the edge up, it hangs about 3.5 mm above the block and does not count.

## Runtime and seeds

- Run limit: 2,400 steps (2 minutes at 20 Hz)
- Wall-clock per trial on one L40S: about 3 minutes, plus about 1 minute of start-up
- Fixed layout (`-E object_jitter=0`); no seed needed
