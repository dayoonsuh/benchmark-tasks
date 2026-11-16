# Brace a dolly and push a wide case off it

The whole task (objects, scoring and scripted demo) is one task file for the `yam_arena` embodiment:
`dolly_unload.py`.

## Offline check (no GPU, a few seconds)

```bash
python tasks/dolly-unload/check_demo.py
```

## Play the scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/dolly-unload/dolly_unload.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the dolly still with one hand and push the heavy case off it onto the marked spot with the other." \
  --scorer task_score \
  --max-steps 500 --store-frames --log-dir ~/yam-eval/unload-demo
```

The right arm presses the dolly's back corner down with its closed fingers; the left arm pushes the case
to the right, off the deck and into the drop zone, then backs off. About 18 s.

`DOLLY_DEMO_NO_HOLD=1` plays the pushing arm alone: the dolly is dragged along (about 11 cm in testing) and the
case stays on it, scoring 0. `DOLLY_DEBUG=1` prints the case and dolly poses once a second.

## Run a trial (Dream-1)

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/dolly-unload/dolly_unload.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the dolly still with one hand and push the heavy case off it onto the marked spot with the other." \
  --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/unload-dream
```

## Scoring

`task_score` gives 1.0 when the episode ends in success (case off the dolly, upright and at rest in the drop
zone, the dolly held within 2 cm of its start throughout, held 1 s); otherwise the task file's `score()` at the
end of the run: 1.0 off and in the zone, 0.5 off but outside it, 0 if any part is still on the deck; halved if the
dolly moved more than 2 cm; 0 if the case was knocked over. `score()` is called every control step to track the
dolly's movement over the run.

## Runtime and seeds

- Run limit: 2,400 steps (2 minutes at 20 Hz)
- Wall-clock per trial on one L40S: about 3 minutes, plus about 1 minute of start-up
- Fixed layout (`-E object_jitter=0`); no seed needed
