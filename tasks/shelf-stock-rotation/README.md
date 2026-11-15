# Restock a shelf lane first-in, first-out

The whole task (objects, scoring and scripted demo) is one task file for the `yam_arena` embodiment:
`shelf_stock_rotation.py`.

## Offline check (no GPU, a few seconds)

```bash
python tasks/shelf-stock-rotation/check_demo.py
```

## Play the scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/shelf-stock-rotation/shelf_stock_rotation.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Restock this shelf lane first-in, first-out: move the two older boxes already on the shelf to the front of the lane, then put the two new boxes from the case behind them." \
  --scorer task_score \
  --max-steps 1400 --store-frames --log-dir ~/yam-eval/rotation-demo
```

The left arm moves the older boxes from slots 3 and 4 to slots 1 and 2 and moves off the shelf; the right arm
then takes the two new boxes from the case into slots 3 and 4 (it lifts the first one out of the case while it waits). About 41 s.

## Run a trial (Dream-1)

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/shelf-stock-rotation/shelf_stock_rotation.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Restock this shelf lane first-in, first-out: move the two older boxes already on the shelf to the front of the lane, then put the two new boxes from the case behind them." \
  --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/rotation-dream
```

## Scoring

`task_score` gives 1.0 when the episode ends in success (older boxes in slots 1-2, new boxes in slots 3-4, all
upright and at rest, held 1 s); otherwise the task file's `score()` at the end of the run: 0.2 per slot holding
the right stock, plus 0.2 when all four do. Doing nothing, or putting the new boxes in front, scores 0.

## Runtime and seeds

- Run limit: 3,600 steps (3 minutes at 20 Hz)
- Wall-clock per trial on one L40S: about 4 minutes, plus about 1 minute of start-up
- Fixed layout (`-E object_jitter=0`); no seed needed