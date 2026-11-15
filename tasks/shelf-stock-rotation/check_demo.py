"""Offline checks, no Isaac Sim or GPU: the demo's IK, the gripper opening, and the scorer on made-up states.

    python tasks/shelf-stock-rotation/check_demo.py
"""

import math
from pathlib import Path

import numpy as np

from yam_arena import station, task_file
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import ArmPlan

TASK = Path(__file__).with_name("shelf_stock_rotation.py")
DT = 1.0 / 20
task = task_file.load(TASK)
m = task.module
ok = True

# 1) the demo is within reach
for arm, waypoints in task.demo().items():
    plan = ArmPlan(list(waypoints), station.BASES[arm])
    for t in np.arange(0.0, plan.duration + DT, DT):
        plan.action7(float(t))
    good = plan.worst_ik_mm < 5.0 and plan.worst_ik_deg < 3.0
    ok &= good
    print(f"IK {arm:5s}: {plan.duration:5.1f} s, worst {plan.worst_ik_mm:.1f} mm / {plan.worst_ik_deg:.1f} deg  {'PASS' if good else 'FAIL'}")

# 2) what the gripper closes across
print(f"gripper: FINGER_OPEN={FINGER_OPEN} FINGER_CLOSED={FINGER_CLOSED} PAD_TIP_PAST_TCP={PAD_TIP_PAST_TCP}")
print(f"  closes across {m.PY * 100:.1f} cm (box width); finger tips {(m.TOP + m.PZ / 2 - PAD_TIP_PAST_TCP - m.TOP - m.DIV_H) * 1000:.0f} mm above the dividers")


# 3) the scorer
def st(spec, pos=None, tilt_deg=0.0):
    pos = np.array(spec.pos if pos is None else pos, dtype=float)
    h = np.array(spec.size) / 2
    half = math.radians(tilt_deg) / 2
    return task_file.ObjectState(pos, np.array([math.sin(half), 0, 0, math.cos(half)]), np.zeros(3), np.zeros(3), pos - h, pos + h)


def objs(**where):
    """where: box name -> slot index (0..3), or a (x, y) position, or ("tipped", slot)."""
    out = {b.name: st(b) for b in (*m.OLD, *m.NEW)}
    for name, w in where.items():
        spec = next(b for b in (*m.OLD, *m.NEW) if b.name == name)
        z = m.TOP + m.PZ / 2
        if isinstance(w, int):
            out[name] = st(spec, (m.SLOTS[w], m.LANE_Y, z))
        elif w[0] == "tipped":
            out[name] = st(spec, (m.SLOTS[w[1]], m.LANE_Y, z), tilt_deg=60)
        else:
            out[name] = st(spec, (w[0], w[1], z))
    return out


cases = {
    "nothing done": (objs(), 0.0, False),
    "rotated correctly": (objs(old_1=0, old_2=1, new_1=2, new_2=3), 1.0, True),
    "rotated, older boxes swapped": (objs(old_1=1, old_2=0, new_1=3, new_2=2), 1.0, True),
    "older boxes pulled forward only": (objs(old_1=0, old_2=1), 0.4, False),
    "new stock put in front (no rotation)": (objs(new_1=0, new_2=1), 0.0, False),
    "new in front, old behind": (objs(new_1=0, new_2=1, old_1=2, old_2=3), 0.0, False),
    "one new box tipped over": (objs(old_1=0, old_2=1, new_1=2, new_2=("tipped", 3)), 0.6, False),
    "one older box left in the case area": (objs(old_1=0, old_2=(m.CASE_X, m.CASE_Y + 0.08), new_1=2, new_2=3), 0.6, False),
}
for name, (state, want, want_ok) in cases.items():
    got, got_ok = m.score(state), m.success(state)
    good = abs(got - want) < 1e-6 and got_ok == want_ok
    ok &= good
    print(f"score {name:40s} {got:.2f} success {got_ok!s:5s} (want {want:.2f} {want_ok!s:5s})  {'PASS' if good else 'FAIL'}")
print("all checks passed" if ok else "SOME CHECKS FAILED")
