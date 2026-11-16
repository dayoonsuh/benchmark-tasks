"""Offline checks, no Isaac Sim or GPU: the demo's IK and the scorer on made-up states.

    python tasks/dolly-unload/check_demo.py
"""

import math
from pathlib import Path

import numpy as np

from yam_arena import station, task_file
from yam_arena.demo import ArmPlan

TASK = Path(__file__).with_name("dolly_unload.py")
DT = 1.0 / 20
task = task_file.load(TASK)
m = task.module
ok = True

for arm, waypoints in task.demo().items():
    plan = ArmPlan(list(waypoints), station.BASES[arm])
    for t in np.arange(0.0, plan.duration + DT, DT):
        plan.action7(float(t))
    good = plan.worst_ik_mm < 5.0 and plan.worst_ik_deg < 3.0
    ok &= good
    print(f"IK {arm:5s}: {plan.duration:5.1f} s, worst {plan.worst_ik_mm:.1f} mm / {plan.worst_ik_deg:.1f} deg  {'PASS' if good else 'FAIL'}")


def st(spec, pos=None, tilt_deg=0.0):
    pos = np.array(spec.pos if pos is None else pos, dtype=float)
    h = np.array(spec.size) / 2
    half = math.radians(tilt_deg) / 2
    return task_file.ObjectState(pos, np.array([math.sin(half), 0, 0, math.cos(half)]), np.zeros(3), np.zeros(3), pos - h, pos + h)


def objs(case=None, dolly_dy=0.0, tilt=0.0):
    d = st(m.DOLLY, np.array(m.DOLLY.pos) + [0, dolly_dy, 0])
    c = st(m.CASE, case, tilt) if case is not None else st(m.CASE)
    return {"dolly": d, "case": c}


def slide(n, dy_end):
    return [objs(None, dy_end * k / n) for k in range(1, n + 1)]


ON_TABLE = m.TOP + m.CASE_H / 2
IN_ZONE = (m.DOLLY_X, m.CASE_GOAL_Y, ON_TABLE)
PAST_ZONE = (m.DOLLY_X, m.ZONE_Y0 - 0.05, ON_TABLE)
HALF_OFF = (m.DOLLY_X, m.DOLLY_Y - m.DOLLY_DY / 2, m.DECK_TOP + m.CASE_H / 2)
cases = {
    "nothing done": ([objs()], 0.0, False),
    "unloaded into the zone, dolly held": ([objs(), objs(IN_ZONE)], 1.0, True),
    "unloaded past the zone": ([objs(), objs(PAST_ZONE)], 0.5, False),
    "half off the deck": ([objs(), objs(HALF_OFF)], 0.0, False),
    "unloaded into the zone, dolly slid 4 cm away": ([objs()] + slide(10, 0.04) + [objs(IN_ZONE, 0.04)], 0.5, False),
    "case knocked over, then righted": ([objs(), objs(IN_ZONE, tilt=80), objs(IN_ZONE)], 0.0, False),
}
for name, (states, want, want_ok) in cases.items():
    m._reset_state()
    for s in states:
        got = m.score(s)
    got_ok = m.success(states[-1])
    good = abs(got - want) < 1e-6 and got_ok == want_ok
    ok &= good
    print(f"score {name:42s} {got:.2f} success {got_ok!s:5s} (want {want:.2f} {want_ok!s:5s})  {'PASS' if good else 'FAIL'}")
m._reset_state()
print("all checks passed" if ok else "SOME CHECKS FAILED")
