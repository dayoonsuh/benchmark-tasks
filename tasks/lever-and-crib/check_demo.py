"""Offline checks, no Isaac Sim or GPU: the demo's IK and the scorer on made-up states.

    python tasks/lever-and-crib/check_demo.py
"""

import math
from pathlib import Path

import numpy as np

from yam_arena import station, task_file
from yam_arena.demo import ArmPlan

TASK = Path(__file__).with_name("lever_and_crib.py")
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


def st(spec, pos=None, roll_deg=0.0, pivot_y=None):
    """A state for spec; roll_deg rotates it about the x axis (left edge up for positive), about pivot_y if given."""
    pos = np.array(spec.pos if pos is None else pos, dtype=float)
    r = math.radians(roll_deg)
    if pivot_y is not None:  # rotate about the bottom edge at pivot_y
        piv = np.array([pos[0], pivot_y, pos[2] - spec.size[2] / 2])
        rel = pos - piv
        rel = np.array([rel[0], rel[1] * math.cos(r) - rel[2] * math.sin(r), rel[1] * math.sin(r) + rel[2] * math.cos(r)])
        pos = piv + rel
    q = np.array([math.sin(r / 2), 0, 0, math.cos(r / 2)])
    h = np.array(spec.size) / 2
    c, s_ = abs(math.cos(r)), abs(math.sin(r))
    ext = np.array([h[0], h[1] * c + h[2] * s_, h[1] * s_ + h[2] * c])
    return task_file.ObjectState(pos, q, np.zeros(3), np.zeros(3), pos - ext, pos + ext)


def roll_for(rise):
    """The roll that lifts the machine's left edge by rise, pivoting on its right bottom edge."""
    return math.degrees(math.asin(rise / m.MACHINE.size[1]))


BLOCK_IN = (m.MX0 + 0.007 + m.BLOCK_X / 2 - 0.007 + 0.0, m.BLOCK_Y_POS, m.TOP + m.BLOCK_Z / 2)
RISE_ON_BLOCK = (m.BLOCK_Z - m.PLINTH_H) * m.MACHINE.size[1] / (m.MACHINE.size[1] - m.OVERHANG / 2 + m.BLOCK_Y / 2 - 0.0)


def objs(rise=0.0, block=None, machine_shift=0.0):
    pos = np.array(m.MACHINE.pos) + [0, machine_shift, 0]
    mach = st(m.MACHINE, pos, roll_for(rise), pivot_y=pos[1] - m.MACHINE.size[1] / 2) if rise else st(m.MACHINE, pos)
    return {"machine": mach, "pry_bar": st(m.BAR), "crib_block": st(m.BLOCK, block) if block else st(m.BLOCK)}


cases = {
    "nothing done": ([objs()], 0.0, False),
    "lifted 1.5 cm, never cribbed": ([objs(), objs(0.015), objs()], 0.3, False),
    "cribbed": ([objs(), objs(0.015), objs(RISE_ON_BLOCK, BLOCK_IN)], 1.0, True),
    "block under, edge not on it (still pressed high)": ([objs(), objs(0.015, BLOCK_IN)], 0.3, False),
    "block under, edge held 3.5 mm above it": ([objs(), objs(0.015), objs(RISE_ON_BLOCK + 0.0035, BLOCK_IN)], 0.3, False),
    "machine shoved 3 cm": ([objs(), objs(0.0, None, 0.03)], 0.0, False),
}
for name, (states, want, want_ok) in cases.items():
    m._reset_state()
    for s in states:
        got = m.score(s)
    got_ok = m.success(states[-1])
    good = abs(got - want) < 1e-6 and got_ok == want_ok
    ok &= good
    print(f"score {name:48s} {got:.2f} success {got_ok!s:5s} (want {want:.2f} {want_ok!s:5s})  {'PASS' if good else 'FAIL'}")
m._reset_state()
print("all checks passed" if ok else "SOME CHECKS FAILED")
