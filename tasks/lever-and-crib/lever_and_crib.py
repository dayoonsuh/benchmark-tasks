"""On the station table, raise one side of a heavy machine with a pry bar and crib it up on a block.

- Machine (loose, 2 kg): a grey housing 10 cm front to back x 16 cm side to side x 6 cm tall, far too wide to grip.
  It stands on a fixed steel plinth 4 cm tall that it overhangs by 3 cm on its left side, so its left edge sits
  4 cm above the table with a gap underneath.
- Pry bar (loose, 200 g): a 12 cm steel bar lying side to side across a fixed fulcrum block (2.7 cm tall). Its right
  tip is under the machine's overhang; its left end sticks out 8 cm past the fulcrum (2:1 leverage).
- Crib block (loose, 100 g): a wooden block 5 cm front to back x 2.5 cm wide x 4.8 cm tall, behind the machine. It is
  taller than the 4 cm gap, so it only fits under the overhang while the edge is raised.

One arm presses the bar's free end down, which lifts the machine's left edge; while it holds the edge up, the other
arm pushes the block under the overhang; then the bar is let go and the machine's edge settles on the block.

Switches (environment variables), for checking the task:
  CRIB_PROBE=lift        only the bar arm moves (press, hold, release): how far does the edge rise?
  CRIB_DEMO_NO_LIFT=1    only the block arm moves: the block should be stopped by the overhang
  CRIB_DEBUG=1           print the machine edge height and the block and bar poses once a second
"""

import math
import os

import numpy as np

from yam_arena.constants import PAD_TIP_PAST_TCP
from yam_arena.demo import FINGER_CLOSED, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL = (0.45, 0.47, 0.50)
# ---------------------------------------------------------------- plinth (fixed) and machine (loose)
PLINTH_H = 0.04
PLINTH_TOP = TOP + PLINTH_H
MX0, MX1 = 0.22, 0.32  # machine, front to back
MY0, MY1 = -0.13, 0.03  # machine, right to left; its left 3 cm overhang the plinth
M_H = 0.06
OVERHANG = 0.03
PLINTH = Box(
    "plinth",
    pos=((MX0 + MX1) / 2, (MY0 - 0.01 + MY1 - OVERHANG) / 2, TOP + PLINTH_H / 2),
    size=(MX1 - MX0, MY1 - OVERHANG - (MY0 - 0.01), PLINTH_H),  # no wider than the machine: the pushing fingers pass its back
    color=STEEL,
)
MACHINE = Box(
    "machine",
    pos=((MX0 + MX1) / 2, (MY0 + MY1) / 2, PLINTH_TOP + M_H / 2 + 0.0005),
    size=(MX1 - MX0, MY1 - MY0, M_H),
    mass=2.0,
    color=(0.55, 0.58, 0.55),
)

# ---------------------------------------------------------------- fulcrum (fixed) and pry bar (loose)
BAR_X = 0.30  # near the machine's front, clear of where the block goes
BAR_T, BAR_W = 0.012, 0.02
BAR_TIP_Y = MY1 - OVERHANG + 0.005  # 5 mm in from the plinth's edge, under the overhang
FULCRUM_Y = MY1 + 0.015  # 1.5 cm left of the machine's edge
BAR_END_Y = FULCRUM_Y + 2 * (FULCRUM_Y - BAR_TIP_Y)  # 2:1 leverage
FULCRUM_H = PLINTH_H - 0.001 - BAR_T  # the bar's top sits 1 mm under the overhang
FULCRUM = Box("fulcrum", pos=(BAR_X, FULCRUM_Y, TOP + FULCRUM_H / 2), size=(0.03, 0.01, FULCRUM_H), color=STEEL)
BAR = Box(
    "pry_bar",
    pos=(BAR_X, (BAR_TIP_Y + BAR_END_Y) / 2, TOP + FULCRUM_H + BAR_T / 2 + 0.0002),
    size=(BAR_W, BAR_END_Y - BAR_TIP_Y, BAR_T),
    mass=0.2,
    color=(0.25, 0.25, 0.28),
)

# ---------------------------------------------------------------- crib block (loose)
BLOCK_X, BLOCK_Y, BLOCK_Z = 0.05, 0.025, 0.048
BLOCK_Y_POS = MY1 - OVERHANG / 2  # in line with the middle of the overhang
BLOCK = Box(
    "crib_block",
    pos=(MX0 - 0.04 - BLOCK_X / 2, BLOCK_Y_POS, TOP + BLOCK_Z / 2 + 0.0005),
    size=(BLOCK_X, BLOCK_Y, BLOCK_Z),
    mass=0.1,
    color=(0.72, 0.55, 0.32),
)

OBJECTS = [PLINTH, FULCRUM, MACHINE, BAR, BLOCK]
FREE = {o.name: o for o in (MACHINE, BAR, BLOCK)}

# ---------------------------------------------------------------- scoring
RAISED = 0.005  # the edge counts as raised 5 mm above the plinth top
LIFTED = 0.008  # ... and as lifted (partial credit) once it has been 8 mm up at any time
MOVE_MAX = 0.015  # the machine may not be shoved more than 1.5 cm


def _rot(s) -> np.ndarray:
    x, y, z, w = (float(v) for v in s.quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _tilt_deg(s) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(s)[2, 2])))))


def edge_z(machine) -> float:
    """Height of the middle of the machine's left bottom edge."""
    p = np.asarray(machine.pos, dtype=float) + _rot(machine) @ np.array([0.0, MACHINE.size[1] / 2, -M_H / 2])
    return float(p[2])


def machine_ok(machine) -> bool:
    shifted = math.hypot(float(machine.pos[0]) - MACHINE.pos[0], float(machine.pos[1]) - MACHINE.pos[1])
    return shifted <= MOVE_MAX and _tilt_deg(machine) <= 15.0


def cribbed(machine, block) -> bool:
    """The block stands upright on the table under the overhang and the machine's edge rests on it."""
    bx, by = float(block.pos[0]), float(block.pos[1])
    under = float(machine.low[0]) + 0.01 <= bx <= float(machine.high[0]) - 0.005 and MY1 - OVERHANG - 0.005 <= by <= MY1 + 0.005
    upright = _tilt_deg(block) <= 10.0 and float(block.low[2]) <= TOP + 0.003
    e = edge_z(machine)
    # within 2 mm: with the bar still pressed the edge hangs about 3.5 mm above the block, which must not count
    return under and upright and e >= PLINTH_TOP + RAISED and abs(float(block.high[2]) - e) <= 0.002


# score() is called after every control step; it keeps the highest the edge has been in this episode.
_state: dict = {}


def _reset_state() -> None:
    _state.clear()
    _state.update(peak=0.0, last={}, n=0)


def _new_episode(objects) -> bool:
    """A reset puts the loose objects back where they started in one step; nothing moves that far that fast."""
    last = _state.get("last")
    if not last:
        return True
    for name, spec in FREE.items():
        p = np.asarray(objects[name].pos, dtype=float)
        if np.linalg.norm(p - last[name]) > 0.05 and math.hypot(p[0] - spec.pos[0], p[1] - spec.pos[1]) < 0.035:
            return True
    return False


def _update(objects) -> None:
    if _new_episode(objects):
        _reset_state()
    _state["peak"] = max(_state["peak"], edge_z(objects["machine"]) - PLINTH_TOP)
    _state["last"] = {n: np.asarray(objects[n].pos, dtype=float).copy() for n in FREE}


DEBUG = os.environ.get("CRIB_DEBUG") == "1"


def _debug(objects) -> None:
    _state["n"] += 1
    if _state["n"] % 20:
        return
    m, b, r = objects["machine"], objects["crib_block"], objects["pry_bar"]
    print(
        f"[crib t={_state['n'] / 20:5.1f}s] edge {(edge_z(m) - PLINTH_TOP) * 1000:5.1f} mm (peak {_state['peak'] * 1000:5.1f}) "
        f"machine shift {math.hypot(float(m.pos[0]) - MACHINE.pos[0], float(m.pos[1]) - MACHINE.pos[1]) * 1000:4.1f} mm "
        f"| block x {float(b.pos[0]) * 100:5.1f} y {float(b.pos[1]) * 100:5.1f} top {(float(b.high[2]) - TOP) * 1000:5.1f} mm "
        f"tilt {_tilt_deg(b):4.1f} | bar end z {(float(r.low[2]) - TOP) * 1000:5.1f} mm | cribbed {cribbed(m, b)}",
        flush=True,
    )


def score(objects) -> float:
    """1.0 machine edge resting raised on the upright block; 0.3 if the edge was lifted 8 mm at some point but is
    not cribbed at the end; 0 if the machine was shoved off its place or knocked over."""
    _update(objects)
    if DEBUG:
        _debug(objects)
    m, b = objects["machine"], objects["crib_block"]
    if not machine_ok(m):
        return 0.0
    if cribbed(m, b):
        return 1.0
    return 0.3 if _state["peak"] >= LIFTED else 0.0


def success(objects) -> bool:
    """Cribbed, with the machine and block at rest; must hold 1 s (so a bar still being pressed doesn't count once
    released). Doesn't update the record."""
    m, b = objects["machine"], objects["crib_block"]
    return machine_ok(m) and cribbed(m, b) and m.resting() and b.resting()


def distance(objects) -> float:
    """Metres from the block's centre to the middle of the overhang."""
    b = objects["crib_block"]
    return math.hypot(float(b.pos[0]) - (MX0 + 0.025), float(b.pos[1]) - BLOCK_Y_POS)


# ---------------------------------------------------------------- demo
TRAVEL_Z = TOP + 0.14
PRESS_Y = BAR_END_Y - 0.01  # 1 cm in from the bar's free end
BAR_TOP0 = TOP + FULCRUM_H + BAR_T  # the bar's top before it is pressed
TOUCH_Z = BAR_TOP0 + PAD_TIP_PAST_TCP + 0.005
PRESS_Z = TOP + BAR_T + PAD_TIP_PAST_TCP - 0.004  # the bar's end pushed to the table, tips 4 mm further
FINGER_HALF = 0.012  # rough half-thickness of the closed finger pair along x (its face pushes the block)
PUSH_Z = TOP + 0.005 + PAD_TIP_PAST_TCP  # closed finger tips 5 mm above the table
PUSH_X0 = BLOCK.pos[0] - BLOCK_X / 2 - FINGER_HALF - 0.012  # 1.2 cm behind the block
PUSH_X1 = MX0 - 0.007 - FINGER_HALF  # the fingers stop 7 mm short of the machine's back face: the block ends with
# its back 7 mm out from under the overhang, its front 4.3 cm in (under the edge, clear of the bar)
PUSH_S = 4.0


def _bar(hold_s: float) -> list[Waypoint]:
    x, y = BAR_X, PRESS_Y
    return [
        Waypoint(1.5, (x, y, TRAVEL_Z), FINGER_CLOSED, "over the bar's free end, fingers closed"),
        Waypoint(1.5, (x, y, TOUCH_Z), FINGER_CLOSED, "onto the bar"),
        Waypoint(2.0, (x, y, PRESS_Z), FINGER_CLOSED, "press it down: the edge rises"),
        Waypoint(hold_s, (x, y, PRESS_Z), FINGER_CLOSED, "hold the edge up"),
        Waypoint(2.0, (x, y, TOUCH_Z + 0.01), FINGER_CLOSED, "ease off: the edge settles on the block"),
        Waypoint(1.0, (x, y, TRAVEL_Z), FINGER_CLOSED, "rise"),
    ]


def _block(start_wait: float) -> list[Waypoint]:
    y = BLOCK_Y_POS
    return [
        Waypoint(start_wait, (PUSH_X0, y, TRAVEL_Z), FINGER_CLOSED, "behind the block, fingers closed"),
        Waypoint(1.5, (PUSH_X0, y, PUSH_Z), FINGER_CLOSED, "down behind the block"),
        Waypoint(PUSH_S, (PUSH_X1, y, PUSH_Z), FINGER_CLOSED, "push it under the raised edge"),
        Waypoint(0.8, (PUSH_X1 - 0.03, y, PUSH_Z), FINGER_CLOSED, "back off"),
        Waypoint(1.0, (PUSH_X1 - 0.03, y, TRAVEL_Z), FINGER_CLOSED, "rise"),
    ]


def demo():
    """Left arm presses the bar's free end and holds the machine's edge up; right arm pushes the crib block under it
    and backs off; then the left arm eases off the bar and the edge settles on the block."""
    pressed = 1.5 + 1.5 + 2.0
    if os.environ.get("CRIB_PROBE") == "lift":
        return {"left": _bar(3.0)}
    block = _block(pressed + 0.5)
    if os.environ.get("CRIB_DEMO_NO_LIFT") == "1":
        return {"right": block}
    block_done = sum(w.duration for w in block[:4])  # backed off
    return {"left": _bar(block_done + 0.5 - pressed), "right": block}
