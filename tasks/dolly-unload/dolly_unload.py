"""On the station table, unload a heavy case from a dolly onto a marked spot.

- Dolly (in the middle, loose, 300 g): a dark blue deck 20 cm front to back x 14 cm side to side, 4 mm thick, lying on
  the table. A case dragged across it drags it along, so it has to be held still.
- Case (on the dolly, loose, 600 g): a low brown carton 10 x 10 x 3 cm, much wider than the gripper opens, so it
  can't be picked up with one hand: it is pushed, not lifted. (A taller 8 x 8 x 6 cm case tipped over when pushed.)
- Drop zone: the patch of table just right of the dolly, between two low yellow corner marks.

One arm braces the dolly (presses its back corner down so it doesn't slide); the other pushes the case to the right,
off the deck and onto the drop zone. (Pushing a case UP onto a deck isn't possible here: a flat-bottomed case pushed
against even a 2 mm step tips over it; stepping down off the deck is fine.)

Switches (environment variables), for checking the task:
  DOLLY_DEMO_NO_HOLD=1  only the pushing arm moves: the dolly should be dragged along and the case not unloaded
  DOLLY_DEBUG=1         print the case and dolly poses once a second
"""

import math
import os

import numpy as np

from yam_arena.constants import PAD_TIP_PAST_TCP
from yam_arena.demo import FINGER_CLOSED, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# ---------------------------------------------------------------- dolly (loose)
DOLLY_X, DOLLY_Y = 0.25, 0.10
DOLLY_DX, DOLLY_DY, DOLLY_T = 0.20, 0.14, 0.004
DOLLY = Box(
    "dolly",
    pos=(DOLLY_X, DOLLY_Y, TOP + DOLLY_T / 2 + 0.0005),
    size=(DOLLY_DX, DOLLY_DY, DOLLY_T),
    mass=0.3,
    color=(0.12, 0.20, 0.45),
)
DECK_TOP = TOP + DOLLY_T

# ---------------------------------------------------------------- case (loose, on the dolly)
CASE_W, CASE_H = 0.10, 0.03
CASE = Box(
    "case", pos=(DOLLY_X, DOLLY_Y, DECK_TOP + CASE_H / 2 + 0.0005), size=(CASE_W, CASE_W, CASE_H), mass=0.6, color=(0.62, 0.45, 0.26)
)

# ---------------------------------------------------------------- drop zone (marked by two fixed corner marks)
ZONE_X0, ZONE_X1 = DOLLY_X - 0.06, DOLLY_X + 0.06
ZONE_Y1 = DOLLY_Y - DOLLY_DY / 2 - 0.005  # its left edge: 5 mm right of the dolly
ZONE_Y0 = ZONE_Y1 - 0.12
MARK = 0.01
YELLOW = (0.95, 0.80, 0.10)
MARKS = [
    Box("zone_mark_back", pos=(ZONE_X0 - MARK / 2, ZONE_Y0 + MARK / 2, TOP + 0.001), size=(MARK, MARK, 0.002), color=YELLOW),
    Box("zone_mark_front", pos=(ZONE_X1 + MARK / 2, ZONE_Y0 + MARK / 2, TOP + 0.001), size=(MARK, MARK, 0.002), color=YELLOW),
]

OBJECTS = [DOLLY, CASE, *MARKS]
FREE = {o.name: o for o in (DOLLY, CASE)}

# ---------------------------------------------------------------- scoring
STEADY_MOVE = 0.02  # the dolly may shift at most 2 cm at any time in the run
UPRIGHT_DEG = 10.0


def _tilt_deg(s) -> float:
    x, y = float(s.quat_xyzw[0]), float(s.quat_xyzw[1])
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def off_dolly(case, dolly) -> bool:
    """No part of the case over the deck, and the case standing on the table."""
    clear = any(float(case.high[i]) <= float(dolly.low[i]) or float(case.low[i]) >= float(dolly.high[i]) for i in (0, 1))
    return clear and float(case.low[2]) <= TOP + 0.002


def in_zone(case) -> bool:
    cx, cy = float(case.pos[0]), float(case.pos[1])
    return ZONE_X0 <= cx <= ZONE_X1 and ZONE_Y0 <= cy <= ZONE_Y1


# score() is called after every control step; it keeps how far the dolly has moved so far in this episode.
_state: dict = {}


def _reset_state() -> None:
    _state.clear()
    _state.update(move=0.0, tipped=False, last={}, n=0)


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
    d = objects["dolly"]
    _state["move"] = max(_state["move"], math.hypot(float(d.pos[0]) - DOLLY.pos[0], float(d.pos[1]) - DOLLY.pos[1]))
    if _tilt_deg(objects["case"]) > 30.0:
        _state["tipped"] = True  # a case knocked over has been dropped, whatever happens next
    _state["last"] = {n: np.asarray(objects[n].pos, dtype=float).copy() for n in FREE}


def _steady() -> bool:
    return _state["move"] <= STEADY_MOVE


DEBUG = os.environ.get("DOLLY_DEBUG") == "1"


def _debug(objects) -> None:
    _state["n"] += 1
    if _state["n"] % 20:
        return
    c, d = objects["case"], objects["dolly"]
    print(
        f"[dolly t={_state['n'] / 20:5.1f}s] case x {float(c.pos[0]) * 100:5.1f} y {float(c.pos[1]) * 100:5.1f} "
        f"bottom {float(c.low[2] - TOP) * 1000:5.1f} mm tilt {_tilt_deg(c):4.1f} | dolly x {float(d.pos[0]) * 100:5.1f} "
        f"y {float(d.pos[1]) * 100:5.1f} (max shift {_state['move'] * 1000:5.1f} mm) | off {off_dolly(c, d)} in zone {in_zone(c)}",
        flush=True,
    )


def score(objects) -> float:
    """1.0 case off the dolly, upright, in the drop zone; 0.5 if off the dolly and upright but outside the zone;
    halved if the dolly ever slid more than 2 cm; 0 if the case was ever knocked over."""
    _update(objects)
    if DEBUG:
        _debug(objects)
    c, d = objects["case"], objects["dolly"]
    if _state["tipped"] or _tilt_deg(c) > UPRIGHT_DEG or not off_dolly(c, d):
        return 0.0
    base = 1.0 if in_zone(c) else 0.5
    return round(base * (1.0 if _steady() else 0.5), 3)


def success(objects) -> bool:
    """Case off the dolly, upright and at rest in the drop zone, the dolly held steady throughout; must hold 1 s.
    Reads the record that score() keeps and doesn't update it."""
    if not _state or _state["tipped"] or not _steady():
        return False
    c, d = objects["case"], objects["dolly"]
    return off_dolly(c, d) and in_zone(c) and _tilt_deg(c) <= UPRIGHT_DEG and c.resting()


def distance(objects) -> float:
    """Metres from the case's centre to the drop zone's centre."""
    c = objects["case"]
    return math.hypot(float(c.pos[0]) - (ZONE_X0 + ZONE_X1) / 2, float(c.pos[1]) - (ZONE_Y0 + ZONE_Y1) / 2)


# ---------------------------------------------------------------- demo
TRAVEL_Z = TOP + 0.14
FINGER_HALF = 0.012  # rough half-thickness of the closed finger pair along y (its face pushes the case)
PUSH_Z = DECK_TOP + 0.005 + PAD_TIP_PAST_TCP  # closed finger tips 5 mm above the deck: push low, so the case slides, not tips
PUSH_Y0 = CASE.pos[1] + CASE_W / 2 + FINGER_HALF + 0.012  # start 1.2 cm left of the case
CASE_GOAL_Y = ZONE_Y1 - 0.06  # the case's centre ends 6 cm into the drop zone (its near edge 1 cm in)
# The pushing arm falls short: it sags toward its base, and stalls against the case's friction (a 1.5 kg case stopped
# 8 cm short, half on the deck). The case is lighter and the push is aimed 5 cm past the goal, so the arm keeps pushing
# until the case is off; aimed exactly, an unhindered push would still leave the case inside the drop zone.
LEFT_PUSH_FIX = 0.05
PUSH_Y1 = CASE_GOAL_Y + CASE_W / 2 + FINGER_HALF - LEFT_PUSH_FIX
PUSH_S = 10.0  # slow and steady
BRACE_XY = (DOLLY_X - DOLLY_DX / 2 + 0.015, DOLLY_Y - 0.02)  # the deck's back corner area, 3.5 cm behind the case's path
BRACE_TOUCH_Z = DECK_TOP + PAD_TIP_PAST_TCP + 0.005
BRACE_PRESS_Z = DECK_TOP + PAD_TIP_PAST_TCP - 0.006  # tips commanded 6 mm into the deck: press it onto the table


def _push(start_wait: float) -> list[Waypoint]:
    x = CASE.pos[0]
    return [
        Waypoint(start_wait, (x, PUSH_Y0, TRAVEL_Z), FINGER_CLOSED, "left of the case, fingers closed"),
        Waypoint(1.5, (x, PUSH_Y0, PUSH_Z), FINGER_CLOSED, "down beside the case"),
        Waypoint(PUSH_S, (x, PUSH_Y1, PUSH_Z), FINGER_CLOSED, "push it off the dolly"),
        Waypoint(0.8, (x, PUSH_Y1 + 0.02, PUSH_Z), FINGER_CLOSED, "back off"),
        Waypoint(1.2, (x, PUSH_Y1 + 0.02, TRAVEL_Z), FINGER_CLOSED, "rise"),
    ]


def _brace(hold_s: float) -> list[Waypoint]:
    x, y = BRACE_XY
    return [
        Waypoint(1.5, (x, y, TRAVEL_Z), FINGER_CLOSED, "over the dolly's back corner, fingers closed"),
        Waypoint(1.5, (x, y, BRACE_TOUCH_Z), FINGER_CLOSED, "lower onto the deck"),
        Waypoint(0.8, (x, y, BRACE_PRESS_Z), FINGER_CLOSED, "press it down"),
        Waypoint(hold_s, (x, y, BRACE_PRESS_Z), FINGER_CLOSED, "hold it still"),
        Waypoint(1.0, (x, y, TRAVEL_Z), FINGER_CLOSED, "rise"),
    ]


def demo():
    """Right arm presses the dolly's back corner down; left arm pushes the case off it to the right, backs off and
    rises; then the right arm lifts off."""
    braced = 1.5 + 1.5 + 0.8
    if os.environ.get("DOLLY_DEMO_NO_HOLD") == "1":
        return {"left": _push(1.5)}
    push = _push(braced + 0.5)
    pushed = sum(w.duration for w in push[:4])  # backed off the case
    return {"left": push, "right": _brace(pushed + 0.5 - braced)}