"""On the station table, one shelf lane to restock first-in, first-out.

- Shelf (fixed): three lanes running front to back (front = the side nearest the robot), 5.5 cm wide inside, between
  grey dividers 1 cm tall, with a 1.2 cm price-channel lip along the front and a 1.2 cm back stop. The middle lane is
  the one to restock; the lanes either side are full of other products (fixed).
- Older stock: two identical orange product boxes (4 cm deep x 3.5 cm wide x 5 cm tall, 50 g), already in the middle
  lane but at the back (slots 3 and 4): shoppers have taken the ones in front. Loose.
- New stock: two more of the same boxes, just delivered, each in its own snug cell of an open cardboard case (1 cm
  walls) in front of the right arm. Loose.

First-in, first-out: the older boxes go to the front of the lane (slots 1 and 2), the new ones behind them (slots 3
and 4), so the older stock sells first.
"""

import math
import os

from yam_arena.demo import FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# ---------------------------------------------------------------- product
PX, PY, PZ = 0.04, 0.035, 0.05  # depth (front to back), width, height: 8 cm boxes met the gripper body before the fingers
# reached them; at 5 cm the top sits 2 cm above the TCP, as the example's cube does
PRODUCT = (0.95, 0.55, 0.12)

# ---------------------------------------------------------------- shelf (fixed)
LANE_Y = -0.025  # 2.5 cm toward the right arm, which sets boxes down in the far slots; the left arm only picks there
LANE_IN = 0.055  # inside width: 1 cm clear of a box on each side (5 mm let a box land on a divider)
DIV_T, DIV_H = 0.004, 0.01  # low, so the finger tips clear them
LANE_PITCH = 0.11  # neighbouring lanes, far enough out that the open fingers clear their products (7 cm was too close)
LANE_X0 = 0.10  # inside face of the front lip
SLOT_PITCH = PX + 0.01  # 1 cm between boxes: with 4 mm, a box set down beside another caught on it and tipped
SLOTS = [LANE_X0 + 0.003 + PX / 2 + k * SLOT_PITCH for k in range(4)]  # slot 1 (front) .. slot 4 (back)
LANE_X1 = SLOTS[-1] + PX / 2 + 0.004  # inside face of the back stop
LIP_T, LIP_H = 0.006, 0.012
STOP_T, STOP_H = 0.006, 0.012  # low, like the lip: a 6 cm stop caught the gripper over the back slot
SHELF_GREY = (0.62, 0.64, 0.66)
NEIGHBOUR_COLORS = {-1: (0.20, 0.55, 0.30), 1: (0.75, 0.15, 0.20)}


def _shelf() -> list[Box]:
    parts = []
    lanes = (-1, 0, 1)
    span_y = (len(lanes) - 1) * LANE_PITCH + LANE_IN + 2 * DIV_T
    length = LANE_X1 - LANE_X0
    mid_x = (LANE_X0 + LANE_X1) / 2
    parts.append(Box("shelf_lip", pos=(LANE_X0 - LIP_T / 2, LANE_Y, TOP + LIP_H / 2), size=(LIP_T, span_y, LIP_H), color=SHELF_GREY))
    parts.append(Box("shelf_stop", pos=(LANE_X1 + STOP_T / 2, LANE_Y, TOP + STOP_H / 2), size=(STOP_T, span_y, STOP_H), color=SHELF_GREY))
    for i in lanes:
        y = LANE_Y + i * LANE_PITCH
        for side in (-1, 1):
            parts.append(
                Box(
                    f"divider_{i + 1}_{'r' if side < 0 else 'l'}",
                    pos=(mid_x, y + side * (LANE_IN / 2 + DIV_T / 2), TOP + DIV_H / 2),
                    size=(length, DIV_T, DIV_H),
                    color=SHELF_GREY,
                )
            )
        if i:  # the neighbouring lanes are full of other products
            for k, x in enumerate(SLOTS):
                parts.append(Box(f"neighbour_{i + 1}_{k}", pos=(x, y, TOP + PZ / 2), size=(PX, PY, PZ), color=NEIGHBOUR_COLORS[i]))
    return parts


# ---------------------------------------------------------------- stock (loose)
def _box(name, x, y):
    return Box(name, pos=(x, y, TOP + PZ / 2 + 0.0005), size=(PX, PY, PZ), mass=0.05, color=PRODUCT)


OLD = [_box("old_1", SLOTS[2], LANE_Y), _box("old_2", SLOTS[3], LANE_Y)]
CASE_X, CASE_Y = 0.28, -0.25
# The case holds each box in its own snug cell (3 mm clear on each side), so a box can't turn as the fingers close
# on it: in a loose case the right arm picked boxes up turned, and set them down caught on a divider.
CELL_X, CELL_Y = PX + 0.006, PY + 0.006
CASE_T, CASE_H = 0.004, 0.01
CELL_OFF = CELL_X / 2 + CASE_T / 2  # cell centres either side of the case's middle wall
NEW = [_box("new_1", CASE_X - CELL_OFF, CASE_Y), _box("new_2", CASE_X + CELL_OFF, CASE_Y)]

# ---------------------------------------------------------------- delivery case (fixed, low so the fingers clear it)
CASE_IN_X, CASE_IN_Y = 2 * CELL_X + CASE_T, CELL_Y
CARDBOARD = (0.70, 0.55, 0.36)


def _case() -> list[Box]:
    ox, oy = CASE_IN_X / 2 + CASE_T / 2, CASE_IN_Y / 2 + CASE_T / 2
    z = TOP + CASE_H / 2
    return [
        Box("case_back", pos=(CASE_X - ox, CASE_Y, z), size=(CASE_T, CASE_IN_Y + 2 * CASE_T, CASE_H), color=CARDBOARD),
        Box("case_front", pos=(CASE_X + ox, CASE_Y, z), size=(CASE_T, CASE_IN_Y + 2 * CASE_T, CASE_H), color=CARDBOARD),
        Box("case_right", pos=(CASE_X, CASE_Y - oy, z), size=(CASE_IN_X, CASE_T, CASE_H), color=CARDBOARD),
        Box("case_left", pos=(CASE_X, CASE_Y + oy, z), size=(CASE_IN_X, CASE_T, CASE_H), color=CARDBOARD),
        Box("case_middle", pos=(CASE_X, CASE_Y, z), size=(CASE_T, CASE_IN_Y, CASE_H), color=CARDBOARD),
    ]


OBJECTS = [*_shelf(), *_case(), *OLD, *NEW]

# ---------------------------------------------------------------- scoring
SLOT_HALF = SLOT_PITCH / 2  # slot regions are contiguous along the lane
UPRIGHT_DEG = 10.0
WANT = {0: "old", 1: "old", 2: "new", 3: "new"}  # slot index -> which stock belongs there


def _tilt_deg(s) -> float:
    x, y = float(s.quat_xyzw[0]), float(s.quat_xyzw[1])
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def slot_of(s):
    """The slot (0 = front .. 3 = back) a box stands in, upright on the shelf inside the middle lane; else None."""
    x, y = float(s.pos[0]), float(s.pos[1])
    if abs(y - LANE_Y) > LANE_IN / 2 or _tilt_deg(s) > UPRIGHT_DEG or float(s.low[2]) > TOP + 0.01:
        return None
    for k, cx in enumerate(SLOTS):
        if abs(x - cx) <= SLOT_HALF:
            return k
    return None


def _filled(objects) -> list[bool]:
    """For each slot: holds exactly the right kind of stock (one box, at rest)."""
    out = []
    for k, kind in WANT.items():
        here = [n for n in (*(b.name for b in OLD), *(b.name for b in NEW)) if slot_of(objects[n]) == k]
        out.append(len(here) == 1 and here[0].startswith(kind) and objects[here[0]].resting())
    return out


DEBUG = os.environ.get("SHELF_DEBUG") == "1"  # SHELF_DEBUG=1: print every box's place once a second
_n = [0]


def _debug(objects, filled) -> None:
    _n[0] += 1
    if _n[0] % 20:
        return
    parts = []
    for b in (*OLD, *NEW):
        s = objects[b.name]
        k = slot_of(s)
        parts.append(
            f"{b.name}: x {float(s.pos[0]) * 100:5.1f} y {float(s.pos[1]) * 100:5.1f} z {float(s.low[2] - TOP) * 100:4.1f} cm "
            f"tilt {_tilt_deg(s):4.1f} slot {'-' if k is None else k + 1}"
        )
    print(f"[shelf t={_n[0] / 20:5.1f}s] " + " | ".join(parts) + f" | slots ok {['Y' if f else 'n' for f in filled]}", flush=True)


def score(objects) -> float:
    """0.2 per slot holding the right stock (older boxes in slots 1-2, new boxes in 3-4); +0.2 when all four are."""
    filled = _filled(objects)
    if DEBUG:
        _debug(objects, filled)
    return round(0.2 * sum(filled) + (0.2 if all(filled) else 0.0), 3)


def success(objects) -> bool:
    """Older boxes in the two front slots, new boxes in the two back slots, all upright and at rest; holds 1 s."""
    return all(_filled(objects))


def distance(objects) -> float:
    """Summed metres from the older boxes to the two front slots and the new ones to the two back slots."""
    total = 0.0
    for boxes, slots in ((OLD, SLOTS[:2]), (NEW, SLOTS[2:])):
        for b in boxes:
            p = objects[b.name].pos
            total += min(math.hypot(float(p[0]) - sx, float(p[1]) - LANE_Y) for sx in slots)
    return total


# ---------------------------------------------------------------- demo
TRAVEL_Z = TOP + 0.14  # as in the example task
GRASP_UP = 0.005  # grasp 5 mm above each box's centre: the finger tips stay above the dividers and the case walls
PLACE_Z = TOP + PZ / 2 + GRASP_UP + 0.005  # set down like the example: 5 mm above where it rests


HOVER_UP = 0.025  # stop with the box's bottom 2.5 cm over the shelf, clear of the dividers, before setting it down
# In simulation the right arm, reaching across with a box, sets it down about 1.4 cm short in y (toward its base) and
# on target in x: uncorrected, both new boxes landed on the right divider's edge and tipped (measured with
# SHELF_DEBUG=1). Its set-down targets are moved 1.2 cm the other way in y. Scoring is unaffected.
RIGHT_PLACE_FIX = {2: (0.0, 0.012), 3: (0.0, 0.012)}  # slot index -> (x, y) in metres


def _pick_place(box, slot, fix=(0.0, 0.0)):
    """pick_place_waypoints, slowed where it matters. Picking: the arm comes to rest open around the box before the
    fingers close slowly. Placing: it stops just above the slot and settles, so a far-reaching arm has stopped
    swinging before it lowers the box between the dividers (it set boxes down on a divider's edge)."""
    wps = pick_place_waypoints(
        (box.pos[0], box.pos[1], box.pos[2] + GRASP_UP),
        (SLOTS[slot] + fix[0], LANE_Y + fix[1], PLACE_Z),
        travel_z=TRAVEL_Z,
        speed=1.3,
    )  # travel faster; the moves near the boxes are re-timed below
    k = next(i for i, w in enumerate(wps) if w.label == "close")
    d = wps[k - 1]  # "descend"
    wps[k - 1] = Waypoint(2.0, d.pos, d.finger, d.label, d.yaw)  # come down slowly
    wps[k] = Waypoint(1.2, wps[k].pos, wps[k].finger, wps[k].label, wps[k].yaw)  # close slowly
    wps.insert(k, Waypoint(0.6, d.pos, d.finger, "settle", d.yaw))
    k = next(i for i, w in enumerate(wps) if w.label == "lower")
    lo = wps[k]
    hover = (lo.pos[0], lo.pos[1], lo.pos[2] + HOVER_UP)
    wps[k] = Waypoint(1.2, lo.pos, lo.finger, "set down", lo.yaw)  # lower slowly
    wps[k:k] = [Waypoint(0.8, hover, lo.finger, "over the slot", lo.yaw), Waypoint(0.8, hover, lo.finger, "settle", lo.yaw)]
    return wps


def demo():
    """Left arm pulls the older boxes forward (slot 3 -> 1, slot 4 -> 2). Meanwhile the right arm lifts the first new
    box out of the case and holds it there; once the left arm has let go of its last box and risen, the right arm
    carries the new boxes into slots 3 and 4. Grasps near each box's centre, as the example does."""
    left = []
    for box, slot in ((OLD[0], 0), (OLD[1], 1)):
        left += _pick_place(box, slot)
    left_risen = sum(w.duration for w in left)  # the left arm has let go of its last box and risen
    left.append(Waypoint(1.5, (0.20, 0.28, TRAVEL_Z), FINGER_OPEN, "clear of the shelf"))

    first = _pick_place(NEW[0], 2, RIGHT_PLACE_FIX[2])
    k = next(i for i, w in enumerate(first) if w.label == "carry")
    picked = sum(w.duration for w in first[:k])  # holding the box above the case
    right = first[:k] + _hold(first[:k], max(0.5, left_risen - picked), "hold it above the case") + first[k:]
    right += _pick_place(NEW[1], 3, RIGHT_PLACE_FIX[3])
    right.append(Waypoint(1.5, (CASE_X, CASE_Y, TRAVEL_Z), FINGER_OPEN, "clear of the shelf"))
    return {"left": left, "right": right}


def _hold(waypoints, seconds, label="wait"):
    w = waypoints[-1]
    return [Waypoint(seconds, w.pos, w.finger, label, w.yaw)]