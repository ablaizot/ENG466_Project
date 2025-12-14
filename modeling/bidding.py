import math
from .constants import *

def dist(x0, y0, x1, y1):
    return math.hypot(x0 - x1, y0 - y1)


class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def orientation(a: Vec2, b: Vec2, c: Vec2):
    v = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y)
    if v > 0:
        return 1  # clockwise
    if v < 0:
        return 2  # counter-clockwise
    return 0      # collinear

def onSegment(a: Vec2, x: Vec2, b: Vec2):
    return (min(a.x, b.x) <= x.x <= max(a.x, b.x) and
            min(a.y, b.y) <= x.y <= max(a.y, b.y))

def segmentsIntersect(A: Vec2, B: Vec2, C: Vec2, D: Vec2):
    return (orientation(A, B, C) != orientation(A, B, D) and
            orientation(C, D, A) != orientation(C, D, B))

def intersectionPoint(A: Vec2, B: Vec2, C: Vec2, D: Vec2):
    x1, y1 = A.x, A.y
    x2, y2 = B.x, B.y
    x3, y3 = C.x, C.y
    x4, y4 = D.x, D.y

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-9:
        return None  # parallel or collinear

    px = ((x1*y2 - y1*x2)*(x3 - x4) -
          (x1 - x2)*(x3*y4 - y3*x4)) / denom

    py = ((x1*y2 - y1*x2)*(y3 - y4) -
          (y1 - y2)*(x3*y4 - y3*x4)) / denom

    P = Vec2(px, py)

    if not onSegment(A, P, B) or not onSegment(C, P, D):
        return None

    return P

def distance_walls(start, target):
    start_x, start_y = start
    target_x, target_y = target
    startPos = Vec2(start_x, start_y)
    targetPos = Vec2(target_x, target_y)

    # Wall 1
    line1_A = Vec2(-0.45 - 0.1875, 0.0)
    line1_B = Vec2(-0.45 + 0.1875, 0.0)

    # Wall 2
    line2_A = Vec2(0.125, 0.225 - 0.425)
    line2_B = Vec2(0.125, 0.225 + 0.425)

    distance = dist(start_x, start_y, target_x, target_y)

    # Wall 1 check
    if segmentsIntersect(startPos, targetPos, line1_A, line1_B):
        hit = intersectionPoint(startPos, targetPos, line1_A, line1_B)
        if hit:
            distance = dist(startPos.x, startPos.y, line1_B.x, line1_B.y)
            distance += dist(line1_B.x, line1_B.y, targetPos.x, targetPos.y)

    # Wall 2 check
    if segmentsIntersect(startPos, targetPos, line2_A, line2_B):
        hit = intersectionPoint(startPos, targetPos, line2_A, line2_B)
        if hit:
            distance = dist(startPos.x, startPos.y, line2_A.x, line2_A.y)
            distance += dist(line2_A.x, line2_A.y, targetPos.x, targetPos.y)

    return distance


def calculate_time_value(time, worked_time, clock):
    work_time_remain = MAX_WORK_TIME - worked_time
    simulation_time_remain = MAX_SIMULATION_TIME - clock

    time_factor = (simulation_time_remain - work_time_remain) / (MAX_SIMULATION_TIME - MAX_WORK_TIME)

    return AVG_TASK_PER_SECOND * time * time_factor


def get_path_waypoints(robot, task):
    start_x, start_y = robot.pos
    target_x, target_y = task.pos
    startPos = Vec2(start_x, start_y)
    targetPos = Vec2(target_x, target_y)

    # Wall 1
    line1_A = Vec2(-0.45 - 0.1875, 0.0)
    line1_B = Vec2(-0.45 + 0.1875, 0.0)

    # Wall 2
    line2_A = Vec2(0.125, 0.225 - 0.425)
    line2_B = Vec2(0.125, 0.225 + 0.425)

    waypoints = []

    # Wall 1 check
    if segmentsIntersect(startPos, targetPos, line1_A, line1_B):
        waypoints += [(line1_B.x, line1_B.y)]

    # Wall 2 check
    if segmentsIntersect(startPos, targetPos, line2_A, line2_B):
        waypoints += [(line2_A.x, line2_A.y)]

    waypoints += [task.pos] 

    return waypoints