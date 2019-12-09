#!/usr/bin/env python3
"""Helper functions for working with geometry."""
import math
import numpy as np

def point_in_contour(point, contour):
    return False

def intersect_segment_with_ray(start_pt, end_pt, ray_origin_pt, ray_direction_vector):
    # Convert to parametric form.
    a = np.array(start_pt)
    b = np.array(end_pt)
    o = np.array(ray_origin_pt)
    d = np.array(ray_direction_vector)
    d = d / np.linalg.norm(d)

    v1 = o - a
    v2 = b - a # direction vector of parameterized line segment
    v3 = np.array([-d[1], d[0]])

    # Check if the two lines are collinear by testing any 3 points.
    # https://stackoverflow.com/questions/3813681/checking-to-see-if-3-points-are-on-the-same-line
    if abs((start_pt[1] - end_pt[1]) * (start_pt[0] - ray_origin_pt[0]) -
           (start_pt[1] - ray_origin_pt[1]) * (start_pt[0] - end_pt[0])) < 1e-6:
        # Return false in this case of collinear lines.
        # In the context of a contour, another segment sharing the point will
        # register as a intersection.
        print("lines are collinear.")
        return False

    # Check if two lines are parallel but not collinear.
    angle1 = (math.atan2(v2[1], v2[0]) + 2*math.pi) % (2*math.pi) # make angle positive
    angle2 = (math.atan2(d[1], d[0]) + 2*math.pi) % (2*math.pi) # make angle positive
    if abs(angle1 - angle2) < 1e-6:
        return False

    # General Case:
    # https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
    # Note: the general check totally breaks if both are colinear.
    t1 = np.linalg.norm(np.cross(v2, v1))/np.dot(v2, v3)
    t2 = np.dot(v1, v3)/np.dot(v2, v3)

    print(f"T1: {t1} | T2: {t2}")
    # Check constraints:
    if t1 < 0: # ray is intersecting from the wrong direction.
        return False
    if 0.0 <= t2 <= 1.0:
        return True
    return False


def intersect_ray_with_arc(center_xy, radius, start_angle_rad, end_angle_rad, end_xy,
                           ray_origin_xy, point_on_ray_xy):
    return 0


def clamp_angle(angle):
    """Constrain angle to the range (-2pi, 2pi)."""
    # https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
    angle = angle%(2*math.pi) # reduce angle
    # angle = (angle + 2*math.pi)%(2*math.pi) # make angle positive
    return angle


