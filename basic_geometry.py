#!/usr/bin/env python3
"""Helper functions for working with geometry."""
import math
import random
import numpy as np
import unittest

class IntersectError(BaseException):
    pass


def point_in_contour(pt, contour):

    while True:
        theta = random.random()*2*math.pi # random angle in range [0, 2pi)
        ray_dir = (math.cos(theta), math.sin(theta))
        #print(f"theta: {theta * 180.0/math.pi}")
        # Ray cast from the point in question
        # Ray should start from here and intersect the point in question.
        intersection_count = 0
        intersect_error = False
        for entity in contour:
            if entity.dxftype() == "LINE":
                # TODO: put a try/except here to catch edge cases and change angle.
                try:
                    #print("INTERSECTION TEST:")
                    #print(f"    seg start: {entity.dxf.start[0]},{entity.dxf.start[1]}")
                    #print(f"    seg end:   {entity.dxf.end[0]},{entity.dxf.end[1]}")
                    #print(f"    ray start: {pt}")
                    #print(f"    ray dir:   {ray_dir}")
                    #print(f"    ray angle:   {math.atan2(math.sin(theta), math.cos(theta))*180.0/math.pi}")
                    if intersect_segment_with_ray((entity.dxf.start[0], entity.dxf.start[1]),
                                                  (entity.dxf.end[0], entity.dxf.end[1]),
                                                  pt,
                                                  ray_dir):
                        intersection_count += 1
                except IntersectError:
                    # We can't trust this result because of the crescent moon edge case.
                    # Ignore this point.
                    intersect_error = True
                    break
            elif entity.dxftype() == "ARC":
                print("Arc intersection calculation not yet supported.")

        if intersect_error:
            continue
        #print(f"Total intersections: {intersection_count}")
        #print()
        if intersection_count > 0 and intersection_count%2 == 1:
            return True
        return False

def intersect_segment_with_ray(start_pt, end_pt, ray_origin_pt, ray_direction_vector):
    """Return True if the segment intersects the ray. Return False otherwise.
        param ray_direction_vector: a vector pointing in the direction of the ray. (No need to normalize.)
    """
    # https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282

    # Convert to parametric form.
    # segment equation
    q = np.array(start_pt)
    #s = (np.array(end_pt) - np.array(start_pt))/np.linalg.norm(np.array(end_pt) - np.array(start_pt))
    s = (np.array(end_pt) - np.array(start_pt)) # Do not normalize!
    # ray equation
    p = np.array(ray_origin_pt)
    r = np.array(ray_direction_vector)/np.linalg.norm(np.array(ray_direction_vector))

    #print(f"Segment start, end = {start_pt}, {end_pt}")
    #print(f"Segment eqn {q}, {s}")
    #print(f"Ray eqn: {p}, {r}")

    # Check if the two lines are collinear by testing any 3 points.
    # https://stackoverflow.com/questions/3813681/checking-to-see-if-3-points-are-on-the-same-line
    if abs((start_pt[1] - end_pt[1]) * (start_pt[0] - ray_origin_pt[0]) -
           (start_pt[1] - ray_origin_pt[1]) * (start_pt[0] - end_pt[0])) < 1e-6:
        # Return false in this case of collinear lines.
        # In the context of a contour, another segment sharing the point will
        # register as a intersection.
        #print("lines are collinear.")
        return False

    # Check if two lines are parallel but not collinear.
    # TODO: write tests for this.
    angle1 = (math.atan2(r[1], r[0]) + 2*math.pi) % (2*math.pi) # make angle positive
    angle2 = (math.atan2(s[1], s[0]) + 2*math.pi) % (2*math.pi) # make angle positive
    if abs(angle1 - angle2) < 1e-6:
        #print("lines are parallel but not collinear")
        return False

    # General Case:
    t1 = np.cross((q - p), r) / np.cross(r, s) # segment equation paramter
    t2 = np.cross((q - p), s)/np.cross(r, s) # ray equation parameter
    #print(f"T1: {t1} | T2: {t2}")
    #print(f"supposed intersection at {q + t1*s}")

    # Check constraints:
    if 0.0 <= t1 <= 1.0 and t2 >= 0:
        # Check if ray-and-segment intersect at the exact endpoint of a segment.
        if abs(np.linalg.norm(q + t1*s - np.array(start_pt))) < 1e-6 or \
            abs(np.linalg.norm(q + t1*s - np.array(end_pt))) < 1e-6:
            raise IntersectError("Intesection happened exactly on the tip of a line segment edge.")
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


class MyTests(unittest.TestCase):
    """Tests to be run must start with 'test'"""

    def test_intersect_on_segment_end(self):
        with self.assertRaises(IntersectError):
            intersect_segment_with_ray([0, 1], [0, -1], [-1, -1], [1, 0])

    def test_intersect_on_segment_start(self):
        with self.assertRaises(IntersectError):
            intersect_segment_with_ray([0, 1], [0, -1], [-1, 1], [1, 0])

    def test_intersect_at_midpoint_from_the_right(self):
        assert intersect_segment_with_ray([0, 1], [0, -1], [1, 0], [-1, 0]) == True

    def test_intersect_at_midpoint_from_the_left(self):
        assert intersect_segment_with_ray([0, 1], [0, -1], [-1, 0], [1, 0]) == True

    def test_intersect_with_big_numbers(self):
        assert intersect_segment_with_ray([0, 100], [0, -100], [-1, 0], [1, 0]) == True

    def test_off_segment_intersection(self):
        assert intersect_segment_with_ray([0, 1], [0, -1], [-1, 2], [1, 0]) == False

    def test_intersect_with_vertical_ray(self):
        assert intersect_segment_with_ray([0, 100], [100, 100], [50, 25], [0, 1]) == True

if __name__ == "__main__":
    unittest.main()
