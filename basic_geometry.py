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
            #print("INTERSECTION TEST:")
            try:
                if entity.dxftype() == "LINE":
                    # TODO: put a try/except here to catch edge cases and change angle.
                    #print(f"    seg start: {entity.dxf.start[0]},{entity.dxf.start[1]}")
                    #print(f"    seg end:   {entity.dxf.end[0]},{entity.dxf.end[1]}")
                    #print(f"    ray start: {pt}")
                    #print(f"    ray dir:   {ray_dir}")
                    #print(f"    ray angle:   {math.atan2(math.sin(theta), math.cos(theta))*180.0/math.pi}")
                    intersection_count += \
                        intersect_segment_with_ray((entity.dxf.start[0], entity.dxf.start[1]),
                                                   (entity.dxf.end[0], entity.dxf.end[1]),
                                                   pt, ray_dir)
                elif entity.dxftype() == "ARC":
                    intersection_count += \
                        intersect_arc_with_ray((entity.dxf.center[0], entity.dxf.center[1]),
                                               entity.dxf.radius,
                                               entity.dxf.start_angle*math.pi/180.0,
                                               entity.dxf.end_angle*math.pi/180.0,
                                               pt, ray_dir)
                elif entity.dxftype() == "CIRCLE":
                    intersection_count += \
                        intersect_arc_with_ray((entity.dxf.center[0], entity.dxf.center[1]),
                                               entity.dxf.radius,
                                               0,
                                               2*math.pi,
                                               pt, ray_dir)
            except IntersectError:
                # We can't trust this result because of the crescent moon edge case.
                # Ignore this point.
                intersect_error = True
                break
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
        return 0

    # Check if two lines are parallel but not collinear.
    # TODO: write tests for this.
    angle1 = (math.atan2(r[1], r[0]) + 2*math.pi) % (2*math.pi) # make angle positive
    angle2 = (math.atan2(s[1], s[0]) + 2*math.pi) % (2*math.pi) # make angle positive
    if abs(angle1 - angle2) < 1e-6:
        #print("lines are parallel but not collinear")
        return 0

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
        return 1
    return 0


def intersect_arc_with_ray(center_pt, radius, start_angle_rad, end_angle_rad,
                           ray_origin_pt, ray_direction_vector):
    # https://www.geometrictools.com/Documentation/IntersectionLine2Circle2.pdf
    intersections = 0

    def dotperp(a, b):
        return np.dot(a, np.array([b[1], -b[0]]))

    center_pt = np.array(center_pt)
    ray_origin_pt = np.array(ray_origin_pt)
    ray_direction_vector = np.array(ray_direction_vector)

    delta = ray_origin_pt - center_pt # technically big delta
    d = np.dot(ray_direction_vector, delta)**2 - \
        np.linalg.norm(ray_direction_vector)**2 * (np.linalg.norm(delta)**2 - radius**2)

    # points for checking whether intersection is in the span of the arc
    a_pt = np.array((center_pt[0] + radius*math.cos(start_angle_rad),
                     center_pt[1] + radius*math.sin(start_angle_rad)))
    b_pt = np.array((center_pt[0] + radius*math.cos(end_angle_rad),
                     center_pt[1] + radius*math.sin(end_angle_rad)))

    if d < 0: # No intersection
        return 0
    if d == 0: # ray is tangent to the arc. This is an edge case.
        raise IntersectError("Arc and ray are tangent to each other.")
    else: # General case
        t1 = (-np.dot(ray_direction_vector, delta) + math.sqrt(d))/np.linalg.norm(ray_direction_vector)**2
        t2 = (-np.dot(ray_direction_vector, delta) - math.sqrt(d))/np.linalg.norm(ray_direction_vector)**2
        #print(f"T1: {t1} | T2: {t2}")
        # Check if either t is in arc. Ignore negative cases, since they're off the ray.
        if t1 >= 0:
            intersection_pt1 = ray_origin_pt + t1*ray_direction_vector
            #print(f"Supposed intersection at {intersection_pt1}")
            # Check arc
            if dotperp((intersection_pt1 - a_pt), (b_pt - a_pt)) >= 0:
                intersections += 1
        if t2 >= 0:
            intersection_pt2 = ray_origin_pt + t2*ray_direction_vector
            #print(f"Supposed intersection at {intersection_pt2}")
            # Check arc
            if dotperp((intersection_pt2 - a_pt), (b_pt - a_pt)) >= 0:
                intersections += 1

    return intersections


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

    # RAY-ARC intersection
    def test_basic_ray_arc_intersection(self):
        assert intersect_arc_with_ray((0.0,0.0), 5, 0, math.pi/2.0,
                                      (0.0,0.0), (1.0, 1.0)) == 1

    def test_basic_ray_double_arc_intersection(self):
        assert intersect_arc_with_ray((0.0,0.0), 5, 0, math.pi/2.0,
                                      (1,5), (1.0, -1.0)) == 2

    def test_basic_ray_double_arc_nonintersection(self):
        assert intersect_arc_with_ray((0.0,0.0), 5, 0, math.pi/2.0,
                                      (-1,5), (1.0, -1.0)) == 0

if __name__ == "__main__":
    unittest.main()
