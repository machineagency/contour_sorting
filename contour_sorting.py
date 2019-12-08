#!/usr/bin/env python3
"""An optimized contour sorting post-processing scheme for DXF files."""

# Assume no self-intersecting contours.
# TODO: handle parts-in-parts case.
# TODO: handle overlapping polygon case.

import ezdxf
import pprint
import inspect
import math

class Contour(object):
    """A contour class for closed contours"""

    def __init__(self):
        self.segments = [] # references to arc, lines, etc.

    def __getitem__(self, index):
        return self.segments.__getitem__(index)

    def __reversed__(self):
        return self.segments.__reversed__()

class SubContour(Contour):
    """A class for an incomplete contour with conveniences for accessing the endpoints quickly."""
    EPS = 1e-5

    def __init__(self):
        super().__init__()
        self.y_min = None
        self.y_max = None

    def merge_back(self, other_sub_contour, reverse_order=False):
        if not reverse_order:
            for entity in other_sub_contour:
                self.push_back(entity)
        else:
            for entity in reversed(other_sub_contour):
                print("pushing to the back.")
                flip_entity(entity)
                self.push_back(entity)

    def merge_front(self, other_sub_contour, reverse_order=False):
        """ if not reversed, merge input list head-to-tail into front of list.
            if reversed, merge input list tail-to-head into front of list"""
        if not reverse_order:
            for entity in reversed(other_sub_contour):
                self.push_front(entity)
        else:
            for entity in other_sub_contour:
                flip_entity(entity)
                self.push_front(entity)

    @property
    def start_x(self):
        if self.segments[0].dxftype() == "LINE":
            return self.segments[0].dxf.start[0]
        elif self.segments[0].dxftype() == "ARC":
            return self.segments[0].dxf.radius * math.cos(self.segments[0].dxf.start_angle*math.pi/180.0) + \
                self.segments[0].dxf.center[0]
        raise RuntimeError
    @property
    def start_y(self):
        if self.segments[0].dxftype() == "LINE":
            return self.segments[0].dxf.start[1]
        elif self.segments[0].dxftype() == "ARC":
            return self.segments[0].dxf.radius * math.sin(self.segments[0].dxf.start_angle*math.pi/180.0) + \
                self.segments[0].dxf.center[1]
        raise RuntimeError

    @property
    def end_x(self):
        if self.segments[-1].dxftype() == "LINE":
            return self.segments[-1].dxf.end[0]
        elif self.segments[-1].dxftype() == "ARC":
            return self.segments[-1].dxf.radius * math.cos(self.segments[-1].dxf.end_angle*math.pi/180.0) + \
                self.segments[-1].dxf.center[0]

    @property
    def end_y(self):
        if self.segments[-1].dxftype() == "LINE":
            return self.segments[-1].dxf.end[1]
        elif self.segments[-1].dxftype() == "ARC":
            return self.segments[-1].dxf.radius * math.sin(self.segments[-1].dxf.end_angle*math.pi/180.0) + \
                self.segments[-1].dxf.center[1]

    @property
    def front(self):
        return (self.start_x, self.start_y)

    @property
    def back(self):
        return (self.end_x, self.end_y)

    def match_front(self, x, y):
        return abs(self.start_x - x) < self.__class__.EPS and abs(self.start_y - y) < self.__class__.EPS

    def match_back(self, x, y):
        return abs(self.end_x - x) < self.__class__.EPS and abs(self.end_y - y) < self.__class__.EPS

    def push_front(self, entity):
        self.segments.insert(0, entity)
        # Check the y values to see if we need to update them.

    def push_back(self, entity):
        self.segments.insert(len(self.segments), entity)
        # Check the y values to see if we need to update them.

    def is_closed(self):
        return abs(self.start_x - self.end_x) < self.__class__.EPS and \
            abs(self.start_y - self.end_y) < self.__class__.EPS

class Part(object):
    """A shape class."""

    def __init__(self):
        self.contours = []

    def get_part_edge(self):
        try:
            return self.contours[0]
        except IndexError:
            return None

def flip_entity(entity):
    """Reverse the direction of an entity based on type."""
    if entity.dxftype() == "LINE":
        old_start = entity.dxf.start
        old_end = entity.dxf.end
        entity.dxf.start = old_end
        entity.dxf.end = old_start
    elif entity.dxftype() == "ARC":
        old_start_angle = entity.dxf.start_angle
        old_end_angle = entity.dxf.end_angle
        entity.dxf.start_angle = old_end_angle
        entity.dxf.end_angle = old_start_angle
    else:
        raise RuntimeError(f"flip_entity cannot flip direction of type: {entity.dxftype()}.")


def create_contours(entity_iterable):
    """ Create contours from segment soup."""
    contours = []
    sub_contours = []
    start_x = None
    start_y = None
    end_x = None
    end_y = None

    # Grow sub-contours. Export completed contours.
    for entity in entity_iterable:
        # Special cases for already-closed entities: circle, ellipse.
        if entity.dxftype() in ["ELLIPSE", "CIRCLE"]:
            print(f"Ignoring {entity.dxftype()}, which is already closed.")
            #contours.append(entity) # TODO: we can't do this yet.
            continue
        elif entity.dxftype() not in ["LINE", "ARC"]:
            print(f"found unknown entity: {entity}")

        # Pack the entity into a subcontour of size:1
        new_sub_contour = SubContour()
        new_sub_contour.push_back(entity)

        elongated_sub_contour = None
        for index, sub_contour in enumerate(sub_contours):
            # 4 cases for sub-contour elongation!
            if elongated_sub_contour is None:
                if sub_contour.match_front(*new_sub_contour.back):
                    sub_contour.merge_front(new_sub_contour)
                    elongated_sub_contour = sub_contour
                elif sub_contour.match_front(*new_sub_contour.front):
                    sub_contour.merge_front(new_sub_contour, reverse_order=True)
                    elongated_sub_contour = sub_contour
                elif sub_contour.match_back(*new_sub_contour.front):
                    sub_contour.merge_back(new_sub_contour)
                    elongated_sub_contour = sub_contour
                elif sub_contour.match_back(*new_sub_contour.back):
                    sub_contour.merge_back(new_sub_contour, reverse_order=True)
                    elongated_sub_contour = sub_contour

                if elongated_sub_contour:
                    # Try to close the contour and remove it.
                    if sub_contour.is_closed():
                        contours.append(sub_contour)
                        del sub_contours[index]
                        break
                    # If closing fails, skip to next iteration where we try to merge with remaining contours.
                    continue

            # Iterate through the remaining list items and try to stitch together subcontours.
            if elongated_sub_contour:
                # attempt to stitch to the front
                if elongated_sub_contour.match_front(*sub_contour.front):
                    elongated_sub_contour.merge_front(sub_contour, reverse_order=True)
                    del sub_contours[index]
                    break
                elif elongated_sub_contour.match_front(*sub_contour.back):
                    elongated_sub_contour.merge_front(sub_contour)
                    del sub_contours[index]
                    break
                # attempt to stitch to the back
                if elongated_sub_contour.match_back(*sub_contour.front):
                    elongated_sub_contour.merge_back(sub_contour)
                    del sub_contours[index]
                    break
                elif elongated_sub_contour.match_back(*sub_contour.back):
                    elongated_sub_contour.merge_back(sub_contour, reverse_order=True)
                    del sub_contours[index]
                    break

        # If we couldn't elongate a contour, create a new sub contour
        if elongated_sub_contour is None:
            sub_contours.append(new_sub_contour)

    for contour in contours:
        print("Contour:")
        for entity in contour:
            print(f"  {entity}")

    return contours

def create_parts_from_contours(contours):
    """Sort contours into parts."""
    # TODO: handle pre-closed contours. (Circles, ellipses, etc.)
    parts = []
    z_intervals = {}

    # Find min/max heights of all contours. Create list (flattened dict) of height pts.
    # Also create interval tree.
    for contour in contours:
        # Compute height and hash it.
        z_min = 0;
        z_max = 0;
        z_heights[()] = contour
        # Insert into tree...?
        pass

    # Construct all polygon in-out relationships.
    for z_height in z_heights:
        contour_subset = None # FIXME.
        for a_index, contour_a in enumerate(contour_subset):
            for b_index, contour_b in enumerate(contour_subset[a_index+1:])
                # Check if a is in b. If so, insert pair relationship into tree. bail
                # Check if b is in a. If so, insert pair relationship into tree.
                pass
        pass

    # Create the parts.
    # handle case with multiple polygons at the same level.

    return parts

def main():
    """ main fn. """

    doc = ezdxf.readfile("10mm_square_with_5mm_center_hole.DXF")
    msp = doc.modelspace()

    lines = []

    for e in msp:
        if e.dxftype() == 'LINE':
            lines.append(((e.dxf.start[0], e.dxf.start[1]), (e.dxf.end[0], e.dxf.end[1])))
            #print_entity(e)

    # Open a document.
    doc = ezdxf.readfile("30mm_square_with_holes_and_fillets.DXF")
    msp = doc.modelspace()

    contours = create_contours(msp)
    parts = create_parts_from_contours(contours)
    parts = sort_parts(parts) # Sort to minimize travel moves.
    # Recreate the DXF, and we're done!

    ## Create a new document.
    #new_doc = ezdxf.new('R2010')  # create a new DXF R2010 drawing, official DXF version name: 'AC1024'
    #new_msp = new_doc.modelspace()  # add new entities to the modelspace
    #for entity in reversed_e:
    #    #pprint.pprint(entity.dxf.dxftype)
    #    if entity.dxf.dxftype == "ARC":
    #        arc_arg_list = inspect.getfullargspec(msp.add_arc).args[1:] # drop [self]
    #        dxfattribs = entity.dxfattribs()
    #        arc_args = {k:v for k,v in dxfattribs.items() if k in arc_arg_list}
    #        new_msp.add_arc(**arc_args)
    #        #new_msp.add_arc(entity.dxf.center, entity.dxf.radius, entity.dxf.start_angle, entity.dxf.end_angle)
    #    elif entity.dxf.dxftype == "LINE":
    #        new_msp.add_line(entity.dxf.start, entity.dxf.end)
    #    elif entity.dxf.dxftype == "CIRCLE":
    #        new_msp.add_circle(entity.dxf.center, entity.dxf.radius)
    #new_doc.saveas('reversed_30mm_square_with_holes_and_fillets.DXF')


def print_entity(e):
    print("LINE on layer: %s\n" % e.dxf.layer)
    print("start point: %s\n" % e.dxf.start)
    print("end point: %s\n" % e.dxf.end)


if __name__ == "__main__":
    main()
