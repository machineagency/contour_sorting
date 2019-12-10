#!/usr/bin/env python3
"""An optimized contour sorting post-processing scheme for DXF files."""

# Assume no self-intersecting contours.
# TODO: handle parts-in-parts case.
# TODO: handle overlapping polygon case.
# TODO: handle edge cases of ray-casting intersection.

import ezdxf
import math
from interval_tree import IntervalTree
from basic_geometry import *
from anytree import Node, RenderTree, LevelOrderGroupIter
from collections import OrderedDict


# TODO: should contour subclass SubContour?
class Contour(object):
    """A contour class for closed contours"""

    def __init__(self):
        self.segments = [] # references to arc, lines, etc.

    def name(self):
        """ Because contours do not share lines, we can give them unique names from a segment name."""
        return f"Contour_with_{str(self.segments[0])}"

    def __getitem__(self, index):
        return self.segments.__getitem__(index)

    def __reversed__(self):
        return self.segments.__reversed__()

class SubContour(Contour):
    """A class for an incomplete contour with conveniences for accessing the endpoints quickly."""
    EPS = 1e-5

    def __init__(self):
        super().__init__()
        self.y_min = math.inf
        self.y_max = -math.inf
        self.x_min = math.inf
        self.x_max = -math.inf

    def point_on_contour_edge(self):
        """Return a point that sits on the edge of the contour"""
        # Check for circles:
        if len(self.segments) == 1 and self.is_closed():
            # TODO: handle other single-segment contours that are closed, like ellipses.
            return (self.x_max, self.y_max)
        # Handle arcs, line segments:
        return (self.start_x, self.start_y)

    def merge_back(self, other_sub_contour, reverse_order=False):
        if not reverse_order:
            for entity in other_sub_contour:
                self.push_back(entity)
        else:
            for entity in reversed(other_sub_contour):
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
        raise RuntimeError("Contour has no defined start or finish.")
    @property
    def start_y(self):
        if self.segments[0].dxftype() == "LINE":
            return self.segments[0].dxf.start[1]
        elif self.segments[0].dxftype() == "ARC":
            return self.segments[0].dxf.radius * math.sin(self.segments[0].dxf.start_angle*math.pi/180.0) + \
                self.segments[0].dxf.center[1]
        raise RuntimeError("Contour has no defined start or finish.")

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
        """Push a new line or arc to the front of the subcontour. Update extreme bounding box points"""
        if self.is_closed():
            raise RuntimeError("We can't prepend to a closed contour.")
        self.segments.insert(0, entity)
        # Check the x,y min/max values to see if we need to update them.
        if entity.dxftype() == "CIRCLE":
            self.x_min = entity.dxf.center[0] - entity.dxf.radius
            self.x_max = entity.dxf.center[0] + entity.dxf.radius
            self.y_min = entity.dxf.center[1] - entity.dxf.radius
            self.y_max = entity.dxf.center[1] + entity.dxf.radius
        if entity.dxftype() == "LINE":
            if self.start_x < self.x_min:
                self.x_min = self.start_x
            if self.start_x > self.x_max:
                self.x_max = self.start_x
            if self.start_y < self.y_min:
                self.y_min = self.start_y
            if self.start_y > self.y_max:
                self.y_max = self.start_y
        # TODO: this assumes we open the angle counterclockwise.
        elif entity.dxftype() == "ARC":
            start_angle = clamp_angle(entity.dxf.start_angle*math.pi/180.0)
            end_angle = clamp_angle(entity.dxf.end_angle*math.pi/180.0)

            # Start by taking the extremes of the angle endpoints at the relevant end of the contour.
            y_max = self.start_y
            y_min = self.start_y
            x_max = self.start_x
            x_min = self.start_x

            for candidate_angle in [90, 270]:
                y_val = entity.dxf.radius * math.sin(candidate_angle*math.pi/180.0) + entity.dxf.center[1]
                if start_angle < candidate_angle < end_angle:
                    y_max = max([y_max, y_val])
                    y_min = min([y_min, y_val])
            self.y_max = max([y_max, self.y_max])
            self.y_min = min([y_min, self.y_min])

            for candidate_angle in [0, 180]:
                x_val = entity.dxf.radius * math.cos(candidate_angle*math.pi/180.0) + entity.dxf.center[0]
                if start_angle < candidate_angle < end_angle:
                    x_max = max([x_max, x_val])
                    x_min = min([x_min, x_val])
            self.x_max = max([x_max, self.x_max])
            self.x_min = min([x_min, self.x_min])

    def push_back(self, entity):
        """Push a new line or arc to the back of the subcontour. Update extreme bounding box points"""
        if self.is_closed():
            raise RuntimeError("We can't append to a closed contour.")
        self.segments.insert(len(self.segments), entity)
        # Check the y values to see if we need to update them.
        if entity.dxftype() == "CIRCLE":
            self.x_min = entity.dxf.center[0] - entity.dxf.radius
            self.x_max = entity.dxf.center[0] + entity.dxf.radius
            self.y_min = entity.dxf.center[1] - entity.dxf.radius
            self.y_max = entity.dxf.center[1] + entity.dxf.radius
        elif entity.dxftype() == "LINE":
            if self.end_x < self.x_min:
                self.x_min = self.end_x
            if self.end_x > self.x_max:
                self.x_max = self.end_x
            if self.end_y < self.y_min:
                self.y_min = self.end_y
            if self.end_y > self.y_max:
                self.y_max = self.end_y
        # TODO: this assumes we open the angle counterclockwise.
        elif entity.dxftype() == "ARC":
            start_angle = clamp_angle(entity.dxf.start_angle*math.pi/180.0)
            end_angle = clamp_angle(entity.dxf.end_angle*math.pi/180.0)

            # Start by taking the extremes of the angle endpoints at the relevant end of the contour.
            y_max = self.end_y
            y_min = self.end_y
            x_max = self.end_x
            x_min = self.end_x

            for candidate_angle in [90, 270]:
                y_val = entity.dxf.radius * math.sin(candidate_angle*math.pi/180.0) + entity.dxf.center[1]
                if start_angle < candidate_angle < end_angle:
                    y_max = max([y_max, y_val])
                    y_min = min([y_min, y_val])
            self.y_max = max([y_max, self.y_max])
            self.y_min = min([y_min, self.y_min])

            for candidate_angle in [0, 180]:
                x_val = entity.dxf.radius * math.cos(candidate_angle*math.pi/180.0) + entity.dxf.center[0]
                if start_angle < candidate_angle < end_angle:
                    x_max = max([x_max, x_val])
                    x_min = min([x_min, x_val])
            self.x_max = max([x_max, self.x_max])
            self.x_min = min([x_min, self.x_min])

    def is_closed(self):
        # Handle special case for circles and other single contours that are closed:
        if len(self.segments) == 0:
            return False
        if self.segments[0].dxftype() == "CIRCLE":
            return True
        # Handle general case:
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
        # Special cases for already-closed entities: circle.
        if entity.dxftype() == "CIRCLE":
            new_sub_contour = SubContour()
            new_sub_contour.push_back(entity)
            contours.append(new_sub_contour)
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


def sort_contours_by_level(contours):
    """Sort contours into parts.
        Returns a sorted list of lists, where inner lists represent contours at the same depth,
        and the outer list organizes inner lists by decreasing depth.
    """
    # TODO: handle pre-closed contours. (Circles, ellipses, etc.)
    parts = []
    height_interval_to_contours = {} # items are contour lists, since multiple contours can have the same height interval.
    contour_tree = IntervalTree()
    heights = set()
    contours_by_name = {}
    nested_contour_tree_items = {} # dict of contour nodes

    # Find min/max heights of all contours.
    layout_y_min = math.inf
    layout_y_max = -math.inf
    for contour in contours:
        # Store contours by name.
        contours_by_name[contour.name()] = contour
        # Store contour in a dict by height interval. Some contours can have the same height, so use lists.
        # This data structure is the input to build the interval tree.
        if (contour.y_min, contour.y_max) in height_interval_to_contours:
            height_interval_to_contours[(contour.y_min, contour.y_max)].append(contour)
        else:
            height_interval_to_contours[(contour.y_min, contour.y_max)] = [contour]
        # Update the extremes of the layout.
        if contour.y_min < layout_y_min:
            layout_y_min = contour.y_min
        if contour.y_max > layout_y_max:
            layout_y_max = contour.y_max
        # Add the contour's midpoint to the height intervals.
        heights.add((contour.y_max - contour.y_min)/2 + contour.y_min)

    # Create interval tree.
    print("Packing Contours into Interval Tree for sorting speedup.")
    contour_tree.build(layout_y_min, layout_y_max, height_interval_to_contours)

    # Construct all contour in-out relationships.
    print("Constructing in-out contour relationships.")
    for height in heights:
        # Extract all the contours that exist at this height.
        contour_subset_lists = contour_tree.query(height)
        contour_subset_lists = [item[1] for item in contour_subset_lists] # remove the keys.
        contour_subset_lists = [item for sublist in contour_subset_lists for item in sublist] # flatten remaining lists.

        # Build the In-Out relationship tree.
        for a_index, contour_a in enumerate(contour_subset_lists):
            contour_a_node = nested_contour_tree_items.get(contour_a.name(), Node(contour_a.name()))
            for b_index, contour_b in enumerate(contour_subset_lists[a_index+1:]):
                point_a = contour_a.point_on_contour_edge()
                point_b = contour_b.point_on_contour_edge()
                # Check if a is in b. If so, insert pair relationship into tree.
                if point_in_contour(point_a, contour_b):
                    # contour_b is contour_a's parent. Add back to the dict
                    contour_b_node = nested_contour_tree_items.get(contour_b.name(), Node(contour_b.name()))
                    contour_a_node.parent = contour_b_node
                    nested_contour_tree_items[contour_b.name()] = contour_b_node
                # Check if b is in a. If so, insert pair relationship into tree.
                elif point_in_contour(point_b, contour_a):
                    # contour_a is contour_b's parent. Add back to the dict
                    contour_b_node = nested_contour_tree_items.get(contour_b.name(), Node(contour_b.name()))
                    contour_b_node.parent = contour_a_node
                    nested_contour_tree_items[contour_b.name()] = contour_b_node
            nested_contour_tree_items[contour_a.name()] = contour_a_node


    print("Organizing contours by depth")
    # A dict, keyed by level (int) of contours that live at that level.
    depth_lists = OrderedDict()

    # Contours may be sorted in multiple separate trees.
    # Pull contours out of the dict representation and put into lists sorted by depths
    while len(nested_contour_tree_items):
        # Find the root(s) and print out the tree from there.
        node=None
        # Pull an arbitrary item out from the nesting.
        node_key = list(nested_contour_tree_items.keys())[0]
        # Get the root of this tree.
        node = nested_contour_tree_items[node_key]
        while node.parent is not None:
            node = node.parent
        # https://anytree.readthedocs.io/en/latest/api/anytree.iterators.html#anytree.iterators.levelordergroupiter.LevelOrderGroupIter
        list_o_lists = [[node.name for node in children] for children in LevelOrderGroupIter(node)]
        for index, depth_list in enumerate(list_o_lists):
            old_depth_list = depth_lists.get(index, [])
            for contour_name in depth_list:
                old_depth_list.append(contours_by_name[contour_name])
                del nested_contour_tree_items[contour_name]
            depth_lists[index] = old_depth_list

    # Serialize tree.
    return [v for k,v in depth_lists.items()]


def main():
    """ main fn. """
    # Open a document.
    doc = ezdxf.readfile("30mm_square_with_holes_and_fillets.DXF")
    #doc = ezdxf.readfile("contour_sorting_test_simple.DXF")
    #doc = ezdxf.readfile("contour_sorting_test.DXF")
    msp = doc.modelspace()

    contours = create_contours(msp)
    contours_by_level = sort_contours_by_level(contours)
    import pprint
    pprint.pprint(contours_by_level)

    circuits = create_circuits(contours_by_level)

    # TODO: Handle circles and arcs
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
