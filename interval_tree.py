#!/usr/bin/env python3
"""An interval tree implementaion."""

class IntervalTree(object):
    """A python-based interval tree"""

    def __init__(self):
        self.segments = [] # references to arc, lines, etc.
        self.left_sorted_contents = []
        self.right_sorted_contents = []

        self.left_child = None
        self.right_child = None
        self.midpoint = None


    def build(self, lower_bound, upper_bound, intervals):
        self.midpoint = (upper_bound - lower_bound)/2.0 + lower_bound
        remaining_intervals = {}
        intervals_to_remove = []
        for interval, item in intervals.items():
            if interval[0] < self.midpoint < interval[1]:
                self.left_sorted_contents.append((interval, item))
                self.right_sorted_contents.append((interval, item))
                # Since we can't modify dict size while iterating, mark this item to be removed.
                intervals_to_remove.append(interval)
            else:
                remaining_intervals[interval] = item # Prune the tree.

        self.left_sorted_contents.sort(key=lambda interval_pair: interval_pair[0])
        self.right_sorted_contents.sort(key=lambda interval_pair: interval_pair[1], reverse=True)

        # Create new dicts based on remaining tree items.
        lower_intervals = {}
        upper_intervals = {}
        for interval, item in remaining_intervals.items():
            if self.midpoint - interval[1] > 0:
                lower_intervals[interval] = item
                del intervals[interval]
            elif interval[0] - self.midpoint > 0:
                upper_intervals[interval] = item
                del intervals[interval]

        for interval in intervals_to_remove:
            del intervals[interval]
        if len(intervals):
            raise RuntimeError("This shouldn't happen.")

        # Recurse!
        if len(lower_intervals):
            self.left_child = IntervalTree()
            self.left_child.build(lower_bound, self.midpoint, lower_intervals)
        if len(upper_intervals):
            self.right_child = IntervalTree()
            self.right_child.build(self.midpoint, upper_bound, upper_intervals)

    def query(self, query_point):
        results = []

        if query_point < self.midpoint:
            for interval, item in self.left_sorted_contents:
                if interval[0] <= query_point or abs(interval[0] - query_point) < 1e-5:
                    results.append((interval, item))
                else:
                    break
            if self.left_child is not None:
                results += self.left_child.query(query_point)
        else:
            for interval, item in self.right_sorted_contents:
                if interval[1] >= query_point or abs(interval[1] - query_point) < 1e-5:
                    results.append((interval, item))
                else:
                    break
            if self.right_child is not None:
                results += self.right_child.query(query_point)
        return results


def main():

    my_tree = IntervalTree()

    my_data =   {
                    (0, 1): "0 to 1",
                    (0.4, 1.2): "0.4 to 1.2",
                    (1, 2): "1 to 2",
                    (1.55, 2): "1.55 to 2",
                    (2, 3): "2 to 3",
                    (3, 4): "3 to 4",
                    (4, 5): "4 to 5",
                }

    my_tree.build(0, 5, my_data)
    print(my_tree.query(1.6))

if __name__ == "__main__":
    main()
