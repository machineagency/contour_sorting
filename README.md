# Contour Sorting
a dxf post-processor that sorts nested contours inside-edge to outside-edge and
takes a nearest-neighbor approach to minimizing travel distances between contours.


## Installation:
You will need Python 3.6 or later. From there, it's just:
    
    pip install -r requirements.txt
    

## Usage
    
    python contour_sorting.py /path/to/my_vector_graphic.dxf
    

## Limitations

### Limited Geometry
Currently, this script only works with **segments**, **arcs**, and **circles**.

### Contour Topology
All contours must be closed. Furthermore, all contours must not intersect or self-intersect.
Typically, if you're exporting surfaces from a 3D CAD program, this shouldn't be
an issue, but if you're creating your cut file in a 2D environment, be forwarned!
