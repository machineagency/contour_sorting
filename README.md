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
