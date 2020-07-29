# Create static map to sense obstacles from

The static obstacles are sensed from a static map.
The static map is defined by a `.sdf` file.
This file should be saved as `[envname]Model.sdf`.
The rectangles that define the static map are stored in a list.
This list is created in `obstacleToRect.py` and saved as `rect_list_[envname].data`.
To create this file, run the following:

```python obstacleToRect.py [envname]```
