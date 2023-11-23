How to Represent the World for Path Planning
=============================================

Previously, ``Occupancy Grids`` have been discussed as a means of representing objects on a 2D grid.  They may be binary (occupied/freespace), trinary (occupied/freespace/unknown) and probabilistic (0-1 i.e. freespace to occupied).

For an occupancy grid, a path is valid if it does not cross any cells which are designated as occupied (i.e. the robot cannot drive through an obstacle).  This simplicity leads to an immediate drawback when being used for path planning.

Robots Are Not Points
-----------------------

Suppose we have a circular robot with diameter 0.5 metres.  The figure below shows

IMAGE


Costmaps
---------
**Costmaps** are an evolution of this concept, which store values (usually integer 8 bit (0-255)) which represent how preferred it is to travel through a cell in the grid.  Now it is possible to not only say if a cell is forbidden (occupied) but if there is a cell we simply don't like for some reason.



Costmap Layers
---------------

Static Layer
`````````````


Obstacle Layer
```````````````


Inflation Layer
````````````````

Other Layers
`````````````


How to Define a Costmap Configuration in .yaml
-----------------------------------------------

