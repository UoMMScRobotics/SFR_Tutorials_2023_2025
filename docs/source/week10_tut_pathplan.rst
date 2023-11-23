Adding Path Planning & Navigation from Nav2
================================================

In the Nav2 Navigation Stack, the so-called ``Planner`` portion provides the general route from the start to the goal, whilst avoiding any known obstacles based on a map (path planning).  The ``Controller`` is an algorithm to generate velocity commands to move the robot, as well as ensure it tries to follow the path and avoid obstacles.  The ``Controller`` is therefore the most clever and most critical part of the navigation stack.

For both the ``Planner`` and the ``Controller``, each algorithm is suited to a particular design of robot, and may not support your configuration.  For example, not all Planners and Controllers support Ackermann steering (like a car).  You can check the full list of supported `Planners <https://navigation.ros.org/plugins/index.html#planners>`_ and `Controllers <https://navigation.ros.org/plugins/index.html#controllers>`_ in the main documentation.

The Planner
-------------

The default planner in ROS is ``NavFn``.  It contains simple path planning algorithms, either Dijkstra or A* (typically A* is preferred).  It is supplied with a start and end goal, and it computes a valid path between the two.  How this path is calculated will not be covered here, but in the context of costmaps, it attempts to minimise the total cost of a path given the cost going through each cell.

.. note::
    The NavFn (Navigation Function) package is based on the paper:
    `Brock, O. and Oussama K. (1999). High-Speed Navigation Using the Global Dynamic Window Approach. IEEE. <https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf>`_
    Practically every ROS package can trace its heritage back to academic papers.


To operate correctly it requires a "Global" Costmap (i.e. it covers the entire area you would wish to navigate in), as well as robot pose information.  This data of course all come via ROS topics.

Writing the Planner Config File
`````````````````````````````````

An example config file for the NavFN package would look like the file below:


Writing the Global Costmap Config File
```````````````````````````````````````


The Controller
----------------








Adding A Planner and Controller to A Launch File
-------------------------------------------------

