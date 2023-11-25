Adding Path Planning from Nav2
================================================

In the Nav2 Navigation Stack, the so-called ``Planner`` portion provides the general route from the start to the goal, whilst avoiding any known obstacles based on a map (path planning).  The ``Controller`` is an algorithm to generate velocity commands to move the robot, as well as ensure it tries to follow the path and avoid obstacles.  The ``Controller`` is therefore the most clever and most critical part of the navigation stack, we will cover that later.

For both the ``Planner`` and the ``Controller``, each algorithm is suited to a particular design of robot, and may not support your configuration.  For example, not all Planners and Controllers support Ackermann steering (like a car).  You can check the full list of supported `Planners <https://navigation.ros.org/plugins/index.html#planners>`_ and `Controllers <https://navigation.ros.org/plugins/index.html#controllers>`_ in the main documentation.

The Planner
-------------

The ``Planner`` server consists of a ROS Node, which is supplied with a start and end goal, and it computes a valid path between the two.  A planner "plugin" actually computes the path, we have freedom on what plugin to choose, giving us flexibility over which algorithm(s) to use.

The default planner plugin in ROS is ``NavFn``.  It contains simple path planning algorithms, either Dijkstra or A* (typically A* is preferred).  It is suitable for differential drive robots, therefore, suitable for our simulation.

.. note::
    The NavFn (Navigation Function) package is based on the paper:
    `Brock, O. and Oussama K. (1999). High-Speed Navigation Using the Global Dynamic Window Approach <https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf>`_.
    
    Practically every ROS package can trace its heritage back to academic papers.


Along side robot pose estimates, the Planner requires a "Global" Costmap (i.e. it covers the entire global area you would wish to navigate in), this is handled by the planner server as well.  The data to make the costmap and provide the start and end poses are of course all communicated via various ROS topics.

Writing the Planner Config File
`````````````````````````````````

The format of the configuration is taken from the `planner documentation <https://navigation.ros.org/configuration/packages/configuring-planner-server.html>`_.  An example config file for the NavFn package would look like the file below.  Copy this example into a file called ``planner.yaml`` in the ``config`` directory.


.. literalinclude:: ../../ros_ws/src/navigation_demos/config/planner_only.yaml
   :language: xml
   :linenos:

If you look under ``plugin: "nav2_navfn_planner/NavfnPlanner"``, notice there are additional parameters (*tolerance*, *use_astar*, *allow_unknown*).  These parameters are specific to ``NavFn`` as per its `documentation <https://navigation.ros.org/configuration/packages/configuring-navfn.html>`_.  These options are explained in the table below.

.. list-table:: NavFn Plugin Options (non-exhaustive)
   :widths: 20 20 56
   :header-rows: 1

   * - Option
     - Default Value
     - Notes
   * - tolerance
     - 0.5
     - Tolerance in meters between requested goal pose and end of path.
   * - use_astar
     - False
     - Whether to use A*. If false, uses Dijkstra’s expansion.
   * - allow_unknown
     - True
     - Whether to allow planning in unknown space.



Writing the Global Costmap Config File
```````````````````````````````````````
For the Global Costmap, we can simply use our SLAM Map (as a static layer), an obstacle layer using the lidar to catch objects before the static map updates, and include an inflation layer based on the robot size.  Edit the ``planner.yaml`` file where the ``NavFn`` parameters were added earlier to match the example below.

.. literalinclude:: ../../ros_ws/src/navigation_demos/config/planner.yaml
   :language: xml
   :emphasize-lines: 12-48
   :linenos:

.. The use of the obstacle layer is somewhat overkill, as will be shown in the ``Controller`` part, we can include unknown obstacles there instead.

There are various parameters associated with the costmap (e.g. *global_frame*, *use_sim_time*, *resolution*) but also for each layer there are additional parameters.  It is clearly visible which parameters below to which seciton by the indentation scheme that these xml format files use.  For a full list of costmap parameters check out the `costmap_2d github <https://github.com/ros-planning/navigation2/blob/3ed4c2dfa1ef9b31e117ccb5c35486b599e6b97e/nav2_costmap_2d/src/costmap_2d_ros.cpp#L90-L116>`_.

The footprint of the robot is used to calculate if a robot can fit through gaps, and as part of the inflation of the costmap based on physical size of the robot.  It is possible to declare a specific polygon for the footprint of the robot (e.g. four points could define a rectangular chassis), however, to keep things conceptually simpler we will only deal with a radius.

Adding a Planner to a Launch File
----------------------------------

Open the ``nav_demo.launch.py`` file and add the following lines.

.. literalinclude:: ../../ros_ws/src/navigation_demos/launch/planner_only.launch.py
   :language: python
   :emphasize-lines: 19-23, 27, 61-69, 71-78, 87-88
   :linenos:


.. note::
  **What is this lifecycle manager thing?**  It allows for the nodes in the navigation stack to start in a set pattern.  In ROS 1 in particular, nodes may have started in any old order, this could really cause problems.  The lifecycle node system is a method to circumvent this annoying problem.  You can find more technical details in the `online docs <https://navigation.ros.org/configuration/packages/configuring-lifecycle.html>`_.

Perform the usual ``colcon build``, ``source install/setup.bash`` and check the launch file runs.

If everything is running correctly, in rviz it should be possible to view the global costmap topic similar to the image below (note that the specific colour palette comes from selecting "costmap" as the "Color Scheme").

.. image:: ../../figures/rviz_planner_globalcostmap.png
  :width: 600
  :alt: Global Costmap generated by the Planner.

