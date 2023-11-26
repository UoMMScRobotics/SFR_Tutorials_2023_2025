Sending Goals to the Robot Navigation Stack
================================================

Now everything is ready, if you haven't already start the simulation, SLAM and the navigation stack with ``nav_demo.launch.py`` using the normal,

.. code-block:: console

    ros2 launch navigation_demos nav_demo.launch.py 


Navigation Action Servers
---------------------------

Open a new terminal and perform the command

.. code-block:: console

    ros2 action list

It should return something like:

.. code-block:: console
    
    /backup
    /compute_path_through_poses
    /compute_path_to_pose
    /drive_on_heading
    /follow_path
    /navigate_through_poses
    /navigate_to_pose
    /spin
    /wait


These ``Action Servers`` are all supplied by the various plugins.

There is interdependancy between these action servers also, for example, the user would make a call to ``/navigation_to_pose``, which then uses ``compute_path_to_pose`` (Planner) followed by ``follow_path`` (Controller).  It can be a bit of a tangled mess at first glance, but in reality its more linear than it appears.

In a new terminal, run the command ``ros2 run rqt_graph rqt_graph`` or simply run ``rqt`` in a terminal and select the plugin (whichever you are more comfortable with).

In the top left corner, select ``Nodes/Topics (all)`` to get a total overview of all the connections between nodes and their topics.  Press the "refresh" arrows button to update the window.  Hover over the oval containing "/planner_server", the green arrows (published topics) go to "/compute_path_to_pose/_action", "/compute_path_through_pose/_action" and a topic called "/plan".  The "/controller_server" node is similar but provides the "/follow_path/_action" topics, a "/local_plan" and most importantly the "/cmd_vel" velocity commands to the robot.

.. image:: ../../figures/rqt_graph_wk10.png
  :width: 600
  :alt: ROS graph of node and topic interconnectivity.


Send a Goal Pose Manually
---------------------------

To send the robot to a goal we need to provide a goal to the "/navigation_to_pose" action server.  An autonomous algorithm/agent would publish goal messages directly to the action server.  Like all things in ROS, we can send messages in the terminal to emulate this.

Ensure you have RVIZ visible on the screen, and in a new terminal (placed somewhere as to not block your view of RVIZ) send the command below.

.. code-block:: console

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose:
      header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: 'map'
      pose:
        position:
          x: 1.0
          y: 0.0
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    behavior_tree: ''" 

Hooray!  The robot should be navigating!  You should see the robot drive forward, whilst publishing the global path and the trajectory the controller is attempting to take.


Send a Goal Using Visual Tools
--------------------------------

It is much easier as a human to select a point on the map, rather than estimate the coordinates of a position.  In RVIZ, along the top bar there is a button called ``Nav 2 Goal``, read the steps below, then head to rviz to try it out.

1. Press the ``Nav2 Goal`` button to enable the tool
2. Hover over a specific point in the map you wish to navigate to
3. Press and HOLD the left mouse button
4. Drag your mouse around to change the direction of the arrows
5. Release the left mouse button

The base of the arrow indicates the pose position, whereas the arrow indicates the pose orientation.  Once you release the left mouse button, the goal is sent.

.. image:: ../../figures/rviz_Nav2GoalArrow.png
  :width: 600
  :alt: Sending a navigation goal via RVIZ visual tools.

Hooray!  The robot should be driving to where your arrow was!





