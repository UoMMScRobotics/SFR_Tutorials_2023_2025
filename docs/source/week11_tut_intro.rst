Behaviour Trees Introduction
================================================

Behaviour trees are a really powerful means for us to add some autonomy to our robots, taking them from glorified remote control cars to realised robots.  Considered more flexible and modular than more traditional Finite State Machines, they have become an integral part of the ROS2 navigation stack.

.. note::
    In ROS 1, the navigation stack known broadly as the ``move_base`` package, relied on a `state machine architecture <http://wiki.ros.org/move_base#Expected_Robot_Behavior>`_.  This is not to say that state machines are bad, quite the opposite, for example the lifecycle node implementation in ROS2 is state machine based.

This tutorial, rather than explaining behaviour trees (which is covered adequately elsewhere), is focussed on how to construct behaviour trees using the existing nodes available in the Nav2 stack of ROS2 (Humble).  Therefore, you should be familiar with the concepts:

- An action node
- A condition node
- A control node
- A decorator Node
- Leaf nodes and parents
- Ticks
- Behaviour tree blackboards
- Outcomes (``SUCCESS``, ``FAILURE``, ``RUNNING``)

You should also be aware that we will be talking about nodes, and do not mean normal ROS nodes.  Furthermore, action nodes usually call ROS action servers, so keep your wits about you.  In this tutorial, behaviour tree nodes will be called nodes, whereas any ROS action servers will be explicitly referred as an "action server".

This tutorial will build up an autonomous navigation behaviour tree, covering the additional behaviour controls which are available through Nav2 (beyond the classic ``Fallback`` and ``Sequence`` controls).  There is nothing to stop you writing behaviour trees for non-navigation tasks (e.g. grasping)!

The Behaviour Tree functionality of Nav2 actually comes from a separate solution called `BehaviorTree.CPP <https://github.com/BehaviorTree/BehaviorTree.CPP>`_ as well as a companion GUI called `Groot <https://github.com/BehaviorTree/Groot/tree/master>`_.  As a new version of both BehaviorTree.CPP and Groot are in the works, this tutorial will likely not be valid for much longer.  Keep an eye out for new versions, and more importantly when ROS2/Nav2 actually implements/supports any new versions.

.. note::
    As a British English speaker, I have a habit of writing "behavio*u*r" with a "u".  How you chose to spell the word does not bother me.  Links to pages or packages will have the explicit spelling they use, therefore, you may see a mix of with "u" and without "u".

Installation of BehaviorTree.CPP and Groot
-------------------------------------------
BehaviorTree.CPP should already be installed to support the Nav2 stack.  A quick check using,

.. code-block:: console

    ros2 pkg list | grep behaviortree

should result in ``behaviortree_cpp_v3``.  If this is not the case, it can be installed the same way any ROS package is installed:

.. code-block:: console

    sudo apt install ros-humble-behaviortree-cpp-v3

Groot in comparison is not installed, and we will need to clone the available repository and ``colcon build``.  You can either use an existing ros workspace or make a new one.  For the following guide we will assume the workspace is called ``groot_ws``.

.. code-block:: console

    cd ~/groot_ws/src/
    git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git
    cd ..
    colcon build
    source install/setup.bash

You can chose to manually source the workspace (``source ~/groot_ws/install/setup.bash``) or add it to your ``.bashrc`` file.

Making a Package for This Tutorial
-----------------------------------

We will utilise a new ROS2 package called ``bt_demos`` for these activities, which will contain three additional directories ``behavior_tree_xml``, ``launch``, and ``config``.  The workspace ``MY_ROS_WS`` is a placeholder for where you wish to build this package.

Make a package the usual way:

.. code-block:: console

    cd ~/MY_ROS_WS/src/
    ros2 pkg create bt_demos --build-type ament_python
    cd bt_demos
    mkdir behavior_tree_xml
    mkdir launch
    mkdir config

Ensure you have made the additional modifications to the setup.py file.

.. literalinclude:: ../../ros_ws/src/bt_demos/setup.py
   :language: python
   :emphasize-lines: 2,3,14-17
   :linenos: