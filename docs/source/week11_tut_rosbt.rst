Behaviour Trees in ROS
=======================

As ROS2 uses the existing BehaviorTree.CPP library, behaviour trees are written in `.xml format <https://www.behaviortree.dev/docs/learn-the-basics/xml_format/>`_.  To actually write new nodes in the tree requires writing C++, constucting a tree however only needs an .xml file.  This tutorial only concerns itself with the latter, not the former.

The typical nodes in a behaviour tree are ``Fallback`` and ``Sequence`` control nodes.  The leaf nodes then report ``SUCCESS``, ``FAILURE`` or ``RUNNING`` and the system evolves based on the response of the leaf node.

Nav2 has `additional control nodes <https://navigation.ros.org/behavior_trees/overview/nav2_specific_nodes.html>`_ along side the expected action nodes (e.g. FollowPath), control nodes (e.g. isBatteryLow), and decorator nodes (e.g. RateController).  These are `PipelineSequence <https://navigation.ros.org/behavior_trees/overview/nav2_specific_nodes.html#control-pipelinesequence>`_, `Recovery <https://navigation.ros.org/behavior_trees/overview/nav2_specific_nodes.html#control-recovery>`_, and `RoundRobin <https://navigation.ros.org/behavior_trees/overview/nav2_specific_nodes.html#control-roundrobin>`_.

PipelineSequence
------------------

This acts like a normal ``Sequence`` control node, but it reticks any earlier nodes.  This is useful if the first node is a condition node.

