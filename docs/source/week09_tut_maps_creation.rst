Maps creations
=========================

In the previous part of this tutorial you have been shown how to use existing map of an environment and how to create one by driving around in simulated world. Here we will show you how to create and import map into gazebo from nothing.

This skill is useful for example if you want to quickly test your robot in a new map or you collected LIDAR scan of real environment and you want to have in your Gazebo for testing.

Generally there are few ways you can go about it:

 - Model your environment as .sdf
 - Model your environment in design software (such as solidworks) and export it as .stl file
 - Draw (or collected using real robot) your maps as .pgm, create assosciated .yaml file and convert those to .stl file.

In SfR module only third method is explored. In this method following steps are necessary

- Draw your map as .pgm in your favourite image creation tool (here GIMP will be used)
- Create assosciated .yaml file (this can be done automatically using Python script)
- Create map .stl file using 'map2gazebo <https://github.com/Adlink-ROS/map2gazebo>' _ package
- Import into gazebo


Drawing the map
=========================

If you look inside .pgm file you will observe it is simply greyscale image with empty spaces represented by white colour and taken by black colour (for the sake of simplicty we will ignore unknown spaces, you are not required to know how to deal with those).

We can easily create such a picture using any graphical editing software. Here we will use GIMP. You can install GIMP by running:

.. code-block:: console
  sudo apt install gimp

Start gimp and you should see:

click *file* > *new*

You should see:

just click *ok* to accept the default settings. You can now draw a simple map using paintbrush tool as highlighted in picture below. 

Just write ROS like so:


click *file* > *save* and save it in ``.../example_gz_robot/world/`` as custom_map.xcf. We also need to export to .pgm. to do this click *file* > *export*, Change file extension to *.pgm* like so:

Click *export* then *export* again in the pop-up window. This completes steps 1
