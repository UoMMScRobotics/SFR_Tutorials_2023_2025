Maps creations
=========================

In the previous part of this tutorial you have been shown how to use existing map of an environment and how to create one by driving around in simulated world. Here we will show you how to create and import map into gazebo from nothing.

This skill is useful for example if you want to quickly test your robot in a new map or you collected LIDAR scan of real environment and you want to have in your Gazebo for testing.

Generally there are few ways you can go about it

- Model your environment as .sdf
- Model your environment in design software (such as solidworks) and export it as .stl file
- Draw (or collected using real robot) your maps as .pgm, create assosciated .yaml file and convert those to .stl file.

In SfR module only third method is explored. In this method following steps are necessary

- Draw your map as .png in your favourite image creation tool (here GIMP will be used)
- Create assosciated .yaml and .pgm file (this can be done automatically using Python script)
- Create map .stl file using `map2gazebo <https://github.com/Adlink-ROS/map2gazebo>`_ package
- Import into gazebo

Drawing the map
---------------------------------------

If you look inside .pgm file you will observe it is simply greyscale image with empty spaces represented by white colour and occupied by black colour (for the sake of simplicty we will ignore unknown spaces, you are not required to know how to deal with those).

We can easily create such a picture using any graphical editing software. Here we will use GIMP. You can install GIMP by running:

.. code-block:: console
  sudo apt install gimp

Start gimp and you should see:

.. image:: ../../figures/week09/gimp_initial.png
  :width: 800
  :alt: starting window from GIMP.
  :align: center 


click *file* > *new* You should see:

.. image:: ../../figures/week09/new_image_window.png
  :width: 800
  :alt: New image creation in GIMP.
  :align: center 


just click *ok* to accept the default settings. You can now draw a simple map using paintbrush tool as highlighted in picture below. 

.. image:: ../../figures/week09/blank_image.png
  :width: 800
  :alt: Blank image in GIMP.
  :align: center 

Just write ROS using paintbrush like so:

.. image:: ../../figures/week09/ROS_written.png
  :width: 800
  :alt: ROS map.
  :align: center 


The particular size does not matter as long as all the letters are clearly visible. The scale and dimensions will be dealt with in the next step. Click *file* > *save* and save it in ``.../example_gz_robot/worlds/`` as custom_map.xcf. We also need to export to .png. to do this click *file* > *export*, Change file extension to *.png* like so:

.. image:: ../../figures/week09/Saving.png
  :width: 800
  :alt: Saving ros map.
  :align: center 

Click *export* then *export* again in the pop-up window. This completes step 1.

Create .yaml and .pgm file
---------------------------------------

the .YAML file contains information which allows determination of size of the map (i.e. length of each pixel), while .pgm is ROS acceptable format for maps. While we could do it manually it is a lot more convinent to do so using a script. Please download `script <../../ros_ws
/src/MakeROSMap.py>`_ (this script was originally part of `ROS-Map-Generator <https://github.com/ycprobotics/ROS-Map-Generator/tree/master>`_ , It was modified here to to work with Python3) and put it in ``.../example_gz_robot/worlds/`` directory. Open terminal in ``.../example_gz_robot/worlds/`` folder and type:

.. code-block:: console

  python3 MakeROSMap.py

If you get any error run following python installation commands:

.. code-block:: console

  pip3 install trimesh
  pip3 install numpy
  pip3 install pycollada
  pip3 install scipy
  pip3 install networkx
  pip3 install opencv-contrib-python 

This will open interactive command-line tool where you have to type the following:

.. code-block:: console

  custom_map.png

and press *Enter*

Now we have to select two x-coordinates and two Y coordinates for package to measure dimension. For x coordinate we want to select bottom of letter 'R' and 'S' as highlighted in picture below with red dots.

.. image:: ../../figures/week09/ROS_coordinates_x.png
  :width: 800
  :alt: RViz screen capture of a published map.
  :align: center 

This does not have to be exact right. Once two x-coordinates are selected you should type 

.. code-block:: console

  4

and press *Enter*

To indicate we want this distance to be 4 meters. then to indicate Y-coordinates top and bottom of R should be marked as outlined below.

.. image:: ../../figures/week09/ROS_coordinates_y.png
  :width: 800
  :alt: RViz screen capture of a published map.
  :align: center 

We also type: 

.. code-block:: console

  4

and press *Enter*. then for question about the new name we just type:

.. code-block:: console

  custom_map

and press *Enter* twice. We should know see in Nautilus two new files being added, custom_map.pgm and custom_map.yaml, both needed by ROS.

.. image:: ../../figures/week09/Files_Ready.png
  :width: 800
  :alt: RViz screen capture of a published map.
  :align: center 

Create .stl file
---------------------------------------

Now that we have map file we can use in map server, we also need to create assosciated .stl file for usage in Gazebo. STL files are 3D graphical files (you can find more info `here <https://www.adobe.com/creativecloud/file-types/image/vector/stl-file.html>`_). So far we have 2D map only, we will create 3D equivalent by simply extruding walls from exisitng map. To do so we will use `map2gazebo <https://github.com/Adlink-ROS/map2gazebo>`_ repository. To install follow the installation instruction from the repository. After installation we will be using offline instructions. Thus open new terminal in  ``.../example_gz_robot/worlds/`` folder and type in:

.. code-block:: console
 python3 ~/map2gz_ros2_ws/src/map2gazebo/map2gazebo/map2gazebo_offline.py --map_dir custom_map.pgm --export_dir .

This should create custom_map.stl file which we can use in gazebo

Importing into Gazebo
---------------------------------------

To import into gazebo, first copy custom_map.stl into ``meshes`` folder (just to keep your folder clean). Then, we need .sdf file. Download .sdf file from `here <.../../ros_ws/src/example_gz_robot/worlds/model.sdf>`_ (this is slightly modified sdf file from `map2gazebo <https://github.com/Adlink-ROS/map2gazebo>`_ repository) and put it inside ``worlds`` folder. In essence this file tells gazebo where to look for STL file as well as some basic properties of it. Look inside sdf file, in the lines outlined below we defined the name of our custom stl file:

.. literalinclude:: ../../ros_ws/src/example_gz_robot/worlds/model.sdf
    :language: xml
    :linenos:
    :lines: 20
    :lineno-start: 20


.. literalinclude:: ../../ros_ws/src/example_gz_robot/worlds/model.sdf
    :language: xml
    :linenos:
    :lines: 28
    :lineno-start: 28

 for the sake of this tutorial you don't need to know or understand .sdf files beyond the two highlighted lines.Then we have to tell our *example_gz_robot* package to use our sdf file.

This is simple modification of Launch file ``simulation_bringup.launch.py``in line 41. 

.. literalinclude:: ../../ros_ws/src/example_gz_robot/launch/simulation_bringup.launch.py
    :language: python
    :linenos:
    :lines: 41


with

.. code-block:: python

 sdf_path = os.path.join(get_package_share_directory('example_gz_robot'), 'worlds', 'model.sdf')

This just tells launch file to look for new sdf file. Now you can fully use your new map. Launch the SLAM like before:

.. code-block:: console

    source ~/<YOUR_ROS_WS>/install/setup.bash
    ros2 launch example_gz_robot slam.launch.py

You should see your new map in Gazebo and laser scan in RVIZ. Drive the robot around to complete the map in RVIZ.



