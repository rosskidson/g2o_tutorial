g2o_tutorial
============

simple tutorial for g2o using ROS

==========================================
Installation
==========================================

This has been programmed for ROS groovy using rosbuild
should work for older versions of ROS too
I don't know anything about this catkin business so please don't ask!

First install the sys debs:
sudo apt-get install libglew1.6-dev libglewmx1.6-dev glew-utils
sudo apt-get install libqt4-dev libqglviewer-qt4-dev
sudo apt-get install libsuitesparse-dev

Then compile:
g2o_tutorial/g2o_viewer$ rosmake

or compile one by one (compile output less boring) 

g2o_tutorial/g2o$ make
g2o_tutorial/g2o_example$ make
g2o_tutorial/g2o_viewer$ make

==========================================
Run the example
==========================================

#Create a g2o graph using synthetic data
rosrun g2o_example example

#View and optimize
rosrun g2o_viewer example example_g2o.g2o

Select a variant of lm for optimizing!

==========================================
TODO
==========================================

Bundle adjustment with real data
Example of optimization for extrinsic calibration
