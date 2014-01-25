g2o tutorial
============

g2o is an open source c++ general graph optimization framework.  It has been developed in the context of robotics and simultaneous localization and mapping however it is a general optimization framework and may easily be used for other purposes.

g2o github:

https://github.com/RainerKuemmerle/g2o

Original publication:

http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf

Introduction/tutorial paper:

http://openslam.informatik.uni-freiburg.de/data/svn/g2o/trunk/g2o/doc/g2o.pdf

I found the example code in g2o not to be the best for getting started because there are no examples with visualizations, and all the code resides in the library, that is, they do not show how to use the library and related tools in your own project.  This repository attempts to provide such examples.

I have just one example so far: 3D keyframe SLAM with a monocular camera.  Bundle adjusts keyframe positions and landmarks using reprojection error assuming a pinhole camera model.  Keyframe poses, landmarks and landmark observations are all synthetically generated

Installation
--

This has been programmed for ROS groovy using rosbuild.  Should work for older versions of ROS too.  This assumes you have cloned this repos into a ros workspace that works with rosbuild

I don't know anything about this catkin business so please don't ask!

First install the sys debs:
```
sudo apt-get install libglew1.6-dev libglewmx1.6-dev glew-utils libqt4-dev libqglviewer-qt4-dev libsuitesparse-dev
```

Then compile:
```
g2o_tutorial/g2o_viewer$ rosmake
```

or compile one by one (g2o takes forever to compile and the compile output is less boring) 
```
g2o_tutorial/g2o$ make
g2o_tutorial/g2o_example$ make
g2o_tutorial/g2o_viewer$ make
```
Run an example
--

Create a g2o graph using synthetic data
```
rosrun g2o_example example
```

View and optimize
```
rosrun g2o_viewer g2o_viewer example_g2o.g2o
```

Select a variant of lm for optimizing!

Please read the wiki for more information about using these examples!

TODO
--

-  Include code to show how to optimize the graph (so far only optimizing with g2o viewer)
-  Bundle adjustment with real data
-  Example of optimization for extrinsic calibration


