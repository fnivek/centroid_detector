# Centroid Detector

## Overview

This package implements a centroid detector for mobile manipulation.
It works by doing the following actions:
* Croping out all points outside of a axis aligned bounding box in the robots `/base_link`
* Clustering points together with euclidean clustering
* Then the most center cluster's centroid is returned

The source code is released under a [TODO license](TODO).

**Author(s): Kevin French
Maintainer: Kevin French kdfrench@umich.edu
Affiliation: Laboratory for Progress University of Michigan**

[![Build Status](TODO)](TODO)

![Teaser](TODO)


## Installation

### Building from Source
TODO

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [pcl](http://pointclouds.org) (a standalone, large scale, open project for 2D/3D image and point cloud processing)
- [tf](http://wiki.ros.org/tf) (a package that lets the user keep track of multiple coordinate frames over time)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

  cd catkin_workspace/src
  git clone TODO
  cd ../
  catkin_make


### Unit Tests

TODO


## Usage

Run the main node with

```
  rosrun centroid_detector centroid_detector
```

## Nodes

### centroid_detector

Runs the centroid detector.


#### Subscribed Topics

* **`/head_camera/depth/points`** ([sensor_msgs/PointCloud2])

    The point cloud produced by the robot

#### Published Topics

None


#### Services

* **`/centroid_detector`** ([centroid_detector_msgs/DetectCentroid])

  Returns the center most centroid of the clustered points in a crop box

#### Actions

None


#### Parameters

None


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://progress-gitlab.eecs.umich.edu/4progress/state_manager/issues).
