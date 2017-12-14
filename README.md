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

```
  cd catkin_workspace/src
  git clone git@progress-gitlab.eecs.umich.edu:4progress/centroid_detector.git
  cd ../
  catkin_make
```


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

* **`~/min_pc_x`** (float, default 0)

    The minimum x for the crop box

* **`~/max_pc_x`** (float, default 1)

    The maximum x for the crop box

* **`~/min_pc_y`** (float, default -1)

    The minimum y for the crop box

* **`~/max_pc_y`** (float, default 1)

    The maximum y for the crop box

* **`~/min_pc_z`** (float, default 0.82)
 
    The minimum z for the crop box

* **`~/max_pc_z`** (float, default 1.016)

    The maximum z for the crop box

* **`~/nearest_neighbor_radius`** (float, default 0.03)

    The radius for nearest neighbor search in the euclidean clustering

* **`~/min_centroid_seperation`** (float, default 0.05)

    The clossest two centroid y values can be before they are considered the same cluster


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://progress-gitlab.eecs.umich.edu/4progress/centroid_detector/issues).
