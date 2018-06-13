## Intro

ROS wrapper for CFTld with multiple objects, a long-term visual tracker, Based on [CFTld tracker](https://github.com/klahaag/CFtld),
Forked from [CFTld ROS wrapper](https://github.com/AutonomyLab/cftld_ros),

## Misc

Debug the nodelet with GDB:

```bash
$ rosrun --prefix "gdb --args" nodelet nodelet standalone multi_cftld_ros/MultiCFtldRosNodelet```

### Some resources

- https://github.com/ros-perception/image_pipeline/tree/indigo/image_proc/src
- https://groups.google.com/forum/?utm_medium=email&utm_source=footer#!msg/ros-sig-perception/K5__71SX7eU/mxWwn3AeAwAJ
- http://wiki.ros.org/opencv3
- http://answers.ros.org/question/214043/use-ros-indigo-opencv3-alongside-248/
- https://github.com/ros-perception/vision_opencv/issues/91
- http://tayyabnaseer.blogspot.ca/2013/04/porting-nodes-to-nodelets-in-ros.html
