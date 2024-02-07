# LES
Locally exploitative sampling for robot path planning. 

Please refer to the paper here for more details on the approach: https://arxiv.org/pdf/2102.13064.pdf

## Dependencies 
This repository is a ROS-Kinetic package. All the code is written using the popular OMPL framework. 
Also requires python and matplotlib.

Clone this package into your catkin workspace and compile with 
```
cd catkin_ws/src
cd ..
catkin_make
```

This repo contains the following custom OMPL planners:

RRTsharp: conventional RRTsharp with informed sampling 

ESTRELRRTsharp: RRTsharp with relevant region sampling framework 

LESDRRT: Custom DRRT planner with a few bug fixes from the original code

TRUSTRRTsharp: RRTsharp running the proposed LES framework.

For running a 2D planning case, navigate to trust_region_les/src and do
```
rosrun trust_region trustregion_visualize
python plothpath.py
```
to quickly plot the graph and solution.
