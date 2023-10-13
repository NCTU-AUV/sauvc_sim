# Gazebo SAUVC environment

Gazebo classic SAUVC environment integrated with ROS noetic

[![](https://img.youtube.com/vi/jII8SlZvBcM/0.jpg)](https://www.youtube.com/watch?v=jII8SlZvBcM)


## Installation

```
export ORCA_WS=/path/to/your/workspace
```

```
mkdir -p ${ORCA_WS}/src
cd ${ORCA_WS}/src
git clone git@github.com:NCTU-AUV/sauvc_sim.git
```

Build the image:

```
cd ${ORCA_WS}/src/sauvc_sim
docker build -f docker/Dockerfile -t sauvc_sim .
```

## Getting started

Start the container:

```
docker compose -f docker-compose.yml run sauvc_sim
```

Build the code:

```
catkin_make
```

Launch the environment

```sh
# lauch the SAUVC arena
roslaunch sauvc_sim sauvc.launch
# lauch an empty world with an auv inside
roslaunch sauvc_sim empty_world.launch
```

Launch keyboard controller

```sh
rosrun sauvc_sim teleop.py
```
