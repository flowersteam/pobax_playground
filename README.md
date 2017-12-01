# POBAX Playground
## Install package and dependencies

1. Setup [ROS and Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) (we're assuming you're on ROS Jade)
2. Clone the dependencies and compile:
```
cd ~/catkin_ws/src  # Or ros_ws if you created such workspace
git clone https://github.com/3rdHand-project/thr_infrastructure.git -b calibrations 

# The packages below are deps of THR Infrastructure
git clone https://github.com/RethinkRobotics/baxter_pykdl.git
git clone https://github.com/baxter-flowers/optitrack_publisher.git
git clone https://github.com/baxter-flowers/baxter_commander.git
git clone https://github.com/baxter-flowers/trac_ik_baxter.git
git clone https://github.com/baxter-flowers/trac_ik.git
sudo apt install libnlopt-dev libnlopt0 ros-jade-nlopt
sudo apt install python-pip ros-jade-moveit  # Moveit has deps for Baxter Commander
sudo pip install xmltodict

cd ~/catkin_ws
catkin_make
```

## Start
```
cd ~/catkin_ws
./baxter.sh   # Make sure you have copied it and edited its variables first. See Baxter workstation setup
roslaunch thr_interaction_controller manual.launch
```

## Troubleshooting
### Object grasping misses precision
Please recalibrate Optitrack using the wanding bar in Motive *and* [the Baxter-Optitrack calibration](https://github.com/baxter-flowers/optitrack_publisher#calibrate).

### Could not load the xml from parameter server: /robot_description
have you executed `./baxter.sh`? it looks like you're using a local ROS Master instead of Baxter's one.
