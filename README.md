# Rosbot_workspace


# COSC2781/COSC2814 ROSBot and AIIL Workspace Setup
​
This is TheConstruct examples and setup with the Husarion ROSBot and AIIL standard code.

## BEFORE running anything
​
At the moment, it seems that ROSJect are not saving installed APT package status. This means NOTHING will work unless the required packages are installed, and ROS2 Humble is updated to the latest version.
​
Thus **before** running any commands for the first time when opening this ROSJect you need to:
​
1. Update apt
```
sudo apt update
```
1. Run the AIIL build script. Don't forget the cd! Also don't forget the correct flags
```
cd ~/aiil_workspace
./bin/build_setup.py -g -t
```
   - Say "yes" to:
      - Installing ROS packages
      - Installing TheConstruct packes
      - Building repositories
   - Say "no" to everything else
1. Update all packages. This can take 30mins
```
sudo apt upgrade
```
​## RViz
​
* Running the standard RViz display
```
rviz2 -d ~/aiil_workspace/humble_workspace/src/aiil_rosbot_demo/rviz/gazebo.rviz
```

## AIIL Gazebo Package
​
Within the AIIL repository is the ``aiil_gazebo`` package. This package contains a collection world files that can be used with the Husarion ``rosbot_gazebo`` package for launching Gazebo simulations with the ROSbot. These are also launch files for using SLAM mapping and Navigation. The launch file do not launch RViz by default, instead you can use the typical command above.
​
Here are some sample Gazebo Worlds:
* Empty world - an empty world that only spawns a ROSbot
    - ``ros2 launch rosbot_gazebo simulation.launch.py world:=empty.sdf``
* Husarion World - Husarion's deafult world and used in the Husarion ROS2 tutorials
    - ``ros2 launch rosbot_gazebo simulation.launch.py``
* Maze - an enclosed maze developed for AIIL assignments
    - ``ros2 launch rosbot_gazebo simulation.launch.py world:=/home/user/aiil_workspace/humble_workspace/src/aiil_gazebo/worlds/maze.sdf``
* Maze with Hazard signs - the enclosed maze with Hazard signs scattered around the maze. This will be useful for the Search & Navigation challenge.
    - ``ros2 launch rosbot_gazebo simulation.launch.py world:=/home/user/aiil_workspace/humble_workspace/src/aiil_gazebo/worlds/hazards.sdf``
    
If you wish to learn about how simulation are configured, take the "Introduction to Gazebo Sim with ROS2" TheConstruct course.
​
### Additional launch files
​
The above launch scipts only start the simulation with the robot sensors. To run other software use the following launch files
​
* Keyboard Teleop Control
    - ``ros2 run teleop_twist_keyboard teleop_twist_keyboard``
* SLAM - uses the configuration slam files in aiil_gazebo which may differ slightly from the ROSBot
    - ``ros2 launch aiil_gazebo slam.launch.py``
* Navigation - uses the configuration slam files in aiil_gazebo which may differ slightly from the ROSBot
    - ``ros2 launch aiil_gazebo nav.launch.py``
​
### Sourcing ROS2 workspace
​
If you need to re-source the workspace, use:
```
source ~/aiil_workspace/humble_workspace/install/setup.bash
```
​
### Typical Workflow
​
The typical workflow for using the Gazebo simulations and AIIL software is:
​
1. Launch the chosen world
1. Launch RViz separately
1. Launch your own ROS package(s) separately
​
For example, if you have chosen to launch the hazards world:
* You should see a Gazebo simulation similar to this
* You should be able to see RViz with the simulated camera, and various map information
* You can change topics to see the simulated depth camera image
* You can specify a navigation point through RViz (if you choose)
* Altneratively, launch your own code

## Using your own ROS Packages
​
To write your own code you will need to create a new ROS package. You will want to create this package within the ``humble_workspace`` of the AIIL codebase. This should follow a similar process to getting your code in other places. In brief
​
​
1. Clone your repository into the ``humble_workspace/src`` folder. For example (you can replace the ``my_packges`` folder name as you desire:
    ```
    git clone <url> ~/aiil_workspace/humble_workspace/src/my_packages
    ```
    This cloned repo repository will serve as the root folder in which you will place all your packages. Colcon (and ROS) allows sub-folder layouts. That is, your folder structure will eventually look like:
    ```
    ├── aiil_workspace/humble_workspace/src
    │                                   ├── aiil_rosbot_demo/
    │                                   ├── aiil_gazebo/
    │                                   ├── my_packages/
    │                                   │   ├── package_1/
    │                                   │   ├── package_2/
    │                                   │   ├── ...
    ```
1. Now, go to the ``humble_workspace`` folder and run Colcon.
```
cd ~/aiil_workspace/humble_workspace
colcon build
```
1. Now test out the code:
    1. Once Gazebo has launched, in a second terminal, run *your* package.
    ```
    ros2 launch <package> <launch_file>
    ```

## Running your code on the ROSbot
​
At present, we do not have the ability to run code from within TheConstruct's platform on the ROSbots directly. Instead you will need to follow the usual steps (documented on the wiki) to run code on the robot

## Known Issues
​
**Gazebo multiple robot spawn**
​
If Gazebo (IGN) doesn't shutdown properly, then it will not launch correctly the next time. You will need to manually kill the ign process
​
* Find the IGN process
```
ps aux | grep -i ign
```
* Using the above get the process id
* Kill the process
```
kill -9 <process_id>
```
​
​
**SSH Key reset/removed/missing**
​
On relaunch, sometimes the ssh keys may not be present in the ``~/.ssh/`` folder. In particular, the default SSH key may be missing. A copy of the default keys is mainted in ``~/notebook_ws/sshkey``. The default SSH keys can be restored by linking to the notebook copies by executing:
​
```
ln -s ~/notebook_ws/sshkey/default_rmit_public_vm ~/.ssh/id_rsa
ln -s ~/notebook_ws/sshkey/default_rmit_public_vm.pub ~/.ssh/id_rsa.pub
```

