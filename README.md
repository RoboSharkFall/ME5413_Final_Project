# ME5413 Final Project

------

This work conducts the Mapping and Navigation of the robot in the Gazebo world **Assemble Line**, there are mainly 2 works conducted, which are shown below.

> * Mapping the environment by using the Jackal Robot.
> * Navigate the robot to the assembly line, packing area, and vehicle respectively.

Now it is going to introduce how to run every work :

# Dependencies
------------------------------------------------------------
> * **System Requirements**:
 * Ubuntu 20.04 (18.04 not yet tested)
 * ROS Noetic (Melodic not yet tested)
 * C++11 and above
 * CMake: 3.0.2 and above

# Installation
------------------------------------------------------------
This repo is a ros workspace, containing three rospkgs :
 > * `interactive_tools` are customized tools to interact with the gazebo and your robot
 > * `jackal_description` contains the modified jackal robot model descriptions
 > * `me5413_world` the main pkg containing the gazebo world, and the launch files
  > * `dji_nav` a navigation package using Dijkstra global planner and base local planner, which has the best performance
  > * `astar_nav` are navigation package using A* global planner and base local planner.
  > * `avoid_nav` a navigation package using the Dijkstra global planner and Teb local planner
  > * `my_nav` navigation package is a package provided to new users who want to try their own method
  > * `calibrate_imu` is a package used to calibrate the IMU sensor.
    > * `costmap_prohibition_layer` are a package used to produce the prohibition layer in costmap.
     > * `master` is a package used to provide Velodyne simulator.
  
  
 
Use the following command to download the repository : 
```
 # Clone your fork of this repo (assuming home here `~/`)
 **cd**
git clone https://github.com/RoboSharkFall/ME5413_Final_Project.git
cd ME5413_Final_Project
# Install all dependencies
rodeo install --from-paths src --ignore-src -r -y
# Build
catkin_make
# Source 
source devel/setup.bash
```
To properly load the gazebo world, you must have the necessary model files in the `~/.gazebo/models/` directory.

There are two sources of models needed:
> * **Gazebo Model**
```
# Create the destination directory
cd
mkdir -p .gazebo/models
# Clone the official gazebo models repo (assuming home here `~/`)
git clone https://github.com/osrf/gazebo_models.git
# Copy the models into the `~/.gazebo/models` directory
cp -r ~/gazebo_models/* ~/.gazebo/models
```
> * **Our customized models**
```
# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y
# Build
catkin_make
# Source 
source devel/setup.bash
```

# Working Pipeline
------------------------------------------------------------
**0. Gazebo World**
This command will launch the gazebo with the project world : 
> \# Launch Gazebo World together with our robot
    roslaunch me5413_world world.launch
    
**1. Manual Control**
If you wish to explore the gazebo world a bit, we provide you a way to manually control the robot around:
> \# Only launch the robot keyboard teleop control
  roslaunch me5413_world manual.launch
  
**Note**: This robot keyboard teleop control is also included in all other launch files, so you don't need to launch this when you do mapping or navigation.  

**2. Mapping**
 After launching **Step 0**, in the second terminal:
> \# Launch GMapping
roslaunch me5413_world mapping.launch

After finishing mapping, run the following command in the thrid terminal to save the map:
> \# Save the map as `my_map` in the `maps/` folder
roscd me5413_world/maps/
rosrun map_server map_saver -f my_map map:=/map

You may want to use other mapping method, in this work we use other two method : A-LOAM and Fast-LIO, and the EVO tool is used for mapping performance evaluation. The repositories are as follows :
> **A-LOAM** (https://github.com/nuslde/aloam_lidar_odom_result_generate)
**Fast-LIO** (https://github.com/hku-mars/FAST_LIO)
**EVO tool** (https://github.com/MichaelGrupp/evo)

The data you need for mapping and evaluation are as follows :
> `/mid/points` : **Topic publish the 3D lidar data.**
`/imu/data` : **Topic publish the IMU data.**
`/gazebo/ground_truth/state` : **Topic publish the ground truth data.**

During running the mapping, run the following command to save the point cloud

> rosrun pcl_ros pointcloud_to_pcd /input:=/globalmap

In order to do dowm sample and RANSAC to remove ground points, run the following python code : 

```python
pcd = o3d.io.read_point_cloud(pcl_file)
pcd_rotate = copy.deepcopy(pcd)
R = pcd_rotate.get_rotation_matrix_from_xyz(rotation=[0, 0, 5 * np.pi / 12])
pcd_rotate.rotate(R=R, center=np.array([0, 0, 0]))
o3d.visualization.draw_geometries([pcd, pcd_rotate])
downsample_pcd = pcd.voxel_down_sample(voxel_size=0.2)
o3d.visualization.draw_geometries([downsample_pcd])
o3d.io.write_point_cloud('path/for/saving/downsampled.pcd', downsample_pcd)
plane_model, inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=1000)
inlier_cloud = pcd.select_by_index(inliers)
outlier_cloud = pcd.select_by_index(inliers, invert=True)
if debug:
    print("Segmentation Time", time.time() - t)
    o3d.visualization.draw_geometries([outlier_pts])
o3d.io.write_point_cloud('path/for/saving/output.pcd', outlier_pts)
```

In order to turn the `.pcd` file to `.pgm` map and the `yaml` file for navigation, running the following command
> mkdir -p ~/pcd2pgm_ws/src
cd ~/pcd2pgm_ws/src
catkin_init_workspace
git clone https://github.com/hujiax380/pcd2pgm.git
cd ~/pcd2pgm_ws

Then go to the `/home/USERNAME/pcd2pgm_ws/src/pcd2pgm/pcd2pgm/src/test.cpp` change the following code :
```C++
//Line 57
private_nh.param("file_directory", file_directory, std::string("/home/YOUR_USER_NAME/"));  //name of your device
ROS_INFO("*** file_directory = %s ***\n", file_directory.c_str());
private_nh.param("file_name", file_name, std::string("PCD_FILE_NAME"));  //Your pcd file name
```
After changing the code, run the following command : 
> catkin_make
source devel/setup.bash
rosrun pcd2pgm pcd2topic

then run the command **rosrun map_server map_saver** to save the map. Then change the map name to `my_map.pgm` and `my_map.yaml`, and modify code of `my_map.yaml` : **image: map.pgm** to **image: my_map.pgm**. After modify, copy the files to ``ME5413_Final_Project/src/me5413_world/maps/``, the map is complete.

**3. Navigation**
After install all the dependence and package, you can try to navigate the robot run the following command.
we need to launch the gazebo model first to provide basic data and information:
> \# Launch the robot 
  cd ~/ME5413_Final_Project/
source devel/setup.bash
roslaunch me5413_world world.launch
  
  Using 'dji_nav' package, which is Dijkstra global planner and base local planner(Recommened)
  > \# Launch the dji_nav
  source ~/ME5413_Final_Project/devel/setup.bash
roslaunch me5413_world dji_navigation.launch

  using 'astar_nav' package, which  using A* global planner and base local planner.
    > \# Launch the astar_nav 
source ~/ME5413_Final_Project/devel/setup.bash
roslaunch me5413_world atar_navigation.launch

  using  `avoid_nav` package, which using A* global planner and TEB local planner.
    > \# Launch the avoid_nav 
source ~/ME5413_Final_Project/devel/setup.bash
roslaunch me5413_world avoid_navigation.launch
