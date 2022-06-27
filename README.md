# Omnidirectional Robot Localization and Mapping

This repository contains the second laboratory project for the [Robotics course 2021/2022](https://www4.ceda.polimi.it/manifesti/manifesti/controller/ManifestoPublic.do?EVN_DETTAGLIO_RIGA_MANIFESTO=evento&aa=2021&k_cf=225&k_corso_la=481&k_indir=T2A&codDescr=089013&lang=IT&semestre=2&idGruppo=4336&idRiga=271084) of the Polytechnic University of Milan.

The project is implemented in ROS, an open-source robotics middleware suit.

## Requirements

- C++
- Python 3
- Ubuntu 18.04 (20.04 is okay too)
- [ROS Melodic (Noetic is okay too)](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [ROS gmapping](http://wiki.ros.org/gmapping)
- [ROS map server](http://wiki.ros.org/map_server)
- [ROS amcl](http://wiki.ros.org/amcl)

## Project structure 

The project contains one package (`omnirobot_loc_and_mapping`), which root folder is the [/src/omnirobot_loc_and_mapping/](src/omnirobot_loc_and_mapping/) folder. We will consider this as root folder.

The source files are under the [src/](src/omnirobot_loc_and_mapping/src/) folder:
- `odom_tf.cpp`: it deals with the need of having the odometry published as a tf

The Python scripts are under [scripts](src/omnirobot_loc_and_mapping/scripts/) folder:
- `map_smoother.py`: it computes some post-processing techniques to the map, in order to have it smoother.
- `trajectory_saver.py`: it is in charge of receiving the current positions of the robot, given from amcl_pose, in order to draw the trajectory on the map and save it.
- `trajectory_drawer.py`: it is in charge of drawing the trajectory on rviz.


The [launch files](src/omnirobot_loc_and_mapping/launch/) contains all the files to start the nodes for mapping, localization, laser merge and trasformation. It also contains the xml files for gmapping and amcl, where it is possible to tune all the parameters.

Under [maps](src/omnirobot_loc_and_mapping/maps/) it is possible to find the generated map (gmapping) and the maps with trajectories (amcl).

Finally are defined `CMakeLists.txt` and `package.xml` for compilation purposes.

## Getting Started

You can either clone the repository and change directory:
```
git clone https://github.com/davide-giacomini/omnidirectional-robot-localization-and-mapping.git
cd omnidirectional-robot-localization-and-mapping/
```

Or you can download the zip file, and unzip it where you prefer and change the name of the directory in `omnidirectional-robot-localization-and-mapping/`. Then, you change directory:
```
unzip <directory_name.zip> -d <destination_folder>
cd <path/to/destination_folder>
mv <directory_name/> omnidirectional-robot-localization-and-mapping/
cd omnidirectional-robot-localization-and-mapping
```

Build the environment with catkin:
```
catkin_make
```

Add ROS worspace to your system. Add the end of the `bashrc` file the path to the `devel/setup.bash` file:
```
echo "source </path/to/project/folder/>omnidirectional-robot-localization-and-mapping/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Warning: In case you have already defined in your `bashrc` file a path for another workspace, comment or delete it. Each machine must have only one workspace defined.

Warning: Remember to give executable rights to the scripts in python in order to run them as nodes (otherwise, the launch files will throw an error): `chmod +x <script_name>.py`.

In this project you can choose either to perform the mapping or the localization of the environment. For these tasks there are two launch files (one for localization and one for mapping) and three bag files, introduced [`Project Structure`](https://github.com/davide-giacomini/omnidirectional-robot-localization-and-mapping#project-structure).

During localization, it is possible to save the trajectory of the robot using a service call.

### Performing mapping

To start the mapping is necessary to launch [mapping_launcher.launch](src/omnirobot_loc_and_mapping/launch/mapping_launcher.launch):
```
roslaunch omnirobot_loc_and_mapping mapping_launcher.launch
```

We used `bag 2` for mapping because, after some experiments it seemd to be the one that gave the best quality map. Just notice that also map 1 gave a good quality map, instead bag 3 did not have a good mapping capability.
Then, you can start one of the tree bags, as you want. 
```
rosbag play --clock <bag_name>.bag
```

Remember the `--clock` argument to let the node synchronize with the bag's clock.

With `rviz` you can see the creation of the map in real-time.

When the bag ends, it is possible to save the map using the `map_saver` service of the package [map server](http://wiki.ros.org/map_server#map_saver):
```
rosrun map_server map_saver -f map
```

We also implemented a script to process the image of the map and correct some minor imprecisions that can be easily detected. It can be run as a ros node:
```
rosrun omnirobot_loc_and_mapping map_smoother.py
```

It process the map and publishes the smoothed map on the topic `/map_smoothed`. To save it, the map server can be run again, specifying which topic to read, as the default is `/map`:
```
rosrun map_server map_saver -f map map:=/map_smoothed
``` 

We have previously computed both the map without running the script ([raw_map.pgm](previously_generated_maps/raw_map.pgm)) and the map running it ([map.pgm](previously_generated_maps/map.pgm)). We used the bag 2 to generate it. Notice that the `amcl` node requires the file of the map named `map.pgm` to load the static map correctly. When using `map_saver`, the map must be named `map` thus overwriting the one already present (the previous bash scripts already do it). This is the only way amcl can use it (without changing the code inside [`localization_launcher.launch`](src/omnirobot_loc_and_mapping/launch/localization_launcher.launch)). If you wish to use our generated map for localization without generating one yourselves, just move our map *and the `.yaml` file* under the folder [maps/](src/omnirobot_loc_and_mapping/maps/).

### Performing localization

To start the localization is necessary to launch [localization_launcher.launch](src/omnirobot_loc_and_mapping/launch/localization_launcher.launch):
```
roslaunch omnirobot_loc_and_mapping localization_launcher.launch
```

Then, you can start one of the tree bags, as you want (we used `bag 1 and bag 3` for localization, for the reasons described above):
```
rosbag play --clock <bag_name>.bag
```

Remember the `--clock` argument to let the node synchronize with the bag's clock.

With `rviz` you can see the localization process in real-time.

During or after the localization process, it is possible to save the the robot followed trajectory so far. In order to do it, we used the OpenCV library. We created a node called `trajectory_saver`, it subscribes to 'amcl_pose' topic receiving all the current positions of the robots and storing them locally. Whenever the service is called, we create the trajectory drawing segments between subsequent positions, using the function `cv2.line`. Note here that, in order to map the position of the robot into pixel of the image, we needed to perform a convertion, using the origin and resolution of the image, contained in the metadata of the map.

We built a service that takes as input, from command line, the name of the image that you want to save, and it automatically saves an image with the trajectory of the robot under the [/maps/path_images/](src/omnirobot_loc_and_mapping/maps/path_images/) folder:
```
rosservice call /save_trajectory <name_of_the_image>
```

For example: `rosservice call /save_trajectory robot_trajectory_bag1`.

We already saved the trajectories of the robot with bag 1 and 3 under the folder [previously_generated_maps/](previously_generated_maps/).

## Gmapping parameters
- MaxURagnge must be <= MaxRange, and we put it equal because we empirically saw a better map in this way, rather than 15 and 16, we put 16 and 16. We chose 16 looking at the data sheet.
- We tried the minimum score at 50, 200 and 400. At 50, we considered it to be less realistic than the one at 200. In particular, at 50 there are some clusters of obstacles in the middle of the map that are likely to be non existent. At 400, a lot of points that are likely to be the wall of the room, expecially on the south, are not sensed (they probably don't score enough points) and are put out of bounds.
- We tried different values for the number of particles, and 30 seems a good trade-off
- We tried the map_update_interval to 2 seconds, 0.1 seconds and 0.001 seconds. The update each 0.1 seconds seems to be the best accurate.

Under [assets/images/maps/](assets/images/maps/) is it possible to find all the maps obtained during the parameters tuning described above. 

## Laser merger parameters

For merging the two lasers (one in the front and one in the rear of the robot), we used the package [`ira_laser_tools`](http://wiki.ros.org/ira_laser_tools). We created the node `laserscan_multi_merger` which subscribes to `/front/scan` and `/rear/scan topics`, merges the data coming from the 2 lasers and publishes, in the `/multi_scan topic`, the merged data. 

Here we tuned some parameters, accordingly to the Laser (YDLIDAR G4) characteristics, found on [`YDLIDAR G4 Data Sheet`](https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20G4%20Data%20sheet%20V2.0(220411).pdf). 
In particular:
-	We set the min angle to `-Pi` and max angle to `Pi`,  for a total `360°` of field of view.
-	We set scan_time to `0.0833333` because the max motor frequency (so position changing) is `12Hz`, so the max update should be every 1/12 seconds.
-	We set the angle_increment to `0.00837758` because the angle resolution of the laser, at `12Hz` of frequency, is `0.48` degree, so 0.48/180 * Pi = 0.00837758 radians. 
-	We set range_min to `0.28` and range_max to `16` because those are the values indicated in the manual of the laser.


## AMCL parameters 

- Initial poses to zero
- laser max range to `16`, as indicated in the Data Sheet of the laser.
- map_topic
- scan_topic to `multi_scan`, as described above it is the topic where the merged data from the two laser, will be published
- frames id
- odom_model_type = omni
- number of particles are okay because it represents the actual path that the robot does


## TF tree

The TF tree structure is shown below:

![tf-tree](assets/images/TF_tree.png)

The root of the structure is the frame `map`, which is used by `amcl` to correct the dead reckoning between the odometry frame `odom` and the robot frame `base_link`. In fact, `map` &#10132; `odom` is broadcasted by the node `/amcl`.

The node `/odom_tf` takes as input the odometry information from the topic `/odom` from the bag and broadcasts the correspondent transformation `odom` &#10132; `base_link`. The source code of this node is [src/odom_tf.cpp](src/omnirobot_loc_and_mapping/src/odom_tf.cpp).

Given that the laser sensors are static, for the transformations `base_link` &#10132; `laser_front/rear` a static transform has been used. The same goes for `base_link` &#10132; `multi_scan`, just because it's an identity. The static transformations can be found in the file [static_transforms.launch.xml](src/omnirobot_loc_and_mapping/launch/static_transforms.launch.xml).

The bags also publish `base_footprint` &#10132; `base_link`, which in theory is used to differentiate the center of gravity in 3D from the center of the robot in 2D, for obstacle avoidance, but in this case it was an identity, hence we decided to ignore it and directly link `odom` to `base_link`.

## Authors

- Davide Giacomini ([GitHub](https://github.com/davide-giacomini), [Linkedin](https://www.linkedin.com/in/davide-giacomini/), [email](mailto://giacomini.davide@outlook.com)) --- Person Code: 10567357
- Giuseppe Cerruto ([GitHub](https://github.com/GiuseppeCerruto)) --- Person Code: 10749409
- Matteo Barin ([GitHub](https://github.com/teobarin)) --- Person Code: 10618370
