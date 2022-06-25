# Omnidirectional Robot Localization and Mapping

This repository contains the second laboratory project for the [Robotics course 2021/2022](https://www4.ceda.polimi.it/manifesti/manifesti/controller/ManifestoPublic.do?EVN_DETTAGLIO_RIGA_MANIFESTO=evento&aa=2021&k_cf=225&k_corso_la=481&k_indir=T2A&codDescr=089013&lang=IT&semestre=2&idGruppo=4336&idRiga=271084) of the Polytechnic University of Milan.

The project is implemented in ROS, an open-source robotics middleware suit.

## Requirements

- C++
- Ubuntu 18.04 (20.04 is okay too)
- [ROS Melodic (Noetic is okay too)](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [ROS gmapping](http://wiki.ros.org/gmapping)
- [ROS map server](http://wiki.ros.org/map_server)
- [ROS amcl](http://wiki.ros.org/amcl)

## Project structure (TODO copied from first project, to change)

TODO

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

Then, you can start one of the tree bags, as you want. Just notice that one of the three bags doesn't have a good mapping capability:
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

Then, you can start one of the tree bags, as you want:
```
rosbag play --clock <bag_name>.bag
```

Remember the `--clock` argument to let the node synchronize with the bag's clock.

With `rviz` you can see the localization process in real-time.

During or after the localization process, it is possible to save the the robot followed trajectory so far. We built a service that takes as input, from command line, the name of the image that you want to save, and it automatically saves an image with the trajectory of the robot under the [/maps/path_images/](src/omnirobot_loc_and_mapping/maps/path_images/) folder:
```
rosservice call /save_trajectory <name_of_the_image>
```

For example: `rosservice call /save_trajectory robot_trajectory_bag1`.

We already saved the trajectories of the robot with bag 1 and 3 under the folder [previously_generated_maps/](previously_generated_maps/).

## Gmapping parameters
- MaxURagnge must be <= MaxRange, and we put it equal because we empirically saw a better map in this way, rather than 15 and 16, we put 16 and 16. We chose 16 looking at the data sheet.
We tried the minimum score at 50, 200 and 400. At 50, we considered it to be less realistic than the one at 200. In particular, at 50 there are some clusters of obstacles in the middle of the map that are likely to be non existent. At 400, a lot of points that are likely to be the wall of the room, expecially on the south, are not sensed (they probably don't score enough points) and are put out of bounds.
We tried the number of particles, and 30 seems a good trade-off
We tried the map_update_interval to 2 seconds, 0.1 seconds and 0.001 seconds. The update each 0.1 seconds seems to be the best accurate.

TODO continue it

## Laser merger parameters

TODO explain it

## AMCL parameters 

- Initial poses to zero
- laser max range
- map_topic
- scan_topic
- frames id
- odom_model_type = omni
- number of particles are okay because it represents the actual path that the robot does

TODO explain


## How the map has been created

TODO explain:
- The launch file amcl.launch.xml has been launched
- The SECOND bag has been played with `--clock` argument active
- After the end of the bag, the script `map_smoother` has been launched: `rosrun omnirobot_loc_and_mapping map_smoother.py`
- After the script says "Custom map published in topic /map_smoothed", from command line is called the map server: `rosrun map_server map_saver -f map map:=/map_smoothed`. In this way, the map saved has been processed with openCV to correct some minor imprecisions. In fact, the borders are better defined and some very small obstacles are changed in considering the cell free. It's been saved also the normal map with `rosrun map_server map_saver -f raw_map`, but in this case we saved it in the folder `asset` to let you see the difference.


## How the trajectory has been created

TODO explain:
After launching the localization, you can run the service `save trajectory`, passing as argument the name of the image that you want to save inside `maps/path_images`: `rosservice call /save_trajectory <name_of_the_image>`. `rosservice call /save_trajectory robot_trajectory_bag1` is an example. You can call this service when you prefer, and the service will save an image with the path done by the robot so far. Notice that if you give a name that already exists, the new image will overwrite the old one.