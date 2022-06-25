# Omnidirectional Robot Localization and Mapping

This repository contains the second laboratory project for the [Robotics course 2021/2022](https://www4.ceda.polimi.it/manifesti/manifesti/controller/ManifestoPublic.do?EVN_DETTAGLIO_RIGA_MANIFESTO=evento&aa=2021&k_cf=225&k_corso_la=481&k_indir=T2A&codDescr=089013&lang=IT&semestre=2&idGruppo=4336&idRiga=271084) of the Polytechnic University of Milan.

The project is implemented in ROS, an open-source robotics middleware suit.

## Requirements

TODO

## Project structure (TODO copied from first project, to change)

TODO

## Getting Started

You can either clone the repository and change directory:
```
$ git clone https://github.com/davide-giacomini/omnidirectional-robot-localization-and-mapping.git
$ cd omnidirectional-robot-localization-and-mapping/
```

Or you can download the zip file, and unzip it where you prefer and change the name of the directory in `omnidirectional-robot-localization-and-mapping/`. Then, you change directory:
```
$ unzip <directory_name.zip> -d <destination_folder>
$ cd <path/to/destination_folder>
$ mv <directory_name/> omnidirectional-robot-localization-and-mapping/
$ cd omnidirectional-robot-localization-and-mapping
```

Build the environment with catkin:
```
$ catkin_make
```

Add ROS worspace to your system. Add the end of the `bashrc` file the path to the `devel/setup.bash` file:
```
$ echo "source </path/to/project/folder/>omnidirectional-robot-localization-and-mapping/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Warning: In case you have already defined in your `bashrc` file a path for another workspace, comment or delete it. Each machine must have only one workspace defined.

With the launch file you can start either the localization or the mapping. In both the cases, the launch file will open `rviz` too, with some basic configurations to facilitate the visualization.

To launch the mapping you type:
```
$ roslaunch omnirobot_loc_and_mapping mapping_launcher.launch
```

To launch the localization you type:
```
$ roslaunch omnirobot_loc_and_mapping localization_launcher.launch
```

Once started, you can use one of the three bags to perform mapping or localization, depending on what you started.

TODO: write about which bag has been used for map creation and how to save an image and the trajectory of the robot through a service

### Using other services

TODO

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