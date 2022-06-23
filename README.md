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