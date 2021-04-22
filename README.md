# experiments-loop-functions-ros
Prototypes of ARGoS loop functions that enable importing or exporting data to ROS

Requires:

Tested with ROS Noetic and Ubuntu 20.04

To compile, enter the following commands 

`cd experiments-loop-functions-ros` 

`catkin_make`

Experiment files are provided in the scenario directory. To run an experiment

`roscore`  -> if it was not running before

`argos3 -c experiment_file.argos`
