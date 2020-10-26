# HW3

## shell A
`$ cd ~/self-driving-cars-course/SDC_HW3/catkin_ws/`

`$ catkin_make`

`$ source devel/setup.bash`

`$ roslaunch robot_localization ekf_template.launch `

or

`$ roslaunch robot_localization ukf_template.launch `



## shell B
`$ cd ~/self-driving-cars-course/SDC_HW3/`

`$ rosbag play sdc_hw3.bag --clock`

## shell C
`$ cd ~/self-driving-cars-course/SDC_HW3/`

`$ rosrun rviz rviz -d sdc_ekf_hw.rviz `


