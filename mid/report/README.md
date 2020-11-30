## 程式執行
在建立workspace，執行完catkin_make和source devel/setup.bash後

使用roslaunch來執行相關node

第一題：`roslaunch localization_309605008 Q1.launch`

第二題：`roslaunch localization_309605008 Q2.launch`

第三題：`roslaunch localization_309605008 Q3.launch`


## 讀取map路徑修改
第一題 catkin_ws/src/localization_309605008/src/icp_locolization1.cpp57行

`if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/bory/sdc/localization/map/itri_map.pcd", *load_map) == -1)`

第二題 catkin_ws/src/localization_309605008/src/icp_locolization2.cpp59行

`if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/bory/sdc/localization/map/nuscenes_map.pcd", *load_map) == -1)`

第三題 catkin_ws/src/localization_309605008/src/icp_locolization3.cpp59行

`if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/bory/sdc/localization/map/nuscenes_map.pcd", *load_map) == -1)`
