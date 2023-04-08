# Map作成
 ```
 $ mv path/to/map_dir
 ~~$ rosrun tf static_transform_publisher x y z yaw pitch roall map odom 10 ~~
 $ roslaunch fetch_navigation build_map.launch
 $ rosrun rviz rviz # check map, add Map topic and Robot Model 
 $  rosrun map_server map_saver -f map

 ```
 # 確認
 ```
$ rosrun rviz rviz
$ ~~rosrun tf static_transform_publisher x y z yaw pitch roall map odom 10~~
<!-- $ rosrun map_server map_server ${create_map.yaml} -->
 $ roslaunch fetch_test base_nav.launch
 # add RobotModel and map topic
 # change Fixed Frame to [map]  #いらない？
 # rosrun fetch_test get_odom.py # 現在のロボットの座標を取得
 # rosrun
 ```
