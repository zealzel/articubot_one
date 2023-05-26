## Launch the robot

```bash
ros2 launch articubot_one launch_robot.launch.py
```

## teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## slam first, nav next

#### slam mapping

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$(ros2 pkg prefix articubot_one)/share/articubot_one/config/mapper_params_online_async.yaml"
```

#### rviz

##### method1: utilaize rviz_launch.py with default config for nav

```bash
ros2 launch nav2_bringup rviz_launch.py
```

##### method2: raw rviz2 with manually adding map, tf, scan etc

```bash
rviz2
```

#### save the map

##### save old map format using nav2 (.pgm, .yaml)

```bash
ros2 run nav2_map_server map_saver_cli -f mymap
```

##### save new serialized map format (map pose-graph) using slam_toolbox

useable for continued mapping, slam_toolbox localization, offline manipulation

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: '$HOME/maps/mymap'"
```

#### navigation

##### method1: use bringup_launch.py

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:="/home/pi/maps/office3_res0.02_0523.yaml" \
  params_file:="/home/pi/zbot_ws/src/articubot_one/config/nav2_params1.yaml"
```

##### method2: use bringup_launch.py

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:="/home/pi/maps/office3_res0.02_0523.yaml"
ros2 launch nav2_bringup navigation.launch.py \
  params_file:="/home/pi/zbot_ws/src/articubot_one/config/nav2_params1.yaml"
```

## slam & nav simultaneously

##### method1: use bringup_launch.py

```bash
ros2 launch nav2_bringup bringup_launch.py map:=none \
  params_file:="$(ros2 pkg prefix articubot_one)/share/articubot_one/config/nav2_params.yaml"
ros2 launch nav2_bringup rviz_launch.py
```

##### method2: use slam_toolbox & nav2_bringup separately

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False
ros2 launch nav2_bringup navigation.launch.py \
  params_file:="$(ros2 pkg prefix articubot_one)/share/articubot_one/config/nav2_params.yaml"
```
