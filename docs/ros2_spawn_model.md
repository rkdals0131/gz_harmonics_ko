# Spawn a Gazebo model from ROS 2

Gazebo는 시작 시 제공된 월드 파일에 포함된 모든 모델을 스폰합니다.
또한, 언제든지 새로운 모델을 스폰할 수 있습니다. ROS를 사용하여 이를 수행하기 위해 다음과 같은 메커니즘을 제공했습니다:

## Spawn a model using the launch file included in `ros_gz_sim`.

`ros_gz_sim` 패키지에는 `gz_spawn_model.launch.py`라는 launch 파일이 포함되어 있습니다. 이를 사용하여 기존 시뮬레이션에 새 모델을 스폰할 수 있습니다. 다음은 예입니다:

```bash
ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5
```

이 launch 파일에서 허용하는 모든 다른 매개변수를 알아보려면 소스 코드의 [this block](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/gz_spawn_model.launch.py#L26-L45)을 확인하십시오.

## Spawn a model from a custom launch file.

사용자 정의 launch 파일에서 모델을 스폰하는 것도 가능합니다. 이를 위해 XML 또는 YAML launch 파일에서 사용할 수 있는 `<gz_spawn_model/>` 태그를 만들었습니다. 이 경우 인수는 이 태그 내의 속성으로 전달됩니다. 다음은 예입니다:

```xml
<launch>
  <arg name="world" default="" />
  <arg name="file" default="" />
  <arg name="model_string" default="" />
  <arg name="topic" default="" />
  <arg name="entity_name" default="" />
  <arg name="allow_renaming" default="False" />
  <arg name="x" default="" />
  <arg name="y" default="" />
  <arg name="z" default="" />
  <arg name="roll" default="" />
  <arg name="pitch" default="" />
  <arg name="yaw" default="" />
  <gz_spawn_model
    world="$(var world)"
    file="$(var file)"
    model_string="$(var model_string)"
    topic="$(var topic)"
    entity_name="$(var entity_name)"
    allow_renaming="$(var allow_renaming)"
    x="$(var x)"
    y="$(var y)"
    z="$(var z)"
    roll="$(var roll)"
    pitch="$(var pitch)"
    yaw="$(var yaw)">
  </gz_spawn_model>
</launch>
```

이 경우 `<gz_spawn_model>` 매개변수는 명령줄에서 읽습니다.
이는 옵션이지만 일부 값을 하드코딩하거나 모든 매개변수를 사용하지 않기로 결정할 수 있으므로 반드시 필요한 것은 아닙니다.


## Spawning a model alongside launching ros_gz_bridge

XML용 예제 launch 파일은 [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/ros_gz_spawn_model.launch)에서 볼 수 있습니다.
Python용 예제 launch 파일은 [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/ros_gz_spawn_model.launch.py)에서 볼 수 있습니다.

터미널에서 이러한 launch 파일을 직접 사용하는 예제 명령:
```bash
ros2 launch ros_gz_sim ros_gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5 bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file>
```

`ros_gz_bridge`에 대한 자세한 정보는 [here](ros2_integration)에서 볼 수 있습니다.