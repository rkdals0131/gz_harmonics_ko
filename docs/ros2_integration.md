# Use ROS 2 to interact with Gazebo

이 튜토리얼에서는 ROS 2를 사용하여 Gazebo와 통신하는 방법을 배웁니다.
이는 여러 측면에서 도움이 될 수 있습니다. ROS로부터 데이터(예: 조인트 상태, TF)를 수신하거나 명령을 받아 Gazebo에 적용할 수 있으며, 그 반대도 가능합니다. 또한 RViz가 Gazebo 월드에서 동시에 시뮬레이션되는 로봇 모델을 시각화하는 데 도움이 될 수 있습니다.

## ros_gz_bridge

[`ros_gz_bridge`](https://github.com/gazebosim/ros_gz)는 ROS 2와 [Gazebo Transport](https://github.com/gazebosim/gz-transport) 간의 메시지 교환을 가능하게 하는 네트워크 브리지를 제공합니다. 지원은 특정 메시지 유형으로 제한됩니다. 사용하려는 메시지 유형이 브리지에서 지원되는지 확인하려면 이 [README](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md)를 확인하십시오.

브리지 사용 예제는 [`ros_gz_sim_demos`](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos)에서 찾을 수 있으며, 여기에는 모든 주요 액추에이션 및 센서 유형의 브리징을 포함한 데모 실행 파일이 포함되어 있습니다.

## Launching the bridge manually

양방향 브리지를 초기화하여 ROS를 게시자로, Gazebo를 구독자로 사용하거나 그 반대로 사용할 수 있습니다. 구문은 `/TOPIC@ROS_MSG@GZ_MSG`이며, 여기서 `TOPIC`은 Gazebo 내부 토픽, `ROS_MSG`는 이 토픽에 대한 ROS 메시지 유형, `GZ_MSG`는 Gazebo 메시지 유형입니다.

예를 들어:

```
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

`ros2 run ros_gz_bridge parameter_bridge` 명령어는 `ros_gz_bridge` 패키지의 `parameter_bridge` 코드를 실행합니다. 그런 다음 메시지가 전송될 토픽 `/TOPIC`을 지정합니다. 첫 번째 `@` 기호는 토픽 이름을 메시지 유형과 구분합니다. 첫 번째 `@` 기호 다음에는 ROS 메시지 유형이 옵니다.

ROS 메시지 유형 다음에는 `@`, `[`, 또는 `]` 기호가 오며, 여기서:

* `@`는 양방향 브리지입니다.
* `[`는 Gazebo에서 ROS로의 브리지입니다.
* `]`는 ROS에서 Gazebo로의 브리지입니다.

ROS에서 Gazebo로, 또는 그 반대로 통신 연결을 만드는 방법을 설명하는 다음 [예제들](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md#example-1a-gazebo-transport-talker-and-ros-2-listener)을 살펴보십시오.

또한 ROS Launch를 `ros_gz_bridge`와 함께 사용하고, 토픽을 yaml 형식으로 표현하여 실행 시 브리지에 제공할 수도 있습니다.

```yaml
- ros_topic_name: "scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS  # BIDIRECTIONAL or ROS_TO_GZ
```

설정 파일은 브리지될 ROS와 Gazebo 토픽 간의 매핑을 포함하는 YAML 파일입니다. 브리지될 각 토픽 쌍에 대해 다음 매개변수가 허용됩니다:

* `ros_topic_name`: ROS 측의 토픽 이름.
* `gz_topic_name`: Gazebo 측의 해당 토픽 이름.
* `ros_type_name`: 이 ROS 토픽의 유형.
* `gz_type_name`: 이 Gazebo 토픽의 유형.
* `subscriber_queue`: ROS 구독자 큐 크기.
* `publisher_queue`: ROS 게시자 큐 크기.
* `lazy`: 지연 구독자가 있는지 여부. 실제 구독자가 없으면 브리지는 내부 구독자도 생성하지 않습니다. 이는 성능을 향상시킬 것입니다.
* `direction`: `GZ_TO_ROS`, `ROS_TO_GZ` 및 `BIDIRECTIONAL`을 지정할 수 있습니다.

유효한 설정 파일의 예는 [이 예제](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/test/config/full.yaml)를 참조하십시오.

## Launching the bridge using the launch files included in `ros_gz_bridge` package.

`ros_gz_bridge` 패키지에는 `ros_gz_bridge.launch.py`라는 이름의 launch 파일이 포함되어 있습니다. 이를 사용하여 ROS 2와 Gazebo 브리지를 시작할 수 있습니다.

다음은 예제입니다:
```bash
ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file>
```

컴포지션으로 실행하기:
```bash
ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file> use_composition:=True create_own_container:=True
```
또는 기존 컨테이너가 이미 실행 중인 경우, `container_name` 매개변수를 사용하여 브리지를 시작할 때 해당 이름을 전달할 수 있습니다. 컴포지션에 대한 자세한 정보는 [여기](ros2_overview.md#composition)에서 볼 수 있습니다.

이 launch 파일에서 허용하는 모든 다른 매개변수를 알아보려면 소스 코드의 [이 블록](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/launch/ros_gz_bridge.launch.py#L25-L68)을 확인하십시오.

QOS 재정의:

QOS 재정의는 launch 파일의 `bridge_params` 인수를 사용하여 브리지에 추가 매개변수로 전달할 수 있습니다. 다음은 예시입니다:
```bash
ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file> bridge_params:={'qos_overrides./topic_name.publisher.durability': 'transient_local', 'qos_overrides./another_topic_name.publisher.durability': 'transient_local'}
```
`bridge_params`에 전달하는 내용에서 다음을 생략할 수 있습니다: `{}`, ` `, `"`, `'`.

QOS 재정의 사용에 대한 자세한 정보는 [여기](https://docs.ros.org/en/jazzy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html#using-qos-overrides)에서 찾을 수 있습니다.

## Launching the bridge from a custom launch file in XML.

사용자 지정 launch 파일에서 브리지를 트리거할 수도 있습니다. 이를 위해 XML 또는 YAML launch 파일에서 사용할 수 있는 `<ros_gz_bridge/>` 태그를 만들었습니다. 이 경우 인수는 이 태그 내의 속성으로 전달됩니다. 다음은 단순화된 예이며, 더 포괄적인 예는 [여기](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/launch/ros_gz_bridge.launch)에서 볼 수 있습니다:

```xml
<launch>
  <arg name="bridge_name" />
  <arg name="config_file" />
  <ros_gz_bridge
    bridge_name="$(var bridge_name)"
    config_file="$(var config_file)">
  </ros_gz_bridge>
</launch>
```

이 경우 `<ros_gz_bridge>` 매개변수는 명령줄에서 읽습니다. 이는 옵션이지만, 일부 값을 하드코딩하거나 모든 매개변수를 사용하지 않기로 결정할 수 있으므로 반드시 필요한 것은 아닙니다.

## Launching the bridge from a custom launch file in Python.

다음은 Python에서 브리지를 로드하는 데 사용되는 Python launch 파일의 단순화된 예입니다. 더 포괄적인 예는 [여기](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/launch/ros_gz_bridge.launch.py)에서 볼 수 있습니다:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():

    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', description='Name of ros_gz_bridge node'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', description='YAML config file'
    )

    # Create the launch description and populate
    ld = LaunchDescription([
        RosGzBridge(
            bridge_name=LaunchConfiguration('bridge_name'),
            config_file=LaunchConfiguration('config_file'),
        ),
    ])

    # Declare the launch options
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)

    return ld
```

## Publish key strokes to ROS

`Key Publisher` Gazebo 플러그인을 사용하여 ROS로 메시지를 보내 봅시다.

**참고:** 필요한 모든 작업 공간(ROS, Gazebo 및 `ros_gz`...)이 소싱되었는지 확인하십시오.

먼저 `Key Publisher` 플러그인이 메시지를 보내는 토픽과 메시지 유형을 지정하여 ROS와 Gazebo 간의 브리지를 시작합니다:

```
ros2 run ros_gz_bridge parameter_bridge /keyboard/keypress@std_msgs/msg/Int32@gz.msgs.Int32
```

`/keyboard/keypress` 토픽에 `Int32` 유형의 메시지를 사용하는 브리지를 시작했습니다.
ROS의 경우 `std_msgs/msg/Int32`이고 Gazebo의 경우 `gz.msgs.Int32`입니다.

다른 터미널에서 Gazebo Sim 월드를 실행합니다. 예를 들어 `empty.sdf` 월드입니다:

```
gz sim empty.sdf
```

그런 다음 오른쪽 상단 드롭다운 메뉴에서 `Key Publisher` 플러그인을 추가합니다.

![empty_world_with_KeyPublisher](tutorials/ros2_integration/empty_world.png)

다른 터미널에서 ROS 리스너를 시작합니다:

```
ros2 topic echo /keyboard/keypress
```

이 명령은 `/keyboard/keypress` 토픽을 통해 전송된 메시지를 수신합니다.

Gazebo 창에서 키보드 키를 누르면 리스너 터미널에 데이터가 표시됩니다.

이제 여러분의 차례입니다! ROS에서 Gazebo로 데이터를 전송해 보세요. 다른 데이터 유형과 다른 통신 방향을 시도해 볼 수도 있습니다.

## Video walk-through

이 튜토리얼의 비디오 연습은 YouTube 채널에서 볼 수 있습니다: [Gazebo tutorials: ROS 2 Foxy integration](https://youtu.be/IpZTNyTp9t8).

<iframe width="560" height="315" src="https://www.youtube.com/embed/IpZTNyTp9t8" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Visualize in RViz

한 단계 더 나아가 [`ros_gz_sim_demos`](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos)의 데모를 시도해 보세요.

`sdf_parser` 데모의 경우, colcon 작업 공간에 [`ros_gz`](https://github.com/gazebosim/ros_gz/tree/jazzy)와 파서 플러그인 `sdformat_urdf`를 소스에서 설치하십시오.
`sdformat_urdf`에 대한 자세한 내용은 [여기](https://github.com/ros/sdformat_urdf/blob/jazzy/sdformat_urdf/README.md)에서 읽어보십시오.

rviz 실행 인수가 설정된 상태로 데모 launch 파일을 실행합니다:

```bash
ros2 launch ros_gz_sim_demos sdf_parser.launch.py rviz:=True
```

Gazebo에서 시뮬레이션을 시작하고 TF가 게시될 때까지 몇 초 기다립니다.

다른 터미널에서 차량이 원을 그리며 움직이도록 ROS 또는 Gazebo 명령을 보냅니다:

```bash
gz topic -t "/model/vehicle/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: -0.1}"
ros2 topic pub /model/vehicle/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}
```

Gazebo와 RViz에서 차량의 궤적이 일치하는지 확인합니다.

![gz_rviz](tutorials/ros2_integration/gz_rviz.gif)

이 데모 구현에 대한 자세한 내용은 [ROS 2 Interoperability](ros2_interop)를 참조하십시오.