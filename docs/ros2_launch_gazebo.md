# Launch Gazebo from ROS 2

Gazebo는 ROS 2 실행 시스템에서 여러 가지 방법으로 실행할 수 있습니다:

## Using the launch files included in
[ros_gz_sim](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim).

`ros_gz_sim` 패키지에는 `gz_server.launch.py`와 `gz_sim.launch.py`라는 두 개의 실행 파일이 포함되어 있습니다. 이를 사용하여 각각 Gazebo 서버 또는 Gazebo (서버 및 GUI)를 시작할 수 있습니다.

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

또는 서버만 시작할 수도 있습니다:

```bash
ros2 launch ros_gz_sim gz_server.launch.py world_sdf_file:=empty.sdf
```

각 실행 파일이 허용하는 다양한 매개변수에 대해 알아보려면 각 실행 파일의 인수 블록을 [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/gz_sim.launch.py.in#L75-L96) 및 [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/gz_server.launch.py#L27-L38)에서 참조하십시오.

## Using a custom launch file.

### XML
사용자 정의 실행 파일에서 Gazebo를 시작하는 것도 가능합니다. 이를 위해 XML 실행 파일에서 사용할 수 있는 사용자 정의 `<gz_server/>` 태그를 만들었습니다. 이 경우 인수는 이 태그 내에서 속성으로 전달됩니다. 다음은 Gazebo 서버를 시작하는 예입니다:

```xml
<launch>
  <arg name="world_sdf_file" default="empty.sdf" />
  <arg name="world_sdf_string" default="" />
  <arg name="container_name" default="ros_gz_container" />
  <arg name="create_own_container" default="False" />
  <arg name="use_composition" default="False" />
  <gz_server
    world_sdf_file="$(var world_sdf_file)"
    world_sdf_string="$(var world_sdf_string)"
    container_name="$(var container_name)"
    create_own_container="$(var create_own_container)"
    use_composition="$(var use_composition)">
  </gz_server>
</launch>
```

이 경우 `<gz_server>` 매개변수는 명령줄에서 읽어옵니다. 이는 옵션이지만 일부 값을 하드코딩하거나 모든 매개변수를 사용하지 않기로 결정할 수 있으므로 반드시 필요한 것은 아닙니다.

### Python
Python 실행 파일은 XML 실행 파일에 비해 더 낮은 수준의 사용자 정의 및 논리를 제공합니다. 예를 들어 환경 변수를 설정하고 Python 관련 함수 및 논리를 포함할 수 있습니다.
다음 예에서 사용자는 예제 패키지, 월드 및 브리지된 토픽을 자신만의 것으로 바꿀 수 있습니다. 이것은 자체적으로 실행할 수 있는 것보다 스캐폴딩으로 의도되었습니다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    example_pkg_path = FindPackageShare('example_package')  # Replace with your own package name
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([example_pkg_path, 'models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([example_pkg_path, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([example_pkg_path, 'worlds/example_world.sdf'])],  # Replace with your own world file
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU',],
            remappings=[('/example_imu_topic',
                         '/remapped_imu_topic'),],
            output='screen'
        ),
    ])
```

다음은 Python에서 `gzserver`를 시작하기 위해 더 높은 수준의 작업을 사용하는 또 다른 예입니다:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros_gz_sim.actions import GzServer


def generate_launch_description():

    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file', default_value='',
        description='Path to the SDF world file')

    # Create the launch description and populate
    ld = LaunchDescription([
        GzServer(
            world_sdf_file=LaunchConfiguration('world_sdf_file')
        ),
    ])

    # Declare the launch options
    ld.add_action(declare_world_sdf_file_cmd)

    return ld
```


## Launching with ros_gz_bridge

XML용 예제 실행 파일은 [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/ros_gz_sim.launch)에서 볼 수 있습니다.
Python용 예제 실행 파일은 [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/ros_gz_sim.launch.py)에서 볼 수 있습니다.

터미널에서 이러한 실행 파일을 직접 사용하는 예제 명령:
```bash
ros2 launch ros_gz_sim ros_gz_sim.launch.py world_sdf_file:=empty.sdf bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file> use_composition:=True create_own_container:=True
```

위의 실행 파일에서 `ros_gz_bridge`의 `create_own_container` 인수가 `False`로 하드코딩되어 있음을 알 수 있습니다. 이는 두 개의 중복 컨테이너(`gz_server`용 하나와 `ros_gz_bridge`용 다른 하나)가 생성되는 것을 방지하고 대신 `ros_gz_bridge`가 `gz_server`에서 생성한 컨테이너를 사용하도록 하기 위한 것입니다. 이에 대한 자세한 정보는 [here](https://github.com/gazebosim/ros_gz/pull/620#issue-2595570189)에서 볼 수 있습니다.

`ros_gz_bridge`에 대한 자세한 정보는 [here](ros2_integration)에서 볼 수 있습니다.
컴포지션에 대한 자세한 정보는 [here](ros2_overview.md#composition)에서 볼 수 있습니다.