# ROS 2 integration overview

Gazebo는 ROS 2 시스템 내에 통합될 수 있습니다. Gazebo와 ROS 간에 달성할 수 있는 다양한 통합 유형에 대해 설명해 보겠습니다.

* ROS를 사용하여 Gazebo 실행하기: ROS는 시스템에 필요한 모든 구성 요소를 시작하는 특정 방법을 규정합니다. 모든 구성 요소를 조율하고 주변의 많은 도구를 관리하기 위한 전용 [launch mechanism](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)이 있습니다. Gazebo는 이 방식으로 실행될 수 있습니다.

* [`ros_gz` bridge](https://github.com/gazebosim/ros_gz)를 통해 ROS를 사용하여 Gazebo 토픽과 상호 작용하기: Gazebo가 실행되면 시뮬레이션과 통신하는 것이 매우 일반적입니다. 이 통신을 수행하는 일반적인 방법은 토픽을 이용하는 것입니다. Gazebo는 ROS와 매우 유사한 토픽 및 서비스 세트를 노출하는 자체 미들웨어인 Gazebo Transport를 가지고 있습니다. `ros_gz` 브리지를 사용하면 필요에 따라 Gazebo Transport와 ROS 2 간에 변환하는 Gazebo와 ROS 시스템 간의 브리지를 만들 수 있습니다.

* ROS를 사용하여 Gazebo 모델 스폰하기: Gazebo 월드는 시작 시 로드되는 모델을 포함할 수 있습니다. 그러나 때로는 런타임에 모델을 스폰해야 합니다. 이 작업은 ROS 2를 사용하여 수행할 수 있습니다.

## Requirements

이 튜토리얼을 시작하기 전에 [Install Gazebo and ROS document](ros_installation)를 따르십시오. 더 진행하려면 ROS 2와 Gazebo의 작동 가능한 설치가 필요합니다.

중요: 이 기능의 대부분은 ROS 2 Rolling에서만 사용할 수 있습니다. 곧 ROS 2 Jazzy로 백포트할 예정입니다.

## Composition

다음 튜토리얼에 언급된 launch 파일의 매개변수를 검사하면 대부분의 경우 `use_composition`과 `create_own_container`라는 두 개의 매개변수를 포함했음을 알 수 있습니다. `use_composition` 매개변수가 `True`로 설정되면 연결된 ROS 노드가 ROS 컨테이너 내에 로드됩니다. 이런 일이 발생하면 동일한 ROS 컨테이너 내의 모든 노드는 동일한 프로세스를 공유하고 프로세스 간 통신을 활용할 수 있습니다.

`create_own_container` 매개변수는 `use_composition`이 `True`로 설정된 경우에만 의미가 있습니다. 이 매개변수를 사용하면 구성 가능한 노드에 대한 ROS 컨테이너를 시작할지 아니면 외부 ROS 컨테이너에 위임할지를 제어할 수 있습니다.

항상 `use_composition` 매개변수를 `True`로 설정하고 구성에 따라 자체 컨테이너를 생성해야 하는지 결정하는 것이 좋습니다. 일반적으로 자신의 launch 파일만 다루는 경우 `create_own_container`를 `True`로 설정할 것입니다. 반면에 ROS 컨테이너가 이미 존재하는 더 복잡한 시작의 일부로 launch 파일을 사용하는 경우 `create_own_container`를 `False`로 설정하고 대신 `container_name` 매개변수를 기존 컨테이너 이름으로 설정해야 합니다.

이러한 방식으로 Gazebo, 브리지 및 기타 잠재적 ROS 노드 간의 통신은 프로세스 간 통신이 됩니다.

![composition_options](images/composition_options.png)

이 그림은 컴포지션 개념을 설명합니다. 왼쪽 다이어그램은 컴포지션을 사용하지 않는 아이디어를 나타냅니다. 세 가지 예제 노드는 모두 독립 실행형 노드이며 브리지를 사용하여 프로세스 간 통신을 통해 통신할 수 있습니다.
가운데 다이어그램은 Gazebo와 브리지를 모두 포함하는 `ros_gz` launch 파일에 의해 생성된 ROS 컨테이너와 함께 컴포지션을 사용하고 제어할 수 없는 추가 소비자 노드가 있는 시나리오를 나타냅니다. Gazebo와 브리지 간의 모든 통신은 프로세스 내 통신이며 외부 소비자 노드와 브리지 간에는 프로세스 간 통신입니다.
오른쪽 다이어그램은 모든 노드에서 컴포지션을 사용하지만 `ros_gz` launch 파일은 자체 컨테이너를 직접 시작하지 않습니다. 이 설정 자체는 외부 ROS 컨테이너를 시작할 때까지(수동으로 또는 별도의 launch 파일을 통해) 작동하지 않습니다. 이 다이어그램에서 외부 ROS 소비자 노드가 컨테이너를 시작합니다. 외부 ROS 2 소비자 노드의 예로 Nav2 로고를 사용하고 있습니다.

ROS 컴포지션에 대한 자세한 내용은 [이 튜토리얼](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html)에서 확인할 수 있습니다.

## What's next?

다음은 Gazebo/ROS 통합에 대해 자세히 알아볼 수 있는 후속 튜토리얼입니다.

* [How to launch Gazebo from ROS 2](ros2_launch_gazebo).
* [How to use ROS 2 to interact with Gazebo](ros2_integration).
* [Example of using ROS 2 to load a model and interact with it in Gazebo](ros2_interop).
* [How to spawn a Gazebo model from ROS 2](ros2_spawn_model).