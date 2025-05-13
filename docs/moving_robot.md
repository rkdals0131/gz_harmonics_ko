# Moving the robot

이 튜토리얼에서는 로봇을 움직이는 방법을 배웁니다. [Build your own robot](building_robot) 튜토리얼에서 만든 로봇을 사용할 것입니다. 로봇은 [here](https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/building_robot/building_robot.sdf)에서 다운로드할 수 있습니다. 이 튜토리얼의 완성된 월드는 [here](https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/moving_robot/moving_robot.sdf)에서 찾을 수 있습니다.

## What is a plugin

로봇을 움직이기 위해 `diff_drive` plugin을 사용할 것입니다. 하지만 그 전에 "플러그인이란 무엇인가?"라는 질문에 답해 봅시다. 플러그인은 공유 라이브러리로 컴파일되어 시뮬레이션에 삽입되는 코드 덩어리입니다. 플러그인을 사용하면 월드, 모델 등 시뮬레이션의 여러 측면을 제어할 수 있습니다.

### Diff_drive plugin

`diff_drive` plugin은 로봇, 특히 차동 구동이 가능한 로봇을 제어하는 데 도움이 됩니다. 로봇에 플러그인을 설정해 봅시다. `building_robot.sdf`를 열고 `vehicle_blue` model 태그 안에 다음 코드를 추가합니다.

```xml
<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

`<plugin>` 태그에는 두 가지 속성이 있습니다. `filename`은 라이브러리 파일 이름을 받고 `name`은 플러그인 이름을 받습니다.
`<left_joint>` 및 `<right_joint>` 태그에서는 왼쪽 및 오른쪽 바퀴를 로봇 본체에 연결하는 조인트를 정의합니다. 우리 경우에는 `left_wheel_joint` 및 `right_wheel_joint`입니다. `<wheel_separation>`은 두 바퀴 사이의 거리를 받습니다.
우리 로봇은 `chassis`를 기준으로 y축에서 `left_wheel`이 0.6m, `right_wheel`이 -0.6m에 있으므로 `wheel_separation`은 1.2m입니다.
`<wheel_radius>`는 바퀴 링크 아래 `<radius>` 태그에 정의된 바퀴의 반지름을 받습니다.
`<odom_publish_frequency>`는 `/model/vehicle_blue/odometry`에서 오도메트리가 게시되는 빈도를 설정합니다.
`cmd_vel`은 `DiffDrive` plugin의 입력 `<topic>`입니다.

## Topics and Messages

이제 우리 모델이 준비되었습니다. 모델에 명령(메시지)을 보내기만 하면 됩니다.
이 메시지들은 위에서 정의한 `cmd_vel` topic에 게시(전송)될 것입니다.

topic은 특정 메시지 집합이나 특정 서비스를 그룹화하기 위한 이름일 뿐입니다.
우리 모델은 `cmd_vel` topic으로 전송된 메시지를 구독(수신)할 것입니다.

로봇 월드를 실행합니다:

`gz sim building_robot.sdf`

다른 터미널에서 로봇에 메시지를 보냅니다:

`gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"`

이제 시뮬레이션에서 로봇이 움직이는 것을 볼 수 있습니다.

**Note:** 시뮬레이션에서 재생 버튼을 누르는 것을 잊지 마십시오.

명령은 `-t` 옵션 뒤에 게시할 topic을 지정합니다.
`-m` 뒤에는 메시지 유형을 지정합니다.
우리 로봇은 `linear`와 `angular` 두 가지 구성 요소로 이루어진 `Twist` 유형의 메시지를 예상합니다.
`-p` 옵션 뒤에는 메시지의 내용(값)을 지정합니다: 선형 속도 `x: 0.5` 및 각속도 `z: 0.05`.

**Hint:** 다음 명령을 사용하여 각 topic 옵션이 무엇을 하는지 알 수 있습니다: `gz topic -h`

Gazebo의 `Topics` 및 `Messages`에 대한 자세한 내용은 [Transport library tutorials](https://gazebosim.org/api/transport/13/tutorials.html)를 확인하십시오.

## Moving the robot using the keyboard

터미널에서 메시지를 보내는 대신 키보드 키를 사용하여 메시지를 보낼 것입니다. 이를 위해 `KeyPublisher`와 `TriggeredPublisher`라는 두 개의 새로운 플러그인을 추가할 것입니다.

### KeyPublisher

`KeyPublisher`는 키보드의 키 입력을 읽어 기본 topic인 `/keyboard/keypress`로 전송하는 `gz-gui` plugin입니다.
다음과 같이 이 플러그인을 사용해 봅시다:

* 한 터미널에 다음을 입력합니다.

    `gz sim building_robot.sdf`

* 오른쪽 상단 모서리에서 플러그인 드롭다운 목록(수직 줄임표)을 클릭하고 Key Publisher를 클릭합니다.

* 다른 터미널에 다음을 입력합니다.

    `gz topic -e -t /keyboard/keypress`

마지막 명령은 `/keyboard/keypress` topic으로 전송된 모든 메시지를 표시합니다.

Gazebo 창에서 다른 키를 누르면 `gz topic -e -t /keyboard/keypress` 명령을 실행한 터미널에 데이터(숫자)가 표시되어야 합니다.

```
$ gz topic -e -t /keyboard/keypress
data: 68

data: 85

data: 72

data: 74

data: 81

data: 16777235

data: 16777234

data: 16777237

data: 16777236
```

이러한 키 입력을 `Twist` 유형의 메시지로 매핑하여 우리 모델이 수신하는 `/cmd_vel` topic에 게시하려고 합니다.
`TriggeredPublisher` plugin이 이 작업을 수행할 것입니다.

### Triggered Publisher

`TriggeredPublisher` plugin은 사용자가 지정한 기준과 일치하는 입력 메시지에 응답하여 사용자가 지정한 메시지를 출력 topic에 게시합니다.
`<world>` 태그 아래에 다음 코드를 추가합시다:

```xml
<!-- Moving Forward-->
<plugin filename="gz-sim-triggered-publisher-system"
        name="gz::sim::systems::TriggeredPublisher">
    <input type="gz.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777235</match>
    </input>
    <output type="gz.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

이 코드는 `triggered-publisher` plugin을 정의합니다.
`/keyboard/keypress` topic에서 `gz.msgs.Int32` 유형의 메시지를 수신하고 `data` 필드의 값이 `16777235`(위쪽 화살표 키)와 일치하면 `cmd_vel` topic에 `x: 0.5`, `z: 0.0` 값을 가진 `Twist` 메시지를 출력합니다.

이제 `building_robot.sdf`를 실행한 다음 Key Publisher plugin을 추가하면 위쪽 화살표 키 &#8593;를 누를 때 로봇이 앞으로 움직여야 합니다 (위쪽 화살표 키를 누른 후 로봇이 앞으로 움직이는 것을 보려면 재생 버튼을 눌러 시뮬레이션을 시작해야 합니다).

[Triggered Publisher](https://github.com/gazebosim/gz-sim/blob/gz-sim7/tutorials/triggered_publisher.md)가 어떻게 작동하는지 설명하는 데모가 있습니다.

### Moving using arrow keys

화살표를 누를 때 `/keyboard/keypress` topic으로 어떤 값이 전송되는지 확인하려면 `--echo` 또는 `-e` 옵션을 사용할 수 있습니다.

* 한 터미널에서 모델을 실행합니다:

    `gz sim building_robot.sdf`

* 오른쪽 상단 모서리에서 플러그인 드롭다운 목록(수직 줄임표)을 클릭하고 Key Publisher를 클릭합니다.

* 다른 터미널에서 다음 명령을 실행합니다:

    `gz topic -e -t /keyboard/keypress`

화살표 키를 누르기 시작하고 어떤 값을 제공하는지 확인하십시오:

* Left &#8592;  : 16777234
* Up  &#8593;   : 16777235
* Right &#8594; : 16777236
* Down &#8595;  : 16777237

각 화살표 키에 대해 `Triggered publisher` plugin을 추가할 것입니다.
예를 들어, 아래쪽 화살표:

```xml
<!-- Moving Backward-->
<plugin filename="gz-sim-triggered-publisher-system"
        name="gz::sim::systems::TriggeredPublisher">
    <input type="gz.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777237</match>
    </input>
    <output type="gz.msgs.Twist" topic="/cmd_vel">
        linear: {x: -0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

뒤쪽 화살표에서 했던 것처럼 각 화살표(키 입력)를 원하는 메시지(움직임)와 매핑하십시오:

* Left &#10142; 16777234 &#10142; linear: {x: 0.0}, angular: {z: 0.5}
* Up &#10142; 16777235 &#10142; linear: {x: 0.5}, angular: {z: 0.0}
* Right &#10142; 16777236 &#10142; linear: {x: 0.0}, angular: {z: -0.5}
* Down &#10142; 16777237 &#10142; linear: {x: -0.5}, angular: {z: 0.0}

이제 여러분 차례입니다. 다른 키를 사용하여 로봇을 움직여 보십시오.

[next tutorial](sdf_worlds)에서는 SDF를 사용하여 자신만의 시뮬레이션 월드를 만드는 방법을 배웁니다.

## Video walk-through

이 튜토리얼의 비디오 연습은 YouTube 채널에서 볼 수 있습니다: [Gazebo tutorials: Moving robot](https://youtu.be/oHtQYPDGk3Y).

<iframe width="560" height="315" src="https://www.youtube.com/embed/oHtQYPDGk3Y" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>