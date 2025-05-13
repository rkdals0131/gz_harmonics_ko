# Feature comparison

[Gazebo-classic](https://github.com/osrf/gazebo/) 버전 11에 있는 기능 목록과
[Gazebo Harmonic](https://gazebosim.org/)으로의 마이그레이션 상태입니다.

아래의 모든 이슈는 GitHub에서
[close the gap](https://github.com/search?q=org%3Agazebosim+label%3A%22close+the+gap%22&type=Issues)
라벨이 붙어 있습니다.

## Sensors

Sensor | Gazebo-classic | Gazebo Sim
-- | -- | --
Air pressure | ✕  | ✓
Altimeter | ✓ | ✓
Bounding Box camera | ✕ | ✓
Camera | ✓ | ✓
Contact sensor | ✓ | ✓
Depth camera | ✓ | ✓
Doppler Velocity Log (DVL) | ✕ | ✓
Force-torque | ✓ | ✓
GPS / NavSat | ✓ |  ✓
GPU Ray | ✓ | ✓ GPU Lidar로 이름 변경됨
IMU | ✓ | ✓
Logical audio sensor | ✕ | ✓
Logical camera | ✓ | ✓
Magnetometer | ✓ | ✓
Multi-camera | ✓ | ✕ 동일한 업데이트 속도를 가진 개별 카메라 사용
Optical tactile sensor | ✕ | ✓
Ray | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/26)
RFID sensor and tag | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/27)
RGBD camera | ✕ | ✓
Segmentation camera | ✕ | ✓
Sonar | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/19)
Thermal camera | ✕  | ✓
Triggered camera | ✕ | ✓
Wide-angle camera | ✓ | ✓
Wireless | ✓ | ✓ RFComms

Sensor features | Gazebo-classic | Gazebo Sim
-- | -- | --
Custom update rate | ✓ | ✓
Gaussian noise | ✓ | ✓
Custom sensors | ✓ |  ✓
Laser retroreflection | ✓ | ✓
Camera distortion | ✓ | ✓
Performance metrics | ✓ |  ✓

## SDF Features

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
SDF frame semantics |✓| ✓
SDF parametrization | ✕ | [✓](http://sdformat.org/tutorials?tut=param_passing_proposal)
Load models from local files | ✓ | [✓](https://gazebosim.org/api/gazebo/6.6/resources.html)
Closed kinematic chains | ✓  | [Issue](https://github.com/gazebosim/gz-physics/issues/25)
Nested models | ✓ | ✓
Populations | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/240)
Actors | ✓ | ✓
Markers | ✓ | ✓
Heightmaps | ✓ | ✓
DEM (Digital Elevation Models) | ✓ | ✓
Polylines | ✓ | ✓
World plugins | ✓ | ✓ 현재 System plugin이라고 불림
Model plugins | ✓ | ✓ 현재 System plugin이라고 불림
Sensor plugins | ✓ | ✓ 현재 System plugin이라고 불림
Visual plugins | ✓ | ✓
GUI plugins | ✓ | ✓ Gazebo GUI plugins 및 Gazebo GUI systems
System plugins | ✓ | ✓ Gazebo Launch를 통해
SDF python bindings | x | ✓ | sdformat13에 있음
SDF <-> Mujoco MJCF | x | ✓ | sdformat13에 있음, [documentation](https://github.com/gazebosim/gz-mujoco/blob/main/sdformat_mjcf/README.md)

## Plugins

### Model plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
AckermannSteering | ✕ | ✓
ActorPlugin | ✓ | ✕ Actor API 데모는 [FollowActor](https://github.com/gazebosim/gz-sim/blob/main/src/systems/follow_actor/FollowActor.hh) 참고
ActuatorPlugin | ✓ |
ArduCopterPlugin | ✓ |
AttachLightPlugin | ✓ | ✕ 적용되지 않음, SDF 사용
Breadcrumbs | ✕ | ✓
BuoyancyPlugin | ✓ | [✓](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/buoyancy.sdf)
CartDemoPlugin | ✓ | ✕
CessnaPlugin | ✓ | ✕
DetachableJoint | ✕ | ✓
DiffDrivePlugin | ✓ | ✓
ElevatorPlugin | ✓ | ✓
FlashLightPlugin | ✓ |
FollowerPlugin | ✓ |
GimbalSmall2dPlugin | ✓ |
GravityCompensationPlugin | ✓ |
HarnessPlugin | ✓ |
HydraDemoPlugin | ✓ |
InitialVelocityPlugin | ✓ | ✓ (VelocityControl 또는 JointController 사용)
JointControlPlugin | ✓ (SDF로부터 force / pos / vel) | ✓ (msg로부터 vel)
JointStatePublisher | ✕ | ✓
JointTrajectoryPlugin | ✓ | ✓
KeysToCmdVelPlugin | ✓ | `gz::gui::KeyPublisher`를 `gz::gazebo::systems::TriggeredPublisher`와 함께 사용
KeysToJointsPlugin | ✓ | `gz::gui::KeyPublisher`를 `gz::gazebo::systems::TriggeredPublisher`와 함께 사용
LedPlugin | ✓ |
LiftDragPlugin | ✓ | ✓
LinearBatteryConsumerPlugin | ✓ | ✓
LinearBatteryPlugin | ✓ | ✓
LinkPlot3DPlugin | ✓ | ✓ (Plot3D로 이름 변경됨)
MecanumDrive | ✕ | ✓
MudPlugin | ✓ |
MulticopterMotorModel | ✕ | ✓
OdometryPublisherPlugin | ✕ | ✓
PlaneDemoPlugin | ✓ |
PosePublisher | ✕ | ✓
RandomVelocityPlugin | ✓ |
RegionEventBoxPlugin | ✓ |
SimpleTrackedVehiclePlugin | ✓ | ✓
SkidSteerDrivePlugin | ✓ | ✓
SphereAtlasDemoPlugin | ✓ | ✕
TouchPlugin | ✓ | ✓
TrackedVehiclePlugin | ✓ | ✓
VariableGearboxPlugin | ✓ |
VehiclePlugin | ✓ |
WheelSlipPlugin | ✓ | ✓
WheelTrackedVehiclePlugin | ✓ | ✓
KineticEnergyMonitor | ✕ | ✓
Buoyancy engine | ✕ | ✓

### World plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
ArrangePlugin | ✓ |
ContainPlugin | ✓ | 부분적으로 포팅됨, [Issue](https://github.com/gazebosim/gz-sim/issues/162)
HydraPlugin | ✓ |
JoyPlugin | ✓ | ✓ Gazebo Launch plugin으로 마이그레이션됨
MisalignmentPlugin | ✓ |
RubblePlugin | ✓ |
StaticMapPlugin | ✓ |
TransporterPlugin | ✓ |
WindPlugin | ✓ | ✓

### Sensor plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
BreakableJointPlugin | ✓ |
CameraPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
ContactPlugin | ✓ | ✓
DepthCameraPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
FiducialCameraPlugin | ✓ |
ForceTorquePlugin | ✓ | ✓
GpuRayPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
ImuSensorPlugin | ✓ | ✓
LensFlareSensorPlugin | ✓ |
MagnetometerPlugin | ✕ | ✓
OpticalTactilePlugin | ✕ | ✓
PressurePlugin | ✓ |
RayPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
RaySensorNoisePlugin | ✓ | ✕ SDF 사용
SonarPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)

### Visual plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
AmbientOcclusionVisualPlugin | ✓ |
BlinkVisualPlugin | ✓ |
HeightmapLODPlugin | ✓ |
ShaderParamVisualPlugin | ✓ | ✓

### GUI plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
CessnaGUIPlugin | ✓ |
KeyboardGUIPlugin | ✓ | `gz::gui::KeyPublisher`
LookAtDemoPlugin | ✓ |
TimerGUIPlugin | ✓ |

### System plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
ColladaWorldExporter | ✕ | ✓
ModelPropShop | ✓ | [✓](https://gazebosim.org/api/gazebo/5.4/model_photo_shoot.html)
RestUiPlugin | ✓ |
RestWebPlugin | ✓ |
StopWorldPlugin | ✓ |

## GUI

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Play / pause / step | ✓ | ✓
Reset world / models | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/203)
World stats | ✓ | ✓
Topic echo | ✓ | ✓
Image viewer | ✓ | ✓
Translate / rotate | ✓ | ✓
Scale models | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/195)
Insert models from Fuel | 부분 지원 | ✓
Insert models from disk | ✓ | ✓
Insert simple shapes | ✓ | ✓
Insert simple lights | ✓ | ✓
Delete models | ✓ | ✓
World tree | ✓ | ✓
Scene properties | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/246)
Log recording / playback | ✓ | ✓
Plotting | ✓ | ✓
Video recording | ✓ | ✓
Screenshot | ✓ | [✓](https://gazebosim.org/api/gui/3.5/screenshot.html)
View angles | ✓ | ✓
Apply force / torque | ✓ |
Visualize as transparent | ✓ | ✓
Visualize as wireframe | ✓ | ✓
Visualize joints | ✓ |  ✓
Visualize collisions | ✓ | ✓
Visualize inertia | ✓ | ✓
Visualize CoM | ✓ |  ✓
Visualize contacts | ✓ | ✓
Visualize lights | ✓ | ✓
Follow / move to | ✓ | ✓
Copy / paste | ✓ | ✓
Building editor | ✓ |
Model editor | ✓ | [Issues](https://github.com/gazebosim/gz-sim/issues?q=is%3Aissue+is%3Aopen+label%3Aeditor)
FPS view control | ✓ |
Orthographic projection | ✓ | ✓
Undo / redo | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/104)
Save world | ✓ | ✓
Save GUI configuration | ✓ | ✓
Color scheme and themes | ✕ | ✓
Position, resize and configure widgets | ✕ | ✓
Load GUI plugins from menu | ✕ | ✓
Edit model pose | ✓ | ✓
Edit light properties | ✓ | ✓
Edit physics properties | ✓ | ✓

## Physics

Gazebo Physics에서는 물리 엔진이 플러그인으로 통합되므로, Gazebo에서와 같이 핵심 소스 코드를 변경하지 않고도 모든 엔진을 통합할 수 있습니다.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
ODE engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/63)
Bullet engine | ✓ | ✓
DART engine | ✓ | ✓ gz-physics와 함께 제공되는 플러그인
Simbody engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/63)
TPE engine | ✕ | ✓
Custom engine plugins | ✕ | ✓
Collide bitmasks | ✓ | ✓
Restitution coefficient | ✓ | ✓
Collision detector | ✓ |  ✓
Solver | ✓ |  ✓

## Rendering

Gazebo Rendering에서는 렌더 엔진이 플러그인으로 통합되므로 핵심 소스 코드를 변경하지 않고도 모든 엔진을 통합할 수 있습니다.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Ogre 1.x engine | ✓ | ✓
Ogre 2.x engine | ✕ | ✓
Optix engine | ✕ | ✓ 부분 지원
Custom engine plugins | ✕ | [✓](https://gazebosim.org/api/rendering/5.0/renderingplugin.html)
Sky | ✓ | ✓
Fog | ✓ |
Material scripts | ✓ (Ogre 1.x 스크립트) | 적용되지 않음
Physically Based Rendering (PBR) | ✕ | ✓ (Ogre 2와 같이 이를 지원하는 엔진 포함)
Normal maps | ✓ | ✓
Environment maps | ✕  | ✓
Lightmaps | ✕  | ✓
Particle effects | ✕  | ✓
Render order | ✕  | ✓
Projector | ✕  | ([ogre 1.x 전용](https://github.com/gazebosim/gz-sim/pull/1979))

## ROS integration

[ros_gz](https://github.com/gazebosim/ros_gz) 패키지를 통한 ROS 통합.

지원 버전:

* ROS 2 Jazzy (바이너리에서) / Rolling (소스에서)

**ROS 2 Rolling**의 경우, Rolling 배포판은 [REP-2000](https://www.ros.org/reps/rep-2000.html)에 정의된 ROS 2의 다음 미래 릴리스와 함께 이동합니다.

## Platforms

Platform | Gazebo-classic | Gazebo Sim
-- | -- | --
Ubuntu | ✓ | ✓
OSX | ✓ | 대부분의 스택이 작동하며, 해결되지 않은 문제: [render window](https://github.com/gazebosim/gz-sim/issues/44)
Windows | ✓ | 모든 라이브러리가 컴파일되며, 하위 수준 라이브러리가 잘 작동함: [Issue](https://github.com/gazebosim/gz-sim/issues/168)

## Others

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Nested models | ✓ | ✓
Log / playback | ✓ | ✓
Web client (GzWeb) | ✓ |
COLLADA meshes | ✓ | ✓
OBJ meshes | ✓ | ✓
STL meshes | ✓ | ✓
USD meshes | ✕ | [✓](https://github.com/gazebosim/sdformat/tree/sdf12/examples/usdConverter)
Code introspection | ✓ | 모든 시뮬레이션 상태는 시스템 플러그인 또는 `SceneBroadcaster`의 상태 토픽을 통해 접근 가능합니다
Distribute simulation across processes | ✕ | (출시 예정)
Incrementally load levels | ✕ | ✓
Online model database | [gazebo_models repository](https://github.com/osrf/gazebo_models/) | [Gazebo Fuel](https://app.gazebosim.org/fuel/models)
Saved simulation states | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/137)
Sphere, cylinder and box primitives | ✓ | ✓
Ellipsoid and capsule primitives | ✕ | ✓
Hydrodynamics | ✕  | ✓
Ocean currents | ✕  | ✓
Test fixture | ✓ | [✓](https://gazebosim.org/api/gazebo/6.6/test_fixture.html)
Spherical coordinates | ✓ | ✓
Generic comms system | ✕ | [✓](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/perfect_comms.sdf)
Acoustic communication | ✕ | ✓
Static linked plugins | ✕ | ✓