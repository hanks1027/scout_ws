# Livox Mid-360 Gazebo Sim for ROS 2

Livox Mid-360의 비반복 스캔 패턴을 Gazebo Sim에서 재현하는 ROS 2 시스템
플러그인입니다. 특정 로봇에 종속되지 않으며, 사용자의 URDF/xacro에 LiDAR
링크와 플러그인 매크로를 추가해 사용할 수 있습니다.

시뮬레이터는 실제 Livox 드라이버와 동일한
`livox_ros_driver2/msg/CustomMsg`를 발행합니다. 따라서 실제 로봇과
시뮬레이션에서 동일한 SLAM·인식 파이프라인을 사용할 수 있습니다.

## 주요 기능

- 실제 Mid-360 데이터에서 얻은 CSV 기반 비반복 스캔 패턴
- `livox_ros_driver2/msg/CustomMsg` 호환 출력
- 디버깅용 `sensor_msgs/msg/PointCloud2` 동시 출력
- 스캔 중 로봇 움직임을 반영한 포인트별 시간과 자세 보간
- 거리 노이즈, 고정 bias, 시간에 따른 drift, point dropout 설정
- 최초 LiDAR 자세 기준 ground-truth point cloud와 odometry 출력
- ROS 2 Humble의 Gazebo Fortress와 최신 Gazebo 계열을 고려한 빌드 구성
- 재사용 가능한 xacro 매크로와 최소 실행 데모 제공

## 지원 환경

현재 검증된 환경은 다음과 같습니다.

| 항목 | 검증 환경 |
| --- | --- |
| 운영체제 | Ubuntu 22.04 |
| ROS 2 | Humble |
| Gazebo Sim | Fortress (`ign gazebo`) |
| 빌드 도구 | colcon, CMake 3.16 이상, C++17 |

Gazebo Garden과 Harmonic용 CMake 탐색 코드도 포함하지만 실제 실행 검증은 아직
완료하지 않았습니다.

## 의존성

### ROS/Gazebo 패키지

Humble 기준으로 다음 패키지가 필요합니다.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-robot-state-publisher \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-xacro
```

### `livox_ros_driver2`

이 플러그인은 `/livox/lidar`를 실제 드라이버와 같은 `CustomMsg`로 발행하기
위해 `livox_ros_driver2` 패키지의 메시지 정의를 사용합니다.

- 시뮬레이션 중 실제 Mid-360 장비나 네트워크 연결은 필요하지 않습니다.
- 빌드할 때는 ROS 2용 `livox_ros_driver2` 패키지를 찾을 수 있어야 합니다.
- 깨끗한 워크스페이스에서는 먼저
  [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)와
  [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)를 해당
  프로젝트의 공식 ROS 2 Humble 절차에 따라 준비합니다.

이미 `livox_ros_driver2`가 빌드된 워크스페이스에서는 별도 드라이버 실행 없이
이 패키지만 추가하면 됩니다.

## 설치

ROS 2와 Gazebo 명령은 ROS 개발 컨테이너 또는 ROS 2 Humble 환경 안에서
실행하는 것을 권장합니다.

```bash
mkdir -p ~/mid360_ws/src
cd ~/mid360_ws/src

git clone git@github.com:jongbob1918/livox_mid360_gz_sim_ros2.git \
  livox_mid360_gz_sim
```

같은 워크스페이스에 `livox_ros_driver2`가 준비된 후 빌드합니다.

```bash
cd ~/mid360_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to livox_mid360_gz_sim
source install/setup.bash
```

빌드 중 아래 오류가 발생하면 `livox_ros_driver2`가 ROS 2 패키지로 준비되지
않은 상태입니다.

```text
Could not find a package configuration file provided by "livox_ros_driver2"
```

## 빠른 실행

패키지에 포함된 최소 로봇과 world를 실행합니다.

```bash
cd ~/mid360_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_mid360_gz_sim demo.launch.py
```

다른 터미널에서 발행 상태를 확인합니다.

```bash
cd ~/mid360_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic info /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --no-arr
```

RViz에서는 `PointCloud2` display를 추가하고 Topic을
`/livox/lidar/points`, Fixed Frame을 `laser_link`로 선택합니다.

## 기본 출력

| 토픽 | 메시지 타입 | 설명 |
| --- | --- | --- |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 실제 드라이버 호환 스캔 |
| `/livox/lidar/points` | `sensor_msgs/msg/PointCloud2` | RViz·디버깅용 측정 point cloud |
| `/ground_truth/livox/points` | `sensor_msgs/msg/PointCloud2` | 노이즈 적용 전 ground truth |
| `/ground_truth/odometry` | `nav_msgs/msg/Odometry` | 최초 LiDAR 자세 기준 상대 pose |

`/livox/lidar/points`와 ground-truth point cloud는 구독자가 있을 때만 실제
메시지를 구성해 발행합니다. `/livox/lidar`는 항상 발행합니다.

## 내 로봇 모델에 적용하기

### 1. LiDAR 링크와 장착 조인트 정의

플러그인은 로봇의 외형이나 장착 위치를 만들지 않습니다. 사용자의 URDF/xacro가
LiDAR 링크와 부모 링크에 연결되는 고정 조인트를 소유해야 합니다.

```xml
<link name="mid360_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="0.265"/>
    <inertia ixx="0.0002" ixy="0" ixz="0"
             iyy="0.0002" iyz="0" izz="0.0003"/>
  </inertial>
</link>

<joint name="mid360_joint" type="fixed">
  <parent link="base_link"/>
  <child link="mid360_link"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
</joint>

<!-- Gazebo가 fixed joint를 합쳐 mid360_link를 제거하지 않도록 유지합니다. -->
<gazebo reference="mid360_joint">
  <disableFixedJointLumping>true</disableFixedJointLumping>
</gazebo>
```

`link_name`으로 지정한 링크가 Gazebo 모델 안에 실제로 남아 있어야 합니다.
`disableFixedJointLumping`을 빠뜨리면 플러그인이 링크를 찾지 못할 수 있습니다.

### 2. ROS frame alias 정의

`frame_id`를 `mid360_link`로 사용할 경우 이 단계는 생략할 수 있습니다. 운영
시스템이 `laser_link`를 사용한다면 같은 좌표의 alias를 정의합니다.

```xml
<link name="laser_link"/>
<joint name="laser_link_alias_joint" type="fixed">
  <parent link="mid360_link"/>
  <child link="laser_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### 3. 플러그인 xacro 추가

로봇 xacro의 `<robot>` 내부에서 매크로 파일을 include하고 호출합니다.

```xml
<xacro:include
  filename="$(find livox_mid360_gz_sim)/urdf/mid360_gz_plugin.xacro"/>

<xacro:livox_mid360_gz_plugin
  link_name="mid360_link"
  node_name="livox_mid360_gz_sim"
  frame_id="laser_link"
  topic="/livox/lidar"
  points_topic="/livox/lidar/points"
  update_rate="10.0"/>
```

시뮬레이션과 실기에서 하나의 xacro를 공유한다면 `use_sim` 조건 안에서 호출하는
것이 좋습니다.

```xml
<xacro:if value="$(arg use_sim)">
  <xacro:livox_mid360_gz_plugin
    link_name="mid360_link"
    frame_id="laser_link"/>
</xacro:if>
```

### 4. World 시스템 확인

데모처럼 spawn service와 장면 갱신을 사용할 경우 world에 최소한 Physics,
UserCommands, SceneBroadcaster 시스템이 필요합니다.

```xml
<plugin filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system"
        name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster"/>
```

## xacro 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `link_name` | `mid360_link` | Gazebo 안에서 ray 원점으로 사용할 링크 |
| `node_name` | `livox_mid360_gz_sim` | 플러그인이 생성하는 ROS 2 노드 이름 |
| `frame_id` | `laser_link` | 출력 메시지의 frame ID |
| `topic` | `/livox/lidar` | `CustomMsg` 출력 토픽 |
| `points_topic` | `/livox/lidar/points` | 측정 `PointCloud2` 출력 토픽 |
| `ground_truth_points_topic` | `/ground_truth/livox/points` | ground-truth point cloud 토픽 |
| `ground_truth_odom_topic` | `/ground_truth/odometry` | ground-truth odometry 토픽 |
| `ground_truth_frame_id` | `camera_init` | ground-truth 기준 frame ID |
| `enable_ground_truth` | `true` | ground-truth publisher 활성화 여부 |
| `csv_file_name` | 패키지 기본 CSV | 스캔 패턴 CSV의 절대 경로 |
| `samples` | `20000` | 한 번 발행할 때 진행할 pattern sample 수 |
| `downsample` | `1` | pattern sample 간격. 값이 클수록 계산량과 point 수 감소 |
| `update_rate` | `10.0` | 목표 발행 주기(Hz). CPU 부하에 따라 실제 주기는 낮아질 수 있음 |
| `min_range` | `0.2` | 최소 측정 거리(m) |
| `max_range` | `50.0` | 최대 측정 거리(m) |
| `noise_seed` | `42` | 재현 가능한 난수 seed |
| `range_noise_stddev` | `0.01` | 거리 Gaussian noise 표준편차(m) |
| `range_bias` | `0.0` | 고정 거리 bias(m) |
| `range_drift_per_second` | `0.0002` | 시간에 따라 증가하는 거리 bias(m/s) |
| `dropout_rate` | `0.01` | point별 dropout 확률, 범위 `0.0~1.0` |

여러 로봇 또는 여러 Mid-360을 동시에 실행할 때는 `node_name`, 출력 토픽,
frame ID를 각각 고유하게 지정해야 합니다.

## `env-hooks` 폴더는 무엇인가요?

`env-hooks/`는 사용자가 직접 실행하는 스크립트 폴더가 아닙니다. ROS 2의
ament가 패키지 설치 후 환경변수를 자동으로 구성하기 위한 파일입니다.

이 패키지의
`env-hooks/livox_mid360_gz_sim.dsv.in`은 빌드된 플러그인 라이브러리가 있는
`<workspace>/install/livox_mid360_gz_sim/lib`를 다음 Gazebo 검색 경로에
추가합니다.

- `GZ_SIM_SYSTEM_PLUGIN_PATH`
- `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`

사용자는 다음 명령만 실행하면 됩니다.

```bash
source ~/mid360_ws/install/setup.bash
```

등록 여부는 다음과 같이 확인할 수 있습니다.

```bash
echo "$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "$IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
```

## IMU 사용

이 패키지는 Mid-360의 LiDAR 스캔만 제공합니다. IMU 센서와 LiDAR→IMU
extrinsic은 로봇마다 장착 상태가 다르므로 공통 플러그인에 포함하지 않습니다.

FAST-LIO 등에서 `/livox/imu`가 필요하면 사용자의 로봇 URDF에 Gazebo IMU
센서를 추가하고, 해당 로봇에서 측정한 extrinsic을 설정해야 합니다. 임의의
공통 extrinsic 값을 사용하지 마십시오.

## 문제 해결

### `Failed to load system plugin` 또는 shared library를 찾지 못함

워크스페이스 setup을 source하고 라이브러리와 환경변수를 확인합니다.

```bash
source ~/mid360_ws/install/setup.bash
ls ~/mid360_ws/install/livox_mid360_gz_sim/lib/liblivox_mid360_gz_sim_plugin.so
echo "$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "$IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
```

이전 빌드 결과가 남아 있다면 해당 패키지만 깨끗하게 다시 빌드합니다.

```bash
cd ~/mid360_ws
rm -rf build/livox_mid360_gz_sim install/livox_mid360_gz_sim
colcon build --symlink-install --packages-up-to livox_mid360_gz_sim
source install/setup.bash
```

### `Link not ready yet` 이후 point cloud가 나오지 않음

- `link_name`과 URDF의 실제 링크 이름이 같은지 확인합니다.
- 고정 조인트에
  `<disableFixedJointLumping>true</disableFixedJointLumping>`를 설정합니다.
- 모델 spawn이 성공했는지 Gazebo 로그를 확인합니다.

### 물체가 보이는데 LiDAR가 감지하지 못함

현재 CPU raycaster가 감지하는 collision geometry는 다음 두 종류입니다.

- box
- plane

mesh, cylinder, sphere collision은 현재 raycast 대상이 아닙니다. 테스트할 물체에
box 또는 plane collision을 추가하십시오. visual geometry만 있는 물체도 감지하지
못합니다.

### 실제 발행 주기가 `update_rate`보다 낮음

`update_rate`는 목표 상한입니다. 현재 raycast는 CPU에서 계산되므로 world의
collision 수와 `samples`에 따라 실제 속도가 낮아질 수 있습니다.

```xml
<xacro:livox_mid360_gz_plugin
  samples="10000"
  downsample="2"
  update_rate="10.0"/>
```

`samples`를 줄이거나 `downsample`을 높이면 계산량이 줄어듭니다.

## 개발 및 검증

```bash
cd ~/mid360_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select livox_mid360_gz_sim \
  --cmake-args -DBUILD_TESTING=ON

colcon test --packages-select livox_mid360_gz_sim
colcon test-result --test-result-base build/livox_mid360_gz_sim --verbose
```

## 디렉터리 구조

```text
livox_mid360_gz_sim_ros2/
├── config/       # Mid-360 스캔 패턴 CSV
├── env-hooks/    # Gazebo plugin 검색 경로 자동 등록
├── include/      # 플러그인과 scan timing 헤더
├── launch/       # 최소 데모 launch
├── src/          # Gazebo Sim 시스템 플러그인 구현
├── test/         # scan timing 회귀 테스트
├── urdf/         # 재사용 xacro와 데모 로봇
└── worlds/       # 데모 world
```

## 알려진 제한사항

- box와 plane collision만 감지합니다.
- 20,000 point CPU raycast는 복잡한 world에서 목표 10 Hz를 달성하지 못할 수
  있습니다.
- Garden/Harmonic 런타임 호환성은 아직 검증하지 않았습니다.
- 저장소는 공개되어 있지만 번들 스캔 패턴의 외부 재배포 근거는 별도로 정리되어
  있지 않으며 라이선스는 Proprietary입니다. 공개 열람 가능 여부와 사용·복제·배포
  허가는 서로 다릅니다.

## 라이선스

Copyright © LK Robotics. All rights reserved.

소스 코드와 번들 스캔 패턴은 별도 서면 허가 없이 외부 복제·배포할 수 없습니다.
