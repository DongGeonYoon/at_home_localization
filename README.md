# at_home_localization

TidyBot의 매핑과 측위, costmap 패키지 모음입니다. 센서는 Livox Mid-360이고 ROS 2 Humble에서 실행합니다.

- **매핑**: FAST-LIO로 PCD 맵을 만듭니다. 그 PCD로 2D 점유 격자(png/yaml)를 만듭니다.
- **측위**: FAST-LIO 오도메트리와 PCD 맵 ICP로 `map → camera_init → body → base_link` TF를 냅니다.
- **costmap**: static costmap(구조 맵 + inflation)과 local costmap(실시간 LiDAR)을 냅니다. MPPI 제어기가 이 costmap을 사용합니다.

---

## ⚠️ 맵(PCD)과 yaml은 항상 직접 지정하세요

**실행할 때마다 `pcd_map:=`와 `map_yaml:=`를 반드시 함께 넣어 주세요.**

- launch 기본값(`~/at_home_localization/maps/406.pcd`, `406_new.yaml`)은 **레포를 `~/at_home_localization`에 clone했을 때만** 맞습니다. 다른 위치에 clone했거나 다른 방에서 쓰면 기본값이 틀린 맵을 가리킵니다.
- **PCD와 yaml은 같은 매핑 결과에서 나온 한 쌍이어야 합니다.** 다른 방의 PCD와 yaml을 섞으면 측위 좌표와 costmap 벽 위치가 어긋납니다. 이때 오류는 나지 않습니다.
- PCD 경로가 틀리면 측위 노드 로그에 `Failed to read PCD map or map is empty`가 나오고 측위가 되지 않습니다.
- 실행 로그에서 `Global map loaded: <경로>`를 보고 원하는 맵이 올라왔는지 확인하세요.

매핑할 때도 저장 경로 `map_file_path:=`를 지정해야 합니다. 빠뜨리면 매핑 launch가 시작되지 않습니다([4. 매핑](#4-매핑) 참고).

---

## 1. 구성

```text
at_home_localization/
├── src/
│   ├── FAST_LIO_LOCALIZATION2/  # 측위: FAST-LIO + global_localization.py (ICP) + transform_fusion.py
│   ├── FAST_LIO_ROS2/           # 매핑: FAST-LIO (map_save 서비스, 기존 파일 덮어쓰지 않음)
│   ├── livox_ros_driver2/       # Mid-360 드라이버
│   ├── costmap_host/            # Nav2 Costmap2DROS 단독 실행 + FreshVoxelLayer
│   └── mppi_nav/                # bringup launch, costmap 설정, self_filter
├── maps/                        # PCD 맵과 2D 맵(png/yaml)
├── tools/pcd_to_map.py          # PCD → 2D 점유 격자 생성
└── install_deps.sh              # 의존성 설치 (처음 한 번)
```

| 패키지 | 원본 | 이 레포에서 바꾼 것 |
|---|---|---|
| FAST_LIO_LOCALIZATION2 | [Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2](https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2) | point-to-plane 다중 스케일 ICP, fitness 게이트, 큰 보정 거부, 2D Pose Estimate를 base_link 기준으로 해석, `blind` 조건식 버그 수정 |
| FAST_LIO_ROS2 | [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | 저장 경로를 launch 인자 `map_file_path:=`로 받음(필수), 파일이 있으면 `이름_YYYYmmdd_HHMMSS.pcd`로 저장 |
| livox_ros_driver2 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | `config/MID360s_config.json` (호스트 IP) |

원본 레포의 라이선스 파일(`LICENSE`)을 그대로 두었습니다.

---

## 2. 설치와 빌드

**레포는 `~/at_home_localization`에 clone하세요.** launch 기본 경로가 이 위치를 기준으로 합니다.

**① 의존성 설치 (처음 한 번)**: Ubuntu 22.04에 ROS 2 Humble이 설치된 상태에서 실행합니다.
```bash
cd ~
git clone https://github.com/DongGeonYoon/at_home_localization.git
cd ~/at_home_localization
conda deactivate 2>/dev/null; ./install_deps.sh
```
설치하는 것:
- apt: Nav2, pcl_ros, tf_transformations, rmw_zenoh_cpp, RViz, PCL, Eigen, APR
- **Livox-SDK2**: 소스를 받아 빌드하고 `/usr/local`에 설치합니다. 이미 있으면 건너뜁니다.
- Python(`--user`): `open3d==0.19.0`, `ros2_numpy==0.0.5`, **`numpy==1.24.2`**
  - numpy 2.x가 잡히면 `global_localization.py`가 시작하자마자 종료됩니다. 스크립트는 numpy를 마지막에 1.24.2로 고정합니다.
  - conda 환경이 켜져 있으면 스크립트가 멈춥니다. conda python과 시스템 ROS python이 섞이면 실행이 깨지기 때문입니다. 실행할 때도 conda를 끄세요.

**② 빌드**
```bash
cd ~/at_home_localization
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release
```
`-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`이 없으면 `livox_ros_driver2`가 `LIVOX_INTERFACES_INCLUDE_DIRECTORIES ... NOTFOUND`로 실패합니다.

**③ LiDAR 유선 랜 설정 (처음 한 번)**

Mid-360은 `192.168.1.100`, PC는 **`192.168.1.5`**로 고정되어 있습니다(`src/livox_ros_driver2/config/MID360s_config.json`).
```bash
nmcli device status                         # LiDAR에 연결된 유선 장치 이름 확인 (예: enp3s0)
sudo nmcli connection add type ethernet ifname <장치이름> con-name livox \
  ipv4.method manual ipv4.addresses 192.168.1.5/24 ipv6.method disabled
sudo nmcli connection up livox
ping -c 3 192.168.1.100                     # LiDAR 응답 확인
```
- PC IP를 다르게 쓰려면 `MID360s_config.json`의 `host_ip`를 같은 값으로 바꾸세요. `--symlink-install`로 빌드했다면 다시 빌드하지 않아도 됩니다.
- LiDAR IP가 `192.168.1.100`이 아니면(공장 설정은 `192.168.1.1xx`, 끝자리는 시리얼 번호 끝 두 자리) `lidar_configs[0].ip`를 바꾸세요.

---

## 3. 실행 (측위 + costmap bringup)

LiDAR 드라이버, FAST-LIO, ICP 측위, map_server, static/local costmap, Zenoh router를 한 번에 띄웁니다.

```bash
source /opt/ros/humble/setup.bash
source ~/at_home_localization/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=25

MAP_DIR=$HOME/at_home_localization/maps

ros2 launch mppi_nav slam_costmap_bringup.launch.py \
  pcd_map:=$MAP_DIR/406.pcd \
  map_yaml:=$MAP_DIR/406_new.yaml \
  run_livox:=true run_rviz:=true run_zenoh:=true
```

다른 맵을 쓸 때는 **`pcd_map`과 `map_yaml`을 같은 쌍으로** 바꿉니다([6. 맵 목록](#6-맵-목록) 참고).

```bash
# 412호
  pcd_map:=$MAP_DIR/412.pcd map_yaml:=$MAP_DIR/412_test1.yaml
# 실험실
  pcd_map:=$MAP_DIR/lab_map.pcd map_yaml:=$MAP_DIR/lab_map_structural.yaml
```

**자주 쓰는 인자**

| 인자 | 기본값 | 설명 |
|---|---|---|
| `pcd_map` | `~/at_home_localization/maps/406.pcd` | **항상 지정.** 측위용 PCD |
| `map_yaml` | `~/at_home_localization/maps/406_new.yaml` | **항상 지정.** static costmap용 2D 맵 |
| `run_livox` | `false` | Livox 드라이버도 함께 실행 |
| `run_rviz` | `false` | RViz 실행 |
| `run_zenoh` | `true` | 이 PC에서 Zenoh router(`tcp/[::]:7447`) 실행. 이미 router가 떠 있으면 `false` |
| `zenoh_endpoint` | `""` | 다른 PC의 router에 연결, 예: `tcp/192.168.0.93:7447` |
| `run_static_costmap` / `run_local_costmap` | `true` | costmap 끄기 |
| `use_sim_time` | `false` | bag 재생 시 `true` (`ros2 bag play --clock`) |

**실행 후 확인**
```bash
ros2 topic echo /map --once --field info     # 2D 맵 크기와 origin
ros2 run tf2_ros tf2_echo map base_link       # 로봇 위치
```

### 초기 위치 지정
1. RViz에서 **2D Pose Estimate**로 로봇의 실제 위치와 방향을 찍습니다. 클릭 지점은 **base_link(로봇 바닥 중심)** 기준입니다.
2. 로그에 `ICP relocalization: fitness@0.25m 0.9xx`가 나오면 성공입니다.
3. `Initial pose rejected`가 나오면 이전 추정을 유지한 상태입니다. 실제 위치에 더 가깝게 다시 찍으세요(기준 fitness 0.85).

추적 중 fitness가 0.75보다 낮거나, 한 번에 0.5 m 또는 10°보다 큰 보정이 나오면 그 결과는 반영하지 않습니다.

---

## 4. 매핑

**측위 bringup은 꺼 두세요.** FAST-LIO와 Livox 드라이버가 서로 충돌합니다.

**① 실행** (터미널 두 개). 저장 경로 `map_file_path:=`는 **필수**입니다.
```bash
# 터미널 1: LiDAR 드라이버
source /opt/ros/humble/setup.bash
source ~/at_home_localization/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# 터미널 2: FAST-LIO 매핑
source /opt/ros/humble/setup.bash
source ~/at_home_localization/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml \
  map_file_path:=$HOME/at_home_localization/maps/<맵이름>.pcd
```
- `map_file_path`를 빠뜨리면 `map_file_path is required`로 바로 종료됩니다. 폴더가 없으면 자동으로 만듭니다.
- 파일이 이미 있으면 덮어쓰지 않고 `<맵이름>_YYYYmmdd_HHMMSS.pcd`로 저장합니다.
- 로봇을 바닥에 둔 채로 시작하세요. 맵 원점은 시작할 때 IMU 위치(바닥에서 약 0.40 m)입니다.
- 방 밖까지 쓸 계획이면 문을 열고 매핑하세요.

**② 저장**: 매핑 터미널에서 `Ctrl+C`를 누르면 저장됩니다. 중간에 저장하려면 아래 명령을 씁니다.
```bash
ros2 service call /map_save std_srvs/srv/Trigger
```
로그의 `Map saved to <경로>`에서 실제로 저장된 파일 이름을 확인하세요.

---

## 5. 2D 맵(png/yaml) 만들기

```bash
cd ~/at_home_localization
/usr/bin/python3 tools/pcd_to_map.py \
  --input maps/<맵이름>.pcd \
  --output-prefix maps/<맵이름>_structural
```
- 높이 map z 0.15~1.8 m(바닥에서 약 0.55~2.2 m, map 원점이 바닥 위 0.40 m라서)를 2D로 투영하고 큰 구조물(벽)만 남깁니다. 해상도는 0.05 m입니다. 관측하지 않은 영역은 unknown(-1)입니다.
- 결과: `<맵이름>_structural.png/.yaml`(사용할 맵), 그리고 확인용 `_raw_occupancy.png`, `_classification.png`, `_residual.png`
- 손으로 고칠 때(가구 제거, 가상 벽 추가)는 **png의 픽셀 크기를 바꾸지 말고**, yaml의 `origin`과 `resolution`은 원본 그대로 두세요. 크기가 바뀌면 PCD와 벽 위치가 어긋납니다.

---

## 6. 맵 목록

`pcd_map`과 `map_yaml`은 같은 행의 쌍으로 쓰세요.

| 장소 | `pcd_map` | `map_yaml` | 비고 |
|---|---|---|---|
| 406호 | `406.pcd` | `406_new.yaml` | 사람이 편집한 맵(현재 사용). 두 방 사이가 막혀 있음 |
| 406호 | `406.pcd` | `406_structural.yaml` | 자동 생성. 두 방 사이 통로 약 0.70 m |

406호 파일 전체: `406.pcd`(PCD), `406_new.png/.yaml`(편집한 맵, 기본값), `406_new.jpg`(편집 원본 이미지), `406_structural.png/.yaml`(자동 생성), `406_raw_occupancy.png`, `406_classification.png`, `406_residual.png`(생성 과정 확인용)

| 412호 | `412.pcd` | `412_test1.yaml` | 책상 제거, 벽만 남김 |
| 412호 | `412_20261028_203939.pcd` | `412_test2.yaml` | 412호 두 번째 매핑 |
| 실험실 | `lab_map.pcd` | `lab_map_structural.yaml` | 이미지는 `lab_map_structural_edited_0p05.png` |

---

## 7. 출력 (다른 팀이 쓰는 인터페이스)

| 이름 | 타입 | 내용 |
|---|---|---|
| TF `map → camera_init → body → base_link` | tf2 | `map → camera_init`은 ICP 보정, `body → base_link`는 고정값 (-0.321, -0.023, -0.401) m |
| `/localization` | `nav_msgs/Odometry` | map 기준 body 자세, 50 Hz. twist는 채워지지 않음 |
| `/map` | `nav_msgs/OccupancyGrid` | 2D 구조 맵 |
| `/mppi_static_costmap/costmap` | `nav_msgs/OccupancyGrid` | 구조 맵 + inflation, 0.2 Hz |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 로봇 중심 8 × 8 m, 실시간 LiDAR, 5 Hz |

**costmap 설정** (`src/mppi_nav/config/`)

| 항목 | static | local |
|---|---|---|
| footprint | 사각형 0.54 × 0.50 m + 전면 LiDAR 돌출 (내접 0.25 / 외접 0.368 m) | 같음 |
| inflation 반경 / 기울기 | 0.45 m / 33.5 | 0.79 m / 6.67 |
| 장애물 높이 (map z) | – | -0.25 ~ 0.80 m |
| 장애물 유지 시간 | – | 1.0초 |

`data` 99는 로봇 중심이 그 셀에 오면 어떤 방향이든 충돌한다는 뜻입니다. 100은 장애물 셀입니다.
자세한 인터페이스는 `src/mppi_nav/README_COSTMAP_INTERFACE.md`에 있습니다(일부 값은 이전 기준).
