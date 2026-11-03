# Costmap / TF Interface for the MPPI Controller

이 모듈은 **FAST-LIO 위치추정, 구조물 Occupancy Map, Static Costmap, Local Costmap**까지만 제공합니다.
MPPI Controller, planner, goal 처리, `/cmd_vel`은 여기서 실행하지 않습니다.

아래 수치는 2026-10-02 실제 LiDAR(Mid-360) 연결 상태에서 측정·확인한 값입니다.
실행 중인 값은 언제든지 다음 명령으로 다시 출력할 수 있습니다.

```bash
ros2 run mppi_nav interface_report
```

---

## 0. 실행 환경

팀 결정으로 **모든 노드가 `rmw_zenoh_cpp`를 사용**합니다(DDS 사용 안 함).
rmw_zenoh 노드는 DDS 노드와 통신하지 않으므로, 이 모듈·Livox driver·RViz·MPPI 노드·`ros2` CLI를
실행하는 **모든 터미널**에 아래 환경이 필요합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/fast_lio_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=25
```

rmw_zenoh는 PC마다 **Zenoh router**(`rmw_zenohd`, `tcp/[::]:7447`)가 있어야 노드끼리 서로를 찾습니다.
이 launch는 기본으로 router를 같이 실행합니다(`run_zenoh:=true`).
같은 PC에서 router가 이미 실행 중이면 `run_zenoh:=false`로 실행합니다(포트 7447 중복 방지).

Bringup (map 파일 생성은 [Structural Map 생성](#structural-map-생성) 참고):

```bash
ros2 launch mppi_nav slam_costmap_bringup.launch.py \
  pcd_map:=$HOME/fast_lio_ws/maps/lab_map_20261005_174210.pcd \
  map_yaml:=$HOME/fast_lio_ws/maps/lab_map_20261005_174210_structural.yaml \
  run_livox:=true run_rviz:=true
```

| Launch argument | 기본값 | 설명 |
|---|---|---|
| `run_zenoh` | true | 로컬 Zenoh router(`rmw_zenoh_cpp/rmw_zenohd`) 실행 |
| `zenoh_endpoint` | `""` | 접속할 원격 router, 예: `tcp/192.168.0.93:7447`. 비우면 로컬 router만 사용 |
| `run_livox` | false | true면 Livox driver(`msg_MID360s_launch.py`)도 실행 |
| `run_localization` | true | FAST-LIO + global localization + transform fusion |
| `run_map_server` | true | `/map` publish |
| `run_static_costmap` / `run_local_costmap` | true | 각 costmap |
| `run_rviz` | false | `rviz/costmap_bringup.rviz` |
| `run_local_cloud_filter` | true | 로봇 몸체 점 제거(self_filter). false면 local costmap이 원시 cloud로 marking |
| `local_cloud_input` / `local_cloud_output` | `/cloud_registered_body` / `/local_obstacle_cloud` | self_filter 입출력 |
| `self_filter_min/max_x/y/z` | x -0.32~0.40, y -0.30~0.30, z -0.05~1.00 | 제거할 상자 (base_link 기준, 실측 치수 + 약 5 cm) |
| `pcd_map` / `map_yaml` | `maps/lab_map_20261005_174210.pcd` / `maps/lab_map_20261005_174210_structural.yaml` | 맵 파일 |
| `base_link_x/y/z/roll/pitch/yaw` | -0.321 / -0.023 / -0.401 / 0 / 0 / 0 | `body → base_link` |

`RMW_IMPLEMENTATION`이 `rmw_zenoh_cpp`가 아니면 launch 시작 시 `WARNING: RMW_IMPLEMENTATION is ...`가 출력됩니다.

### 다른 PC(MPPI PC)와 연결

두 PC 모두 router를 실행하고, **한쪽 router가 다른 쪽 router에 접속**하면 됩니다(둘 중 하나만).

```bash
# 방법 A: 로봇 PC에서 MPPI PC router로 접속
ros2 launch mppi_nav slam_costmap_bringup.launch.py ... zenoh_endpoint:=tcp/<MPPI_PC_IP>:7447

# 방법 B: MPPI PC에서 로봇 PC router로 접속
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<ROBOT_PC_IP>:7447"]'
ros2 run rmw_zenoh_cpp rmw_zenohd
```

- 두 PC의 `ROS_DOMAIN_ID`가 같아야 합니다(rmw_zenoh도 domain ID로 통신을 구분).
- Zenoh는 원격에 구독자가 있는 토픽만 네트워크로 보냅니다. MPPI PC에서 RViz로 포인트클라우드
  (`/cloud_registered`, `/livox/lidar`, `/cloud_pcd`)를 구독하면 수 MB/s가 Wi-Fi로 흐르니 주의하세요.
- **원격 연결은 아직 실제로 시험하지 않았습니다.** 2026-10-02 기준 로봇 PC(Wi-Fi `172.100.x`)에서
  `192.168.0.93`으로 ping이 닿지 않았습니다.

### 참고: RMW별 측정 기록

| RMW | FAST-LIO 출력 지연 (`/Odometry` stamp age) |
|---|---|
| `rmw_fastrtps_cpp` | 1.4 ~ 8.5 s (밀림) |
| `rmw_cyclonedds_cpp` | 0.047 s (40 s 동안 안정) |
| `rmw_zenoh_cpp` | **0.029 s** (40 s 동안 안정, 현재 사용) |

rmw_zenoh(2026-10-02) 실측: TF `map→base_link` age 평균 0.087 s(최대 0.154 s),
static costmap ~0.7 Hz, local costmap 3.1~3.6 Hz. Livox driver·lifecycle·`/map`(transient local) 모두 정상.

실행 후 RViz `2D Pose Estimate`로 초기 위치를 한 번 지정해야 `map → base_link`가 올바른 위치가 됩니다.
초기 위치를 지정하면 2 s 뒤 local costmap이 자동으로 초기화됩니다(이전 위치 기준으로 찍힌 장애물 제거).

- 자동 전역 위치추정은 없습니다. **시작할 때마다** 실제 위치에 가깝게 지정해야 합니다(위치 ±0.5 m, 방향 ±30° 이내).
- 클릭한 위치는 ICP로 확인된 뒤에만 반영됩니다. 성공하면 로그에 `ICP relocalization: fitness@0.25m 0.9xx`,
  실패하면 `Initial pose rejected; keeping the previous estimate.`가 나오고 **이전 위치가 그대로 유지**됩니다.
- 이후 1 s마다 `ICP tracking: fitness@0.25m ...`가 출력됩니다. 0.25 m 이내 점 비율이 0.75 미만이거나
  보정이 0.5 m / 10°를 넘으면 그 보정은 버립니다(`Keeping the previous estimate` / `Rejected ICP jump`).

---

## 1. TF — Robot Pose

```text
map ──► camera_init ──► body ──► base_link
```

| Transform | Publisher | Rate | 의미 |
|---|---|---|---|
| `map → camera_init` | `transform_fusion` | ~10 Hz | ICP 전역 위치 보정 (Nav2 `map→odom` 역할) |
| `camera_init → body` | FAST-LIO | ~10 Hz | LiDAR-inertial odometry |
| `body → base_link` | static | – | 측정값 (-0.321, -0.023, -0.401) m, 회전 0 |

MPPI는 **`map → base_link`만 조회**해서 `x [m]`, `y [m]`, `yaw [rad]`를 사용하면 됩니다.

- `base_link`: 베이스 기하 중심을 바닥에 투영한 점, x = 전방, y = 좌측
- `base_link`의 map z는 약 -0.35 m입니다(map 원점이 매핑 시작 시 IMU 위치라 바닥이 z ≈ -0.40). 2D 제어에서는 무시하면 됩니다.
- `odom` 프레임은 없습니다. odometry 프레임이 필요하면 `camera_init`을 사용하세요.
- TF stamp age 측정값: 0.05~0.10 s
- **TF stale 권장 기준: 0.5 s**

---

## 2. Static Map

| 항목 | 값 |
|---|---|
| Topic | `/map` |
| Type | `nav_msgs/msg/OccupancyGrid` |
| QoS | reliable, transient local (1회 publish, 늦게 구독해도 수신) |
| Frame | `map` |
| 내용 | 구조물(벽 등) 중심 정적 맵. 값: `0` free, `100` occupied |

MPPI에서 직접 쓰지 않아도 됩니다. 디버깅과 맵 버전 확인용입니다.

---

## 3. MPPI Static Costmap

| 항목 | 값 |
|---|---|
| Topic | `/mppi_static_costmap/costmap` |
| Type | `nav_msgs/msg/OccupancyGrid` |
| Frame | `map` |
| Layers | StaticLayer(`/map`) + InflationLayer |
| 실시간 장애물 | **포함하지 않음** |
| 갱신 | full map, 5 s마다 재전송(`publish_frequency: 0.2`). 내용은 바뀌지 않음 |
| Node | `/mppi_static_costmap/mppi_static_costmap` |

## 4. Local Costmap

| 항목 | 값 |
|---|---|
| Topic | `/local_costmap/costmap` |
| Type | `nav_msgs/msg/OccupancyGrid` |
| Frame | `map` (rolling window, 로봇 중심으로 이동) |
| Layers | FreshVoxelLayer(`costmap_host`) + InflationLayer |
| 입력 | marking: `/local_obstacle_cloud` (self_filter 출력, frame `body`) / clearing: `/cloud_registered_body` |
| 크기 | 8.0 × 8.0 m (160 × 160 cells) |
| 갱신 | 설정 5 Hz (측정 ~3 Hz) |
| 장애물 높이 | map z -0.25 ~ 0.80 m (map 바닥 ≈ -0.35~-0.40 → 바닥 +0.10~0.15 ~ +1.20 m, 로봇 높이 0.93 m) |
| 장애물 거리 | 센서에서 0.10 ~ 4.0 m (marking), raytrace 8.0 m (clearing) |
| 미관측 셀 | `0` (free)로 보고 (`track_unknown_space: false`) |
| 장애물 제거 | 매 갱신마다 voxel을 비우고 최근 **1.0 s**(`lidar.observation_persistence`)의 관측만 다시 표시 |
| 로봇 몸체 점 | self_filter가 base_link 상자 안의 점을 제거 (footprint 밖의 LiDAR 주변 점 포함) |
| Node | `/local_costmap/local_costmap` |

**잔상 짧게, 대신 기억도 짧게:** 장애물 표시는 마지막으로 보인 뒤 약 1 s 동안만 유지됩니다. 지나간 사람이나
map→odom 보정으로 어긋난 표시는 약 1 s 안에 사라집니다(가상 시나리오 측정 1.05~1.2 s). 반대로 Mid-360 아래쪽
시야각(-7°) 때문에 센서에서 약 3.3 m 안쪽의 낮은 영역(바닥 +0.10~0.40 m)으로 들어간 장애물도 약 1 s 뒤 사라집니다.
유지 시간 0.5 s에서는 점이 듬성듬성해서 갱신마다 장애물 셀의 약 30%가 바뀌어 RViz에서 흐리게 깜빡였습니다.
1.0 s로 14%까지 줄었습니다(10/11 bag 재생 비교, 2026-10-28). `FreshVoxelLayer`의 `blind_memory_time`(기본 0)을
10.0으로 주면 시야 밖 표시를 최대 10 s 기억하지만, 현재 사용하지 않습니다.

Local costmap의 `info.origin`은 로봇을 따라 매 메시지마다 바뀝니다. **매번 메시지의 origin을 사용하세요.**

Static과 local은 해상도(0.05 m)와 frame이 같습니다. local 셀 (r, c)는 static 셀
`(r + round((local.origin.y - static.origin.y)/0.05), c + round((local.origin.x - static.origin.x)/0.05))`에 대응합니다.

---

## 5~7. Grid Metadata

| | `/map`, `/mppi_static_costmap/costmap` | `/local_costmap/costmap` |
|---|---|---|
| resolution | 0.05 m/cell | 0.05 m/cell |
| width | 748 cells (37.40 m) | 160 cells (8.0 m) |
| height | 1318 cells (65.90 m) | 160 cells (8.0 m) |
| origin.x | -8.121 | 메시지마다 변함 |
| origin.y | -42.282 | 메시지마다 변함 |

(`maps/lab_map_structural.yaml` → 편집한 구조물 맵 `lab_map_structural_edited_0p05.png` 기준. **값을 하드코딩하지 말고 메시지의 `info`를 쓰세요.**)

`data[row * width + col]`에서 row 0은 origin.y 쪽(아래), col 0은 origin.x 쪽(왼쪽)입니다.
셀 중심 좌표: `x = origin.x + (col + 0.5) * resolution`, `y = origin.y + (row + 0.5) * resolution`.

---

## 8~10. Robot / Inflation

| 항목 | Static | Local |
|---|---|---|
| footprint (base_link) | 사각형 0.54 × 0.50 m + 앞쪽 LiDAR 돌출부 | 동일 |
| inscribed / circumscribed | 0.25 / 0.368 m | 0.25 / 0.368 m |
| inflation_radius | 0.37 m | **1.05 m** |
| cost_scaling_factor | 25.0 | **6.67** |

Local은 사각형 footprint(내접 0.25 m)에 MPPI planner costmap과 같은 1.05 m / 6.67 기울기입니다(2026-10-28 통일).
원형 0.38 m(99 = 어느 방향이든 충돌)로 해 봤을 때는, 로봇이 장애물을 옆에 두고 0.27~0.38 m로 지나갈 때(실제 반폭 0.25 m라
닿지 않음)도 중심 셀이 99가 되어 MPPI가 경로를 따라가지 못했습니다(안전한 자세의 15.8%를 충돌로 판정). 지금은 99를 0.25 m까지만
두고, 0.25~0.368 m 모서리 위험 구간은 MPPI가 자세별 footprint 검사로 확인합니다.
local `data` 실측: 0.25 m까지 99, 0.30 m 74, 0.35 m 53, 0.40 m 38, 0.50 m 19, 0.70 m 5, 1.00 m 1, 1.10 m 밖 0.

로봇 베이스는 0.50 (W) × 0.54 (L) m이고, 원(`robot_radius`) 대신 실제 모양의 polygon footprint를 씁니다.

```yaml
footprint: [[0.35, 0.05], [0.35, -0.05], [0.27, -0.05], [0.27, -0.25],
            [-0.27, -0.25], [-0.27, 0.25], [0.27, 0.25], [0.27, 0.05]]
```

- `x 0.27 ~ 0.35, y ±0.05`는 앞쪽 가운데에 달린 Mid-360입니다.
- Static: 충돌 영역(`data 99`)은 장애물에서 **0.25 m**(inscribed)까지입니다. 0.25 ~ 0.368 m는 로봇 방향에 따라
  모서리가 닿을 수 있는데 `data 99` 미만으로 표시됩니다.
- Local: `data 99`는 0.25 m까지(어느 방향이든 충돌). 그 밖은 1.05 m까지 cost가 줄어듭니다(0.30 m 74 → 1.00 m 1).
- Static `inflation_radius` 0.37 m / `cost_scaling_factor` 25: `data` 0.255 m에서 86, 0.30 m에서 28, 0.36 m에서 6, 0.37 m 밖 0.

---

## Costmap Data Encoding

Nav2 내부 cost(0~255, `nav2_costmap_2d/cost_values.hpp`)는 `nav2_costmap_2d::Costmap2DPublisher`가
OccupancyGrid로 publish할 때 다음 표로 변환합니다(Humble).

| Nav2 내부 cost | 의미 | **OccupancyGrid `data` 값** |
|---|---|---|
| 0 | FREE_SPACE | `0` |
| 1 ~ 252 | inflation cost | `1 + (97 * (cost - 1)) / 251` → `1 ~ 98` |
| 253 | INSCRIBED_INFLATED_OBSTACLE (로봇 중심이 여기 있으면 충돌) | `99` |
| 254 | LETHAL_OBSTACLE | `100` |
| 255 | NO_INFORMATION | `-1` |

실제 메시지에서 확인한 값 집합: `{0, 1~98 범위의 inflation 값, 99, 100}`. `-1`은 현재 맵에서는 나오지 않습니다.
**`data >= 99`는 "로봇 중심이 여기 있으면 어느 방향이든 충돌"입니다. 그 밖(0.25 ~ 0.368 m)의 충돌은 footprint로 판단해야 합니다.**

---

## 11. Map Version (SHA256)

현재 운용 맵 (2026-10-26, `pcd_map:=lab_map.pcd map_yaml:=lab_map_structural.yaml`):

```text
MAP_VERSION
a34c568880557288b35c44f0454f81e25526a299b69f0f8c5eca6238bed1da62  lab_map.pcd
d0af4b308fd756c8688931d047cc9746949906bd5f9dd7a6b2e8bad3f01dbb1b  lab_map_structural_edited_0p05.png
72eaca1e27ca253feb1403681c577e54ea1e72e25a816fab4ccfea7b7575a08a  lab_map_structural.yaml
```

```bash
sha256sum ~/fast_lio_ws/maps/lab_map.pcd \
          ~/fast_lio_ws/maps/lab_map_structural_edited_0p05.png \
          ~/fast_lio_ws/maps/lab_map_structural.yaml
```

---

## 12. TF Stale 권장 기준

`map → base_link`의 stamp가 **0.5 s**보다 오래되면 제어를 멈추는 것을 권장합니다.
Costmap 메시지도 stamp를 확인하세요. local은 ~0.3 s 간격이 정상입니다. static은 5 s마다 같은 맵을 다시 보내므로 stale 판단에 쓰지 마세요.

---

## 확인 명령

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /mppi_static_costmap/costmap --once --field info
ros2 topic echo /local_costmap/costmap --once --field info
ros2 topic echo /map --once --field info
ros2 run mppi_nav interface_report
```

## 검증용 rosbag

```bash
ros2 bag record -o slam_costmap_mppi_test \
  /tf /tf_static /Odometry /map \
  /mppi_static_costmap/costmap /local_costmap/costmap \
  /cloud_registered /livox/lidar /livox/imu
```

---

## 전달 파일

```text
maps/lab_map_structural.png
maps/lab_map_structural.yaml
src/mppi_nav/config/static_costmap.yaml
src/mppi_nav/config/local_costmap.yaml
src/mppi_nav/README_COSTMAP_INTERFACE.md
maps/lab_map.pcd              (reference)
```

## Structural Map 생성

```bash
python3 ~/fast_lio_ws/maps/pcd_to_map.py \
  --input ~/fast_lio_ws/maps/lab_map.pcd \
  --output-prefix ~/fast_lio_ws/maps/lab_map_structural
```

| 출력 | 용도 |
|---|---|
| `lab_map_structural.png/.yaml` | Nav2 static map (기본 입력) |
| `lab_map_raw_occupancy.png` | 필터 전 원본 occupancy (비교용) |
| `lab_map_residual.png` | 원본 − 구조물 (제거된 셀, 눈으로 확인용) |
| `lab_map_classification.png` | 검정 구조물 / 빨강 불확실 / 파랑 노이즈 |

주요 parameter (`--help` 참고):

| Parameter | 기본값 | 의미 |
|---|---|---|
| `--z-min`, `--z-max` | 0.15, 1.80 (map z) | 투영 높이 범위. map 바닥은 z ≈ -0.40 |
| `--min-points` | 2 | 셀 occupied 최소 점 개수 |
| `--min-noise-cells` | 3 | 이보다 작은 component는 노이즈 |
| `--min-static-component-cells` | 20 | 구조물 최소 셀 수 |
| `--min-static-extent-m` | 1.0 | 구조물 최소 길이 (PCA 주축) |
| `--link-kernel` | 9 | 분류 시 ~0.2 m 간격 셀을 하나로 묶음 (점선 벽). 맵에는 원래 셀만 남음 |
| `--closing-kernel`, `--closing-iterations` | 3, 1 | 최종 morphological closing |

Map origin은 원본 PCD bounds + 0.5 m padding 기준이며 crop하지 않습니다.

---

## 새 PCD 맵으로 교체

Launch, costmap config, RViz config에는 맵 경로가 고정되어 있지 않습니다(launch 인자 기본값만 `lab_map_20261005_174210`).
같은 로봇·같은 LiDAR 장착 상태로 매핑했다면 다음 두 단계로 바로 사용할 수 있습니다.

```bash
# 1) 구조물 맵 생성
python3 ~/fast_lio_ws/maps/pcd_to_map.py \
  --input ~/fast_lio_ws/maps/new_map.pcd \
  --output-prefix ~/fast_lio_ws/maps/new_map_structural

# 2) 두 경로를 넘겨서 실행
ros2 launch mppi_nav slam_costmap_bringup.launch.py \
  pcd_map:=$HOME/fast_lio_ws/maps/new_map.pcd \
  map_yaml:=$HOME/fast_lio_ws/maps/new_map_structural.yaml
```

확인할 것:

1. **높이 값은 매핑 시작 자세에 따라 달라집니다.**
   map 원점 = 매핑 시작 시 IMU 위치입니다. 현재 값(바닥 z ≈ -0.40, local 장애물 -0.25~0.80,
   `voxel_layer.origin_z: -0.40`, `pcd_to_map.py --z-min/--z-max`)은
   **로봇이 바닥에 놓인 상태에서 매핑을 시작했다**는 가정입니다.
   손으로 들고 매핑했거나 LiDAR 장착 높이가 바뀌었다면 바닥 높이를 다시 확인하고
   `config/local_costmap.yaml`과 `pcd_to_map.py` 인자를 맞게 고쳐야 합니다.
2. **`<base>_residual.png`, `<base>_classification.png`를 눈으로 확인합니다.**
   벽이 빨강(불확실)으로 빠졌으면 `--link-kernel`을 키우거나 `--min-static-extent-m`을 낮춥니다.
   문·복도가 막혔으면 `--closing-kernel`/`--closing-iterations`를 줄입니다.
3. **맵 버전(SHA256)이 바뀝니다.** 새 값을 출력해 MPPI 팀에 다시 전달하세요.
   ```bash
   ros2 run mppi_nav interface_report \
     --pcd ~/fast_lio_ws/maps/new_map.pcd \
     --map-yaml ~/fast_lio_ws/maps/new_map_structural.yaml
   ```
   (`interface_report`의 기본 경로도 `lab_map_20261005_174210`입니다.)
4. **매핑 시 로봇 몸체가 찍히지 않게 합니다.**
   로봇 몸체(상단 구조물 포함)는 LiDAR 뒤쪽 약 0.6 m까지 관측됩니다.
   매핑용 `src/FAST_LIO_ROS2/config/mid360.yaml`의 `preprocess.blind`(현재 0.5 m)를
   0.7 m 정도로 올린 뒤 매핑하지 않으면 로봇이 지나간 경로가 장애물로 맵에 남을 수 있습니다.

---

## RViz 도구

`run_rviz:=true`로 실행되는 `rviz/costmap_bringup.rviz`:

| 항목 | 내용 |
|---|---|
| Displays | Structural Map(`/map`), Static Costmap, Local Costmap, Current Scan In Map(`/cur_scan_in_map`), Raw Body Cloud(`/cloud_registered_body`, 기본 꺼짐), Local Obstacle Cloud(`/local_obstacle_cloud`), Robot Footprint(`/local_costmap/published_footprint`), TF, Localization(`/localization`) |
| Tools | Interact, Move Camera, Select, **2D Pose Estimate**(`/initialpose`) |

MPPI 팀이 사용할 수 있는 RViz 도구:

| 도구 | 동작 | 사용 가능 여부 |
|---|---|---|
| **2D Goal Pose** (RViz 기본) | `/goal_pose` (`geometry_msgs/msg/PoseStamped`, frame `map`) publish | ✅ MPPI 노드가 구독하면 바로 사용 가능 |
| **Publish Point** (RViz 기본) | `/clicked_point` (`geometry_msgs/msg/PointStamped`) publish | ✅ |
| Footprint 표시 | `/local_costmap/published_footprint` (`PolygonStamped`) | ✅ 표시 전용 |
| MPPI 경로/궤적 표시 | MPPI 팀 토픽 (`nav_msgs/Path`, `MarkerArray` 등) | ✅ 토픽 이름만 정하면 됨 |
| **Nav2 Goal + Navigation2 패널** (`nav2_rviz_plugins`, 설치됨) | `navigate_to_pose` action 요청 | ❌ `bt_navigator`·planner가 필요하며 이 모듈 범위가 아님 |

2D Goal Pose / Publish Point를 쓰려면 `rviz/costmap_bringup.rviz`의 `Tools:`에 추가합니다.

```yaml
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Value: /clicked_point
```

Footprint 표시는 `Displays:`에 추가합니다.

```yaml
    - Class: rviz_default_plugins/Polygon
      Name: Robot Footprint
      Enabled: true
      Topic:
        Value: /local_costmap/published_footprint
```

---

## MPPI 팀 주의사항

| 항목 | 내용 |
|---|---|
| RMW | `rmw_zenoh_cpp`, `ROS_DOMAIN_ID=25`. MPPI PC에도 router 필요. PC 간 연결은 아직 실제 시험 전 |
| 초기화 | 시작할 때마다 RViz 2D Pose Estimate 필요. 틀린 클릭은 거부되고 이전 위치 유지 |
| 충돌 판정 | 로봇 중심 셀 `data >= 99` = 확실한 충돌. 0.25~0.368 m는 방향에 따라 모서리가 닿을 수 있으므로 footprint로 확인 |
| Local origin | `/local_costmap/costmap`의 `info.origin`은 메시지마다 바뀜 |
| 갱신 주기 | local 설정 5 Hz / 실측 ~3 Hz, static 5 s마다 재전송(내용 불변) |
| TF stale | `map → base_link` stamp가 0.5 s보다 오래되면 정지 권장 |
| 센서 사각지대 | 센서에서 약 3.3 m 이내, 바닥 +0.10~0.40 m의 낮은 장애물은 LiDAR가 보지 못함. local costmap에서도 약 1 s 뒤 사라짐 |
| 속도 | `/localization`, `/Odometry`의 twist는 **채워지지 않음(항상 0)** |
| 전역 경로 | 구조물 맵에는 책상·카트 등 가구가 없음. 전역 경로가 가구를 지날 수 있으니 local 장애물로 재계획 필요 |

실제 주행 전에 필요한 것 (이 모듈 범위 밖):

- `base_link` 회전값(roll/pitch/yaw = 0)은 LiDAR가 수평·정면이라는 가정이며 캘리브레이션하지 않았습니다.
  로봇을 직진시켜 `map → base_link`의 이동 방향이 yaw와 일치하는지 확인하세요.
- `/cmd_vel` 브리지: `~/tidybot2/base_ros_node.py`가 `/cmd_vel`(Twist, `linear.x/y`, `angular.z`)을 베이스로 전달합니다.
  `max_vel (0.5, 0.5, 1.57)`, `max_accel (1.0, 1.0, 3.14)`. `/cmd_vel`이 0.25 s 끊기면 베이스가 스스로 정지합니다
  (`base_controller.py`). CAN은 한 번에 한 프로세스만 사용할 수 있습니다(`gamepad_teleop.py`와 동시 실행 불가).
