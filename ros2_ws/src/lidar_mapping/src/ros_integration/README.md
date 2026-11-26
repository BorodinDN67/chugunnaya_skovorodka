# Интеграция lidar_mapping с ROS 2 Humble

Узел `lidar_mapping_node` преобразует сообщения ROS 2 и передаёт XYZ, метку скана и синхронизированную позу в `lidar_core`. Наблюдения по входным данным приведены в [DATA_ANALYSIS.md](../../../../../agent/DATA_ANALYSIS.md).

## Сборка в стенде

Войти в работающий контейнер `r_hacatonius` из Ubuntu.

```bash
docker exec -it r_hacatonius bash -l
```

Внутри контейнера выполнить сборку из `/ros2_ws`.

```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --packages-select lidar_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /ros2_ws/install/setup.bash
```

## Запуск с MCAP

Откройте два терминала в контейнере и выполните `source /ros2_ws/install/setup.bash` в каждом. Запустите узел перед воспроизведением. Оба launch-файла включают `use_sim_time` по умолчанию.

```bash
ros2 launch lidar_mapping mapping.launch.py mode:=gicp_mapping
```

Во втором терминале запустите данные. Опция `--clock` публикует `/clock`.

```bash
ros2 bag play /ros2_ws/bag_for_mapping --clock 100 \
  --topics /utlidar/cloud_base /utlidar/robot_odom
```

| Режим | Launch | Облако | Результат |
|---|---|---|---|
| Накопление `cloud_deskewed` | `mapping.launch.py mode:=deskewed` | `/utlidar/cloud_deskewed` | `map_baseline.pcd` |
| Накопление по одометрии | `mapping.launch.py mode:=odometry` | `/utlidar/cloud_base` | `map_odometry.pcd` |
| GICP Mapping | `mapping.launch.py mode:=gicp_mapping` | `/utlidar/cloud_base` | `map_gicp.pcd` |
| GICP Localization | `localization.launch.py` | `/utlidar/cloud_base` | Поза и траектория на загруженной карте |

Для deskewed передайте плееру `--topics /utlidar/cloud_deskewed /utlidar/robot_odom`. Узел удаляет нулевой префикс, накапливает глобальные точки без второго преобразования и получает позу для траектории из одометрии при её наличии. Оба режима построения по `cloud_base` выбирают первую синхронизированную одометрию как начальную позу карты. Запускайте один режим на ROS-домен.

Локализация требует существующего PCD и явной начальной позы **в координатах этой карты**. Передайте `INITIAL_POSE` как семь чисел через запятую в порядке `x,y,z,qx,qy,qz,qw`. Поза из второй записи сама по себе дана в `odom` этой записи.

```bash
: "${INITIAL_POSE:?Задайте начальную позу в координатах карты}"
ros2 launch lidar_mapping localization.launch.py \
  map_directory:=/ros2_ws/maps map_input_file:=map_gicp.pcd \
  "initial_pose_xyz_xyzw:=$INITIAL_POSE"
```

```bash
ros2 bag play /ros2_ws/bag_for_localization --clock 100 \
  --topics /utlidar/cloud_base /utlidar/robot_odom
```

## Настройки и выходы

- Параметры по умолчанию находятся в `config/lidar_mapping.yaml`. Другой YAML передаётся через `config:=/path/to/config.yaml`. Можно настроить темы, кадры, размеры буферов, пороги GICP и частоту публикации карты. Параметры адаптера задаются при запуске. Для источника без `/clock` используйте `use_sim_time:=false`.
- Подписки используют `RELIABLE` и `VOLATILE`. Буфер `OdometryBuffer` интерполирует по `header.stamp`, с опорами не дальше 50 мс друг от друга по умолчанию. Скан ждёт до 250 мс реального времени, максимум 32 ожидающих скана. Нет экстраполяции. По истечении срока скан учитывается в `odometry_timeout`. В режиме deskewed облако можно накопить и без позы. `/clock` служит часами ROS, а метки входных сообщений сохраняются как шкала синхронизации. При скачке `/clock` назад очередь, траектория и состояние сессии сбрасываются. Карта локализации остаётся загруженной.
- Выходы под `/lidar_mapping` — `aligned_cloud` (`PointCloud2`, best effort), `map` (`PointCloud2`, transient local), `pose` (`PoseStamped`), `path` (`Path`, transient local) и `diagnostics` (`DiagnosticArray`). Облака, поза и путь в кадре `map` получают исходную метку скана.
- TF публикуется на метку скана. `map → odom` вычисляется как `T_map_base · inverse(T_odom_base)`, а `odom → base_link` берётся из той же интерполированной одометрии. В предоставленных MCAP TF отсутствует. При наличии другого издателя отключите соответствующий параметр `publish.map_to_odom_tf` или `publish.odom_to_base_tf` в YAML.
- Карты сохраняются через сервис или при штатном завершении узла, если включён `output.save_on_shutdown`. Каталог по умолчанию `/ros2_ws/maps`, имя можно задать в `map.output_file`. Сервис в локализации оставляет загруженную карту неизменной. Метрики и выходная поза на каждый скан пишутся в `/ros2_ws/results/<mode>.csv` с указанием времени ожидания и этапов ядра. `output.metrics_file` задаёт другой путь или отключает запись пустой строкой. Новый запуск перезаписывает CSV выбранного режима.

```bash
ros2 service call /lidar_mapping/save_map std_srvs/srv/Trigger '{}'
```
