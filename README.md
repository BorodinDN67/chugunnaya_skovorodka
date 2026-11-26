# KISS-ICP на записи Unitree Go2

Образ `kiss-icp-hackathon:humble` предназначен для Windows 11 x64 с Docker Desktop в режиме Linux containers (WSL 2). Он содержит ROS 2 Humble, поддержку MCAP, сообщения Unitree, RViz и собранные ROS-обёртку и ядро KISS-ICP. Коммит KISS-ICP — `85488b7b6a6e1566cff0f8af4b83a9ea77dca7a0` (v1.2.3). [Исходники KISS-ICP](https://github.com/PRBonn/kiss-icp/commit/85488b7b6a6e1566cff0f8af4b83a9ea77dca7a0)

## Запуск на второй машине

### Готовая карта для RViz

В этой ветке сохранён полный офлайн-прогон `bag_for_mapping` с начальным приближением от одометрии Unitree. Карта `demo/mapping/map.pcd` содержит 55 169 точек, траектория `demo/mapping/trajectory.csv` — 1 688 поз. Оба файла используют систему `odom`. Для просмотра достаточно файлов этой ветки и ROS 2 Humble с RViz. Rosbag и пересборка образа не требуются.

**Windows 11 с Docker Desktop.** Обновите ветку `codex/kiss-icp`, запустите VcXsrv на дисплее `0` и выполните в PowerShell из корня репозитория:

```powershell
.\view-guided.ps1
```

Скрипт использует локальный образ `kiss-icp-hackathon:humble`. Если его ещё нет, скачайте архив и один раз выполните `docker load -i .\kiss-icp-hackathon.tar.gz`. [Скачать Docker-образ](https://github.com/BorodinDN67/chugunnaya_skovorodka/releases/download/kiss-icp-humble-v1/kiss-icp-hackathon.tar.gz)

**Установленный ROS 2 Humble.** В терминале с активным окружением ROS 2 выполните из корня репозитория:

```bash
python3 run_kiss.py view --output demo/mapping --frame odom --rviz-config demo/mapping/guided.rviz
```

Если RViz уже открыт, загрузите `demo/mapping/guided.rviz` через **File → Open Config** и запустите публикацию карты и траектории в той же ROS-среде:

```bash
python3 run_kiss.py view --output demo/mapping --frame odom --publish-only
```

### Воспроизведение исходной записи

Скачайте `bag_for_mapping.zip` и распакуйте в папку `data`. Путь к записи должен указывать на каталог, в котором лежат `metadata.yaml` и `bag_for_mapping_0.mcap`. [Скачать ZIP записи из исходного репозитория](https://media.githubusercontent.com/media/ivr0ma/hackathon/ecbeeb681f7fe1e274402623da27444f63a4c61a/hackathon/ros2_ws/bag_for_mapping.zip)

Скачайте архив образа в корень репозитория и выполните команды в PowerShell. [Образ для загрузки на второй машине](https://github.com/BorodinDN67/chugunnaya_skovorodka/releases/tag/kiss-icp-humble-v1)

```powershell
docker load -i .\kiss-icp-hackathon.tar.gz
.\run-kiss.ps1 -Bag .\data\bag_for_mapping -Topic /utlidar/cloud_base -Output .\results
```

Добавьте `-Seconds 12` для прогона первых 12 секунд или `-NoDeskew` для контрольного прогона без компенсации движения. Время воспроизведения определяется временными метками MCAP и соответствует 1×. Запись монтируется только для чтения. Контейнер обрабатывает данные без GUI и без сети.

Сборка и сохранение образа для разработчика:

```powershell
docker build --platform linux/amd64 -t kiss-icp-hackathon:humble .
docker save -o .\kiss-icp-hackathon.tar kiss-icp-hackathon:humble
```

Для локального несжатого архива используйте `docker load -i .\kiss-icp-hackathon.tar`. Дополнительные пакеты исходного стенда, включая Nav2, SLAM Toolbox и PCL, доступны в цели `--target full`.

## Результаты

- `trajectory.csv` содержит ROS-время скана в наносекундах, позицию в метрах и кватернион XYZW. Система координат — `odom_lidar`.
- `map.pcd` содержит накопленные исходные сканы, преобразованные по их позам KISS-ICP в `odom_lidar`. Файл XYZ бинарный, с прореживанием по сетке 0,10 м. Каждый исходный скан переносится как жёсткое тело. Внутрискановая компенсация движения применяется в оценивателе KISS-ICP.
- `metrics.json` содержит число опубликованных входных сканов, число сканов с полученной позой, число точек карты и частоту поз по монотонным часам. `publish_to_odometry_ms_p50_p95` измеряет интервал от публикации облака до обратного вызова одометрии в процессе воспроизведения. Интервал включает DDS, очереди и исполнение обратных вызовов.

### Просмотр в RViz

Запустите на Windows X-сервер VcXsrv на дисплее `0`, доступный из Docker Desktop, и выполните в PowerShell:

```powershell
$out = (Resolve-Path .\results).Path
docker run --rm -e DISPLAY=host.docker.internal:0 -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 --mount "type=bind,source=$out,target=/out,readonly" kiss-icp-hackathon:humble python3 /opt/go2_playback/run_kiss.py view --output /out
```

Режим просмотра читает PCD и CSV и публикует их в ROS 2 для встроенного конфига `results.rviz`. Для просмотра достаточно папки с результатами.

## Проверка входных данных

В метаданных `bag_for_mapping` указаны MCAP, длительность 121,076 с и 116 175 сообщений. Непосредственное чтение MCAP подтвердило 1 688 сообщений `PointCloud2` на `/utlidar/cloud_base`, частоту 13,952 Гц по меткам записи, `frame_id=base_link` и шаг точки 32 байта.

| Поле | Тип | Смещение, байт |
| --- | --- | ---: |
| `x` | FLOAT32 | 0 |
| `y` | FLOAT32 | 4 |
| `z` | FLOAT32 | 8 |
| `intensity` | FLOAT32 | 16 |
| `ring` | UINT16 | 20 |
| `time` | FLOAT32 | 24 |

Во всех 1 688 сканах метки заголовков уникальны, число точек на скан составляет 508–731, а значения поля `time` конечны и неотрицательны. Минимальный, медианный и максимальный размах времени внутри скана составляет 0,058582, 0,064660 и 0,092103 с соответственно. Масштаб согласуется с относительным временем в секундах при периоде скана около 0,072 с и с форматом Unitree. [Конфигурация Unitree для L1](https://github.com/unitreerobotics/point_lio_unilidar/blob/main/config/unilidar_l1.yaml)

KISS-ICP распознаёт `time` типа FLOAT32 и нормирует его внутри каждого скана, поэтому deskew включён по умолчанию. `/utlidar/cloud_deskewed` запрещён в качестве входа, поскольку его координаты уже приведены к глобальной системе старта (в сообщениях `frame_id=odom`). [Извлечение времени точек в KISS-ICP](https://github.com/PRBonn/kiss-icp/blob/v1.2.3/ros/src/Utils.hpp)

### Прогон первых 12 секунд

`ros2 bag info` в контейнере подтвердил метаданные записи, перечисленные выше. Проверка выполнена в Docker Engine под WSL на Windows 11, с deskew и воспроизведением 1×.

| Метрика | Результат |
| --- | ---: |
| Входные / обработанные сканы | 168 / 168 |
| Частота выходных поз | 13,98 Гц |
| Задержка от публикации до получения позы, p50 / p95 | 6,92 / 14,68 мс |
| Строки траектории CSV | 168 |
| Конечные XYZ-точки в PCD | 14 498 |

Для повтора запустите команду из раздела выше с `-Seconds 12`. Числа задержек зависят от машины.
