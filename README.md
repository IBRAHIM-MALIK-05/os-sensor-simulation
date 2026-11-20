# OS Project – Sensor Simulation

A simulated self-driving car with three sensors (**LIDAR**, **Camera**, **GPS**) that all look at the same road.
This is stage 1 of the "Multithreaded Producer-Consumer Synchronization" project. It produces the sensor data
that the OS part (shared buffer, priority inheritance, paging, scheduling) will later work with.

## Run it

```powershell
.\build.ps1                     # builds sensor_sim.exe (needs g++)
python tools\dashboard.py       # live dashboard in the browser: http://localhost:8765
```
Or run it without the browser: `.\sensor_sim.exe`. It prints one status line per second, then a summary.

If PowerShell blocks the script, use `powershell -ExecutionPolicy Bypass -File .\build.ps1`.

## The three sensors

| Sensor | Priority (planned) | Rate | One reading | Size |
|---|---|---|---|---|
| LIDAR | high | 10 / s | 720 distances around the car (every 0.5°, up to 60 m) | ~2.9 KB |
| Camera | medium | 10 / s | a 640×360 picture of the road ahead | ~675 KB |
| GPS | low | 5 / s | latitude, longitude (±1.5 m noise), speed, heading | 64 B |

The camera produces ~6.6 MB/s, so a 10 MB buffer holds only ~1.5 s of data.

## How the code is organised

```
src/
  main.cpp                    the timeline: asks each sensor for a reading when it is due,
                              combines the newest readings, sends them to the outputs
  world.h / world.cpp         the road, the obstacles and how the car moves
  sensors/
    lidar.h / lidar.cpp       LIDAR:  scanLidar(world, time, scanNo)       -> LidarScan
    camera.h / camera.cpp     Camera: camera.capture(world, time, frameNo) -> CameraFrame
    camera_render.*           draws the camera picture (road, cars, people, ...)
    gps.h / gps.cpp           GPS:    gps.read(world, time, fixNo)         -> GpsFix
  fusion.h / fusion.cpp       combines the readings and decides CRUISE / SLOW / BRAKE
  options.h / options.cpp     command-line options
  output/
    disk_recorder.*           saves the data to sim_output\
    dashboard_stream.*        sends the data to the dashboard
tools/
  dashboard.py                starts sensor_sim.exe and serves the web page
  dashboard.html              the web page
assets/camera/                (optional) your own photos for the camera
```

### Where the data goes
```
sensors/  ──►  main.cpp keeps the newest reading of each sensor (FusedState latest)
               ──►  fusion: decide()  ──►  output/disk_recorder     ──►  sim_output\*.csv
                                       └►  output/dashboard_stream  ──►  dashboard.py ──► browser
```

**Next stage (the OS part):**
- **Producers:** each sensor call in `main.cpp` (`scanLidar`, `camera.capture`, `gps.read`) moves into its
  own producer thread, which puts its reading into a shared 10 MB buffer.
- **Consumer:** the combining step (`decide()` and saving the combined row) becomes the consumer thread (the
  AI processor), which takes readings out of that buffer.

The sensor functions only depend on the time and the world, so they can be called from any thread.

## Output files (`sim_output\`)

| File | Contents |
|---|---|
| `lidar.csv` | one row per scan: time, car position, nearest obstacle in front, then `r0..r719` (all 720 distances) |
| `camera.csv` | one row per picture: time, image name, size, what it sees (type, distance, angle) |
| `gps.csv` | one row per fix: time, lat, lon, speed, heading, accuracy |
| `combined.csv` | one row per LIDAR scan: the newest reading of all three sensors + the decision |
| `camera\` | every 10th picture (every 50th when using the dashboard) |

The files are overwritten every run. In `combined.csv`, `lidar_nearest_m` and `camera_distance_m` agree
within 1 m in about 96% of rows. The rest are objects at the edge of the camera's view.

## Options

`sensor_sim.exe`:

| Option | Meaning | Default |
|---|---|---|
| `--duration SEC` | simulated seconds (0 = until stopped) | 30 |
| `--speed X` | 1 = real time, 2 = twice as fast, 0 = as fast as possible | 1 |
| `--save-images N` | save every Nth camera picture (0 = none) | 10 |
| `--no-csv` | don't write the CSV files | |
| `--photos DIR` | folder with your own camera photos | `assets/camera` |
| `--out DIR` | output folder | `sim_output` |
| `--seed N` | different obstacle layout | 42 |
| `--dashboard` | send live data for `tools/dashboard.py` (used by the dashboard itself) | |

`tools\dashboard.py` takes these options:
- `--speed`, `--duration`, `--photos` and `--seed`, as above. The default duration is to run until Ctrl+C.
- `--port N`: the web page's port.
- `--no-browser`: don't open the browser automatically.
- `--no-save`: don't save the data to `sim_output\`.

## Using your own photos
Put `.jpg`, `.png` or `.bmp` files in `assets\camera\`. The camera then sends your photos instead of drawn
pictures. If a photo's file name contains an obstacle type (`car`, `truck`, `pedestrian`, `cyclist`, `cone`
or `sign`, e.g. `truck_1.jpg`), it is used whenever the camera sees that type.
