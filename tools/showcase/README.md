# ParamCAD showcase videos

These scripts use the existing `pcad capture views` Studio command. Start ParamCAD Studio before capturing. `ffmpeg` must be available on `PATH`.

`pcad capture frame` captures the current Studio display directly to a PNG path without opening a save dialog. `--target cloud|surfaces` shows the cloud and detected-surface presentation while hiding the solid model. `--target model|cad` shows the solid and hides the point cloud. Camera pose and visibility are restored after each frame.

Capture one source STEP model at a fixed orbit elevation:

```powershell
python tools/showcase/capture_step_spin.py --model data/performance-validation/level-01/box-through-hole/model.step --output outputs/performance-showcase/level-01/box-through-hole/step
```

Capture every STEP/STP source found directly in a level's case folders, including one nested folder such as `level-06/local/Y373`. Use `--width` and `--height` to choose the normalized capture frame size:

```powershell
python tools/showcase/capture_level_steps.py --level level-01
```

When multiple Studio processes are running, select one with `--session SESSION_ID`.

Run online cloud loading, surface detection, CAD reconstruction and, after a successful reconstruction, STEP export while capturing each supported stage:

```powershell
python tools/showcase/capture_cloud_stages.py --cloud outputs/performance-validation-reference/level-01/sphere/cloud.ply --output outputs/performance-showcase/level-01/sphere --session SESSION_ID
```

The STEP capture output contains the Studio's original 62-frame capture under `capture/images/`, plus 12 direct viewport PNGs and `spin.mp4` at elevation `-30` degrees. Change the camera angle, frame size or playback rate with `--elevation`, `--width`, `--height` and `--fps`.

`capture_cloud_stages.py` writes the point-cloud and detected-surface videos directly. It also writes an exported STEP and CAD video when solid reconstruction succeeds. Then build a level page and synchronized four-column video:

```powershell
python tools/showcase/build_performance_showcase.py --level level-01
```

The result is `outputs/performance-showcase/<level>/index.html` and `showcase.mp4`. The page and video include every STEP/STP case discovered under the level. Cells without a supplied stage video are marked as missing in HTML and rendered blank in the MP4.
