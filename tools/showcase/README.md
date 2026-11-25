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

For showcase clouds sampled directly from STEP, use a `0.5 mm` surface spacing
and enable the optional edge-fidelity samples. These topology-informed edge
samples are intended to help the showcase reconstruction produce cleaner
display geometry. Do not enable them for correctness validation.

```powershell
$validator = "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe"
$source = "data/performance-validation/level-01/box-through-hole/model.step"
$output = "outputs/performance-validation-showcase-dense/level-01/box-through-hole"
New-Item -ItemType Directory -Path $output -Force | Out-Null
& $validator sample $source "$output/sampling.json" "$output/cloud.ply" 0.5 --preserve-edges
```

Run online cloud loading, surface detection, CAD reconstruction and, after a successful reconstruction, STEP export while capturing each supported stage:

```powershell
python tools/showcase/capture_cloud_stages.py --cloud outputs/performance-validation-reference/level-01/sphere/cloud.ply --output outputs/performance-showcase/level-01/sphere --session SESSION_ID
```

To acquire the point cloud through the client's default structured-light workflow before the analysis stages, pass the source STEP model instead. The resulting structured-light cloud and command response are kept under the case output directory:

```powershell
python tools/showcase/capture_cloud_stages.py --model data/performance-validation/level-01/cylinder/model.step --output outputs/performance-showcase/level-01/cylinder --session SESSION_ID
```

Use `--allow-missing-solid` to keep the acquisition and surface clips when solid reconstruction fails. The reconstruction report is retained and the solid stage remains blank.

The orbit camera stays 45 degrees above the XY base plane and completes one turn around the model center. The STEP capture output contains Studio's multi-angle capture under `capture/images/`, the orbit PNGs under `frames/` and `spin.mp4`. The default of 96 poses at 12 fps yields an eight-second revolution. Use `--frames` and `--fps` to control the number of poses and playback speed. Use `--elevation`, `--width` and `--height` to control the view and capture size. Near-square captures make better use of the available width in levels with few parts.

`capture_cloud_stages.py` writes the point-cloud and detected-surface videos directly. It also writes an exported STEP and CAD video when solid reconstruction succeeds. Then build a level page and synchronized four-column video:

```powershell
python tools/showcase/build_performance_showcase.py --level level-01
```

For a tightly joined Level 1 presentation with a separate title band:

```powershell
python tools/showcase/build_performance_showcase.py --level level-01 --presentation
```

Build and concatenate multiple levels in playback order:

```powershell
python tools/showcase/build_performance_showcase.py --level level-01 --level level-02 --combined outputs/performance-showcase/levels-01-02.mp4
```

The result is `outputs/performance-showcase/<level>/index.html` and `showcase.mp4`. The page and video include every STEP/STP case discovered under the level. Cells without a supplied stage video are marked as missing in HTML and rendered blank in the MP4.
