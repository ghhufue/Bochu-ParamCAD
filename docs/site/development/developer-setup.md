# 开发环境与代码布局

本页面向需要编译和调试仓库的开发者。当前只支持 Windows x64，客户端和两个核心库都使用 C++17；图像重建是独立的 Python 流程。构建前请先准备仓库本地的 VTK、OpenCASCADE、COLMAP 运行时以及 vcpkg 提供的 CGAL、Eigen3。

## 仓库布局

| 目录 | 职责 |
| --- | --- |
| `apps/ParamCAD Studio` | Qt 6 Widgets 桌面客户端，负责交互、流程编排、文件读写、VTK 显示和后台任务。 |
| `core/reconstruction` | 无界面 Python 调度层，调用 COLMAP 把图片转换为稀疏或稠密 PLY。 |
| `core/geometric_reconstruction` | 点云预处理、CGAL 曲面证据、面片与关系、实体识别和候选生成，不依赖 Qt/VTK。 |
| `core/parametric_modeling` | 参数化特征历史、建模命令、原子建模补丁、OpenCASCADE B-Rep 重建和 STEP 导出。 |
| `data` | 示例数据、STEP 工件和性能验证样例。 |
| `docs` | 当前模块契约、VitePress 站点源目录 `docs/site` 和站点配置 `docs/.vitepress`。 |
| `outputs` | 运行输出目录，存放拍摄、重建、截图和验证结果，不提交生成物。 |
| `tools` | 统一入口 `pcad`、批量重建、验证、诊断、夹具和 Python 回归测试。 |
| `packaging` | Windows 完整离线 Release 的构建、校验、压缩和安装脚本。 |
| `deps` | 固定版本的本地二进制 SDK 与运行时，不提交到 Git，也不递归扫描。 |

客户端 `src/` 内部按职责分为 `app`（工作区与命令控制器）、`ui`（面板和视口交互）、`io`（STEP、PLY、工程文件适配）、`rendering`（几何到 VTK actor）和 `core`（场景、相机与草图坐标）。

## 开发依赖与构建目标

| 依赖 | 当前要求 | 说明 |
| --- | --- | --- |
| Visual Studio | 2022，含「使用 C++ 的桌面开发」 | 必须使用 MSVC 2022 x64，不能混用 MinGW。 |
| CMake | 3.24 或更新 | 客户端与核心库统一使用。 |
| Qt | 6.5 或更新，MSVC 2022 x64 | 客户端 UI、进程管理和文件处理。 |
| VTK | 9.7.0，仓库本地 SDK | 位于 `deps/vtk-9.7.0`，需包含 `GUISupportQt`。 |
| OpenCASCADE | 8.0.1，MSVC x64 SDK | 位于 `deps/occt-8.0.1`，提供 STEP、B-Rep 与布尔运算。 |
| vcpkg | 已安装 `cgal:x64-windows`、`eigen3:x64-windows` | 几何重建的曲面检测与数值计算。 |
| Python / COLMAP | Python 3.x；COLMAP 4.2.0 | 仅图像重建需要，位于 `core/reconstruction` 与 `deps/colmap`。 |

从 `apps/ParamCAD Studio` 目录配置并构建 Release。`CMakeLists.txt` 会自动发现仓库本地 VTK 和 OpenCASCADE，只需提供 Qt 包根与 vcpkg 工具链：

```powershell
cmake --preset msvc-debug `
  -DCMAKE_TOOLCHAIN_FILE="D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DCMAKE_PREFIX_PATH="D:/Qt/6.11.2/msvc2022_64"
cmake --build --preset msvc-release
```

`msvc-debug` 是唯一的配置预设，`msvc-release` 在其 `build/msvc-debug` 目录下产出 Release。可执行文件为 `apps/ParamCAD Studio/build/msvc-debug/Release/ParamCAD Studio.exe`，只有这棵构建树参与打包；缓存可疑时删除整个 `build/` 重新配置，不要另建第二棵构建树。

客户端构建开关如下：

| 选项 | 默认值 | 作用 |
| --- | --- | --- |
| `CAD_CLIENT_BUILD_TESTS` | `OFF` | 构建客户端 I/O、交互与任务回归测试。 |
| `CAD_CLIENT_BUILD_VALIDATION_TOOLS` | `OFF` | 构建 `cad_reconstruction_validation` 等验证程序。 |
| `CAD_CLIENT_BUILD_MODULE_CLIS` | `ON` | 构建统一 `pcad.exe`，包含六个算法模块、在线分派、离线 CAD 重建和导出包查询。Studio 构建会同步更新它。 |

两个核心库都能脱离客户端独立配置和测试。客户端通过 `add_subdirectory` 引入它们时会强制关闭核心测试目标，因此单独验证核心库需要直接配置其源目录：

```powershell
cmake -S core/parametric_modeling -B core/parametric_modeling/build `
  -G "Visual Studio 17 2022" -A x64 `
  -DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
cmake --build core/parametric_modeling/build --config Release
ctest --test-dir core/parametric_modeling/build -C Release --output-on-failure

cmake -S core/geometric_reconstruction -B core/geometric_reconstruction/build-msvc-cgal `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
cmake --build core/geometric_reconstruction/build-msvc-cgal --config Release
ctest --test-dir core/geometric_reconstruction/build-msvc-cgal -C Release --output-on-failure
```

## 本地测试与调试

客户端开启 `CAD_CLIENT_BUILD_TESTS=ON` 后，可以用 CTest 运行全部已注册用例，测试程序输出到 `build/msvc-debug/Release/`：

```powershell
ctest --test-dir "apps/ParamCAD Studio/build/msvc-debug" -C Release --output-on-failure
```

重点可执行文件包括 `workspace_interaction_test`、`point_cloud_interaction_test`、`point_cloud_document_test`、`cad_reconstruction_task_test`、`project_store_test`、`project_interaction_test`、`step_model_exporter_test` 和 `module_settings_test`。这些程序同样依赖同目录的 Qt、VTK、OpenCASCADE 与 MSVC 运行库，应从 `Release/` 目录直接启动。

Python 侧分别运行图像重建和工具测试：

```powershell
$env:PYTHONPATH = ".\core\reconstruction"
python -m unittest discover -s .\core\reconstruction\tests -v
python -m unittest discover -s tools\tests -p test_pcad_cli.py -v
```

统一工具入口 `pcad` 使用 Python 3.10 或更新版本，仅依赖标准库；在仓库根目录运行 `& .\tools\install_pcad.ps1` 注册当前用户 PATH 后可使用：

```powershell
pcad studio
pcad clean output --dry-run
pcad clean output
```

常见问题：

- **启动时报缺少 DLL**：Qt、VTK、OpenCASCADE 与 MSVC 运行库由 `POST_BUILD` 暂存到 EXE 同级目录，不要把 `ParamCAD Studio.exe` 复制到别处。
- **改了 `.conf` 但程序仍用旧值**：构建目录中的配置只在文件不存在时复制，避免覆盖用户运行期修改；需删除 `build/.../Release/config/` 或手动复制源码配置。
