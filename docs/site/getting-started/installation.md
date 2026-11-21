# 安装与构建

## 系统与工具链

当前只支持 Windows x64。所有 C++ 二进制依赖必须使用同一套 **MSVC 2022 + x64 + Release + dynamic CRT**，不能混用 MinGW 构建的 Qt 或 VTK。

| 组件 | 当前要求 | 用途 |
| --- | --- | --- |
| Windows | Windows 10/11 x64 | 客户端与发布包运行环境 |
| Visual Studio | 2022，含「使用 C++ 的桌面开发」 | C++17 编译与 MSVC 运行库 |
| CMake | 3.24 或更新 | 配置、构建、测试与运行库收集 |
| Qt | 6.5 或更新，MSVC 2022 x64 | Widgets 界面、进程与文件操作 |
| VTK | 9.7.0，仓库本地 SDK | 三维显示、相机交互与截图 |
| OpenCASCADE | 8.0.1，仓库本地 MSVC x64 SDK | STEP、B-Rep、布尔运算 |
| vcpkg | 已安装 `cgal:x64-windows`、`eigen3:x64-windows` | 曲面检测与数值计算 |
| Python | 3.x；发布包固定 3.13.10 | 无界面图像重建调度 |
| COLMAP | 4.2.0 | SfM、三角化与稠密重建 |

## 依赖准备

本地固定版本 SDK 放在仓库根的 `deps/` 下，不提交到 Git：

- `deps/vtk-9.7.0`：需包含 `lib/cmake/vtk-9.7/vtk-config.cmake`，并与 Qt 使用相同编译器和运行库；
- `deps/occt-8.0.1`：需包含 `cmake/OpenCASCADEConfig.cmake`；
- `deps/colmap`：完整的 COLMAP 运行时，客户端和 `reconstruct.py` 会按固定路径查找 `bin/colmap.exe`。

CGAL 与 Eigen3 通过 vcpkg 提供，需先把 vcpkg 根目录准备好：

```powershell
vcpkg install cgal:x64-windows eigen3:x64-windows
& .\deps\verify.ps1   # 非递归依赖布局检查
```

## 源码构建

### 桌面客户端

从 `apps/ParamCAD Studio` 目录执行。`CMakeLists.txt` 会自动发现仓库本地的 VTK 和 OpenCASCADE SDK，因此只需显式提供 Qt 包根和 vcpkg 工具链：

```powershell
cmake --preset msvc-debug `
  -DCMAKE_TOOLCHAIN_FILE="D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DCMAKE_PREFIX_PATH="D:/Qt/6.11.2/msvc2022_64"
cmake --build --preset msvc-release
```

`msvc-debug` 是唯一的配置预设，生成器为 Visual Studio 17 2022；`msvc-release` 构建预设在该目录下产出 Release 配置，可执行文件为 `apps/ParamCAD Studio/build/msvc-debug/Release/ParamCAD Studio.exe`。只有该目录会被打包和验证；怀疑缓存过期时删除整个 `build/` 重新配置，不要另建第二棵构建树。需要客户端测试时，用 `-DCAD_CLIENT_BUILD_TESTS=ON` 重新配置。

产品版本统一记录在 `apps/ParamCAD Studio/VERSION`。构建时同时记录 Git 提交、工作区状态和未提交改动指纹；构建后可用同一目录中的 `pcad --version` 或 Studio 的“帮助 → 关于”核对，说明见 [CLI 使用指南](/cli/#版本与构建身份)。

### 核心库独立构建

两个核心库都可脱离客户端单独配置和测试。参数化建模核心只需要 OpenCASCADE：

```powershell
cmake -S core/parametric_modeling -B core/parametric_modeling/build `
  -G "Visual Studio 17 2022" -A x64 `
  -DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
cmake --build core/parametric_modeling/build --config Release
ctest --test-dir core/parametric_modeling/build -C Release --output-on-failure
```

几何重建库还需要 vcpkg 提供的 CGAL 与 Eigen3：

```powershell
cmake -S core/geometric_reconstruction `
  -B core/geometric_reconstruction/build-msvc-cgal `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
cmake --build core/geometric_reconstruction/build-msvc-cgal --config Release
ctest --test-dir core/geometric_reconstruction/build-msvc-cgal `
  -C Release --output-on-failure
```

## 启动与环境检查

构建后直接启动 Release 可执行文件。不要把它复制或移动到其他目录：Qt、VTK、OpenCASCADE 与 MSVC 运行库由 CMake 的 `POST_BUILD` 步骤暂存到同级目录，Windows 优先从可执行文件所在目录解析 DLL。移走 EXE 会因找不到 `vtkInteractionWidgets-9.7.dll` 一类依赖而启动失败。

```powershell
& "./build/msvc-debug/Release/ParamCAD Studio.exe"
& .\tools\install_pcad.ps1   # 或注册统一入口后执行 pcad studio
```

首次构建会把 `apps/ParamCAD Studio/config/` 下的模块超参数复制到可执行文件旁的 `config/`，之后已有的运行配置不会被覆盖。图像重建还需要 Python 和 COLMAP：客户端依次查找 `reconstruction/pythonInterpreter` 设置、`deps/python/python.exe`、PATH 上的 `python`/`python3`/`py`；COLMAP 会查找 `--colmap`、`COLMAP_EXECUTABLE`、PATH 和 `deps/colmap/bin/colmap.exe` 等位置。

### 使用发布包

维护者可在仓库根目录生成完整离线包：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\package-client.ps1 `
  -QtRoot "D:\Qt\6.11.2\msvc2022_64" -VcpkgRoot "D:\dev\vcpkg"
```

产物位于 `artifacts/releases/<版本>/`，包含 ZIP 和 `SHA256SUMS.txt`。完整解压后双击 `Start-ParamCADStudio.cmd`，或直接运行 `app/ParamCAD Studio.exe`；不要只复制 EXE。可选运行 `Install-ParamCADStudio.cmd` 离线安装到 `%LOCALAPPDATA%\Programs\ParamCAD Studio`，无需管理员权限，升级会保留已有 `outputs/`。需要 CPU-only 的 COLMAP 时传 `-ColmapFlavor nocuda`。包内 CUDA 版 COLMAP 的稠密重建仍要求兼容的 NVIDIA GPU 和驱动，安装包不包含硬件驱动。
