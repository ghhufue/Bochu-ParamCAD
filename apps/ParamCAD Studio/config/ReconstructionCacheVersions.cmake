# Hash numerical implementations by stage so an unrelated client rebuild can
# still consume earlier-stage artifacts. CONFIGURE_DEPENDS also covers new files.
function(paramcad_cache_version variable)
    set(files)
    foreach(pattern IN LISTS ARGN)
        file(GLOB_RECURSE matched CONFIGURE_DEPENDS RELATIVE "${PARAMCAD_REPOSITORY_ROOT}"
            "${PARAMCAD_REPOSITORY_ROOT}/${pattern}")
        list(APPEND files ${matched})
    endforeach()
    list(REMOVE_DUPLICATES files)
    list(SORT files)
    set(identity "${CMAKE_CXX_COMPILER_ID}|${CMAKE_CXX_COMPILER_VERSION}|${CMAKE_SYSTEM_PROCESSOR}")
    foreach(file IN LISTS files)
        set(path "${PARAMCAD_REPOSITORY_ROOT}/${file}")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${path}")
        file(SHA256 "${path}" sha)
        string(APPEND identity "|${file}=${sha}")
    endforeach()
    string(SHA256 hash "${identity}")
    set(${variable} "${hash}" PARENT_SCOPE)
endfunction()

set(PARAMCAD_REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(reconstruction "core/geometric_reconstruction")
set(modeling "core/parametric_modeling")
set(app "apps/ParamCAD Studio")
set(common "${reconstruction}/src/common/*.h" "${reconstruction}/src/data/PointStore.cpp"
    "${reconstruction}/include/reconstruction/data/*.h")
set(input "${reconstruction}/src/preprocessing/*.cpp" "${reconstruction}/src/preprocessing/*.h"
    "${reconstruction}/include/reconstruction/preprocessing/*.h")
set(detection "${reconstruction}/src/segmentation/*.cpp" "${reconstruction}/src/segmentation/*.h"
    "${reconstruction}/include/reconstruction/segmentation/*.h"
    "${reconstruction}/src/surface/expansion/*.cpp"
    "${reconstruction}/src/surface/SurfaceAnalysis.cpp")
set(surfaces "${reconstruction}/src/surface/*.cpp" "${reconstruction}/src/surface/*.h"
    "${reconstruction}/include/reconstruction/surface/*.h"
    "${reconstruction}/include/reconstruction/settings/SurfaceMeshingSettings.h")
set(recognition "${reconstruction}/src/recognition/*.cpp" "${reconstruction}/src/recognition/*.h"
    "${reconstruction}/src/pipeline/*.cpp" "${reconstruction}/src/pipeline/*.h"
    "${reconstruction}/include/reconstruction/pipeline/*.h"
    "${reconstruction}/include/reconstruction/recognition/*.h")
set(cad "${modeling}/src/*.cpp" "${modeling}/src/*.h" "${modeling}/include/modeling/*.h"
    "${reconstruction}/src/modeling/*.cpp" "${reconstruction}/include/reconstruction/modeling/*.h")
set(assembly "${reconstruction}/src/parts/assembly")
set(assemblyApi "${reconstruction}/include/reconstruction/parts/AssemblyReconstruction.h")
set(projection "${reconstruction}/src/parts/projection/*.cpp" "${reconstruction}/src/parts/projection/*.h"
    "${reconstruction}/include/reconstruction/parts/ProjectionExtrusion.h"
    "${reconstruction}/src/contour_fitting/*.cpp" "${reconstruction}/src/contour_fitting/*.h")

paramcad_cache_version(CACHE_PREPROCESSING ${common} ${input})
paramcad_cache_version(CACHE_EVIDENCE ${common} ${detection})
paramcad_cache_version(CACHE_ANALYSIS ${common} ${detection} ${surfaces})
paramcad_cache_version(CACHE_RECOGNITION ${common} ${input} ${detection} ${surfaces} ${recognition} ${cad})
paramcad_cache_version(CACHE_ASSEMBLY_GROUPING ${common} ${input} "${assembly}/AssemblyReconstruction.cpp"
    "${assembly}/AssemblyInternal.h")
paramcad_cache_version(CACHE_ASSEMBLY_PREPARATION ${common} "${assembly}/AssemblyReconstruction.cpp"
    "${assembly}/BoundaryGeometry.*" "${assembly}/FiniteBoundaries.cpp")
paramcad_cache_version(CACHE_ASSEMBLY_CLOSURE ${assemblyApi} "${assembly}/*.cpp" "${assembly}/*.h"
    ${common} ${input} ${detection} ${surfaces} ${recognition} ${cad})
paramcad_cache_version(CACHE_PROJECTION ${common} ${input} ${detection} ${projection} ${cad}
    "${assembly}/AssemblyReconstruction.cpp")
paramcad_cache_version(CACHE_PRESENTATION ${common} ${input} ${detection} ${surfaces}
    "${app}/src/io/PointCloudAnalysis.cpp" "${app}/src/io/PointCloudAdapter.cpp"
    "${app}/src/app/PointCloudWorkspaceController.cpp" "${app}/src/core/PointCloudRelations.cpp"
    "${app}/src/core/PointCloudSelection.cpp")

configure_file("${CMAKE_CURRENT_LIST_DIR}/ReconstructionCacheVersions.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/ReconstructionCacheVersions.h" @ONLY)
