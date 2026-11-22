cmake_minimum_required(VERSION 3.24)

project(GeometricReconstruction VERSION 0.1.0 LANGUAGES CXX)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

option(GEOMETRIC_RECONSTRUCTION_BUILD_TESTS
    "Build geometric reconstruction tests" ON)

find_package(OpenCASCADE REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Threads REQUIRED)

if(NOT TARGET ContourFitting::ContourFitting)
    add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/contour_fitting"
        "${CMAKE_CURRENT_BINARY_DIR}/contour_fitting"
    )
endif()

if(NOT TARGET CadModelCore::CadModelCore)
    set(CAD_MODEL_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/../parametric_modeling"
        "${CMAKE_CURRENT_BINARY_DIR}/cad_model_core"
    )
endif()

include(cmake/ReconstructionModules.cmake)

function(configure_reconstruction_library target)
    target_compile_features(${target} PUBLIC cxx_std_17)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    # The core libraries never use Qt code generation, even in a Qt host project.
    set_target_properties(${target} PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_library(ReconstructionData STATIC ${RECONSTRUCTION_DATA_SOURCES})
add_library(GeometricReconstruction::Data ALIAS ReconstructionData)
configure_reconstruction_library(ReconstructionData)
target_link_libraries(ReconstructionData PUBLIC CadModelCore::GeometryTypes)

add_library(ReconstructionInput STATIC ${RECONSTRUCTION_INPUT_SOURCES})
add_library(GeometricReconstruction::Input ALIAS ReconstructionInput)
configure_reconstruction_library(ReconstructionInput)
target_link_libraries(ReconstructionInput PUBLIC ReconstructionData PRIVATE Eigen3::Eigen Threads::Threads)

add_library(ReconstructionSegmentation STATIC ${RECONSTRUCTION_SEGMENTATION_SOURCES})
add_library(GeometricReconstruction::Segmentation ALIAS ReconstructionSegmentation)
configure_reconstruction_library(ReconstructionSegmentation)
target_link_libraries(ReconstructionSegmentation PUBLIC ReconstructionData PRIVATE CGAL::CGAL Eigen3::Eigen Threads::Threads)

add_library(ReconstructionSurfaces STATIC ${RECONSTRUCTION_SURFACES_SOURCES})
add_library(GeometricReconstruction::Surfaces ALIAS ReconstructionSurfaces)
configure_reconstruction_library(ReconstructionSurfaces)
target_link_libraries(ReconstructionSurfaces PUBLIC ReconstructionSegmentation ReconstructionInput
    PRIVATE ContourFitting::ContourFitting CGAL::CGAL Eigen3::Eigen Threads::Threads)

# CAD-backed feature/part recognition and workflow orchestration live above the
# numerical libraries. Existing clients retain the same aggregate target name.
add_library(GeometricReconstruction ${RECONSTRUCTION_WORKFLOW_SOURCES})
add_library(GeometricReconstruction::GeometricReconstruction ALIAS GeometricReconstruction)
configure_reconstruction_library(GeometricReconstruction)
target_compile_definitions(GeometricReconstruction PRIVATE
    PARAMCAD_CONFIG_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../apps/ParamCAD Studio/config")
target_link_libraries(GeometricReconstruction PUBLIC ReconstructionSurfaces CadModelCore::CadModelCore
    PRIVATE ContourFitting::ContourFitting CGAL::CGAL Eigen3::Eigen Threads::Threads)

if(GEOMETRIC_RECONSTRUCTION_BUILD_TESTS)
    include(CTest)
    include(cmake/BoundaryTests.cmake)
    add_reconstruction_boundary_tests()
    add_executable(surface_support_validation_test tests/analysis/surface_support_validation_test.cpp)
    target_link_libraries(surface_support_validation_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    add_test(NAME surface_support_validation_test COMMAND surface_support_validation_test)
    add_executable(projection_stage_test tests/projection_stage_test.cpp)
    target_include_directories(projection_stage_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(projection_stage_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    add_test(NAME projection_stage_test COMMAND projection_stage_test)
    set_tests_properties(projection_stage_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}")
    add_executable(module_settings_test tests/settings/module_settings_test.cpp)
    target_include_directories(module_settings_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(module_settings_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    target_compile_definitions(module_settings_test PRIVATE
        RECONSTRUCTION_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../apps/ParamCAD Studio/config")
    add_test(NAME module_settings_test COMMAND module_settings_test)
    set_tests_properties(module_settings_test PROPERTIES TIMEOUT 60
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}")
    add_executable(assembly_reconstruction_test tests/assembly_reconstruction_test.cpp)
    target_include_directories(assembly_reconstruction_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(assembly_reconstruction_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    add_test(NAME assembly_reconstruction_test COMMAND assembly_reconstruction_test)
    set_tests_properties(assembly_reconstruction_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}")
    add_executable(surface_analysis_test tests/analysis/surface_analysis_test.cpp)
    target_include_directories(surface_analysis_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(surface_analysis_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    add_test(NAME surface_analysis_test COMMAND surface_analysis_test)
    set_tests_properties(surface_analysis_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}")
    add_executable(surface_scene_test tests/surface_scene_test.cpp)
    target_link_libraries(surface_scene_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    add_test(NAME surface_scene_test COMMAND surface_scene_test)
    set_tests_properties(surface_scene_test PROPERTIES
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}")
    include(cmake/RegressionSources.cmake)
    add_executable(geometric_reconstruction_test ${GEOMETRIC_RECONSTRUCTION_REGRESSION_SOURCES})
    target_link_libraries(geometric_reconstruction_test PRIVATE
        GeometricReconstruction::GeometricReconstruction)
    target_compile_definitions(geometric_reconstruction_test PRIVATE
        RECONSTRUCTION_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-output"
    )
    if(MSVC)
        target_compile_options(geometric_reconstruction_test PRIVATE
            /W4 /permissive- /utf-8)
    else()
        target_compile_options(geometric_reconstruction_test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME geometric_reconstruction_test
        COMMAND geometric_reconstruction_test)
    set_tests_properties(geometric_reconstruction_test PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}"
    )
endif()
