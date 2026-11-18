function(add_reconstruction_boundary_tests)
    set(reconstruction_root "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
    add_executable(surface_ownership_test "${reconstruction_root}/tests/surface/surface_ownership_test.cpp")
    target_link_libraries(surface_ownership_test PRIVATE GeometricReconstruction::Surfaces)
    set_target_properties(surface_ownership_test PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
    add_test(NAME surface_ownership_test COMMAND surface_ownership_test)

    add_executable(reconstruction_compatibility_test "${reconstruction_root}/tests/surface/compatibility_include_test.cpp")
    target_link_libraries(reconstruction_compatibility_test PRIVATE GeometricReconstruction::GeometricReconstruction)
    set_target_properties(reconstruction_compatibility_test PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
    add_test(NAME reconstruction_compatibility_test COMMAND reconstruction_compatibility_test)
    set_tests_properties(surface_ownership_test reconstruction_compatibility_test PROPERTIES TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${OpenCASCADE_BINARY_DIR}")

    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME reconstruction_boundaries_test COMMAND "${Python3_EXECUTABLE}"
            "${reconstruction_root}/../../tools/tests/test_reconstruction_boundaries.py")
    endif()
endfunction()
