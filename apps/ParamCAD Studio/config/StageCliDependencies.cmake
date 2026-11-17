# Inspect only the PE import closure of the built CLI, never glob SDK trees.
file(GET_RUNTIME_DEPENDENCIES
    EXECUTABLES "${PCAD_EXECUTABLE}"
    DIRECTORIES "${PCAD_DIRECTORY}" "${PCAD_QT_BIN}" "${PCAD_OCCT_BIN}" "${PCAD_VTK_BIN}" "${PCAD_VCPKG_BIN}"
    RESOLVED_DEPENDENCIES_VAR resolved
    UNRESOLVED_DEPENDENCIES_VAR unresolved
    PRE_EXCLUDE_REGEXES "api-ms-.*" "ext-ms-.*" "[Hh][Vv][Ss][Ii][Ff][Ii][Ll][Ee][Tt][Rr][Uu][Ss][Tt][.]dll"
    POST_EXCLUDE_REGEXES ".*[Ss][Yy][Ss][Tt][Ee][Mm]32.*" ".*[Ww][Ii][Nn][Ss][Xx][Ss].*")
if(unresolved)
    message(FATAL_ERROR "Unresolved pcad runtime dependencies: ${unresolved}")
endif()
foreach(dependency IN LISTS resolved)
    get_filename_component(destination "${PCAD_DIRECTORY}" ABSOLUTE)
    get_filename_component(source_directory "${dependency}" DIRECTORY)
    if(NOT source_directory STREQUAL destination)
        get_filename_component(filename "${dependency}" NAME)
        file(COPY_FILE "${dependency}" "${destination}/${filename}" ONLY_IF_DIFFERENT)
    endif()
endforeach()
