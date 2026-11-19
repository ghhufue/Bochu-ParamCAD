cmake_minimum_required(VERSION 3.24)

file(MAKE_DIRECTORY "${OUTPUT_DIRECTORY}")
file(LOCK "${OUTPUT_DIRECTORY}/build-info.lock" TIMEOUT 30)
set(GIT_COMMIT "")
set(SOURCE_STATE "unknown")
set(WORKTREE_FINGERPRINT "")
file(REAL_PATH "${REPOSITORY_ROOT}" root)

if(GIT_EXECUTABLE)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${root}" rev-parse --show-toplevel
        RESULT_VARIABLE root_result OUTPUT_VARIABLE git_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(root_result STREQUAL "0")
        file(REAL_PATH "${git_root}" git_root)
    endif()
    if(root_result STREQUAL "0" AND git_root STREQUAL root)
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${root}" rev-parse --verify HEAD
            RESULT_VARIABLE head_result OUTPUT_VARIABLE head OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(head_result STREQUAL "0" AND head MATCHES "^[0-9a-f]+$")
            set(GIT_COMMIT "${head}")
            set(paths .
                ":(exclude)docs/Outdated"
                ":(exclude)deps/colmap"
                ":(exclude)deps/vtk-9.7.0"
                ":(exclude)deps/occt-8.0.1")
            set(diff_path "${OUTPUT_DIRECTORY}/worktree.diff")
            execute_process(
                COMMAND "${GIT_EXECUTABLE}" -C "${root}" diff --binary --full-index
                    --no-ext-diff --no-textconv --no-renames --ignore-submodules=dirty "${GIT_COMMIT}" -- ${paths}
                RESULT_VARIABLE diff_result OUTPUT_FILE "${diff_path}" ERROR_QUIET)
            execute_process(
                COMMAND "${GIT_EXECUTABLE}" -C "${root}" -c core.quotePath=false
                    ls-files --others --exclude-standard -- ${paths}
                RESULT_VARIABLE untracked_result OUTPUT_VARIABLE untracked ERROR_QUIET ENCODING UTF-8)
            if(diff_result STREQUAL "0" AND untracked_result STREQUAL "0")
                file(SIZE "${diff_path}" tracked_size)
                set(SOURCE_STATE "clean")
                if(tracked_size GREATER 0 OR NOT untracked STREQUAL "")
                    set(SOURCE_STATE "dirty")
                    file(SHA256 "${diff_path}" fingerprint_input)
                    string(REPLACE "\r\n" "\n" untracked "${untracked}")
                    string(REGEX REPLACE "\n$" "" untracked "${untracked}")
                    string(REPLACE ";" "\\;" untracked "${untracked}")
                    string(REPLACE "\n" ";" untracked "${untracked}")
                    set(hash_complete TRUE)
                    foreach(path IN LISTS untracked)
                        set(source "${root}/${path}")
                        if(IS_SYMLINK "${source}")
                            file(READ_SYMLINK "${source}" link_target)
                            string(SHA256 file_hash "symlink:${link_target}")
                        elseif(EXISTS "${source}" AND NOT IS_DIRECTORY "${source}")
                            file(SHA256 "${source}" file_hash)
                        else()
                            set(hash_complete FALSE)
                            break()
                        endif()
                        string(APPEND fingerprint_input "\n${path}\n${file_hash}")
                    endforeach()
                    if(hash_complete)
                        string(SHA256 WORKTREE_FINGERPRINT "${fingerprint_input}")
                    endif()
                endif()
            endif()
            file(REMOVE "${diff_path}")
        endif()
    endif()
endif()

set(template "${CMAKE_CURRENT_LIST_DIR}/ParamCADBuildInfo.h.in")
set(header "${OUTPUT_DIRECTORY}/ParamCADBuildInfo.h")
set(identity_file "${OUTPUT_DIRECTORY}/build-info.sha256")
file(SHA256 "${template}" template_hash)
string(SHA256 identity "${template_hash}|${PRODUCT_VERSION}|${BUILD_CONFIGURATION}|${GIT_COMMIT}|${SOURCE_STATE}|${WORKTREE_FINGERPRINT}")
if(EXISTS "${identity_file}" AND EXISTS "${header}")
    file(READ "${identity_file}" previous_identity)
    if(previous_identity STREQUAL identity)
        return()
    endif()
endif()

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%dT%H:%M:%SZ" UTC)
configure_file("${template}" "${header}" @ONLY)
file(WRITE "${identity_file}" "${identity}")
