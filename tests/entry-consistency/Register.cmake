# 配置级集成验证：使用真实两个入口和父构建选定的依赖，不编造运行层目标。
get_cmake_property(entry_variables VARIABLES)
set(entry_cache "${CMAKE_CURRENT_BINARY_DIR}/entry-dependencies.cmake")
file(WRITE "${entry_cache}" "")
foreach(variable IN LISTS entry_variables)
    if(variable MATCHES "^FETCHCONTENT_SOURCE_DIR_" OR variable STREQUAL "CMAKE_PREFIX_PATH")
        file(APPEND "${entry_cache}" "set(${variable} [==[${${variable}}]==] CACHE STRING \"\" FORCE)\n")
    endif()
endforeach()
foreach(case models_first full_first identical_copy changed_header changed_build unknown_target incremental_change)
    add_test(NAME entry_${case} COMMAND "${CMAKE_COMMAND}"
        "-DFAMILY_SOURCE=${CMAKE_CURRENT_LIST_DIR}/../.."
        "-DTEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/entry-${case}"
        "-DDEPENDENCY_CACHE=${entry_cache}"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        "-DPLATFORM=${CMAKE_GENERATOR_PLATFORM}"
        "-DCASE=${case}"
        -P "${CMAKE_CURRENT_LIST_DIR}/Check.cmake")
    set_tests_properties(entry_${case} PROPERTIES TIMEOUT 120)
endforeach()
