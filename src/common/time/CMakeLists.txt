add_library(libtime
    src/tic_toc.cpp
)

add_library(common::libtime ALIAS libtime)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_include_directories(libtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(libtime
    
)

install(TARGETS libtime
    EXPORT export_time
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

# ament_export_dependencies
ament_export_targets(export_time HAS_LIBRARY_TARGET)
