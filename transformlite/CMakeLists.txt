add_library(transformlite INTERFACE)
target_include_directories(transformlite INTERFACE include/)
target_link_libraries(transformlite INTERFACE boost constexprstring)
target_sources(
    transformlite INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/transformlite/transform_lite.h
)