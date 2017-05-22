add_library(constexprstring INTERFACE)
target_include_directories(constexprstring INTERFACE include/)
target_link_libraries(constexprstring INTERFACE boost)
target_sources(
    constexprstring INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constexprstring/constexpr_string.h
)