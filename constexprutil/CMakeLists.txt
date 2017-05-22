find_package(Eigen3)

add_library(constexprutil INTERFACE)
target_include_directories(constexprutil INTERFACE include/)
target_link_libraries(constexprutil INTERFACE )
target_sources(
    constexprutil INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constexprutil/math/trigonometry.h
)

add_subdirectory(examples)
add_subdirectory(tests)