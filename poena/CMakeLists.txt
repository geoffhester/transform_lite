add_library(poena INTERFACE)
target_include_directories(
    poena INTERFACE
    include/
    #include/poena/graph/
)
target_link_libraries(poena INTERFACE boost)
target_sources(
    poena INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/poena/poena.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/poena/graph/graph.h
)

add_subdirectory(examples)