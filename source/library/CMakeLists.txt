project("SoftBodyLib")

set(${PROJECT_NAME}_headers
    dll_main.h
    ForwardDeclarations.h
    stdafx.h
)

set(${PROJECT_NAME}_sources
    dll_main.cpp
)

add_subdirectory(physics)
add_subdirectory(utils)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME} Utils Physics)
