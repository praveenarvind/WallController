set(controller_SRC
  WallController.cpp
  utils/PointProjector.cpp
)

set(controller_HDR
  WallController.h
  utils/PointProjector.h
)

# ====== Add StabiliPlus headers ======
include_directories(${CMAKE_SOURCE_DIR}/include)

# ====== Add all StabiliPlus .cpp files ======
file(GLOB stabiliplus_SRC
  ${CMAKE_SOURCE_DIR}/src/polytope/*.cpp
  ${CMAKE_SOURCE_DIR}/src/problemDescriptor/*.cpp
  ${CMAKE_SOURCE_DIR}/src/wrapper/*.cpp
)

# === Add this for GLPK ===

# ====== Append StabiliPlus source files to your controller sources ======
list(APPEND controller_SRC ${stabiliplus_SRC})

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DWallController_EXPORTS")

# === Add this to actually link the GLPK lib ===
target_link_libraries(${PROJECT_NAME} PUBLIC glpk)



