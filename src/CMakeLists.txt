set(controller_SRC
  WallController.cpp
)

set(controller_HDR
  WallController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DWallController_EXPORTS")

