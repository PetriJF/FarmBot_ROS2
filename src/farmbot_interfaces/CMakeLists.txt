cmake_minimum_required(VERSION 3.8)
project(farmbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FBPanel.msg"
  "msg/ParameterList.msg"
  "msg/GantryCommand.msg"
  "msg/ServoCommand.msg"
  "msg/HomeCommand.msg"
  "msg/I2CCommand.msg"
  "msg/PinCommand.msg"
  "msg/ParameterCommand.msg"
  "msg/StateCommand.msg"
  "msg/StatusCommand.msg"
  "msg/PlantManage.msg"
  "msg/MapCommand.msg"
  "srv/LedPanelHandler.srv"
  "srv/ParameterConfig.srv"
  "srv/LoadParamConfig.srv"
  "srv/StringRepReq.srv"
  "action/GetUARTResponse.action"
)

ament_package()
