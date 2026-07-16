cmake_minimum_required(VERSION 3.8)
project(farmbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
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
  "msg/ImageMessage.msg"
  "srv/ConfigurePin.srv"
  "srv/LedPanelHandler.srv"
  "srv/MoveServo.srv"
  "srv/ParameterConfig.srv"
  "srv/ReadI2C.srv"
  "srv/ReadParameter.srv"
  "srv/ReadPin.srv"
  "srv/SetI2C.srv"
  "srv/StringRepReq.srv"
  "srv/Watering.srv"
  "srv/WriteParameter.srv"
  "srv/WritePin.srv"
  "action/FarmbotComms.action"
  "action/HomeAxes.action"
  "action/MoveGantry.action"
  DEPENDENCIES sensor_msgs
)

ament_package()
