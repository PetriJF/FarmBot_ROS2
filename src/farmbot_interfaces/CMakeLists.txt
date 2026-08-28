cmake_minimum_required(VERSION 3.8)
project(farmbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImageMessage.msg"
  "msg/SequenceStatus.msg"
  "srv/AddPlant.srv"
  "srv/AddSeedTray.srv"
  "srv/AddTool.srv"
  "srv/CaptureImage.srv"
  "srv/ConfigurePin.srv"
  "srv/EnableCamera.srv"
  "srv/LedPanelHandler.srv"
  "srv/MoveServo.srv"
  "srv/ParameterConfig.srv"
  "srv/ReadI2C.srv"
  "srv/ReadParameter.srv"
  "srv/ReadPin.srv"
  "srv/RemoveMapObject.srv"
  "srv/SetI2C.srv"
  "srv/StringRepReq.srv"
  "srv/UpdateMap.srv"
  "srv/Watering.srv"
  "srv/WriteParameter.srv"
  "srv/WritePin.srv"
  "action/HomeAxes.action"
  "action/LoadingParameters.action"
  "action/MoveGantry.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

ament_package()
