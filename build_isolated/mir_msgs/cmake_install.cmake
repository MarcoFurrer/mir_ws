# Install script for directory: /home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/joel/Desktop/mir_ws/install_isolated")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/joel/Desktop/mir_ws/install_isolated/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/joel/Desktop/mir_ws/install_isolated" TYPE PROGRAM FILES "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/joel/Desktop/mir_ws/install_isolated/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/joel/Desktop/mir_ws/install_isolated" TYPE PROGRAM FILES "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/joel/Desktop/mir_ws/install_isolated/setup.bash;/home/joel/Desktop/mir_ws/install_isolated/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/joel/Desktop/mir_ws/install_isolated" TYPE FILE FILES
    "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/setup.bash"
    "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/joel/Desktop/mir_ws/install_isolated/setup.sh;/home/joel/Desktop/mir_ws/install_isolated/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/joel/Desktop/mir_ws/install_isolated" TYPE FILE FILES
    "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/setup.sh"
    "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/joel/Desktop/mir_ws/install_isolated/setup.zsh;/home/joel/Desktop/mir_ws/install_isolated/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/joel/Desktop/mir_ws/install_isolated" TYPE FILE FILES
    "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/setup.zsh"
    "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/joel/Desktop/mir_ws/install_isolated/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/joel/Desktop/mir_ws/install_isolated" TYPE FILE FILES "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mir_msgs/msg" TYPE FILE FILES
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/AngleMeasurment.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/BMSData.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/BatteryCurrents.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/BrakeState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/ChargingState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Device.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Devices.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/EncoderTestEntry.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Encoders.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Error.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Event.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Events.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/ExternalRobot.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/ExternalRobots.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Gpio.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/GripperState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/HeightState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/HookData.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/HookExtendedStatus.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/HookStatus.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/IOs.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/JoystickVel.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/LocalMapStat.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/MirExtra.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/MirLocalPlannerPathTypes.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/MissionCtrlCommand.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/MissionCtrlState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/MovingState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/PalletLifterStatus.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Path.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Pendant.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/PlanSegment.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/PlanSegments.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Pose2D.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/PowerBoardMotorStatus.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/PrecisionDockingStatus.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Proximity.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/ResourceState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/ResourcesAcquisition.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/ResourcesState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/RobotMode.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/RobotState.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/RobotStatus.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/SafetyStatus.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Serial.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/ServiceResponseHeader.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/SkidDetectionDiff.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/SkidDetectionStampedFloat.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/SoundEvent.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/StampedEncoders.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/TimeDebug.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Trolley.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/Twist2D.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/UserPrompt.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/WebPath.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/WorldMap.msg"
    "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/msg/WorldModel.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mir_msgs/cmake" TYPE FILE FILES "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/mir_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/joel/Desktop/mir_ws/devel_isolated/mir_msgs/include/mir_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/joel/Desktop/mir_ws/devel_isolated/mir_msgs/share/roseus/ros/mir_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/joel/Desktop/mir_ws/devel_isolated/mir_msgs/share/common-lisp/ros/mir_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/joel/Desktop/mir_ws/devel_isolated/mir_msgs/share/gennodejs/ros/mir_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/joel/Desktop/mir_ws/devel_isolated/mir_msgs/lib/python3/dist-packages/mir_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/joel/Desktop/mir_ws/devel_isolated/mir_msgs/lib/python3/dist-packages/mir_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/mir_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mir_msgs/cmake" TYPE FILE FILES "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/mir_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mir_msgs/cmake" TYPE FILE FILES
    "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/mir_msgsConfig.cmake"
    "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/catkin_generated/installspace/mir_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mir_msgs" TYPE FILE FILES "/home/joel/Desktop/mir_ws/src/mir_robot/mir_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/joel/Desktop/mir_ws/build_isolated/mir_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
