execute_process(COMMAND "/home/joel/Desktop/mir_ws/build_isolated/mir_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/joel/Desktop/mir_ws/build_isolated/mir_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
