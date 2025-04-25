# CMake generated Testfile for 
# Source directory: /home/joel/Desktop/mir_ws/src/mir_robot/mir_driver
# Build directory: /home/joel/Desktop/mir_ws/build_isolated/mir_driver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_mir_driver_roslaunch-check_launch "/home/joel/Desktop/mir_ws/build_isolated/mir_driver/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/joel/Desktop/mir_ws/build_isolated/mir_driver/test_results/mir_driver/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/joel/Desktop/mir_ws/build_isolated/mir_driver/test_results/mir_driver" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/joel/Desktop/mir_ws/build_isolated/mir_driver/test_results/mir_driver/roslaunch-check_launch.xml\" \"/home/joel/Desktop/mir_ws/src/mir_robot/mir_driver/launch\" ")
set_tests_properties(_ctest_mir_driver_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/joel/Desktop/mir_ws/src/mir_robot/mir_driver/CMakeLists.txt;67;roslaunch_add_file_check;/home/joel/Desktop/mir_ws/src/mir_robot/mir_driver/CMakeLists.txt;0;")
subdirs("gtest")
