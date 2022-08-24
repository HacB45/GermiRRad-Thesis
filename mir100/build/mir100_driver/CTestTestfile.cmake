# CMake generated Testfile for 
# Source directory: /home/hugo/Desktop/mir100/src/mir100_driver
# Build directory: /home/hugo/Desktop/mir100/build/mir100_driver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_mir100_driver_roslaunch-check_launch "/home/hugo/Desktop/mir100/build/mir100_driver/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/hugo/Desktop/mir100/build/mir100_driver/test_results/mir100_driver/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/hugo/Desktop/mir100/build/mir100_driver/test_results/mir100_driver" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/hugo/Desktop/mir100/build/mir100_driver/test_results/mir100_driver/roslaunch-check_launch.xml\" \"/home/hugo/Desktop/mir100/src/mir100_driver/launch\" ")
set_tests_properties(_ctest_mir100_driver_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/hugo/Desktop/mir100/src/mir100_driver/CMakeLists.txt;67;roslaunch_add_file_check;/home/hugo/Desktop/mir100/src/mir100_driver/CMakeLists.txt;0;")
subdirs("gtest")
