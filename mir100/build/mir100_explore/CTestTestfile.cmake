# CMake generated Testfile for 
# Source directory: /home/hugo/Desktop/mir100/src/mir100_explore
# Build directory: /home/hugo/Desktop/mir100/build/mir100_explore
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_mir100_explore_roslaunch-check_launch "/home/hugo/Desktop/mir100/build/mir100_explore/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/hugo/Desktop/mir100/build/mir100_explore/test_results/mir100_explore/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/hugo/Desktop/mir100/build/mir100_explore/test_results/mir100_explore" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/hugo/Desktop/mir100/build/mir100_explore/test_results/mir100_explore/roslaunch-check_launch.xml\" \"/home/hugo/Desktop/mir100/src/mir100_explore/launch\" ")
set_tests_properties(_ctest_mir100_explore_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/hugo/Desktop/mir100/src/mir100_explore/CMakeLists.txt;81;roslaunch_add_file_check;/home/hugo/Desktop/mir100/src/mir100_explore/CMakeLists.txt;0;")
subdirs("gtest")
