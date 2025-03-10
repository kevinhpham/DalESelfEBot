# CMake generated Testfile for 
# Source directory: /home/jarred/git/DalESelfEBot/ur3_control
# Build directory: /home/jarred/git/DalESelfEBot/ur3_control/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_ik_solver_kdl "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/jarred/git/DalESelfEBot/ur3_control/build/test_results/ur3_control/test_ik_solver_kdl.gtest.xml" "--package-name" "ur3_control" "--output-file" "/home/jarred/git/DalESelfEBot/ur3_control/build/ament_cmake_gtest/test_ik_solver_kdl.txt" "--command" "/home/jarred/git/DalESelfEBot/ur3_control/build/test_ik_solver_kdl" "--gtest_output=xml:/home/jarred/git/DalESelfEBot/ur3_control/build/test_results/ur3_control/test_ik_solver_kdl.gtest.xml")
set_tests_properties(test_ik_solver_kdl PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/jarred/git/DalESelfEBot/ur3_control/build/test_ik_solver_kdl" TIMEOUT "60" WORKING_DIRECTORY "/home/jarred/git/DalESelfEBot/ur3_control/build" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/jarred/git/DalESelfEBot/ur3_control/CMakeLists.txt;27;ament_add_gtest;/home/jarred/git/DalESelfEBot/ur3_control/CMakeLists.txt;0;")
subdirs("gtest")
