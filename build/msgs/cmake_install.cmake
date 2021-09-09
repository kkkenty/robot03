<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
# Install script for directory: /home/robot03/robot03/src/custom_msgs
=======
# Install script for directory: /home/kento/robot03/src/msgs
>>>>>>> dev:build/msgs/cmake_install.cmake

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/robot03/robot03/install")
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
   "/home/robot03/robot03/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
file(INSTALL DESTINATION "/home/robot03/robot03/install" TYPE PROGRAM FILES "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/_setup_util.py")
=======
file(INSTALL DESTINATION "/home/kento/robot03/install" TYPE PROGRAM FILES "/home/kento/robot03/build/msgs/catkin_generated/installspace/_setup_util.py")
>>>>>>> dev:build/msgs/cmake_install.cmake
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/robot03/robot03/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
file(INSTALL DESTINATION "/home/robot03/robot03/install" TYPE PROGRAM FILES "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/env.sh")
=======
file(INSTALL DESTINATION "/home/kento/robot03/install" TYPE PROGRAM FILES "/home/kento/robot03/build/msgs/catkin_generated/installspace/env.sh")
>>>>>>> dev:build/msgs/cmake_install.cmake
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/robot03/robot03/install/setup.bash;/home/robot03/robot03/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
file(INSTALL DESTINATION "/home/robot03/robot03/install" TYPE FILE FILES
    "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/setup.bash"
    "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/local_setup.bash"
=======
file(INSTALL DESTINATION "/home/kento/robot03/install" TYPE FILE FILES
    "/home/kento/robot03/build/msgs/catkin_generated/installspace/setup.bash"
    "/home/kento/robot03/build/msgs/catkin_generated/installspace/local_setup.bash"
>>>>>>> dev:build/msgs/cmake_install.cmake
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/robot03/robot03/install/setup.sh;/home/robot03/robot03/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
file(INSTALL DESTINATION "/home/robot03/robot03/install" TYPE FILE FILES
    "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/setup.sh"
    "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/local_setup.sh"
=======
file(INSTALL DESTINATION "/home/kento/robot03/install" TYPE FILE FILES
    "/home/kento/robot03/build/msgs/catkin_generated/installspace/setup.sh"
    "/home/kento/robot03/build/msgs/catkin_generated/installspace/local_setup.sh"
>>>>>>> dev:build/msgs/cmake_install.cmake
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/robot03/robot03/install/setup.zsh;/home/robot03/robot03/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
file(INSTALL DESTINATION "/home/robot03/robot03/install" TYPE FILE FILES
    "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/setup.zsh"
    "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/local_setup.zsh"
=======
file(INSTALL DESTINATION "/home/kento/robot03/install" TYPE FILE FILES
    "/home/kento/robot03/build/msgs/catkin_generated/installspace/setup.zsh"
    "/home/kento/robot03/build/msgs/catkin_generated/installspace/local_setup.zsh"
>>>>>>> dev:build/msgs/cmake_install.cmake
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/robot03/robot03/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
file(INSTALL DESTINATION "/home/robot03/robot03/install" TYPE FILE FILES "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/custom_msgs/msg" TYPE FILE FILES "/home/robot03/robot03/src/custom_msgs/msg/motor_pwm.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/custom_msgs/cmake" TYPE FILE FILES "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/custom_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/robot03/robot03/devel/.private/custom_msgs/include/custom_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/robot03/robot03/devel/.private/custom_msgs/share/roseus/ros/custom_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/robot03/robot03/devel/.private/custom_msgs/share/common-lisp/ros/custom_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/robot03/robot03/devel/.private/custom_msgs/share/gennodejs/ros/custom_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/robot03/robot03/devel/.private/custom_msgs/lib/python2.7/dist-packages/custom_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/robot03/robot03/devel/.private/custom_msgs/lib/python2.7/dist-packages/custom_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/custom_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/custom_msgs/cmake" TYPE FILE FILES "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/custom_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/custom_msgs/cmake" TYPE FILE FILES
    "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/custom_msgsConfig.cmake"
    "/home/robot03/robot03/build/custom_msgs/catkin_generated/installspace/custom_msgsConfig-version.cmake"
=======
file(INSTALL DESTINATION "/home/kento/robot03/install" TYPE FILE FILES "/home/kento/robot03/build/msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/msgs/msg" TYPE FILE FILES "/home/kento/robot03/src/msgs/msg/Motor.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/msgs/cmake" TYPE FILE FILES "/home/kento/robot03/build/msgs/catkin_generated/installspace/msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kento/robot03/devel/.private/msgs/include/msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/kento/robot03/devel/.private/msgs/share/roseus/ros/msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/kento/robot03/devel/.private/msgs/share/common-lisp/ros/msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/kento/robot03/devel/.private/msgs/share/gennodejs/ros/msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/kento/robot03/devel/.private/msgs/lib/python2.7/dist-packages/msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/kento/robot03/devel/.private/msgs/lib/python2.7/dist-packages/msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kento/robot03/build/msgs/catkin_generated/installspace/msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/msgs/cmake" TYPE FILE FILES "/home/kento/robot03/build/msgs/catkin_generated/installspace/msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/msgs/cmake" TYPE FILE FILES
    "/home/kento/robot03/build/msgs/catkin_generated/installspace/msgsConfig.cmake"
    "/home/kento/robot03/build/msgs/catkin_generated/installspace/msgsConfig-version.cmake"
>>>>>>> dev:build/msgs/cmake_install.cmake
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/custom_msgs" TYPE FILE FILES "/home/robot03/robot03/src/custom_msgs/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/msgs" TYPE FILE FILES "/home/kento/robot03/src/msgs/package.xml")
>>>>>>> dev:build/msgs/cmake_install.cmake
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
  include("/home/robot03/robot03/build/custom_msgs/gtest/cmake_install.cmake")
=======
  include("/home/kento/robot03/build/msgs/gtest/cmake_install.cmake")
>>>>>>> dev:build/msgs/cmake_install.cmake

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD:build/custom_msgs/cmake_install.cmake
file(WRITE "/home/robot03/robot03/build/custom_msgs/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/kento/robot03/build/msgs/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> dev:build/msgs/cmake_install.cmake
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
