This is used to build the software packages. It describes how to build the code and where to install it.

## Overall Structure
- <mark style="background: #FF5582A6;">The order must be followed.</mark>
1. **Required CMake Version** (`cmake_minimum_required`)
	1. the name of the package
2. **Package Name** (project()): 
	1. you can reference the project name anywhere later in the CMake script by using the variable ${PROJECT_NAME} wherever needed.
3. **Find other CMake/Catkin packages needed for build** (`find_package()`)
	1. specify which other CMake packages are needed to build our project
	2. catkin packages can be specified as Components: `find_package(catkin REQUIRED COMPONENTS nodelet roscpp)`
	3. -> <mark style="background: #FFB86CA6;">should be the same as in the</mark> [[package.xml]] file!
4. Enable Python module support (`catkin_python_setup()`)
5. Message/**Service/Action Generators**(`add_message_files()`, `add_service_files()`, `add_action_files()`)
6. **Invoke message/service/action generation** (`generate_messages()`)
7. **Specify package build info export** (`catkin_package()`)
8. **Libraries/Executables to build** (`add_library()/add_executable()/target_link_libraries()`)
9. **Tests to build** (`catkin_add_gtest()`)
10. **Install rules** (`install()`)
Example:
```CMake
cmake_minimum_required(VERSION 2.8.3)
project(robot_brain)  // same name as in the package.xml

catkin_python_setup()

find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	sensor_msg
	 )

catkin_package(
	INCLUDE_DIRS include // in which dir the includes are
	LIBRARIES ${PROJECT_NAME} // if the porject creates a library
	CATKIN_DEPENDS roscpp sensor_msgs // ros dependencies again
	DEPENDS Eigen Boost // non ros dependencies
)

add_executable(${PROJECT_NAME} // decalre the ros node 
	src/${PROJECT_NAME}_node.py)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(myUnitTest test/utest.cpp)
endif()
```

## Specify Build Targets
Build targets can take many forms, but usually they represent one of two possibilities:
- **Executable Target** - programs we can run
- **Library Target** - libraries that can be used by executable targets at build and/or runtime


## Messages, Services, and Action Targets
Messages (.msg), services (.srv), and actions (.action) files in ROS require a special pre-processor build step before being built and used by ROS packages.

There are three macros provided to handle messages, services, and actions respectively:
- add_message_files
- add_service_files
- add_action_files

- <mark style="background: #FFB86CA6;">These macros must then be followed by a call to the macro that invokes generation</mark>: `generate_messages()`
- <mark style="background: #FFB86CA6;">These macros must come BEFORE the catkin_package() macro in order for generation to work correctly</mark>:
```cmake
 find_package(catkin REQUIRED COMPONENTS ...)
 add_message_files(...)
 add_service_files(...)
 add_action_files(...)
 generate_messages(...)
 catkin_package(...)
 ...
```
- <mark style="background: #FFB86CA6;">Your catkin_package() macro must have a CATKIN_DEPENDS dependency on message_runtime</mark>:
```cmake
 catkin_package(
 ...
 CATKIN_DEPENDS message_runtime ...
 ...)
```
- <mark style="background: #FFB86CA6;">You must use find_package() for the package message_generation, either alone or as a component of catkin</mark>:
```cmake
find_package(catkin REQUIRED COMPONENTS message_generation)
```
Example:
```cmake
  # Get the information about this package's buildtime dependencies
  find_package(catkin REQUIRED
    COMPONENTS message_generation std_msgs sensor_msgs)
  # Declare the message files to be built
  add_message_files(FILES
    MyMessage1.msg
    MyMessage2.msg
  )
  # Declare the service files to be built
  add_service_files(FILES
    MyService.srv
  )
  # Actually generate the language-specific message and service files
  generate_messages(DEPENDENCIES std_msgs sensor_msgs)
  # Declare that this catkin package's runtime dependencies
  catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
  )
  # define executable using MyMessage1 etc.
  add_executable(message_program src/main.cpp)
  add_dependencies(message_program ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  # define executable not using any messages/services provided by this package
  add_executable(does_not_use_local_messages_program src/main.cpp)
  add_dependencies(does_not_use_local_messages_program ${catkin_EXPORTED_TARGETS})
```