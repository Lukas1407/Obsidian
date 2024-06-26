This file defines properties about the package such as the *package name*, *version* numbers, *authors*, *maintainers*, and dependencies on other catkin packages.

## Basic structure
```xml
<package format="2">
  <name>foo_core</name>
  <version>1.2.4</version>
  <description>
  This package provides foo capability.
  </description>
  <maintainer email="ivana@osrf.org">Ivana Bildbotz</maintainer>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>BSD</license>
</package>
```

## Dependencies
Packages can have 6 different types of dependencies:
1. **Build Dependencies** `<build_depend>`: specifies which packages are needed to build this package. This is the case if any file from these packages is needed at build time.
2. **Build Export Dependencies** `<build_export_depend>`: specifies which packages are needed to build libraries against this package. This is the case when you transitively include their headers in public headers in this package.
3. **Executions Dependencies** `<exec_depend>`: specifies which packages are needed to run the code in this package. This is the case when you depend on shared libraries in this package.
4. **Test Dependencies** `<test_depend>`: specifies only additional dependencies for unit tests. *They should never duplicate any dependencies already mentioned as build or run dependencies.*
5. **Build Tool Dependencies** `<buildtool_depend>`: specifies build system tools which this package needs to build itself. Typically the only build tool needed is catkin.
6. **Document Tool Dependencies** `<doc_depend>`: specifies documentation tools which this package needs to generate documentation.
==depend== specifies that a dependency is a build, export, and execution dependency. This is the most commonly used dependency tag.
```xml
<package format="2">
  <name>foo_core</name>
  <version>1.2.4</version>
  <description>
    This package provides foo capability.
  </description>
  <maintainer email="ivana@willowgarage.com">Ivana Bildbotz</maintainer>
  <license>BSD</license>
  <url>http://ros.org/wiki/foo_core</url>
  <author>Ivana Bildbotz</author>
  
  <buildtool_depend>catkin</buildtool_depend>
  
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  
  <build_depend>message_generation</build_depend>
  
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>rospy</exec_depend>

  <test_depend>python-mock</test_depend>
  
  <doc_depend>doxygen</doc_depend>
</package>
```

## Metapackages
- Metapackages are a way to <mark style="background: #FFB86CA6;">group multiple packages as a single logical package</mark>. This means that <mark style="background: #FFB86CA6;">you can install or uninstall several related packages at once</mark> by using the name of the metapackage. For example, if you want to install a desktop environment, you can use a metapackage that contains all the necessary packages for that environment.
- Metapackages have a special **export tag** in the package.xml file to indicate that they are metapackages:
```xml
 <export>
   <metapackage />
 </export>
```
- Metapackages can only have **execution dependencies** on the packages that they group, and a **build tool dependency** on catkin
```xml
<package format="2">
  <name>ros_metapackage_example</name>
  <version>1.5.0</version>
  <description>
    Example for such a Metapackage
  </description>
  <maintainer email="ros@osrfoundation.org">ROS Team</maintainer>
  <license>BSD</license>
  <url type="website">http://wiki.ros.org/ros_base</url>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>actionlib</run_depend>
  <exec_depend>bond_core</run_depend>
  <exec_depend>class_loader</run_depend>
  <exec_depend>dynamic_reconfigure</run_depend>
  <exec_depend>nodelet_core</run_depend>
  <exec_depend>pluginlib</run_depend>
  <export>
    <metapackage/>
  </export>
</package>
```
Additionally a metapackage has a required, boilerplate CMakeLists.txt file:
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(<PACKAGE_NAME>)
find_package(catkin REQUIRED)
catkin_metapackage()
```
