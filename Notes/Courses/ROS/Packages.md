A package is an independent unit in the program. For example a "folder" that combines all the [[Nodes]] necessary for hardware control.

Must meet a few requirements:
1. The package must contain a catkin compliant [[package.xml]] file
2. The package must contain a [[CMakeLists.txt]] which uses catkin
3. Each package must have its own folder
	1. -> no nested packages

Example of the simplest possible package:
```
- my_package/
      CMakeLists.txt
      package.xml
```
Recommended to use a [[catkin workspace]]:
```
- workspace_folder/        -- WORKSPACE
      src/                   -- SOURCE SPACE
        CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
        package_1/
          CMakeLists.txt     -- CMakeLists.txt file for package_1
          package.xml        -- Package manifest for package_1
        ...
        package_n/
          CMakeLists.txt     -- CMakeLists.txt file for package_n
          package.xml        -- Package manifest for package_n
```
A more detailed package consists of:
```
- my_package/
      config             -- for parameter files
      include            -- for c++ header files to include
      launch             -- for the launch files to launch the package
      src
      test
      CMakeLists.txt
      package.xml
```
## Create a Package
1. <mark style="background: #FFB86CA6;">Change into the workspace</mark> source folder: `cd ~/catkin_ws/src`
2. <mark style="background: #FFB86CA6;">Use the</mark> `catkin_create_pkg` <mark style="background: #FFB86CA6;">script to create a new package</mark> 
	- Example called 'beginner_tutorials' which depends on <mark style="background: #ADCCFFA6;">std_msgs, roscpp, and rospy</mark>: `catkin_create_pkg beginner_tutorials std_msgs rospy roscpp`
		-> creates a beginner_tutorial folder which contains partially filled out [[package.xml]] and [[CMakeLists.txt]]
	- The <mark style="background: #ADCCFFA6;">Dependencies</mark> are **first-order dependencies** in this case. They can be checked using [[rospack]]. 

## Building the workspace with the packages
A workspace, which may contain multiple packages, can be build using the `catkin_make` command inside the workspace folder:
```linux
cd ~/catkin_ws
catkin_make
```
To add the workspace to your ROS environment you need to source the generated setup file:
	`. ~/catkin_ws/devel/setup.bash`