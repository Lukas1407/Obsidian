
## Create a catkin workspace
```
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/
 catkin_make
```
Next, add the line `source ~/catkin_ws/devel/setup.bash` to your `~/.bashrc` file.

## Using multiple Workspaces
- Create a new workspace like above.
- <mark style="background: #FF5582A6;">You can only use one workspace at once!</mark>
- Switching between workspaces: comment out the other workspaces in the `~/.bashrc` file
