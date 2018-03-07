# ros_sdf
A plugin to expose Gazebo joints specified by an SDF file to ROS.

To build this plugin:

```
git clone https://github.com/daniel-s-ingram/ros_sdf.git
cd ros_sdf
mkdir build
cd build
cmake ../
make
```

Now, tell Gazebo where to find the plugin:

```
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:path_to_ros_sdf/build"
```

Alternatively, you can include the following line in your launch file:

```
<env name="GAZEBO_PLUGIN_PATH" value="${GAZEBO_PLUGIN_PATH}:path_to_ros_sdf/build" />
```

To use the plugin with your SDF model, include this line in the model tag in the SDF file:

```
<plugin name="joint_controller" filename="libros_sdf.so"/>
```

Currently, the plugin subscribes to the /model_name/position_cmd topic. Publish a Float32MultiArray message to this topic to set the positions of each joint. The order of the joints is the order in which they are defined in the SDF file.
