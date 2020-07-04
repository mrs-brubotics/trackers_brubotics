# trackers_brubotics
Trackers developed by the summer 2020 Brubotics interns.

## How to create a new tracker in this package?
There is now an automatic way to do this. Just run the script:

```
sh create_tracker.sh [TRACKER NAMESPACE] [TRACKER NAME] -options
```
As options, you have:
```
-nc or --noconfig: Does not create config files
-c or --config: Creates config files (default)
```

### Convention

when I write "tracker name", I mean the name of the class. It should be written without underscores, with caps, like this:
> ExampleTracker

And when I say "tracker namespace", I mean the name of the namespace, the cpp file and the folder where it's located. It should be written in lowercase with underscores, like this:
> example_tracker

When I write something with brackets in a file path, the brackets should be replaced by the value of whatever is in them, for example:
> [TRACKER NAMESPACE].cpp = example_tracker.cpp

### Writing the code
All trackers are plugins. To begin, you should create a MRS tracker plugin following the `tracker.h` class. As long as you have built the `mrs_workspace` package, you will have no trouble including the class declaration on your tracker via `#include`, and you will also be able to use all the mrs class includes. After the code is written, you need to add the new tracker to the `address.yaml` file in the `config` folder. You can also use the config folder to create config .yaml files for your tracker. If you do that, you should change the `launch/trackers_brubotics.launch` to make sure the parameters for the new tracker are loaded.

#### What to change in the launch file?

In the launch file, you should add your parameter files. You should put your config file `[FILE NAME].yaml` in the folder `config/default`, and if you have different configurations for different run types and/or different uav types you should add them to `config/[RUN_TYPE]/[UAV TYPE]/[FILE NAME].yaml`. If you don't have different configurations, create the file anyway and leave it empty. Then, add the following lines to the launch file, respecting the sections defined by the comments:
```
<!-- Arg definition -->
<arg name="custom_config_[TRACKER NAMESPACE]" default="" />

<!-- Parameter loading -->
<rosparam ns="$(env UAV_NAME)/control_manager/[TRACKER NAMESPACE]" file="$(find tracker_brubotics)/config/default/[FILE NAME].yaml" />
<rosparam ns="$(env UAV_NAME)/control_manager/[TRACKER NAMESPACE]" file="$(find trackers_brubotics)/config/$(arg RUN_TYPE)/$(arg UAV_TYPE)/[FILE NAME].yaml" />
<rosparam if="$(eval not arg('custom_config_[TRACKER NAMESPACE]') == '')" ns="$(env UAV_NAME)/control_manager/[TRACKER NAMESPACE]s" file="$(arg custom_config_[TRACKER NAMESPACE])" />
<param name="$(env UAV_NAME)/control_manager/[TRACKER NAMESPACE]/enable_profiler" type="bool" value="$(arg PROFILER)" />
<remap from="~$(env UAV_NAME)/control_manager/[TRACKER NAMESPACE]/profiler" to="profiler" />
```

#### What to change in the address file?

Add your tracker to the `config/address.yaml` file without erasing what is already there following the model:

```
[TRACKER NAME]:
  address: "trackers_brubotics/[TRACKER NAME]"
```

### Building the plugin

There are some things worth mentioning in the `package.xml` file, in the `CMakeLists`,  and in the `plugins.xml`.

#### In the package.xml
Our new trackers have the same dependencies as the other mrs tracker plugins, so the `depend` tag is exactly the same. The `export` tag indicates that the tracker should be exported as a `mrs_uav_tracker` type plugin

#### In the CMakeLists
It follows precisely the same model as the `CMakeLists` file in the ```mrs_uav_trackers```
1. In the build section, via the `add_library`, `add_dependencies` and `target_link_libraries` commands, the trackers specified are built. To build a new tracker, add the following commmands to the build section:

```
add_library([TRACKER NAMESPACE]
  [PATH TO TRACKER RELATIVE TO THE PACKAGE, EXAMPLE:
  src/tracker_example/tracker_example.cpp]
  )

add_dependencies([TRACKER NAMESPACE]
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries([TRACKER NAMESPACE]
  ${catkin_LIBRARIES}
  )
```

2. In the install command, add our tracker to the list:
```
install(TARGETS
  So3TrackerInterns
  [TRACKER NAMESPACE]
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
```

#### In the plugins.xml
Add your tracker under what's already there, following the model:
```
<library path="lib/lib[TRACKER NAME]">
  <class name="trackers_brubotics/[TRACKER NAME]" type="mrs_uav_trackers::[TRACKER NAMESPACE]::[TRACKER NAME]" base_class_type="mrs_uav_managers::Tracker">
    <description>[short description of your tracker]</description>
  </class>
</library>
```


### Integrating the new tracker with the mrs_uav_manager

The new tracker has to be added to the tracker list in the `mrs_uav_manager` config file `control_manager.yaml`:

```
trackers : [
  "MpcTracker",
  "LineTracker",
  "LandoffTracker",
  "JoyTracker",
  "MatlabTracker",
  "SpeedTracker",
  "NullTracker",
  "[TRACKER NAME]",
]
```

### Launching the new tracker

The `trackers_brubotics.yaml` file takes care of the parameter loading. You shold launch it via
```
roslaunch trackers_brubotics trackers_brubotics.launch
```
**before** launching `core.launch` or `control_manager.launch` from the `mrs_uav` packages.
