# trackers_brubotics
Trackers created by the 2020 interns
## How to create a new tracker in this package when it's placed ?

### Writing the code
All trackers are plugins. To begin, you should create a MRS tracker plugin following the `tracker.h` class. As long as you have built the `mrs_workspace` package, you will have no trouble including the class declaration on your tracker via `#include`, and you will also be able to use all the mrs includes.

### Building the plugin

There are some things worth mentioning in the `package.xml` file, in the `CMakeLists`, in the `plugins.xml`.

#### In the package.xml
Our new trackers have the same dependencies as the other mrs tracker plugins, so the `depend` tag is exactly the same. The `export` tag indicates that the tracker should be exported as a `mrs_uav_tracker` type plugin

#### In the CMakeLists
It follows precisely the same model as the `CMakeLists` file. The only differences are:
1. In the build section, for the `add_library`, `add_dependencies` and `target_link_libraries` commands, the tracker specified is our line tracker instead of the default mrs trackers.
2. In the install command, the plugin specified is our tracker and not the default MRS trackers
3. The name of the package is `trackers_brubotics`
Everything else is the same.

#### In the plugins.xml
It follows the same model as in the MRS trackers package, and even the type is the same. The only difference is that the name is `trackers_brubotics/[tracker name]` instead of `mrs_uav_trackers/[tracker name]`

### Integrating it with the mrs_uav_manager

*This section will probably be modified soon. The goal is to keep the changes to the mrs_workspace folders and files to a minimum*

The only thing that needs to change inside the package is the addition of the parameter loader for our new trackers to the `control_manager.launch` file and some modifications in the config files. The following lines were added after the SO3 parameter loader:
```
<!-- LineTrackerInterns -->
<rosparam ns="line_tracker_interns" file="$(find trackers_brubotics)/config/default/line_tracker_interns.yaml" />
<rosparam ns="line_tracker_interns" file="$(find trackers_brubotics)/config/$(arg RUN_TYPE)/$(arg UAV_TYPE)/line_tracker_interns.yaml" />
<rosparam if="$(eval not arg('custom_config_line_tracker_interns') == '')" ns="line_tracker_interns" file="$(arg custom_config_line_tracker_interns)" />
<param name="line_tracker_interns/enable_profiler" type="bool" value="$(arg PROFILER)" />
<remap from="~line_tracker_interns/profiler" to="profiler" />
```
And the following line was added in the arguments definition:

```
<arg name="custom_config_line_tracker_interns" default="" />
```

To the `control_manager.yaml` file in the config folder, the following lines were added:

```
trackers : [
  "MpcTracker",
  "LineTracker",
  "LandoffTracker",
  "JoyTracker",
  "MatlabTracker",
  "SpeedTracker",
  "NullTracker",
  "LineTrackerInterns",
]
```

And to the `trackers.yaml`, the address of the new tracker was added:
```
LineTrackerInterns:
  address: "trackers_brubotics/LineTrackerInterns"
```
