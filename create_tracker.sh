#!/usr/bin/env bash
helptxt="Usage of this script:
create_tracker.sh <TRACKER NAMESPACE> <TRACKER NAME> [--options]
Options (optional):
  -nc or --noconfig: Does not create config files
  -c or --config: Creates config files (default)"

trackermodel="Put your tracker here
  here"

if [ $# -lt 2 ] || [ $# -gt 3 ]
then
  echo "$helptxt"
  exit 0
fi

pluginmodel="
<library path=\"lib/lib"$2"\">
  <class name=\"trackers_brubotics/"$2"\" type=\"mrs_uav_trackers::"$1"::"$2"\" base_class_type=\"mrs_uav_managers::Tracker\">
    <description>This is the "$2"</description>
  </class>
</library>"

cmakemodel="
  # "$2"

  add_library("$2"
    src/"$1"/"$1".cpp
    )
  add_dependencies("$2"
    \${catkin_EXPORTED_TARGETS}
    \${\${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries("$2"
    \${catkin_LIBRARIES}
    )
    "

addressmodel="
"$2":
  address: \"trackers_brubotics/"$2"\"
"


trackermodel="#define VERSION "0.0.0.0"

#include <ros/ros.h>
#include <mrs_uav_managers/tracker.h>

namespace mrs_uav_trackers
{

namespace $1
{

class $2 : public mrs_uav_managers::Tracker {
public:
  void                          initialize(const ros::NodeHandle &parent_nh, const std::string uav_name, std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  std::tuple<bool, std::string> activate(const mrs_msgs::PositionCommand::ConstPtr &last_position_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);

  const mrs_msgs::PositionCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  const mrs_msgs::TrackerStatus             getStatus();
  const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state);

  const mrs_msgs::ReferenceSrvResponse::ConstPtr           setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::TrackerConstraintsSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

private:

};

  //WRITE THE FUNCTIONS HERE.

}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trackers::$1::$2, mrs_uav_managers::Tracker)"

packpath=$(rospack find trackers_brubotics)
cd $packpath
cd src

mkdir -p $1
cd $1

filename=$1.cpp

if test -f $filename
then
    echo "Tracker already exists. Do you wish to replace it? [y/N]"
    read answer
    if [ "$answer" = "y" ] || [ "$answer" = "Y" ]
    then
      echo "Replacing tracker..."
    else
      exit 0
    fi
fi

echo "$trackermodel" > $filename
cd $packpath
echo "$pluginmodel" >> plugins.xml

linenumber=$(grep -n target_link_libraries CMakeLists.txt | cut -d : -f 1)

for number in $linenumber
do
    line=$number
done

linenumber=$(grep -n \) CMakeLists.txt | cut -d : -f 1)

for number in $linenumber
do
  newline=$number
  if [ $newline -gt $line ]
  then
    break
  fi
done

newline=$(($newline+1))

split -l $newline CMakeLists.txt
echo "$cmakemodel" >> xaa

cat x* > CMakeLists.txt
rm x*

linenumber=$(grep -n "ARCHIVE DESTINATION" CMakeLists.txt | cut -d : -f 1)
linenumber=$(($linenumber-1))

split -l $linenumber CMakeLists.txt
echo "$2" >> xaa
cat x* > CMakeLists.txt
rm x*

cd config
echo "$addressmodel" >> address.yaml

if [ "$3" = "-nc" ] || [ "$3" = "--noconfig" ]
then
  exit 0
fi


run_type_list="simulation uav"
uav_type_list="eaglemk2 f450 f550 m600 t650 eagle naki"


for rtype in $run_type_list
do
  mkdir -p $rtype
  cd $rtype
  for utype in $uav_type_list
  do
    mkdir -p $utype
    cd $utype
    touch $1.yaml
    cd ..
  done
  cd ..
done

cd default
touch $1.yaml

cd ../..

cd launch

argmodel="<arg name=\"custom_config_$1\" default=\"\" />"
paramodel="
<rosparam ns=\"\$(env UAV_NAME)/control_manager/$1\" file=\"\$(find trackers_brubotics)/config/default/$1.yaml\" />
<rosparam ns=\"\$(env UAV_NAME)/control_manager/$1\" file=\"\$(find trackers_brubotics)/config/\$(arg RUN_TYPE)/\$(arg UAV_TYPE)/$1.yaml\" />
<rosparam if=\"\$(eval not arg('custom_config_$1') == '')\" ns=\"\$(env UAV_NAME)/control_manager/$1\" file=\"\$(arg custom_config_$1)\" />
<param name=\"\$(env UAV_NAME)/control_manager/$1/enable_profiler\" type=\"bool\" value=\"\$(arg PROFILER)\" />
<remap from=\"~\$(env UAV_NAME)/control_manager/$1/profiler\" to=\"profiler\" />"

linenumber=$(grep -n "<!-- Parameter loading -->" trackers_brubotics.launch | cut -d : -f 1)
linenumber=$(($linenumber-2))

split -l $linenumber trackers_brubotics.launch
echo "$argmodel" >> xaa
cat x*> trackers_brubotics.launch
rm x*

linenumber=$(grep -n "<!-- Address loading -->" trackers_brubotics.launch | cut -d : -f 1)
linenumber=$(($linenumber-2))

split -l $linenumber trackers_brubotics.launch
echo "$paramodel" >> xaa
cat x* > trackers_brubotics.launch
rm x*
