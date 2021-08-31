# header time stamp
time stamp
# start and end point of the trajectory contained as first and last point of the loaded traj.csv or traj.txt
geometry_msgs/Point traj_start_point
geometry_msgs/Point traj_end_point
# Criteria to determine if and when the uav arrived at end of the trajectory (i.e. arrived_at_traj_end_point = true) is that the uav has a smaller error from the end_point than the arrival_norm_pos_error_treshold value for at least a time of the arrival_period_treshold s.
float32 arrival_norm_pos_error_treshold
float32 arrival_period_treshold
bool arrived_at_traj_end_point 
# the times when the trajectory is started and ended (i.e.when arrival_period_treshold triggers 0->1)
float64 time_at_start_point
float64 time_at_end_point





