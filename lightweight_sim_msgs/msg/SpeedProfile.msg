# Stamp is the initial vehicle-state time, never the solver completion time.
std_msgs/Header header
uint64 run_id
uint64 path_sequence
float64 origin_s
bool valid
string status
SpeedPoint[] points
