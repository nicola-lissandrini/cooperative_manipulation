#
#	MPC Input Data message 
#

# State data: must be a vector of NX components
std_msgs/Float64MultiArray state
# Output reference data: must be a matrix of NY rows and N columns
std_msgs/Float64MultiArray refWindow
# Terminal output reference data: vector of NYN components
std_msgs/Float64MultiArray refTerminal
# Online data
std_msgs/Float64MultiArray onlineData