#
#	MPC Output Data message 
#

# Predicted control inputs, NU * N
std_msgs/Float64MultiArray controlTrajectory
# Predicted state, NX * N
std_msgs/Float64MultiArray stateTrajectory
# Current control sample, correspond to the column of controlTrajectory indexed by openLoopIndex
std_msgs/Float64MultiArray currentControl
# If an MPC tick is triggered without updated data, the MPC increments this index, meaning that the current control is open loop
int64 openLoopIndex
# KKT value
float64 kktTol
# Solver status code
float64 solverStatus
# CPU Time - not implemented
float64 cpuTime
# Number of iterations
int64 nIter
# Objective cost value
float64 objVal
