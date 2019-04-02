package commander;

public interface OutputDataMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "commander/OutputDataMsg";
  static final java.lang.String _DEFINITION = "#\n#\tMPC Output Data message \n#\n\n# Predicted control inputs, NU * N\nstd_msgs/Float64MultiArray controlTrajectory\n# Predicted state, NX * N\nstd_msgs/Float64MultiArray stateTrajectory\n# Current control sample, correspond to the column of controlTrajectory indexed by openLoopIndex\nstd_msgs/Float64MultiArray currentControl\n# If an MPC tick is triggered without updated data, the MPC increments this index, meaning that the current control is open loop\nint64 openLoopIndex\n# KKT value\nfloat64 kktTol\n# Solver status code\nfloat64 solverStatus\n# CPU Time - not implemented\nfloat64 cpuTime\n# Number of iterations\nint64 nIter\n# Objective cost value\nfloat64 objVal\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Float64MultiArray getControlTrajectory();
  void setControlTrajectory(std_msgs.Float64MultiArray value);
  std_msgs.Float64MultiArray getStateTrajectory();
  void setStateTrajectory(std_msgs.Float64MultiArray value);
  std_msgs.Float64MultiArray getCurrentControl();
  void setCurrentControl(std_msgs.Float64MultiArray value);
  long getOpenLoopIndex();
  void setOpenLoopIndex(long value);
  double getKktTol();
  void setKktTol(double value);
  double getSolverStatus();
  void setSolverStatus(double value);
  double getCpuTime();
  void setCpuTime(double value);
  long getNIter();
  void setNIter(long value);
  double getObjVal();
  void setObjVal(double value);
}
