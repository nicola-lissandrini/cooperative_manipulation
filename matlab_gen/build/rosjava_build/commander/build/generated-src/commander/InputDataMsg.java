package commander;

public interface InputDataMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "commander/InputDataMsg";
  static final java.lang.String _DEFINITION = "#\n#\tMPC Input Data message \n#\n\n# State data: must be a vector of NX components\nstd_msgs/Float64MultiArray state\n# Output reference data: must be a matrix of NY rows and N columns\nstd_msgs/Float64MultiArray refWindow\n# Terminal output reference data: vector of NYN components\nstd_msgs/Float64MultiArray refTerminal\n# Online data\nstd_msgs/Float64MultiArray onlineData";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Float64MultiArray getState();
  void setState(std_msgs.Float64MultiArray value);
  std_msgs.Float64MultiArray getRefWindow();
  void setRefWindow(std_msgs.Float64MultiArray value);
  std_msgs.Float64MultiArray getRefTerminal();
  void setRefTerminal(std_msgs.Float64MultiArray value);
  std_msgs.Float64MultiArray getOnlineData();
  void setOnlineData(std_msgs.Float64MultiArray value);
}
