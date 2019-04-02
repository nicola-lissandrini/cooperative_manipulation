package aerial_mpc;

public interface Trajectory extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aerial_mpc/Trajectory";
  static final java.lang.String _DEFINITION = "# Pose trajectory\ngeometry_msgs/Pose[] trajectory\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.util.List<geometry_msgs.Pose> getTrajectory();
  void setTrajectory(java.util.List<geometry_msgs.Pose> value);
}
