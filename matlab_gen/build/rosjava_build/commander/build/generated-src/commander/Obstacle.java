package commander;

public interface Obstacle extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "commander/Obstacle";
  static final java.lang.String _DEFINITION = "# Obstacles are defined as ellipsoid\ngeometry_msgs/Point center\ngeometry_msgs/Vector3 axes";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  geometry_msgs.Point getCenter();
  void setCenter(geometry_msgs.Point value);
  geometry_msgs.Vector3 getAxes();
  void setAxes(geometry_msgs.Vector3 value);
}
