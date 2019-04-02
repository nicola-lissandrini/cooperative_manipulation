package commander;

public interface ObstaclesArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "commander/ObstaclesArray";
  static final java.lang.String _DEFINITION = "# Declare obstacles array\ncommander/Obstacle[] obstacles\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.util.List<commander.Obstacle> getObstacles();
  void setObstacles(java.util.List<commander.Obstacle> value);
}
