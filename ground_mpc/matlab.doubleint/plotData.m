state = csvread ('/home/nicola/ros_ws/state2');
control = csvread ('/home/nicola/ros_ws/control2');


stairs (state(:,3))