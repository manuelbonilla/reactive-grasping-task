% required MATLAB r2015a and the Robotics System Toolbox

rosshutdown;
rosinit;
map_sub = rossubscriber('accel_map_topic','std_msgs/Float64MultiArray');

while 1
  message = receive(map_sub);
  % showdetails(message);
  
  % acceleration map plot
  plot_acceleration_map(message.Data)
end
