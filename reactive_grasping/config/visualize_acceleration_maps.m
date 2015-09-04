% required MATLAB r2015a and the Robotics System Toolbox

window_size = 30;

rosshutdown;
rosinit;
map_sub = rossubscriber('accel_map_topic','std_msgs/Float64MultiArray');

while 1
  message = receive(map_sub);
  % showdetails(message);
  
  % acceleration map plot
  plot_data((1:15*window_size)', message.Data)
  xlabel('Samples', 'Interpreter', 'latex', 'FontSize', 14);
  ylabel('Normalized Acceleration [g]', 'Interpreter', 'latex', 'FontSize', 14);
  hold on;
  plot([1*window_size 1*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([2*window_size 2*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([3*window_size 3*window_size], [-4 4], '--', 'Color', [0.41 0.43 0.45])
  plot([4*window_size 4*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([5*window_size 5*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([6*window_size 6*window_size], [-4 4], '--', 'Color', [0.41 0.43 0.45])
  plot([7*window_size 7*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([8*window_size 8*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([9*window_size 9*window_size], [-4 4], '--', 'Color', [0.41 0.43 0.45])
  plot([10*window_size 10*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([11*window_size 11*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([12*window_size 12*window_size], [-4 4], '--', 'Color', [0.41 0.43 0.45])
  plot([13*window_size 13*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  plot([14*window_size 14*window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
  hold off;
end
