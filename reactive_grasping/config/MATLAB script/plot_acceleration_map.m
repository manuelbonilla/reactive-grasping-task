% Plots Acceleration map

function [] = plot_acceleration_map(data)
  window_size = length(data)/15;

  % acceleration map plot
  plot_data((1:length(data))', data)
  xlabel('Samples', 'Interpreter', 'latex', 'FontSize', 14);
  ylabel('Acceleration [g]', 'Interpreter', 'latex', 'FontSize', 14);
  hold on;
  plot([window_size window_size], [-4 4], '--', 'Color', [0.929 0.694 0.125])
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
