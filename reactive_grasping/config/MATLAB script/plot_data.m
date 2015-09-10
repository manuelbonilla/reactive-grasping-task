% Plots data against time.
% - x: x-axis data (column vector/s);
% - y: x-axis data (column vectors, same length of 'x');
% - id_ref: the y-data-column-id of the first reference. If not specified
% it is assumed that there is no reference signal, otherwise it is assumed
% that at least y has 2*n columns, where n is the number of signals to be
% plotted (x can be a single column vector).
%
% Note: if y has many columns and x just one, it is assumed that every data
% is referred to that vector (like a multi-data plot against common time).

function [] = plot_data(x, y, id_ref)
  if nargin < 3
    id_ref = size(y,2) + 1;
  end

  x_min = min(min(x));
  x_max = max(max(x));
  y_min = min(min(y));
  y_max = max(max(y));
  x_margins = abs(x_max-x_min)/20;
  y_margins = abs(y_max-y_min)/20;

  figure_size = [900 300];
  if isempty(ishandle(findobj('type','figure','name','fig_accel_map')))
    figure('Name', 'fig_accel_map', ...
           'Color', [1 1 1], 'Units', 'pixels', ...
           'Position', [100 100 figure_size(1) figure_size(2)]);
  end
  cla;
  set(gca, 'Units', 'pixels', ...
           'Position', [60 45 figure_size(1)-90 figure_size(2)-75]);

  colors = get(gca, 'ColorOrder');
  hold on;
  grid on;
  
  while id_ref > size(colors,1)
    colors = [colors; colors*0.8];
  end

  for i = 1:id_ref-1
    if size(x,2) > 1
      plot(x(:,i), y(:,i), 'Color', colors(i,:), 'LineWidth', 0.5);
    else
      plot(x(:,1), y(:,i), 'Color', colors(i,:), 'LineWidth', 0.5);
    end
  end

  for i = id_ref:size(y,2)
    if size(x,2) > 1
      plot(x(:,i), y(:,i), '--', 'Color', colors(i-id_ref+1,:), 'LineWidth', 0.1);
    else
      plot(x(:,1), y(:,i), '--', 'Color', colors(i-id_ref+1,:), 'LineWidth', 0.1);
    end
  end

  axis([x_min-x_margins x_max+x_margins y_min-y_margins y_max+y_margins]);

  hold off;
end
