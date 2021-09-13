function [Xavg] = ComputeAverage(X, i_0, i_f)

  % Initialization
  numpoints = length(X);
  
  % Check indexes
  i_0 = max( min(i_0, numpoints) , 0);
  i_f = max( min(i_f, numpoints) , 0);
  if (i_f - i_0 == 0)
    exit;
  end

  % Compute average
  Xavg = sum( X(i_0:i_f,:) ) / (i_f - i_0 + 1);

end