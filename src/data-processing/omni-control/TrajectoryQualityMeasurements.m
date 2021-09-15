function [Error] = TrajectoryQualityMeasurements(Time, Xpos, Xpos_r, range)
%TRAJECTORYQUALITYMEASUREMENTS

  if isempty(range)
    
    % Quality measures
    n = length(Time);
    % - Average absolute error
    Error.avg_abs_x  = mean( abs( Xpos_r(:,1) - Xpos(:,1) ) );
    Error.avg_abs_y  = mean( abs( Xpos_r(:,2) - Xpos(:,2) ) );
    Error.avg_abs_th = rad2deg( mean( abs( wrapToPi( Xpos_r(:,3) - Xpos(:,3) ) ) ) );
    Error.avg_abs_dist = mean( sqrt( (Xpos_r(:,1) - Xpos(:,1)).^2 + (Xpos_r(:,2) - Xpos(:,2)).^2 ) );
    % - Maximum absolute error
    [ Error.max_abs_x  , ix  ] = max( abs( Xpos_r(:,1) - Xpos(:,1) ) );
    [ Error.max_abs_y  , iy  ] = max( abs( Xpos_r(:,2) - Xpos(:,2) ) );
    [ Error.max_abs_th , ith ] = max( abs( wrapToPi( Xpos_r(:,3) - Xpos(:,3) ) ) );
    Error.max_abs_th = rad2deg( Error.max_abs_th );
    [ Error.max_abs_dist , idist ] = max( sqrt( (Xpos_r(:,1) - Xpos(:,1)).^2 + (Xpos_r(:,2) - Xpos(:,2)).^2 ) );
    Error.max_abs_x_t = Time(ix);
    Error.max_abs_y_t = Time(iy);
    Error.max_abs_th_t = Time(ith);
    Error.max_abs_dist_t = Time(idist);
    % - Trajectory error
    Error.trajectory = zeros(n,1);
    for i=1:n
      Error.trajectory(i) = min( sqrt( (Xpos_r(:,1) - Xpos(i,1)).^2 + (Xpos_r(:,2) - Xpos(i,2)).^2 ) );
    end
    Error.avg_trajectory = mean(Error.trajectory);
    Error.max_trajectory = max(Error.trajectory);
    
  else
    
    % Quality measures
    n = range(2) - range(1) + 1;
    i1 = range(1);
    i2 = range(2);
    % - Average absolute error
    Error.avg_abs_x  = mean( abs( Xpos_r(i1:i2,1) - Xpos(i1:i2,1) ) );
    Error.avg_abs_y  = mean( abs( Xpos_r(i1:i2,2) - Xpos(i1:i2,2) ) );
    Error.avg_abs_th = rad2deg( mean( abs( wrapToPi( Xpos_r(i1:i2,3) - Xpos(i1:i2,3) ) ) ) );
    Error.avg_abs_dist = mean( sqrt( (Xpos_r(i1:i2,1) - Xpos(i1:i2,1)).^2 + (Xpos_r(i1:i2,2) - Xpos(i1:i2,2)).^2 ) );
    % - Maximum absolute error
    [ Error.max_abs_x  , ix  ] = max( abs( Xpos_r(i1:i2,1) - Xpos(i1:i2,1) ) );
    [ Error.max_abs_y  , iy  ] = max( abs( Xpos_r(i1:i2,2) - Xpos(i1:i2,2) ) );
    [ Error.max_abs_th , ith ] = max( abs( wrapToPi( Xpos_r(i1:i2,3) - Xpos(i1:i2,3) ) ) );
    Error.max_abs_th = rad2deg( Error.max_abs_th );
    [ Error.max_abs_dist , idist ] = max( sqrt( (Xpos_r(i1:i2,1) - Xpos(i1:i2,1)).^2 + (Xpos_r(i1:i2,2) - Xpos(i1:i2,2)).^2 ) );
    Error.max_abs_x_t = Time(i1 + ix - 1);
    Error.max_abs_y_t = Time(i1 + iy - 1);
    Error.max_abs_th_t = Time(i1 + ith - 1);
    Error.max_abs_dist_t = Time(i1 + idist - 1);
    % - Trajectory error
    Error.trajectory = zeros(n,1);
    j = 1;
    for i=i1:i2
      Error.trajectory(j) = min( sqrt( (Xpos_r(:,1) - Xpos(i,1)).^2 + (Xpos_r(:,2) - Xpos(i,2)).^2 ) );
      j = j + 1;
    end
    Error.avg_trajectory = mean(Error.trajectory);
    Error.max_trajectory = max(Error.trajectory);
    
  end
end