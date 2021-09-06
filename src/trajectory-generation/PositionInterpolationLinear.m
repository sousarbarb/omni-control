function [Xpos] = PositionInterpolationLinear(trajectory, dist_points)
%POSITIONINTERPOLATIONLINEAR

  % Initialization
  Xpos = [];
  Xpos = [ Xpos ; trajectory(1,1:2) ];
  [numwp,~]  = size(trajectory);
  j    = 1;
  
  % Linear interpolation
  for i = 2:numwp
    while true
      dir = atan2( trajectory(i,2) - trajectory(i-1,2)  ,  trajectory(i,1) - trajectory(i-1,1) );
      temppoint    = Xpos(j,1:2);
      temppoint(1) = temppoint(1) + dist_points*cos(dir);
      temppoint(2) = temppoint(2) + dist_points*sin(dir);
      if Dist( trajectory(i,:) , temppoint ) < 0.99*dist_points
        Xpos = [ Xpos ; trajectory(i,1:2) ];
        j = j+1;
        break;
      end
      Xpos = [ Xpos ; temppoint ];
      j = j+1;
    end
  end
end

