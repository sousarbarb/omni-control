function [Xth] = OrientationInterpolationLinear(trajectory, Xpos)
%ORIENTATIONINTERPOLATIONCONSTANT

  % Initialization
  Xth = [];
  Xth = [ Xth ; trajectory(1,3) ];
  [numwp,~]  = size(trajectory);
  j = 2;
  jini = 1;
  
  % Constant interpolation
  for i = 2:numwp
    while true
      if ((Xpos(j,1) == trajectory(i,1)) && (Xpos(j,2) == trajectory(i,2)))
        th1 = trajectory(i-1,3);
        th2 = trajectory(i  ,3);
        if (abs(th2 - th1) > pi)
          th2 = th1 + wrapToPi( th2 - th1 );
        end
        linearization = linspace( th1 , th2 , j - jini + 1 );
        linearization = wrapToPi(linearization(2:end));
        Xth = [ Xth ; linearization' ];
        jini = j;
        j = j+1;
        break;
      end
      j = j+1;
    end
  end
end

