function [X] = GenerateTrajectoryOmni3(trajectory, parameters)

  % Initialization
  [trajectory_numpoints,~] = size(trajectory);

  % - unwrap the trajectory orientation
  for i = 2 : trajectory_numpoints
    trajectory(i,3) = trajectory(i-1,3) + ...
                      wrapToPi(trajectory(i,3) - trajectory(i-1,3));
  end

  % - nominal distance
  distn = parameters.vn * parameters.Tctrl;



  % Generate trajectory without considering the acceleration limits
  % - (x,y) positions:
  [Xpos] = PositionInterpolationLinear(trajectory, distn);
  [Xn_numpoints,~] = size(Xpos);

  % - orientation:
  [Xth] = OrientationInterpolationLinear(trajectory, Xpos);

  for i = 2 : Xn_numpoints
    Xth(i) = Xth(i-1) + ...
             wrapToPi(Xth(i) - Xth(i-1));
  end

  % - interpolated trajectory
  Xn = [ Xpos , Xth ]

  % - compute distance and angular displacement from the end to the trajectory's
  %   start (distance , angular displacement)
  displacement = zeros(Xn_numpoints, 2);

  for i = Xn_numpoints-1 : -1 : 1
    % linear displacement
    displacement(i,1) = displacement(i+1,1) + ...
                        Dist( Xn(i+1,:) , Xn(i,:) );

    % angular displacement
    displacement(i,2) = displacement(i+1,2) + ...
                        abs(wrapToPi( Xn(i+1,3) - Xn(i,3) ));
  end



  % Generate trajectory with acceleration limits
  % - initial pose
  u = 1;
  X = Xn(1,:);

  % - velocity and parameters
  parameters.v_inc = parameters.acc_up   * parameters.Tctrl;
  parameters.v_dec = parameters.acc_down * parameters.Tctrl;
  parameters.dist_dec = ...
      (( parameters.vn-parameters.v_min ) ^ 2) / (2*parameters.acc_down) + ...
      parameters.v_min * ( parameters.vn-parameters.v_min ) / parameters.acc_down;
  v = parameters.v_inc;
  state = 0;  % 0:acceleration ; 1:nominal ; 2: deceleration; 3:minimum velocity

  % - interpolation:
  while u < Xn_numpoints
    u_prev  = u;

    % compute displacement from current velocity
    delta_dist = v * parameters.Tctrl;

    % update indexes
    delta_u = delta_dist / distn;
    u = u + delta_u;



    % compute new pose to add to the generated trajectory
    [ u_0, u_f ] = GetUpperLowerIndex(u, Xn_numpoints);
    Xn_u = Xn(u_0,:) + ...
           ( Xn(u_f,:) - Xn(u_0,:) ) * ( u - u_0 ) / ( u_f - u_0);
    X = [ X ; Xn_u ];



    % state machine (update the velocity)
    switch state
    case 0
      v = v + parameters.v_inc;

      if (displacement(u_f,1) + Dist(Xn_u, Xn(u_f,:)) <= parameters.tol_xy_ctrl)
        state = 3;
        v = parameters.v_min;
      elseif (displacement(u_f,1) + Dist(Xn_u, Xn(u_f,:)) < parameters.dist_dec + parameters.tol_xy_ctrl)
        state = 2;
        v = v - parameters.v_dec;
      elseif  (v > parameters.vn)
        state = 1;
        v = parameters.vn;
      end
    case 1
      v = parameters.vn;

      if (displacement(u_f,1) + Dist(Xn_u, Xn(u_f,:)) <= parameters.tol_xy_ctrl)
        state = 3;
        v = parameters.v_min;
      elseif (displacement(u_f,1) + Dist(Xn_u, Xn(u_f,:)) < parameters.dist_dec + parameters.tol_xy_ctrl)
        state = 2;
        v = v - parameters.v_dec;
      end
    case 2
      v = v - parameters.v_dec;

      if ((displacement(u_f,1) + Dist(Xn_u, Xn(u_f,:)) < parameters.tol_xy_ctrl) || (v < parameters.v_min))
        state = 3;
        v = parameters.v_min;
      end
    case 3
      v = parameters.v_min;
    end

  end

  % - add final pose
  X = [ X ; Xn(end,:) ];
end