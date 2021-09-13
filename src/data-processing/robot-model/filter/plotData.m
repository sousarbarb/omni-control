function [] = plotData(Time,VRob,OldVRob,VrRob)
%PLOTDATA

  %% INITIALISATION
  [~,numWheels] = size(VRob);

  %% ROBOT SPEED
  figure
  % Linear speed
  iv = 1;
  xminplot = Time(1);
  xmaxplot = Time(end);
  yminplot = min([OldVRob(:,iv);VRob(:,iv)]);
  ymaxplot = max([OldVRob(:,iv);VRob(:,iv)]);
  yminplot = yminplot - 0.05*abs(yminplot);
  ymaxplot = ymaxplot + 0.05*abs(ymaxplot);
  subplot(1,3,iv)
  hold on
  plot(Time,OldVRob(:,iv),".-")
  plot(Time,VRob(:,iv),".-")
  plot(Time,VrRob(:,iv),"-")
  grid
  xlabel('time (s) \rightarrow')
  ylabel('v (m/s) \rightarrow')
  xlim([xminplot,xmaxplot])
  ylim([yminplot,ymaxplot])
  legend('old','filtered','ref')
  title("Old VS Filtered Robot\nSpeed - v")
  % Normal speed
  iv = 2;
  xminplot = Time(1);
  xmaxplot = Time(end);
  yminplot = min([OldVRob(:,iv);VRob(:,iv)]);
  ymaxplot = max([OldVRob(:,iv);VRob(:,iv)]);
  yminplot = yminplot - 0.05*abs(yminplot);
  ymaxplot = ymaxplot + 0.05*abs(ymaxplot);
  subplot(1,3,iv)
  hold on
  plot(Time,OldVRob(:,iv),".-")
  plot(Time,VRob(:,iv),".-")
  plot(Time,VrRob(:,iv),"-")
  grid
  xlabel('time (s) \rightarrow')
  ylabel('v_n (m/s) \rightarrow')
  xlim([xminplot,xmaxplot])
  ylim([yminplot,ymaxplot])
  legend('old','filtered','ref')
  title("Old VS Filtered Robot\nSpeed - v_n")
  % Angular speed
  iv = 3;
  xminplot = Time(1);
  xmaxplot = Time(end);
  yminplot = min([OldVRob(:,iv);VRob(:,iv)]);
  ymaxplot = max([OldVRob(:,iv);VRob(:,iv)]);
  yminplot = yminplot - 0.05*abs(yminplot);
  ymaxplot = ymaxplot + 0.05*abs(ymaxplot);
  subplot(1,3,iv)
  hold on
  plot(Time,OldVRob(:,iv),".-")
  plot(Time,VRob(:,iv),".-")
  plot(Time,VrRob(:,iv),"-")
  grid
  xlabel('time (s) \rightarrow')
  ylabel('\omega (rad/s) \rightarrow')
  xlim([xminplot,xmaxplot])
  ylim([yminplot,ymaxplot])
  legend('old','filtered','ref')
  title("Old VS Filtered Robot\nSpeed - w")
end