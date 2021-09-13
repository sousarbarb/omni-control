figure
color_order = get(gca, 'ColorOrder');
hold on

plot(PWM(:,1), ':', 'Color', color_order(1,:) )
plot(PWM(:,2), ':', 'Color', color_order(2,:) )
plot(PWM(:,3), ':', 'Color', color_order(3,:) )

plot(MotVolt(:,1), '-', 'Color', color_order(1,:) )
plot(MotVolt(:,2), '-', 'Color', color_order(2,:) )
plot(MotVolt(:,3), '-', 'Color', color_order(3,:) )

plot(WhW(:,1), '-.', 'Color', color_order(1,:) )
plot(WhW(:,2), '-.', 'Color', color_order(2,:) )
plot(WhW(:,3), '-.', 'Color', color_order(3,:) )

grid on
xlabel('index (1..#points) \rightarrow')
ylabel('\{PWM_{0..1023}\} , \{MotV_{V}\} , \{\omega_{rad/s}\} \rightarrow')
title("Analysis of the motors' deadzone")
