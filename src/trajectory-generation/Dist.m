function [distance] = Dist(X1, X2)
  distance = sqrt( sum( ( X1(1:2) - X2(1:2) ).^2 ) );
end