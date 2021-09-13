function [Xfiltered] = nonCausalAverageFilter(X,windowSize)
%NONCAUSALAVERAGEFILTER
%
%[XFILTERED] = NONCAUSALAVERAGEFILTER(X,WINDOWSIZE)
%
%Inputs:
%  X         : matrix containing the data to be filtered (this function supports
%              the filtering of several "signals").
%  WINDOWSIZE: integer that defines the size of the filter (this function 
%              supports both odd and even window sizes).
%
%Outputs:
%  XFILTERED: matrix containing the filtered data.

  %% INITIALISATION
  itf = length(X);
  [nr,nc] = size(X);
  % Evaluate if X is up or down
  if (nr == itf)
    X = X';
    numExp = nc;
  elseif (nc == itf)
    X = X;
    numExp = nr;
  else
    Xfiltered = [];
    return;
  end
  Xfiltered = zeros(size(X));
  % Evaluate the windowSize value
  if (ceil(windowSize/2) ~= floor(windowSize/2))
    isOdd = true;
  else
    isOdd = false;
  end
  
  %% DATA FILTERING
  % Odd number for the window size
  if isOdd
    for i=1:numExp
      sideLength = 0;
      for j=1:itf
        if (j < windowSize/2)
          Xfiltered(i,j) = mean(X(i,1:j+sideLength));
          sideLength = sideLength + 1;
        elseif (itf-j+1 < windowSize/2)
          sideLength = sideLength - 1;
          Xfiltered(i,j) = mean(X(i,j-sideLength:end));
        else
          Xfiltered(i,j) = mean(X(i,j-sideLength:j+sideLength));
        end
      end
    end
  % Even number for the window size
  else
    sideLength = windowSize/2;
    for i=1:numExp
      sideLength = 0;
      for j=1:itf
        if (j <= windowSize/2)
          Xfiltered(i,j) = mean(X(i,1:j+sideLength));
          sideLength = sideLength + 1;
        elseif (itf-j < windowSize/2)
          sideLength = sideLength - 1;
          Xfiltered(i,j) = mean(X(i,j-sideLength:end));
        else
          Xfiltered(i,j) = ...
            mean([ 0.5*( X(i,j-sideLength) + X(i,j+sideLength) ) , ...
                   X(i,j-sideLength+1:j+sideLength-1) ]);
        end
      end
    end
  end
  
  %% OUTPUT
  Xfiltered = Xfiltered';
end