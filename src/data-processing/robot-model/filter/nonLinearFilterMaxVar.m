function [Xfiltered] = nonLinearFilterMaxVar(X,maxXvar,T,boolLastValidSample)
%NONLINEARFILTERMAXVAR

  %% INITIALISATION
  itf = length(X);
  % Evaluate if X is up or down
  [nr,nc] = size(X);
  if (nr == itf)
    X = X;
    numExp = nc;
  elseif (nc == itf)
    X = X';
    numExp = nr;
  else
    Xfiltered = [];
    return;
  end
  Xfiltered = X;
  % Maximum variation
  maxXvar = abs(maxXvar) .* T;
  
  %% DATA FILTERING
  
  % Implementation considering the variation relative to the last valid sample
  if (boolLastValidSample)
    for j=1:numExp
      boolInvalid = false;
      iDataOkIni  = 1;
      for i=2:itf
        if ((abs(X(i,j)-X(i-1,j)) > maxXvar(j)) && (~boolInvalid))
          boolInvalid = true;
          iDataOkIni  = i-1;
        end
        if ((abs(X(i,j)-X(iDataOkIni,j)) <= maxXvar(j) * (i-iDataOkIni)) && (boolInvalid))
          boolInvalid = false;
          iDataOkFin  = i;
          Xfiltered(iDataOkIni+1:iDataOkFin-1,j) = ones(iDataOkFin-iDataOkIni-1,1) .* X(iDataOkIni,j);
        end
      end
    end
  
  % Implementation considering the variation between samples
  else
    for j=1:numExp
      boolInvalid = false;
      iDataOkIni  = 1;
      for i=2:itf
        if ((abs(X(i,j)-X(i-1,j)) >  maxXvar(j)) && (~boolInvalid))
          boolInvalid = true;
          iDataOkIni  = i-1;
        end
        if ((abs(X(i,j)-X(i-1,j)) <= maxXvar(j)) && (boolInvalid))
          boolInvalid = false;
          iDataOkFin  = i;
          Xfiltered(iDataOkIni+1:iDataOkFin-1,j) = ones(iDataOkFin-iDataOkIni-1,1) .* X(iDataOkIni,j);
        end
      end
    end
  end
end