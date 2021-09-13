function [Xfiltered] = nonLinearFilterMaxDiffRef(X,Xref,maxDiffRefRel)
%NONLINEARFILTERDIFFREF

  %% INITIALISATION
  itf = length(X);
  % Evaluate if X is up or down
  [nr,nc] = size(X);
  if (nr == itf)
    X = X;
    Xref = Xref;
    numExp = nc;
  elseif (nc == itf)
    X = X';
    Xref = Xref';
    numExp = nr;
  else
    Xfiltered = [];
    return;
  end
  Xfiltered = X;
  % Maximum variation
  maxDiffRefRel = abs(maxDiffRefRel);
  
  %% DATA FILTERING
  
  % Values greater than the reference (no need to worry on transient states)
  for j=1:numExp
    boolInvalid = false;
    iDataOkIni  = 1;
    for i=2:itf
      if ((sign(Xref(i,j)) * (X(i,j) - Xref(i,j)) > maxDiffRefRel(j) * abs(Xref(i,j))) && (Xref(i,j) ~= 0) && (~boolInvalid))
        boolInvalid = true;
        iDataOkIni  = i-1;
      end
      if ((sign(Xref(i,j)) * (X(i,j) - Xref(i,j)) <= maxDiffRefRel(j) * abs(Xref(i,j))) && (Xref(i,j) ~= 0) && (boolInvalid))
        boolInvalid = false;
        iDataOkFin  = i;
        Xfiltered(iDataOkIni+1:iDataOkFin-1,j) = ones(iDataOkFin-iDataOkIni-1,1) .* X(iDataOkIni,j);
      end
    end
    if (boolInvalid)
      boolInvalid = false;
      iDataOkFin  = i;
      Xfiltered(iDataOkIni+1:iDataOkFin-1,j) = ones(iDataOkFin-iDataOkIni-1,1) .* X(iDataOkIni,j);
    end
  end
  
%   % Update data
%   X = Xfiltered;
%   
%   % Values lower than the reference (it is necessary to worry about transient states)
%   for j=1:numExp
%     boolInvalid   = false;
%     boolTransient = true;
%     iDataOkIni  = 1;
%     
%     for i=2:itf
%       % Check if transient state finished
%       if (sign(Xref(i,j)) * mean(X(max(1,i-3):min(itf,i+3),j)) > abs(Xref(i,j)))
%         boolTransient = false;
%       end
%       % Check if it will occur a new transsient state
%       if (Xref(i,j) ~= Xref(i-1,j))
%         boolTransient = true;
%         if (boolInvalid)
%           boolInvalid = false;
%           iDataOkFin  = i;
%           Xfiltered(iDataOkIni+1:iDataOkFin-1,j) = ones(iDataOkFin-iDataOkIni-1,1) .* X(iDataOkIni,j);
%         end
%       end
%       
%       if ((~boolTransient) && (sign(Xref(i,j)) * (X(i,j) - Xref(i,j)) < maxDiffRefRel(j) * abs(Xref(i,j))) && (Xref(i,j) ~= 0) && (~boolInvalid))
%         boolInvalid = true;
%         iDataOkIni  = i-1;
%       end
%       if ((~boolTransient) && (sign(Xref(i,j)) * (X(i,j) - Xref(i,j)) >= maxDiffRefRel(j) * abs(Xref(i,j))) && (Xref(i,j) ~= 0) && (boolInvalid))
%         boolInvalid = false;
%         iDataOkFin  = i;
%         Xfiltered(iDataOkIni+1:iDataOkFin-1,j) = ones(iDataOkFin-iDataOkIni-1,1) .* X(iDataOkIni,j);
%       end
%     end
%     if (boolInvalid)
%       boolInvalid = false;
%       iDataOkFin  = i;
%       Xfiltered(iDataOkIni+1:iDataOkFin-1,j) = ones(iDataOkFin-iDataOkIni-1,1) .* X(iDataOkIni,j);
%     end
%   end
end