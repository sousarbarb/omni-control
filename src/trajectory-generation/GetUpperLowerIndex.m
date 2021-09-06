function [ i_0, i_f ] = GetUpperLowerIndex(i, max_i)
  i_f = max( 2 , min( ceil(i), max_i ) );
  i_0 = i_f - 1;
end