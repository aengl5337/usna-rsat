function qfcn pBlendLin(q,t0,t1,thdd,tb)
% INPUTS
% q = 
% t0,t1 = start,end times respectively
% thdd = acceleration for blend segments; if NaN then specifies linear
% interpolation
% tb = duration of parabolic blend at either end

% OUTPUTS
% Qfcn = array of joint variable functions, row = joint index, column = spline index
%     NOTE: jth spline span
%     as a result will have one fewer column than