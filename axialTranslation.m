function D = axialTranslation(s,axis)
% INPUTS:
%
% s = arbitrary length 
% axis = string specifying axis of rotation
% 
% OUTPUTS:
%
% D = homogeneous translation matrix
%
    D = eye(4,4);
    if axis == 'z'
        D(3,4) = s;
    elseif axis == 'x'
        D(1,4) = s;
    elseif axis == 'y'
        D(2,4) = s;
    else
        disp("Invalid input for 'axis' argument.  Choose from {'x', 'y', 'z'}.")
    end
end