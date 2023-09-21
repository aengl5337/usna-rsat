function R = planarRotation(theta, axis)
% INPUTS:
%
% theta = float, angle of planar rotation, degrees
% plane = string, plane of rotation, e.g. 'xy'
%
% OUTPUTS:
%
% R = homogeneous rotation matrix
%
    
    R = [ cosd(theta),-sind(theta);
          sind(theta), cosd(theta)];
    
    if axis == 'z'
        R=[R, zeros(2,1);
            zeros(1,2), 1];
    elseif axis == 'x'
        R=[1, zeros(1,2);
            zeros(2,1), R];
    elseif axis == 'y'
        R=[ R(2,2), 0, R(2,1);
            0,      1, 0;
            R(1,2), 0, R(1,1)];
    else
        disp("Invalid input for 'axis' argument.  Choose from {'x', 'y', 'z'}.")
    end
    
    R = [R  zeros(3,1);
         zeros(1,3), 1]; % Convert to 4x4 homogenous form
end
