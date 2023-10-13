%{
TODO:
() + Add standard D-H version
() + Display option?
() - appears fwd kinematics are negative what they should be -- why?
%}



function T0_i = forwardKin(q, q_types, D_H, D_H_type) 
% INPUTS:
%
% q = column vector of joint variables
% q_types = vector of of booleans indexed by joint specifying {revolute = 0, prismatic = 1}
% D_H = Matrix of (intrinsic) Denavit-Hartenberg parameters:
%       If standard, row i: [theta_i,d_i,alpha_i,a_i]
%       If modified, row i: [alpha_(i-1),a_(i-1),theta_i,d_i]
% D_H_type = string specifying Denavit-Hartenberg notation version
% {"std","mod"}
% 
% OUTPUTS:
%
% T0_i = homogeneous basis transformation matrix betwen frame/joint {i} and {0}
%


    N = numel(q);
    
    if D_H_type=="mod"
    
        % Extract parameters by columns and add joint variables
        alpha = D_H(:,1);
        a = D_H(:,2);
        theta = not(q_types).*q + D_H(:,3); % Adds joint variable if revolute
        d = q_types.*q + D_H(:,4);          % Adds joint variable if prismatic

        Tadj =zeros(4,4,N); % Adjacent basis transformations, (i) to (i-1)

        for i = 1:N % From [1,7]
            % Assign the overall transformation to a data cube, layer indexed
            % by joint [from 6 to 1]
            
            % Emulating equation (3.4) from Craig:
            % ^(i-1)_(i)T = R_x(alpha_i-1)D_x(a_i-1)R_z(theta_i)D_z(d_i)
            R_alpha = planarRotation(alpha(i),'x');
            D_a = axialTranslation(a(i),'x');
            R_theta = planarRotation(theta(i),'z');
            D_d = axialTranslation(d(i),'z');
            
%             Tadj(:,:,i) = R_alpha*D_a*R_theta*D_d;
            Tadj(:,:,i) = [cosd(theta(i))                -sind(theta(i))                 0                a(i);                   
                           sind(theta(i))*cosd(alpha(i))  cosd(theta(i))*cosd(alpha(i)) -sind(alpha(i))  -d(i)*sind(alpha(i));
                           sind(theta(i))*sind(alpha(i))  cosd(theta(i))*sind(alpha(i))  cosd(alpha(i))   d(i)*cosd(alpha(i));
                           0                              0                              0                1];
        end

        %{
        figure(1)

        hold on
        for j=1:nMotors
            x=r(1,j);
            y=r(2,j);
            z=r(3,j);
            plot3(x,y,z,'ro')
        end
        %}
        T0_i = zeros(4,4,N); % Initialize overall basis transformation matrix from {6} to {0}
        for j=1:N % Incrementally matrix-multiply all transformations together, from joint 6 to 1
            if j==1
                T0_i = Tadj(:,:,j);
            else
                T0_i(:,:,j) = T0_i(:,:,j-1)*Tadj(:,:,j);
            end            
        end
        
    end
end
