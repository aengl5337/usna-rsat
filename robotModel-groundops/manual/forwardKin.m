%% forwardKin.m
% Simulates a serial manipulator based on D-H params and joint variables

%{
TODO:
() + Add standard D-H version
() + Display option?
() - appears fwd kinematics are negative what they should be -- why?
(X) -remove dependence on unnecessary calculations
() -combine loops
() -remove inputMode -- phased out by THETAREF in higher wrappers
%}



function T0_i = forwardKin(q, q_types, D_H, D_H_type, inputMode) 
% INPUTS:
%
% q = column vector of joint variables
% q_types = vector of of booleans indexed by joint specifying {revolute = 0, prismatic = 1}
% D_H = Matrix of (intrinsic) Denavit-Hartenberg parameters:
%       If standard, row i: [theta_i,d_i,alpha_i,a_i]
%       If modified, row i: [alpha_(i-1),a_(i-1),theta_i,d_i]
% D_H_type = string specifying Denavit-Hartenberg notation version
% {"std","mod"}
% inputMode = string specifying if joint variables are to be applied
% absolutely or relatively {"abs","rel"}
% 
% OUTPUTS:
%
% T0_i = homogeneous basis transformation matrix betwen frame/joint {i} and {0}
%

    N = numel(q);
    % Extract parameters by columns->rows and add joint variables
    alpha = D_H(:,1)';
    a = D_H(:,2)';
    theta0 = D_H(:,3)';
    d0 = D_H(:,4)';
    % Ensure that all vectors are rows, since doing element-wise
    % multiplication
    q = reshape(q,1,N);
    q_types = reshape(q_types,1,N);
    
    % Interpret inputs based on inputMode
    % Know that entries of q_types are {0,1} for {revolute,prismatic}
    % joints, respectively
    % If rel:
    %   theta = q_fwd + theta0
    % If abs:
    %   theta = q_fwd
    if D_H_type=="mod"
        if inputMode == "rel"
            theta = not(q_types).*q + theta0; % Adds joint variable if revolute
            d = q_types.*q + d0;          % Adds joint variable if prismatic
        elseif inputMode == "abs"
            theta = not(q_types).*q + q_types.*theta0; % Resets joint variable if revolute absolute
            d = q_types.*q + not(q_types).*d0;          % Resets joint variable if prismatic absolute
        else
            disp("Not a valid entry for inputMode")
            return
        end
        
        % Create storage containers
        % This allows us to work flexibly with class sym vars
        c = class(q);
        if c=="sym" % Make a string comparison, as if comparison is char to char it will act as if it's an array
            Tadj =sym('Tadj',[4,4,N]);
            T0_i = sym('T0_i',[4,4,N]);
        else
            Tadj =zeros(4,4,N); % Adjacent basis transformations, (i) to (i-1)
            T0_i = zeros(4,4,N); % Initialize overall basis transformation matrix from {6} to {0}
        end

        for i = 1:N % From [1,7]
            % Assign the overall transformation to a data cube, layer indexed
            % by joint [from 6 to 1]
            
            % Emulating equation (3.4) from Craig:
            % ^(i-1)_(i)T = R_x(alpha_i-1)D_x(a_i-1)R_z(theta_i)D_z(d_i)
%             R_alpha = planarRotation(alpha(i),'x');
%             D_a = axialTranslation(a(i),'x');
%             R_theta = planarRotation(theta(i),'z');
%             D_d = axialTranslation(d(i),'z');
% 
%             Tadj1 = R_alpha*D_a*R_theta*D_d;

            % Same as the above, written out explicitly, more efficient
            Tadj(:,:,i) = [cosd(theta(i))                -sind(theta(i))                 0                a(i);                   
                           sind(theta(i))*cosd(alpha(i))  cosd(theta(i))*cosd(alpha(i)) -sind(alpha(i))  -d(i)*sind(alpha(i));
                           sind(theta(i))*sind(alpha(i))  cosd(theta(i))*sind(alpha(i))  cosd(alpha(i))   d(i)*cosd(alpha(i));
                           0                              0                              0                1];
%             Tadj1==Tadj(:,:,i)
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
        
        for j=1:N % Incrementally matrix-multiply all transformations together, from joint 6 to 1
            if j==1
                T0_i(:,:,j) = Tadj(:,:,j);
            else
                T0_i(:,:,j) = T0_i(:,:,j-1)*Tadj(:,:,j);
            end            
        end
        
    end
end
