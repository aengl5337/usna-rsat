function [T0_6s,Nsol] = tgtWtask2goal(T0_tgt,TASK)
% INPUTS
% T0_tgt
% TASK = {"moveToPoint","inspection","grab","handoff"}
% OUTPUTS
% T0_6s
% Nsol = number of solutions
    %% Define desired pose offset between goal frame and target frame based on assigned task
    
    if TASK == "moveToPoint"
        % Specify transform to (goal) (frame at the tip of the end
        % effector) from (6) (motor6)
        Pgoal_6 = [0 0 -3*obj.lEE]; % FIX THIS (find actual measurement)
        Tgoal_6 = [eye(3,3)   Pgoal_6;
                   zeros(1,3) 1];
       
        % POSITION
        Ptgt_goal = zeros(3,1); % Aligned
        % ROTATION
        Nsol = 4;
        % NOTE: since motor 6's axis points outwards through the wrist, must negate it to get an outward facing normal for the QR code
        % Thus 4 possible orientations that orient the 6 frame 'squarely' to the given pose
        thetasSquare = [0,90,180,360];
        T0_6s = zeros(4,4,Nsol);
        for ii = 1:Nsol
            theta = thetasSquare(ii);
            R2D = [cosd(theta) -sind(theta);
                   sind(theta)  cosd(theta)];
            Rtgt_goal = [R2D zeros(2,1);
                         zeros(1,2) -1];
            Ttgt_goal = [Rtgt_goal  Ptgt_goal;
                         zeros(1,3) 1];
            T0_6s(:,:,ii) = T0_tgt*Ttgt_goal*Tgoal_6;
        end
    else
        disp("Other tasks not yet supported")
end