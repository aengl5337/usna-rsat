%{
TODO
() + Make incremental and absolute input versions (utilize inputMode syntax
for fwdKin and use theta0 to store current position)
() - Display error flexibly with separate function from pieperInvKin
() + Add plotRobotAxes to the class
() + Add THETAREF convertor
%}

classdef RSAT
    properties
        l2
        l4
        l4p
        l6
        t
        RECSmount_w
        RECSmount_l
        RECS_l
        lpinch
        lshaftOffset
        lee
        lindex
        T6_ee
        D_H_type
        a
        alpha
        theta0ext
        theta0stow
        theta0deploy
        theta0abs
        d
        D_H
        N
        q_types 
        mountMult
        MALimRng_relext
        thetaLimRng_relext
        thetaLimRng_relabs
        thetaLimRng_ee
        D_Hrad
        robot
    end
    
    methods
        function obj = RSAT() % INITPOS Constructor
%             if INITPOS == "EXT"
%                 theta0 = theta0ext;
%             elseif INITPOS == "STOW"
%                 theta0 = theta0stow;
%             else % Catch all misinputs
%                 disp("Bad input for INITPOS")
%                 return;
%             end

%             path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
                
            % RECS MOUNTING PARAMS (all dims mm)
            % All (including unlabeled) dims pulled directly from CAD
            % assembly
            obj.RECSmount_w = 55; % mm
            obj.RECSmount_l = 302.7;
            obj.RECS_l = 327.71;
            obj.t = 1.98; % Thickness of RECS shell
            
            % ARM PARAMS (for kinematics)
            % Specify joint types
            obj.N = 6;
            obj.q_types = zeros(obj.N,1); % specify 0 for each revolute joint, and 1 for each prismatic one
            
            % Lengths (mm)
            obj.l2 = 312.24;
            obj.l4 = 182.02; % l3
            obj.l4p = 7.99; % l3p
            obj.l6 = 76.6; %l5

            % Denavit-Hartenberg Parameters (Modified -- Craig*)
            obj.D_H_type = "mod"; % Possible vals: {"mod", "std} -- currently only support modified
            % Link twist angles (alpha, deg)
            obj.alpha = 90*[0 1 1 1 1 -1]'; % i-1
            % Link Lengths (a, mm)
            obj.a = [0 0 0 obj.l4p 0 0]'; % i-1
            % (Intrinsic) Joint angles (theta, deg) -- all reference theta0abs = 0
            obj.theta0ext = 90*[0 1 0 2 0 0]'; % i; these correspond to the 'extended' position
            obj.theta0stow = obj.theta0ext + [0 0 -180 0 +7.2 0]';
            obj.theta0deploy = obj.theta0stow + [40 -90 -40 0 0 0]';

            obj.theta0abs =  zeros(obj.N,1);

            % Link offsets (d, mm)
            obj.d = [0 -obj.l2 0 obj.l4 0 obj.l6]'; % i

            % Modified D-H parameters
            obj.D_H = [obj.alpha,obj.a,obj.theta0abs,obj.d];
            
            % MOUNTING PARAMS
            % Relating motor command sense to kinematic sense based on mounting
            % mountMult(i) = +1 => segment (i) is mounted on motor shaft
            % mountMult(i) = -1 => segment (i) is mounted on motor body 
                        
            obj.mountMult = [-1 -1 1 1 1 1]';
            
            % JOINT LIMITS
            % Angle limits for each joint, relative to the extended position (INITPOS="EXT") (deg)
            obj.MALimRng_relext = [-255,0;        % **guess**
                           -180,180;
                           -180,0;
                           -180,180;
                           -123.95,34.29; % referenced from previous arm's centerline (i.e. extended position)
                           -180,180];
            
            obj.thetaLimRng_relext = zeros(obj.N,2);
            for ii = 1:obj.N
                if obj.mountMult(ii) == -1
                    obj.thetaLimRng_relext(ii,1) = -1*obj.MALimRng_relext(ii,2);
                    obj.thetaLimRng_relext(ii,2) = -1*obj.MALimRng_relext(ii,1);
                else
                    obj.thetaLimRng_relext(ii,:) = obj.MALimRng_relext(ii,:);
                end
            end
            
            obj.thetaLimRng_ee= [-45,135];    % **guess**, note this 7th entry is for the end effector motor!
            % Angle lims relative to the "absolute" position (i.e. theta0 = 0)
            obj.thetaLimRng_relabs = obj.thetaLimRng_relext + obj.theta0ext;
            
            % END EFFECTOR
            % Define transformation into end effector frame
            delta = 2.6; % Between 2.1 and 3.1mm
            obj.lshaftOffset = 7.8+delta;
            obj.lee = obj.lshaftOffset + 15.6/2; % distance to the end effector axis = length of the shaft + half of the width of the motor casing prism
            
            P6_ee = [0 0 obj.lee]';
            R6_ee = [0 -1 0;
                     0 0 -1;
                     1 0 0];
            obj.T6_ee = [R6_ee      P6_ee;
                         zeros(1,3) 1];
            
            obj.lpinch= 44.5-obj.lee;  % Measured from ee axis to pinch point* (define)
            obj.lindex = obj.lpinch+13.7; % Measured from ee axis to end of end effector
            
            
            obj.robot = obj.makeRBT;
            
        end
        
        function rbt = makeRBT(obj)
            % convert to MATLAB's rigidBodyTree type (move this into separate function)
%             obj.D_Hrad = [(pi/180)*obj.alpha,obj.a,(pi/180)*obj.theta0abs,obj.d];
            % Reindex from i-1
            aim1 = obj.a;
            alphaim1 = (pi/180)*obj.alpha;
            di = obj.d;
            thetai = (pi/180)*obj.theta0abs;
            
            dhparams = [aim1 alphaim1 di thetai];
            
            rbt = DH2rigidBodyTree(dhparams,obj.D_H_type,obj.N,obj.q_types);
            % Add another body for the end effector
            ee = robotics.RigidBody('ee');
            ee.Joint = robotics.Joint('ee','fixed');
            setFixedTransform(ee.Joint,obj.T6_ee); % check to make sure this is joint2parent
            % sets the JointToParentTransform property of the rigidBodyJoint object directly with the specified homogenous transformation, tform
            addBody(rbt,ee,rbt.Bodies{1,end}.Name) % connect ee to rbt.Bodies{1,6}
        end

        
        function theta0 = getThetaRef(obj,THETAREF)
            % All of the following are referencing 'absolute' theta (i.e. theta0abs =  zeros(obj.N,1);)
            % THETAREF = {"ext","stow","deploy","abs"} to reflect predetermined reference positions
            if THETAREF == "ext"
                theta0 = obj.theta0ext;
            elseif THETAREF == "stow"
                theta0 = obj.theta0stow;
            elseif THETAREF == "deploy"
                theta0 = obj.theta0deploy;
            elseif THETAREF == "abs"
                theta0 = obj.theta0abs;
            else
                disp("Invalid input for THETAREF")
            end
        end
        
        function thetas = motorAngle2theta(obj,motorAngles) % Works as theta2motorAngle as well
            thetas = reshape(obj.mountMult,6,1).*reshape(motorAngles,6,1); % Ensure is a column vector
        end
        
        function TRECS_d = D435toRECS(obj)
            RRECS_d = [-1 0  0;
                       0 1  0;
                       0 0 -1];
            z_back = obj.t + 32.24; % Backplane of D435, mm
            z_POM = z_back; % Plane of measurement of D435 ** ADD OFFSET
            PRECS_d = [0 0 z_POM]';
            TRECS_d = [RRECS_d  PRECS_d;
                       zeros(1,3) 1];
        end
  
        function TRECS_0 = arm2RECS(obj, whichArm)
            x_hole2mount = (22.48 - obj.t);
            motorHousingRadius = 13.08;
            x_mount2motorHousing = 6.6;
            x_mount2axis = -(x_mount2motorHousing+motorHousingRadius);
            xarm = obj.RECSmount_w/2 + x_hole2mount + x_mount2axis;
            
            y_side2mount = -.65;
            y_mount2axis = -7.5;
            yarm = obj.RECS_l/2 + y_side2mount + y_mount2axis;
            
            z_plate2mount = obj.t + 90.79;
            z_mount2axis = -7.5;
            zarm =  z_plate2mount + z_mount2axis;
            
            if whichArm == 1
                RRECS_0 = [0 0 -1;
                           0 1 0;
                           1 0 0];
                PRECS_0 = [-xarm -yarm zarm]';
            elseif whichArm == 2
                RRECS_0 = [0 0 1;
                           0 -1 0;
                           1 0 0];
                PRECS_0 = [xarm yarm zarm]';                
            else
                disp("Invalid selection for whichArm")
            end            
            TRECS_0 = [RRECS_0    PRECS_0;
                       zeros(1,3) 1];
        end
        
        function T0_i = fwdKin(obj, theta_fwd_rel, THETAREF)
            % Basically just a wrapper for forwardKin.m for ease of use in 
            % RSAT use case
            % THETAREF = {"ext","stow","deploy","abs"} to reflect predetermined positions
            
            %{
            TODO
            () + Add end effector pose as output (ensure that all uses of
            fwdKin account for this!
            (X) - Change inputMode to THETAREF
            %}
            
                        
            theta0 = obj.getThetaRef(THETAREF);
            theta_fwd_abs = theta_fwd_rel + theta0;
            inputMode = "abs";
            T0_i = forwardKin(theta_fwd_abs, obj.q_types, obj.D_H, obj.D_H_type, inputMode);
        end
        
        function theta_invs_rel = invKin(obj, T0_6, THETAREF) % To specified pose, no offset
            % THETAREF = {"ext","stow","deploy","abs"} to reflect predetermined positions
            
            theta0 = obj.getThetaRef(THETAREF);
            
            [theta_invs,T0_i_invs] = pieperInvKin(T0_6, obj.q_types, obj.D_H, obj.D_H_type);
            ts = size(theta_invs);
            theta_invs_rel = zeros(ts);
            for ii = 1:ts(end)
                theta_invs_rel(:,ii) = theta_invs(:,ii) - theta0;
            end
        end
        
        function thetas_inv = testInvSoln(obj,theta_fwd)
            % Propagate fwd kinematics, then solve inv kinematics
            inputMode = "abs";
            T0_i = fwdKin(obj, theta_fwd, inputMode);
            T0_tgt = T0_i(:,:,6);
            theta_invs = invKin(obj, T0_tgt);
        end
        
        function [T0_6s,Nsol,T6_tgts] = tgtWtask2goal(obj,TRECS_tgt,whichArm,TASK)
        %{
            TODO
            ()- offload T6_tgts functionality to another function
        %}                       
        % INPUTS
        % T0_tgt
        % TASK = {"move2pt","inspect","grab","handoff"}
        % OUTPUTS
        % T0_6s = possible 
        % Nsol = number of solutions
        % T6_tgts = possible valid orientations of tgt relative to 6 frame
        % based on given task
        
            TRECS_0 = obj.arm2RECS(whichArm);
            
            T0_tgt = TRECS_0\TRECS_tgt; % inv(TRECS_0)*TRECS_tgt
        
            % Define desired pose offset between goal frame and target frame based on assigned task
            if TASK == "move2pt"
                % POSITION
                Pee_tgt = [1.2*obj.lindex 0 0]'; % conservative offset
                
                % ROTATION (choose arb square orientation for end effector rel to tgt, then
                % generalize to all possible orientations)
                Ree_atgt = [0 0 -1;
                            0 1 0;
                            1 0 0];
                % 4 possible orientations that orient the ee frame
                % 'squarely' to the tgt's 
                
                % This involves rotating the virtual target by increments
                % of 90 about its z axis
                thetasSquare = [0,90,180,270]; 
                axis = "z";
                Nsol = numel(thetasSquare);
                T6_tgts = zeros(4,4,Nsol);
                T0_6s = zeros(4,4,Nsol);
                for ii = 1:Nsol
                    theta = thetasSquare(ii);
                    Tatgt_tgt = planarRotation(theta, axis);
                    Ratgt_tgt = Tatgt_tgt(1:3,1:3); % Pull out only the rotation matrix part
                    Ree_tgt = Ree_atgt*Ratgt_tgt;
                    
                    % Generate transformation matrix from tgt to ee
                    Tee_tgt = [Ree_tgt    Pee_tgt;
                               zeros(1,3) 1];
                    
                    % Now back-calculate possible (6) poses from given
                    % (tgt)                    
                    T6_tgt = obj.T6_ee*Tee_tgt;
                    T6_tgts(:,:,ii) = T6_tgt;
%                     Ttgt_6 = inv(Ttgt_6);
%                     T0_6s(:,:,ii) = T0_tgt*Ttgt_6;
                    T0_6s(:,:,ii) = T0_tgt/T6_tgt; % Matlab inverse
                end
            else
                disp("Other tasks not yet supported")
            end
        end
        
        function logic = enforceThetaJointLims(obj,theta_rel,THETAREF)
        %{
        TODO
            ()- determine if joint angle limits are given in terms of motor
            angle or theta
        %}
        % INPUTS:
        % theta_rel = joint angles relative to THETAREF
        % OUTPUTS:
        % logic = vector of booleans indexed by joint (true = within lims, false = violation)
            
            % Common reference
            theta0 = obj.getThetaRef(THETAREF);
            theta_abs = theta_rel + theta0;
            
            % Enforce
            s = size(theta_abs);
            Njoints = obj.N;
            Nsols = s(2); % Number of column vectors in input array = no solns
            logic = zeros(s);
            for ii = 1:Njoints
                negLim = obj.thetaLimRng_relabs(ii,1);
                posLim = obj.thetaLimRng_relabs(ii,2);
                
                % Row vector of possible solutions for a given joint
                theta_abs_i = theta_abs(ii,:); % This type of indexing works on single columns as well
                
                logic(ii,:) = (theta_abs_i>negLim)&(theta_abs_i<posLim); %https://stackoverflow.com/questions/1379415/whats-the-difference-between-and-in-matlab
            end
        end
        
        
    end
end