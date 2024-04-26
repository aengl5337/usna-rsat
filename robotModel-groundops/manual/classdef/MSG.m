%{
TODO
() + Check all *** annotations
() + Make incremental and absolute input versions (utilize inputMode syntax
for fwdKin and use theta0 to store current position)
() - Display error flexibly with separate function from pieperInvKin
% Add MSG and inter-arm collision protection
        % Use a radius about each arm stick

() - Eliminate static methods? Good for organization though
() - Move initial frame reference math upstream to fwdKin so it's less
laborious in plotExperiment
() + Make plotMSG() more efficient with convex hull stuff?
() - Fix constant need to re-reference RECS = RSAT() ... feel like here be
dragons
() + section off and label all wrapper functions
%}

classdef MSG
    properties
        MSG_X
        MSG_x
        MSG_Y
        MSG_Z
        PLATEoffsetZ
        PLATEmountA
        PLATEmount_lenX
        PLATEmount_lenY
        RECSmount_w
        RECSmount_l
%         delRECScg
%         RECSAmountB
%         RECSBmountB
%         RECSAmountCG
%         RECSBmountCG
        TMSG_RECSA
        TMSG_RECSB
        u
        v
    end
    
    methods
        function obj = MSG() % Constructor
            obj.MSG_X = 500; % mm
            obj.MSG_x = 410; % min 310, max 500
            obj.MSG_Y = 906;
            obj.MSG_Z = 638;
            obj.PLATEoffsetZ = 1;
            
            % MARK MOUNTING CENTROIDS OF RECSA,B ON PLATE WITHIN MSG
            % NOTE: the following coordinates are measured wrt the MSG
            % frame
            obj.PLATEmountA = [48 210.25 obj.PLATEoffsetZ]';
            obj.PLATEmount_lenX = 280;
            obj.PLATEmount_lenY = 459;
%             PLATEmountPts = [
            obj.RECSmount_w = 55; % Redundant with variables in RECS
            obj.RECSmount_l = 302.7;
            delRECScg = [-obj.RECSmount_w/2 obj.RECSmount_l/2 0]'; % Measured in MSG frame from mountB
            %Doesn't account for thickness
            
            RECSBmountB = obj.PLATEmountA + [obj.PLATEmount_lenX+29 78.15 0]';
            RECSAmountB = RECSBmountB + [-2*obj.RECSmount_w 0 0]';
            
            RECSBmountCG = RECSBmountB + delRECScg;
            RECSAmountCG = RECSAmountB + delRECScg;
            
            % DEFINE TRANSFORMS FROM RECSA,B FRAMES TO MSG FRAME
            RMSG_RECS = eye(3,3); % Coordinate directions are the same
            obj.TMSG_RECSA = [RMSG_RECS  RECSAmountCG;
                          zeros(1,3) 1];
            obj.TMSG_RECSB = [RMSG_RECS  RECSBmountCG;
                          zeros(1,3) 1];
                      
            % DEFINE PHYSICAL BOUNDS OF MSG 
            w1 = obj.MSG_X;
            w2 = obj.MSG_x;
            l = obj.MSG_Y;
            h = obj.MSG_Z;
            
            v1 = [0 0 0]';
            v2 = [w1 0 0]';
            v4 = [0 l 0]';
            v3 = v2+v4;
            
            u1 = [0 0 h]';
            u2 = [w2 0 h]';
            u4 = [0 l h]';
            u3 = [w2 l h]';
            
            obj.v = [v1 v2 v3 v4,v1];
            obj.u = [u1 u2 u3 u4,u1];
        end
        
        function TMSG_tgt = MSGplyGrabPose(obj, zmax, CANNED)
            % GUI-search for, open&view, manually select pose points, and return pose
            % of a (rectangular) target in the MSG frame
            
            % zmax = .62 maximum depth displayed in m (D435 has depth noise out to infinity depth)
            % (good values: zmax = .55 for ISS 10JAN24, .62 for Houston test)
            Td_tgt = plyGrabPose(zmax, CANNED);
            % NOTE: D435 is located in RECS A, so determine pose accordingly
            RECSA = RSAT();
            TRECS_d = RECSA.D435toRECS();
            TMSG_d = obj.TMSG_RECSA*TRECS_d;
            
            TMSG_tgt = TMSG_d*Td_tgt;
        end
        
        function TMSG_arm0 = arm2MSG(obj,whichArm)
        % INPUTS
        % whichArm = {"I","II"}
        
            RECS = RSAT();
            
            if whichArm == "I" % RECSA ARM1
                TMSG_RECS = obj.TMSG_RECSA;
                whichRECSarm = 1;
            elseif whichArm == "II" % RECSB ARM2
                TMSG_RECS = obj.TMSG_RECSB;
                whichRECSarm = 2;
            else
                disp("Invalid selection for whichArm")
            end
            
            TRECS_arm0 = RECS.arm2RECS(whichRECSarm);
            
            TMSG_arm0 = TMSG_RECS*TRECS_arm0;
        end
        
        
        function TMSG_is = MSGfwdKin(obj,theta_rel,THETAREF,whichArm)
        % Basically just a wrapper for RECS.fwdKin for ease of use in MSG
        % coords
        
        % INPUTS
        % whichArm = {"I","II"}
            RECS = RSAT();
            Njoints = RECS.N;
            theta_rel = reshape(theta_rel,Njoints,1);
                        
            % Define reference to specified arm's (0) frame
            TMSG_0 = obj.arm2MSG(whichArm);
            % Common reference
            theta0 = RECS.getThetaRef(THETAREF);
            theta_abs = theta_rel + theta0;
            
            % Compute arm fwd kin
            inputMode = "abs";
            T0_is = RECS.fwdKin(theta_abs, inputMode);
            
            % Convert arm joint poses to MSG reference frame
            TMSG_is = zeros(4,4,Njoints);
            for ii=1:Njoints
                T0_i = T0_is(:,:,ii);
                
                TMSG_i = TMSG_0*T0_i;
                
                TMSG_is(:,:,ii) = TMSG_i;
            end
        end
        
        function [T0_6s,Nsol,T6_tgts] = MSGtgtWtask2goal(obj,TMSG_tgt,whichArm,TASK)
        % Basically just a wrapper for RSAT.tgtWtask2goal for ease of use in MSG
        % coords
        
        % INPUTS
        % TMSG_tgt
        % whichArm = {"I","II"}
        % TASK = {"moveToQR","inspection","grab","handoff"}
        % OUTPUTS
        % T0_6s
        % Nsol = number of solutions
            
            % *** PASS OFF THIS FUNCTIONALITY
            if whichArm == "I" % RECSA ARM1
                TMSG_RECS = obj.TMSG_RECSA;
                whichRECSarm = 1;
            elseif whichArm == "II" % RECSB ARM2
                TMSG_RECS = obj.TMSG_RECSB;
                whichRECSarm = 2;
            else
                disp("Invalid selection for whichArm")
            end

            TRECS_tgt = TMSG_RECS\TMSG_tgt; % inv(TMSG_RECS)*TMSG_tgt

            RECS = RSAT();
            [T0_6s,Nsol,T6_tgts] = RECS.tgtWtask2goal(TRECS_tgt,whichRECSarm,TASK);

            %                 
    %         TMSG_6s = zeros(4,4,Nsol);
    %         for ii = 1:Nsol
    %             TMSG_6s(:,:,ii) = T0_6s(:,:,ii);
    %         end
        end
        
        function plotMSG(obj,SEPFIG)
            % INIT FIGURE
            if ~SEPFIG % if not plotting to a separate figure, initiate it here
                figure()
                hold on
            end
            axis equal
            % COLOR MSG PURPLE
            cMSG = '#9370db';
                        
            % PLOT 1ST LAYER
            xv = obj.v(1,:);
            yv = obj.v(2,:);
            zv = obj.v(3,:);
            plot3(xv,yv,zv,'Color',cMSG)
            
            % PLOT 2ND LAYER
            xu = obj.u(1,:);
            yu = obj.u(2,:);
            zu = obj.u(3,:);
            plot3(xu,yu,zu,'Color',cMSG)
                        
            % PLOT CONNECTION
            for ii = 1:4
                xi = [obj.v(1,ii), obj.u(1,ii)];
                yi = [obj.v(2,ii), obj.u(2,ii)];
                zi = [obj.v(3,ii), obj.u(3,ii)];
                plot3(xi,yi,zi,'Color',cMSG)
            end
            
            % CLOSE FIG
            if ~SEPFIG % if not plotting to a separate figure, initiate it here
                hold off
            end
        end
        
        function plotExperiment(obj,theta_armI_relstowed,theta_armII_relstowed,TMSG_tgt)
            %{
            TODO
            () - Trim down useless stuff in the displayed legend
            %}
            RECS = RSAT();
            
            THETAREF = "stow";
            TMSG_armI_is = obj.MSGfwdKin(theta_armI_relstowed,THETAREF,"I");
            TMSG_armII_is = obj.MSGfwdKin(theta_armII_relstowed,THETAREF,"II");
            
            % INIT FIG
            SEPFIG = true;
            figure()
            hold on
            % PLOT arms, end effector, & tgt
            plotRobotAxes(TMSG_armI_is,RECS.T6_ee,NaN,SEPFIG)
            plotRobotAxes(TMSG_armII_is,RECS.T6_ee,TMSG_tgt,SEPFIG)
            
            % PLOT D435
            
            
            % PLOT MSG
            obj.plotMSG(SEPFIG)
            
            % ADMIN
            xlabel("X")
            ylabel("Y")
            zlabel("Z")
            title("Experiment")
            % Specify view
            towardsCamera = [1 1 1];
            view(towardsCamera)
            hold off
        end 
        
        function logic = enforceMSGLims(obj,theta_rel,THETAREF,whichArm)
        % INPUTS:
        % theta_rel = joint angles relative to THETAREF
        % OUTPUTS:
        % logic = vector of booleans indexed by joint (true = within lims, false = violation)
        %{
        TODO
        () +Add ability to 
        %}
            % Common reference
            RECS = RSAT();
            theta0 = RECS.getThetaRef(THETAREF);
            theta_abs = theta_rel + theta0;
            
            % Compute forward kinematics
            TMSG_is = obj.MSGfwdKin(theta_rel,THETAREF,whichArm);
            
            % Define MSG as a convex hull
            ih = [1 0 0]';
            jh = [0 1 0]';
            kh = [0 0 1]';
                        
            v1 = obj.v(:,1);            
            u3 = obj.u(:,3);
            u2 = obj.u(:,2);
            v2 = obj.v(:,2);
                        
            z2 = u2-v2;
            
            % Define outward surface normals
            n_xp = cross(jh,z2);
            n_xp = normalizeVector(n_xp);
            n_xn = -ih;
            n_yp = +jh;
            n_yn = -jh;
            n_zp = +kh;
            n_zn = -kh;
            
            ns = [n_xp,n_xn,n_yp,n_yn,n_zp,n_zn];
            
            % Define intercepts
            x0_xp = u3;
            x0_xn = v1;
            x0_yp = u3;
            x0_yn = v1;
            x0_zp = u3;
            x0_zn = v1;
                        
            x0s = [x0_xp,x0_xn,x0_yp,x0_yn,x0_zp,x0_zn];
            
            Njoints = RECS.N;
            Nsurf = 6;
            logic = zeros(Njoints,Nsurf);
            for ii = 1:Njoints
                PMSG_i = TMSG_is(1:3,4,ii);
                for jj = 1:Nsurf
                    n = ns(:,jj);
                    x0 = x0s(:,jj);
                    b = n'*x0;

                    logic(ii,jj) = (n'*PMSG_i)<b;
                end
            end
        end
        
        function filterSolns(obj,theta_soln_rel,THETAREF)
        % INPUTS
        % 
        % THETAREF = {"ext","stow","deploy","abs"} to reflect predetermined reference positions
        % OUTPUTS
        % 
        
            % Common reference
            RECS = RSAT();
            theta0 = RECS.getThetaRef(THETAREF);
            thetas_abs = thetas_rel + theta0;
            
            % for each solution in array...
            % % FILTER BASED ON PROXIMITY***
            % % Print error between two methods
            % thetas_residual = thetas_fwd - thetas_inv


            % % FILTER BASED ON ROTATION RESTRICTIONS FOR EACH JOINT
            jointLims = RECS.enforceJointLims(thetas_abs,"abs");
            
            % % FILTER BASED ON ROTATION RESTRICTIONS FOR EACH JOINT
            
            
            % % PROVIDE RESULT

        end
        
        
    end
end