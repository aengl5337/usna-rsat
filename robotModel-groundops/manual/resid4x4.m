function [eaxis,theta,delP] = resid4x4(TQR,TEE,PLOT)
%{
Inputs:
    PLOT = boolean, true if want to display result on a 3D plot
Outputs:
Function to measure the angular and translational displacement/residual
between two frames (from 1 to 2), provided as 4x4 matrices in the homogenous formalism

Returns axis, theta in the axis-angle representation, where the axis was
chosen to be the cross product of the two frames' z axes, i.e. the rotation
axis perpendicular to both.

NOTE: assumes both inputs are given with respect to the same coordinate
frame
%}

    %% CALCULATE
    % Translation
    PQR = TQR(1:3,4);
    PEE = TEE(1:3,4);
    
    delP = PEE-PQR;

    % Angle
    khQR = TQR(1:3,3);
    khEE = TEE(1:3,3);

    theta = acosd(dot(khQR,khEE)); % This works since axes already normalized
    
    % Eigenaxis (Normalized)
%     axis = cross(z1,z2);
    if round(sind(theta),4)~=0 % Eliminates small numerical residuals
        %(also corresponds to nonzero axis magnitude)
        eaxis = cross(khQR,khEE)/sind(theta);
    else
        eaxis = cross(khQR,khEE); % Going to be zero anyways, prevent divByZero
    end
    
    
    
    
    %% PLOT
    % Settings borrowed from plotRobotAxes()
    if PLOT
        figure()
        hold on
        axis equal
        % COLOR CODE AXES
    %     c1 = '#EDB120'; % Marigold
    %     c2 = '#D95319'; % Orange
        c3 = '#A2142F'; % Red?
    %     c4 = '#0072BD'; % Dark blue
        c5 = '#4DBEEE'; % Light blue
        c6 = '#00FF00'; % Lime green
        ctgt = '#777777'; % Gray
        ctgt2 = '#888888';


        % 1.PLOT RELEVANT AXES

        sc = 100; % unit vector scaling
        Tstack = reshape([TQR,TEE],[4,4,2]);
        colors = {ctgt,c6};

        for kk = 1:2
            % Extract unit vectors
            R = Tstack(1:3,1:3,kk);
            % Extract origin
            origin = Tstack(1:3,4,kk);

            % Plot axes
            plotAxes(T,sc,colors{kk},true,false,true)

            % Make null plot to construct legend
            h(kk) = plot3(NaN,NaN,NaN,'Color',colors{kk});

            if kk==1 % For QR code origin, also plot plane
                plotSquare(origin,ih,jh,ctgt2)
            end
        end

        % 2.PLOT ERROR METRICS
        % Scale up axes for display purposes
        zEE = sc*khEE;
        zQR = sc*khQR;
        saxis = sc*eaxis; 
        oQR = Tstack(1:3,4,1);

        % position diff (RED)
        qP = quiver3(oQR(1),oQR(2),oQR(3),delP(1),delP(2),delP(3),'Color',c3); 

        % Plot z unit vector projected onto QR origin for display purposes
        q = quiver3(oQR(1),oQR(2),oQR(3),zEE(1),zEE(2),zEE(3),'Color',c6,'LineStyle', '--');
        q.Head.LineStyle = 'solid'; 

        % Plot eigaxis and arc connecting them, representing theta (BLUE)
        qA = quiver3(oQR(1),oQR(2),oQR(3),saxis(1),saxis(2),saxis(3),'Color',c5);
        plotArc(oQR,zEE,zQR,c5)

        % Label with appropriate symbology

        % Enable legend
        colorsL = {'j6/EE','tgt'};
        legend(h,colorsL); % Enable legend
        hold off
    end
end