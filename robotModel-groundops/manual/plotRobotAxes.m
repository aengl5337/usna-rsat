%{
TODO:
() = incomplete, (X) = complete
+ = addition, - = fix/debug

() + Generalize colors to N joints

%}

function plotRobotAxes(T0_is,TN_ee,T0_tgt,SEPFIG)
% INPUTS:
% T0_is = pose of joint frame (i) rel to (0)
% TN_ee pose of ee frame rel to last joint frame (N)
% T0_tgt = target pose (or NaN if want to omit) rel to (0)
% OUTPUTS:

    s = size(T0_is);
    N = s(3);
    
    sc = 100; % unit vector scaling
    
    % INIT FIGURE
    if ~SEPFIG % if not plotting to a separate figure, initiate it here
        figure()
        hold on
    end
    axis equal
    
    % COLOR CODE AXES
    c1 = '#EDB120'; % Marigold
    c2 = '#D95319'; % Orange
    c3 = '#A2142F'; % Red?
    c4 = '#0072BD'; % Dark blue
    c5 = '#4DBEEE'; % Light blue
    c6 = '#00FF00'; % Lime green
    cee = '#9370db'; % med purple (end effector)
    ctgt = '#777777'; % Gray
    ctgt2 = '#888888';
    
    colors = {c1,c2,c3,c4,c5,c6}; % Same colors as my eng drawings
    
    % PLOT ROUGH ARM GEOMETRY
    P0_iorg = T0_is(1:3,4,:);  % Origin of {ith} frame in zeroth frame coordinates
    xs = reshape(P0_iorg(1,:,:),[N,1]);
    ys = reshape(P0_iorg(2,:,:),[N,1]);
    zs = reshape(P0_iorg(3,:,:),[N,1]);
    plot3(xs,ys,zs,'-ok','LineWidth',1.0);
    
    % PLOT MAIN JOINT AXES
    h = zeros(N+2, 1); % See below
    colorsL = cell(N+2,1); % Container for later
    for i = 1:N
        % Pull current transform matrix for unit vectors and origin
        T0_i = T0_is(:,:,i);
        
        % COLOR CODE AND PLOT EACH JOINT&AXES WITH Z AXIS BOLDED
        color = colors{i};
        plotAxes(T0_i,sc,color,true,false,true)
        
        % Make a null plot to enter desired legend
        h(i) = plot3(NaN,NaN,NaN,'Color',color);
        colorsL{i} = strcat('j',num2str(i));
%         figure(10)
%         pause(5)
    end
    
    % PLOT END EFFECTOR AXES
    % COLOR CODE AND PLOT with Z AXIS BOLDED
    T0_N = T0_is(:,:,N);
    T0_ee = T0_N*TN_ee;
    plotAxes(T0_ee,sc,cee,true,true,true)

    % Make a null plot to enter desired legend
    h(N+1) = plot3(NaN,NaN,NaN,'Color',cee);
    colorsL{N+1} = 'ee';
    
    % PLOT TARGET POSE
    if ~isnan(T0_tgt) % evaluates as true for a 4x4 double
        P0_tgt = T0_tgt(1:3,4);
        R0_tgt = T0_tgt(1:3,1:3);
        plotAxes(T0_tgt,sc,ctgt,true,false,true)
    
        % Make a square
        ih = sc*R0_tgt(:,1);
        jh = sc*R0_tgt(:,2);
        kh = sc*R0_tgt(:,3);
        p1 = (ih + jh) + P0_tgt;
        p2 = (-ih + jh) + P0_tgt;
        p3 = (-ih + -jh) + P0_tgt;
        p4 = (ih + -jh) + P0_tgt;

        P = [p1 p2 p3 p4];

        f = [1 2 3 4]; % specifies which points to connect

        patch('Faces',f,'Vertices',P','FaceColor',ctgt2)

        h(N+2) = plot3(NaN,NaN,NaN,'Color',ctgt2);
        colorsL{N+2} = 'tgt';
    else % If tgt specified as NaN
        h = h(1:4);
        colorsL = colorsL(1:4);
    end
    
    legend(h,colorsL); % Enable legend
    axis equal
%     axis([-600 50 -300 300 -300 300]) % Mod*
    if ~SEPFIG % if not plotting to a separate figure, close
        hold off
    end
end