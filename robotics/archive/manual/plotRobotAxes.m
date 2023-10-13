%{
TODO:
() = incomplete, (X) = complete
+ = addition, - = fix/debug

() + Generalize colors to N joints

%}

function plotRobotAxes(T0_i,T0_tgt)
    s = size(T0_i);
    N = s(3);
    
    sc = 100; % unit vector scaling
    
    % INIT FIGURE
    figure()
    hold on
    axis equal
    
    % COLOR CODE AXES
    c1 = '#EDB120'; % Marigold
    c2 = '#D95319'; % Orange
    c3 = '#A2142F'; % Red?
    c4 = '#0072BD'; % Dark blue
    c5 = '#4DBEEE'; % Light blue
    c6 = '#00FF00'; % Lime green
    ctgt = '#777777'; % Gray
    ctgt2 = '#888888';
    
    colors = {c1,c2,c3,c4,c5,c6}; % Same colors as my eng drawings
        
    % PLOT ROUGH ARM GEOMETRY
    P0_iorg = T0_i(1:3,4,:);  % Origin of {ith} frame in zeroth frame coordinates
    xs = reshape(P0_iorg(1,:,:),[N,1]);
    ys = reshape(P0_iorg(2,:,:),[N,1]);
    zs = reshape(P0_iorg(3,:,:),[N,1]);
    plot3(xs,ys,zs,'-ok','LineWidth',1.0);
    
    
    % PLOT AXES
    h = zeros(N+1, 1); % See below
    colorsL = cell(N+1,1); % Container for later
    for i = 1:N
        % Finds unit vectors
        ih = sc*T0_i(1:3,1,i);
        jh = sc*T0_i(1:3,2,i);
        kh = sc*T0_i(1:3,3,i);
        
        % COLOR CODE AND PLOT EACH JOINT&AXES WITH Z AXIS BOLDED
        plot3(P0_iorg(1,:,i),P0_iorg(2,:,i),P0_iorg(3,:,i),'o','MarkerEdgeColor',colors{i},'MarkerFaceColor',colors{i})
%         arrow(P0_iorg,P0_ix,'MarkerFaceColor',colors{i},'MarkerEdgeColor',colors{i})
%         arrow(P0_iorg,P0_iy,'MarkerFaceColor',colors{i},'MarkerEdgeColor',colors{i})
%         arrow(P0_iorg,P0_iz,'MarkerFaceColor',colors{i},'MarkerEdgeColor',colors{i})
%         quiver3(P0_iorg(1,:,i),P0_iorg(2,:,i),P0_iorg(3,:,i),ih(1),ih(2),ih(3),'Color',colors{i})
%         quiver3(P0_iorg(1,:,i),P0_iorg(2,:,i),P0_iorg(3,:,i),jh(1),jh(2),jh(3),'Color',colors{i})
%         quiver3(P0_iorg(1,:,i),P0_iorg(2,:,i),P0_iorg(3,:,i),kh(1),kh(2),kh(3),'Color',colors{i})
        
        qx = quiver3(P0_iorg(1,:,i),P0_iorg(2,:,i),P0_iorg(3,:,i),ih(1),ih(2),ih(3),'Color',colors{i});
%         qy = quiver3(P0_iorg(1,:,i),P0_iorg(2,:,i),P0_iorg(3,:,i),jh(1),jh(2),jh(3),'Color',colors{i});
        qz = quiver3(P0_iorg(1,:,i),P0_iorg(2,:,i),P0_iorg(3,:,i),kh(1),kh(2),kh(3),'Color',colors{i});
        qz.LineWidth = 2;
        
        % Make a null plot to enter desired legend
        h(i) = plot3(NaN,NaN,NaN,'Color',colors{i});
        colorsL{i} = strcat('j',num2str(i));
%         figure(10)
%         pause(5)
    end
    
    % PLOT TARGET POSE
    P0_tgt = T0_tgt(1:3,4);
    R0_tgt = T0_tgt(1:3,1:3);
    ih = sc*R0_tgt(:,1);
    jh = sc*R0_tgt(:,2);
    kh = sc*R0_tgt(:,3);
    plot3(P0_tgt(1,:),P0_tgt(2,:),P0_tgt(3,:),'o','MarkerEdgeColor',ctgt,'MarkerFaceColor',ctgt)
    qx = quiver3(P0_tgt(1,:),P0_tgt(2,:),P0_tgt(3,:),ih(1),ih(2),ih(3),'Color',ctgt);
%         qy = quiver3(P0_tgt(1,:),P0_tgt(2,:),P0_tgt(3,:),jh(1),jh(2),jh(3),'Color',ctgt);
    qz = quiver3(P0_tgt(1,:),P0_tgt(2,:),P0_tgt(3,:),kh(1),kh(2),kh(3),'Color',ctgt);
    qz.LineWidth = 2;
    
    % Make a square
    p1 = (ih + jh) + P0_tgt;
    p2 = (-ih + jh) + P0_tgt;
    p3 = (-ih + -jh) + P0_tgt;
    p4 = (ih + -jh) + P0_tgt;
    
    P = [p1 p2 p3 p4];
    
    f = [1 2 3 4]; % specifies which points to connect
    
    patch('Faces',f,'Vertices',P','FaceColor',ctgt2)
    
    h(N+1) = plot3(NaN,NaN,NaN,'Color',ctgt2);
    colorsL{N+1} = strcat('tgt');
    
    legend(h,colorsL); % Enable legend
    axis equal
%     axis([-600 50 -300 300 -300 300]) % Mod*
    hold off
end