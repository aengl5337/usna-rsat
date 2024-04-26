function plotAxes(T,sc,color,PLOTX,PLOTY,PLOTZ)
% Plots the axes of a given coordinate frame at a specified origin
% Pulls axes from rotation matrix
% Currently plots only x and z (z is bolded), in a specified color
% Should plot within the active figure!
    i = sc*T(1:3,1);
    j = sc*T(1:3,2);
    k = sc*T(1:3,3);
    origin = T(1:3,4);
    hold on
    plot3(origin(1),origin(2),origin(3),'o','MarkerEdgeColor',color,'MarkerFaceColor',color)
    if PLOTX
        qx = quiver3(origin(1),origin(2),origin(3),i(1),i(2),i(3),'Color',color);
    end
    if PLOTY
        qy = quiver3(origin(1),origin(2),origin(3),j(1),j(2),j(3),'Color',color);
    end
    if PLOTZ
        qz = quiver3(origin(1),origin(2),origin(3),k(1),k(2),k(3),'Color',color);
        qz.LineWidth = 2;
    end
    
end