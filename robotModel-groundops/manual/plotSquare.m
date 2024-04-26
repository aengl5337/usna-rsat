function plotSquare(cp,ih,jh,color)
% Makes a square given a centerpoint and unit vectors
% Should plot in the current figure!


    p1 = (ih + jh) + cp;
    p2 = (-ih + jh) + cp;
    p3 = (-ih + -jh) + cp;
    p4 = (ih + -jh) + cp;
    
    P = [p1 p2 p3 p4];
    
    f = [1 2 3 4]; % specifies which points to connect
    
    patch('Faces',f,'Vertices',P','FaceColor',color)
end