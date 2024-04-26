function plotArc(origin, v1,v2,color) % ,zQR,zAXIS
% Plots an arc between two arb vectors, v1 and v2 (defined in plot's native
% coordinates)
% Currently plots arc only in the plane of the two vectors, -- want to 
% generalize given arb normal vector
% Currently only works for this specific application (generalize the
% rotation matrix)
% Should plot within the active figure!

    % Normalize vectors
    V1 = norm(v1);
    V2 = norm(v2);
    a1 = v1/V1;
    a2 = v2/V2;
    
    % Find angle between them
    TH =  acos(dot(a1,a2)); % radians
    
    % Parameterize
    N=100;
    thetas = linspace(0,TH,N); % radians
    r = .5*min(V1,V2); % Choose a good arc radius about half of the lengths
    
    % Define coordinates, taking a1=ihat
    c1 = r*(cos(thetas)-sin(thetas)*cot(TH)); %i.e.  c1=x-c2*cos(TH)
    c2 = r*sin(thetas)/sin(TH);
    v = origin + a1*c1 + a2*c2; % use outer product to multiply onto basis vectors
    
    
    % Plot!
    plot3(v(1,:),v(2,:),v(3,:),'Color',color,'LineWidth',1.0)
    
end