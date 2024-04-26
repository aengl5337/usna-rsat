
%{
TODO:
(~)-should I call forwardKin() inside this function or simply take the
transform matrices as input? -- the latter, for now
() +Expand to include points around the joints, 
() -Ensure got the x axes right for the arms... also check diff between (0)
and (1)'

%}

function logic = checkMSG(T0_i,recsarm) % DEPRECATED AS OF 09JAN24
% INPUTS:
% T0_i
% recsarm = <recs><arm>, i.e. '{A,B}{1,2}', e.g. 'A1' for arm 1 of recs A
% (char so can index recs and arm independently)
% 
% OUTPUTS:
% logic = vector of booleans indexed by joint (true = within lims, 
% % false = violation)
% Structured as (no joint,coordinate):
% [x1 in lims, y1 in lims, z1 in lims;
%  x2 in lims, y2 in lims, z2 in lims;... etc


    %% Define some constants
    % MSG params
    l = 906;
    w1 = 500;
    w2 = 410; % min 310, max 500
    h = 638;
    % recs params
    wrecs = 100;
    lrecs = 300;
    deltal = 303;
    deltaw = 118;
    hoffset = 80.85;
    
    %% Define the boundaries of the MSG, in base coordinates (units are mm)    
       
%     delta = 10; % spatial delta, maximum
    
    ylim = [0,l];
    zlim = [0,h];
    xlim = @(z)[0,(w1-z*(w1-w2)/h)];
    
%     [X,Y,Z] = meshgrid(x,y,z);
    
    %% Transform the arm bulk coordinates to the (MSG) frame
    % Define the (0)=(1)' to (MSG) transform based on 
    recs = recsarm(1);
    arm = recsarm(2);
    
    % Position offsets
    del_recs = zeros(3,1);
    del_arm = zeros(3,1);
    
    del_recs(2) = deltal;
    if recs == 'A'
        del_recs(1) = deltaw;
    elseif recs == 'B'
        del_recs(1) = deltaw+wrecs;       
    else
        disp("Invalid recs value, choose {'A','B'}")
    end
    
    if arm == '1'
        del_arm(1) = 0;
        del_arm(2) = 0;
%         del_arm(3) = 0;
        RMSG_0 = [0  0 1;
                  0 -1 0;
                  1  0 0];
    elseif arm == '2'
        del_arm(1) = wrecs;
        del_arm(2) = lrecs;
%         del_arm(3) = 0;
        RMSG_0 = [0 0 -1;
                  0 1  0;
                  1 0  0];     
    else
        disp("Invalid arm value, choose {'1','2'}")
    end
    
    PMSG_0 = del_recs + del_arm; % also known as del_overall
    PMSG_0(3) = hoffset; % *** fix this/debug
    
    TMSG_0 = [RMSG_0    PMSG_0;
              zeros(1,3)     1];
        
    % Transform arm coordinates
    s = size(T0_i);
    N = s(3);
    TMSG_i = zeros(s);
    logic = zeros(N,3);
    for ii = 1:N
        TMSG_ii = TMSG_0*T0_i(:,:,ii);
        TMSG_i(:,:,ii) = TMSG_ii;
        %% Check if the arm violates the MSG workspace
        PMSG_ii = TMSG_ii(1:3,4);
        x = PMSG_ii(1);
        y = PMSG_ii(2);
        z = PMSG_ii(3);
        
        xlim_z = xlim(z);
        
        logic(ii,1) = (x>xlim_z(1))&(x<xlim_z(2));
        logic(ii,2) = (y>ylim(1))&(y<ylim(2));
        logic(ii,3) = (z>zlim(1))&(z<zlim(2));
    end
end