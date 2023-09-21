function [T0_6,T] = forwardKin(thetas, D_H) % This works in its current form, but needs serious convention rehaul
    
    nMotors = numel(thetas)-1;
    
    % GIVENS    
    a = D_H(:,1);
    d = D_H(:,2);
    alpha = D_H(:,3);

    T=zeros(4,4,nMotors);
    
    for i = 1:nMotors % From [1,7]
        % Reverse index, from [row 7 (joint 6),row 2 (joint 1)]
        i_rev = nMotors-(i-2);
        display(i_rev)
        
        % Generate T(i_rev-2)_(i_rev-1)
        % i.e. for i=1, i_rev=7, generating T5_6
        %      for i=6, i_rev=2, generating T0_1
        
        % D_d, R_theta, and D_a formulas stay the same for every joint
        D_a = axialTranslation(a(i_rev-1),'x');
        D_d = axialTranslation(d(i_rev),'z');
        R_theta = planarRotation(thetas(i_rev),'z');
        
        % R_alpha needs to be specified diferently when considering T0_1
        if i_rev == 2
            % This y here is to rotate base coordinates so they are consistent with NSTAR engineering drawings
            R_alpha = planarRotation(alpha(i_rev-1),'y');
        else
            R_alpha = planarRotation(alpha(i_rev-1),'x');
        end
        
        % Assign the overall transformation to a data cube, layer indexed
        % by joint [from 6 to 1]
        T(:,:,(i_rev-1)) = R_alpha*D_a*R_theta*D_d;
        
    end
      
    %{
    figure(1)
    
    hold on
    for j=1:nMotors
        x=r(1,j);
        y=r(2,j);
        z=r(3,j);
        plot3(x,y,z,'ro')
    end
    %}
    T0_6 = eye(4,4); % Initialize overall transformation matrix from {6} to {0}
    for joint=1:nMotors % Incrementally matrix-multiply all transformations together, from joint 6 to 1
        joint_rev = nMotors-(joint-1);
        T0_6 = T(:,:,joint_rev)*T0_6;
    end
end
