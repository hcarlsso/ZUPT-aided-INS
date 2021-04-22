function [x_h, cov_h, dT]=RawINS(u,zupt, simdata)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Initialize the data fusion          %%       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get  the length of the IMU data vector
N=length(u);

% Initialize the filter state covariance matrix P, the processes noise
% covariance matrix Q, the pseudo measurement noise covariance R, and the
% observation matrix H.
[P, Q, R, H]=init_filter(simdata);          % Subfunction located further down in the file.

% Allocate vecors
[x_h, cov_h, Id]=init_vec(N,P,simdata);     % Subfunction located further down in the file.

% Initialize the navigation state vector x_h, and the quaternion vector
% quat.
[x_h(1:9,1), quat]=init_Nav_eq(u,simdata);  

W = simdata.Start_window;
% Remove bias in gyroscopes
u(4:6,:)=u(4:6,:)-mean(u(4:6,1:W),2);

if ~all(zupt(1:W))
    disp("Not still in pre window")
end
for k = 2:W
    x_h(:,k) = x_h(:,1); 
end

dT = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Run the filter algorithm          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=W+1:N
    if zupt(k)==1
        [x_h(1:9,k), quat]=init_Nav_eq(u(:,k-simdata.Window_size:k),simdata);
        x_h(1:6,k)=x_h(1:6,k-1);
    else
        
        dT=dT+simdata.Ts;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %           Time  Update         %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Compensate the IMU measurements with the current estimates of the
        % sensor errors
        u_h=comp_imu_errors(u(:,k),x_h(:,k-1),simdata); % Subfunction located further down in the file.
        
        
        % Update the navigation equations.
        [x_h(:,k), quat]=Navigation_equations(x_h(:,k-1),u_h,quat,simdata); % Subfunction located further down in the file.
        
        
        % Update state transition matrix
        [F, G]=state_matrix(quat,u_h,simdata); % Subfunction located further down in the file.
        
        
        % Update the filter state covariance matrix P.
        P=F*P*F'+G*Q*G';
        
        % Make sure the filter state covariance matrix is symmetric.
        P=(P+P')/2;
        
    end 
    % Store the diagonal of the state covariance matrix P.
    cov_h(:,k)=diag(P);
end

end
