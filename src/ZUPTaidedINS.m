%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%> @file ZUPTaidedINS.m
%>
%> @brief This file contains all the functions needed to implement a zero-
%> velocity aided inertial navigation system. 
%>
%> @details This file contains all the functions needed to implement a 
%> zero-velocity aided inertial navigation system, given a set of IMU data
%> and a vector that indicates when the system has zero-velocity. 
%>
%> @authors Isaac Skog, John-Olof Nilsson
%> @copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% MAINFUNCTION


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [x_h cov_h]=ZUPTaidedINS(u,zupt)
%
%> @brief Function that runs the zero-velocity aided INS Kalman filter 
%> algorithm. 
%>
%> @details Function that runs the zero-velocity aided INS Kalman filter 
%> algorithm. All settings for the filter is done in setting.m. 
%>
%> @param[out]  x_h     Matrix with the estimated navigation states. Each row holds the [position, velocity, attitude, (biases, if turned on),(scale factors, if turned on)] for time instant k, k=1,...N.        
%> @param[out]  cov     Matrix with the diagonal elements of the state covariance matrices.
%> @param[in]   u       The IMU data vector.     
%> @param[in]   zupt    Vector with the decisions of the zero-velocity.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_h, cov_h]=ZUPTaidedINS(u,zupt, posupt, simdata)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Initialize the data fusion          %%       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get  the length of the IMU data vector
N=length(u);

% Initialize the filter state covariance matrix P, the processes noise
% covariance matrix Q, the pseudo measurement noise covariance R, and the
% observation matrix H.
[P, Q, R, H]=init_filter(simdata);          % Subfunction located further down in the file.

% Position updates
H_pos = zeros(size(H));
H_pos(1:3,1:3) = eye(3);
R_pos = simdata.R_pos;
assert(all(size(R_pos) == [3,3]))

% Allocate vecors
[x_h, cov_h, Id]=init_vec(N,P,simdata);     % Subfunction located further down in the file.

% Initialize the navigation state vector x_h, and the quaternion vector
% quat.
[x_h(1:9,1), quat]=init_Nav_eq(u,simdata);  % Subfunction located further down in the file.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Run the filter algorithm          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=2:N
    
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
    
    % Store the diagonal of the state covariance matrix P.
    cov_h(:,k)=diag(P);
   
     
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      Zero-velocity update      %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Check if a zero velocity update should be done. If so, do the
    % following
    if zupt(k)==true
        
        % Calculate the Kalman filter gain
        K=(P*H')/(H*P*H'+R);
        
        % Calculate the prediction error. Since the detector hypothesis 
        % is that the platform has zero velocity, the prediction error is 
        % equal to zero minus the estimated velocity.    
        z=-x_h(4:6,k);   
        
        % Estimation of the perturbations in the estimated navigation
        % states
        dx=K*z;
        
        
        % Correct the navigation state using the estimated perturbations. 
        % (Subfunction located further down in the file.)
        [x_h(:,k), quat]=comp_internal_states(x_h(:,k),dx,quat);     % Subfunction located further down in the file.
    
        
        % Update the filter state covariance matrix P.
        P=(Id-K*H)*P;
        
        % Make sure the filter state covariance matrix is symmetric. 
        P=(P+P')/2;
    
        % Store the diagonal of the state covariance matrix P.
        cov_h(:,k)=diag(P);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %      Position update      %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Check if a zero velocity update should be done. If so, do the
    % following
    if all(~isnan(posupt(:,k)))
        
        % Calculate the Kalman filter gain
        K=(P*H_pos')/(H_pos*P*H_pos' + R_pos);
        
        % Calculate the prediction error. 
        z = posupt(:,k) - x_h(1:3,k);   
        
        % Estimation of the perturbations in the estimated navigation
        % states
        dx=K*z;
        
        
        % Correct the navigation state using the estimated perturbations. 
        % (Subfunction located further down in the file.)
        [x_h(:,k), quat]=comp_internal_states(x_h(:,k),dx,quat);     % Subfunction located further down in the file.
    
        
        % Update the filter state covariance matrix P.
        P=(Id-K*H_pos)*P;
        
        % Make sure the filter state covariance matrix is symmetric. 
        P=(P+P')/2;
    
        % Store the diagonal of the state covariance matrix P.
        cov_h(:,k)=diag(P);
    end
        
end

end



























