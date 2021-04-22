%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [P Q R H]=init_filter()
%
%> @brief Function that initializes the Kalman filter.
%>
%> @details Function that initializes the Kalman filter. That is, the
%> function generates the initial covariance matrix P, the process noise
%> covariance matrix Q, the measurement noise covariance matrix R, and
%> observation matrix H, based upon the settings defined in the function
%> settings.m
%>
%> @param[out]   P     Initial state covariance matrix.
%> @param[out]   Q     Process noise covariance matrix.
%> @param[out]   R     Measurement noise covariance matrix.
%> @param[out]   H     Measurement observation matrix.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [P, Q, R, H] = init_filter(simdata)


% Check which errors that are included in the state space model and
% allocate P,Q, and H matrices with the right size.

if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on')) % Both scale and bias errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_bias.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_bias.^2);
    P(16:18,16:18)=diag(simdata.sigma_initial_acc_scale.^2);
    P(19:21,19:21)=diag(simdata.sigma_initial_gyro_scale.^2);
    
    
    
    Q=zeros(12);
    Q(7:9,7:9)=diag(simdata.acc_bias_driving_noise.^2);
    Q(10:12,10:12)=diag(simdata.gyro_bias_driving_noise.^2);
    
    
    H=zeros(3,9+6+6);
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off') % Scale errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_scale.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_scale.^2);
    
    % Process noise covariance matrix
    Q=zeros(6);
    
    % Observation matrix
    H=zeros(3,9+6);
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on') % Bias errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_bias.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_bias.^2);
    
    % Process noise covariance matrix
    Q=zeros(12);
    Q(7:9,7:9)=diag(simdata.acc_bias_driving_noise.^2);
    Q(10:12,10:12)=diag(simdata.gyro_bias_driving_noise.^2);
    
    % Observation matrix
    H=zeros(3,9+6);
    
else % Only the standard errors included
    
    %  Initial state covariance matrix
    P=zeros(9);
    
    % Process noise covariance matrix
    Q=zeros(6);
    
    % Observation matrix
    H=zeros(3,9);
end


% General values for the observation matrix H
H(1:3,4:6)=eye(3);

% General values for the initial covariance matrix P
P(1:3,1:3)=diag(simdata.sigma_initial_pos.^2);
P(4:6,4:6)=diag(simdata.sigma_initial_vel.^2);
P(7:9,7:9)=diag(simdata.sigma_initial_att.^2);

% General values for the process noise covariance matrix Q
Q(1:3,1:3)=diag(simdata.sigma_acc.^2);
Q(4:6,4:6)=diag(simdata.sigma_gyro.^2);

% General values for the measurement noise matrix R
R=diag(simdata.sigma_vel.^2);

end
