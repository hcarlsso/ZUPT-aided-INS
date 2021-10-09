 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [x quat]=init_Nav_eq(u)
%
%> @brief Function that calculates the initial state of the navigation
%> equations.
%>
%> @details Function that calculates the initial state of the navigation
%> equations. That is, it does a simple initial alignment of the navigation
%> system, where the roll and pitch of the system is estimated from the
%> 20 first accelerometer readings. All other states are set according to
%> the information given in the function "settings.m".
%>
%> @param[out]  x     Initial navigation state vector.
%> @param[out]  quat  Quaternion vector, representating the initial attitude of the platform.
%> @param[in]   u     Matrix with the IMU data.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x, quat]=init_Nav_eq(u, simdata)


% Under the assumption that the system is stationary during the first 20
% samples, the initial roll and pitch is calculate from the 20 first
% accelerometer readings.
f_u=mean(u(1,1:20));
f_v=mean(u(2,1:20));
f_w=mean(u(3,1:20));

roll=atan2(-f_v,-f_w);
pitch=atan2(f_u,sqrt(f_v^2+f_w^2));


% Set the attitude vector
attitude=[roll pitch simdata.init_heading]';

% Calculate quaternion corresponing to the initial attitude
Rb2t=Rt2b(attitude)';
quat=dcm2q(Rb2t);

% Set the initial state vector
x=zeros(9,1);
x(1:3,1)=simdata.init_pos;
x(7:9,1)=attitude;
end
