%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [F G]=state_matrix(q,u)
%
%> @brief Function for calculating the state transition matrix F and 
%> the process noise gain matrix G. 
%>
%> @details Function for calculating the state transition matrix F and 
%> the process noise gain matrix G, given the current orientation of 
%> the platform and the specific force vector.  
%>
%> @param[out]   F     State transition matrix.
%> @param[out]   G     Process noise gain matrix.
%> @param[in]    u     IMU data [specific force, angular rates].
%> @param[in]    q     Old quaternions
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [F,G]=state_matrix(q,u, simdata)


% Convert quaternion to a rotation matrix
Rb2t=q2dcm(q);

% Transform measured force to force in
% the navigation coordinate system.
f_t=Rb2t*u(1:3);

% Create a ske symmetric matrix of the specific fore vector
St=[0 -f_t(3) f_t(2); f_t(3) 0 -f_t(1); -f_t(2) f_t(1) 0];

% Zero matrix
O=zeros(3);

% Identity matrix
I=eye(3);

% Diagonal matrices with the specific fore and angular rate along the
% diagonals.
Da=diag(u(1:3));
Dg=diag(u(4:6));

% Correlation constant for accelerometer and gyro biases
B1=-1/simdata.acc_bias_instability_time_constant_filter*eye(3);
B2=-1/simdata.gyro_bias_instability_time_constant_filter*eye(3);


% Check which errors that are included in the state space model
if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    % Both scale and bias errors included
    Fc=[O I O   O     O     O         O    ;
        O O St Rb2t   O    Rb2t*Da    O    ;
        O O O   O   -Rb2t   O     -Rb2t*Dg ;
        O O O   B1    O     O         O    ;
        O O O   O     B2    O         O    ;
        O O O   O     O     O         O    ;
        O O O   O     O     O         O   ];
    
    % Noise gain matrix
    Gc=[O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I; O O O O; O O O O];
    
    
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    % Scale errors included
    Fc=[O I O       O       O    ;
        O O St  Rb2t*Da     O    ;
        O O O       O   -Rb2t*Dg ;
        O O O       O       O    ;
        O O O       O       O];
    
    % Noise gain matrix
    Gc=[O O; Rb2t O ; O -Rb2t; O O; O O];
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    % Bias errors included
    Fc=[O I O O O;
        O O St Rb2t O;
        O O O O -Rb2t;
        O O O B1 O;
        O O O O B2];
    
    % Noise gain matrix
    Gc=[O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I];
    
else
    Fc=[O I O;
        O O St;
        O O O];
    
    % Noise gain matrix
    % Should not have minus sign here. But does not influence solution
    % since G is squared.
    Gc=[O O; Rb2t O; O Rb2t];
    
end


% Approximation of the discret time state transition matrices
F=eye(size(Fc))+simdata.Ts*Fc;
G=simdata.Ts*Gc;
end
