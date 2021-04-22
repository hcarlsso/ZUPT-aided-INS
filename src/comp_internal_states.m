%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  function [x_out q_out]=comp_internal_states(x_in,dx,q_in)
%
%> @brief Function that corrects the estimated navigation states with 
%> the by the Kalman filter estimated  system perturbations (errors). 
%>
%> @details Function that corrects the estimated navigation states with 
%> the by the Kalman filter estimated system perturbations (errors). That
%> is, the current position an velocity estimates of the navigation 
%> platform is corrected by adding the estimated system perturbations to 
%> these states. To correct the orientation state (Euler angles and 
%> quaternion vector), the quaternion vector are first converted into a
%> rotation matrix, which then is corrected using the estimated orientation 
%> perturbations. The corrected rotation matrix is then transformed back 
%> into a quaternion vector, as well as the equivalent vector of Euler 
%> angles.         
%>
%> @param[out]   x_out     Corrected (posteriori) navigation state vector.
%> @param[out]   q_out     Corrected (posteriori) quaternion vector.
%> @param[in]    x_in      A priori estimated navigation state vector.
%> @param[in]    q_in      A priori estimated quaternion vector.
%> @param[in]    dx        Vector of system perturbations
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_out, q_out]=comp_internal_states(x_in,dx,q_in)

% Convert quaternion to a rotation matrix
R=q2dcm(q_in);

% Correct the state vector
x_out=x_in+dx;

% Correct the rotation matrics
epsilon=dx(7:9);
OMEGA=[0 -epsilon(3) epsilon(2); 
       epsilon(3) 0 -epsilon(1); 
       -epsilon(2) epsilon(1) 0];
R=(eye(3)-OMEGA)*R;


% Get the corrected roll, pitch and heading from the corrected rotation
% matrix
x_out(7)=atan2(R(3,2),R(3,3));
x_out(8)=-atan(R(3,1)/sqrt(1-R(3,1)^2));
x_out(9)=atan2(R(2,1),R(1,1));

% Calculte the corrected quaternions
q_out=dcm2q(R);



end
