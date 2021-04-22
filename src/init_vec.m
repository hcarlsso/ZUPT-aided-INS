%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [x_h cov Id]=init_vec(N,P)
%
%>
%> @brief Function that allocates memmory for the output of zero-velocity
%> aided inertial navigation algorithm.
%>
%> @param[out]  x_h     Matrix with the estimated navigation states. Each row holds the [position, velocity, attitude, (biases, if turned on),(scale factors, if turned on)] for time instant k, k=1,...N.
%> @param[out]  cov     Matrix with the diagonal elements of the state covariance matrices.
%> @param[out]  Id      Identity matrix.
%> @param[in]   N       The length of the IMU data vector u, i.e., the number of samples.
%> @param[in]   P       Initial state covariance matrix.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_h, cov_h, Id]=init_vec(N,P, simdata)




% Check which errors that are included in the state space model
if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    % Both scale and bias errors included
    cov_h=zeros(9+6+6,N);
    x_h=zeros(9+6+6,N);
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    % Scale errors included
    cov_h=zeros(9+6,N);
    x_h=zeros(9+6,N);
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    % Bias errors included
    cov_h=zeros(9+6,N);
    x_h=zeros(9+6,N);
else
    % Only the standard errors included
    cov_h=zeros(9,N);
    x_h=zeros(9,N);
end


Id=eye(size(P));
cov_h(:,1)=diag(P);
end