%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  function u_out=comp_imu_errors(u_in,x_h)
%
%> @brief Function that compensats for the artifacts in the IMU  
%> measurements with the current estimates of the sensors biases and/or  
%> scale factor errors.  
%>
%> @details Function that compensats for the artifacts in the IMU  
%> measurements with the current estimates of the sensors biases and/or  
%> scale factor errors. If the sensor errors are not included in the state 
%> space model used in the Kalman filter, no correction/compensation is 
%> done.   
%>     
%> @param[out]   u_out     Corrected/compensated IMU measurements.
%> @param[in]    u_in      Raw IMU measurements.
%> @param[in]    x_h       Navigation state vector, where the last states are the estimated sensor errors. 
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u_out=comp_imu_errors(u_in,x_h, simdata)


% Check which errors that are included in the state space model

if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    
    % Both scale and bias errors included
    temp=1./(ones(6,1)-x_h(16:end));
    u_out=diag(temp)*u_in+x_h(10:15);
    
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    
    % Scale errors included
    temp=1./(ones(6,1)-x_h(10:end));
    u_out=diag(temp)*u_in;
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    
    % Bias errors included
    u_out=u_in+x_h(10:end);
    
else
    
    % Only the standard errors included
    u_out=u_in;
end

end