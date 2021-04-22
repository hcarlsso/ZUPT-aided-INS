%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%> @file zero_velocity_detector.m
%>
%> @brief Functions for implementing different zero-velocity detection 
%> algorithms. 
%>
%> @details Functions for implementing different zero-velocity detection 
%> algorithms, as well as a wrapper function for easy to use purpose. The
%> settings used by the wrapper function "zero_velocity_detector()" is  
%> specified in the file \a setting.m. Details about the detectors can be 
%> found in papers
%> 
%> \li <A href="http://dx.doi.org/10.1109/TBME.2010.2060723">Zero-Velocity Detection -- An Algorithm Evaluation</A> 
%> \li <A href="http://dx.doi.org/10.1109/IPIN.2010.5646936">Evaluation of Zero-Velocity Detectors for Foot-Mounted Inertial Navigation Systems</A>
%>   
%>
%> @authors Isaac Skog, John-Olof Nilsson
%> @copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [zupt T] = zero_velocity_detector(u) 
%
%>
%> @brief Wrapper function for running the zero-velocity detection 
%> algorithms. 
%>
%> @details A wrapper function that runs the zero-velocity detection 
%> algorithm that was specified in the file \a setting.m. 
%>
%> @param[out]  zupt       Vector with the detector decsions. [ true = zero velocity, false = moving]    
%> @param[out]  T          The test statistics of the detector 
%> @param[in]   u          The IMU data vector.     
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [zupt,T]=zero_velocity_detector(u, simdata)

% Allocate memmory
zupt=true(1,length(u));

% Run the desired detector type. Each detector return a vector with their 
% calculated test statistics T. 
switch simdata.detector_type
    
    case 'GLRT'
        T=GLRT(u, simdata);    
    
    case 'MV'
        T=MV(u, simdata);
        
    case 'MAG'
        T=MAG(u, simdata);
        
    case 'ARE'
        T=ARE(u, simdata);
        
    otherwise
        disp('The choosen detector type not recognized. The GLRT detector is used')
        T=GLRT(u, simdata);
end

% Check if the test statistics T are below the detector threshold. If so, 
% chose the hypothesis that the system has zero velocity 
W=simdata.Window_size;
for k=1:length(T)
    if T(k)>simdata.gamma
       zupt(k:k+W-1)=false(1,W); 
    end    
end

% Fix the edges of the detector statistics
T=[max(T)*ones(1,floor(W/2)) T max(T)*ones(1,floor(W/2))];
end





%% SUBFUNCTIONS 










