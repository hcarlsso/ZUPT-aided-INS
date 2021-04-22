function [x] = dcm2euler(Rb2t)
%DCM2EULER Summary of this function goes here
%   Detailed explanation goes here
x = zeros(3,1);
% roll
x(1)=atan2(Rb2t(3,2),Rb2t(3,3));

% pitch
x(2)=-atan(Rb2t(3,1)/sqrt(1-Rb2t(3,1)^2));

%yaw
x(3)=atan2(Rb2t(2,1),Rb2t(1,1));

end

