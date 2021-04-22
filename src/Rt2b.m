%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  function R=Rt2b(ang)
%
%>
%> @brief Function that calculates the rotation matrix for rotating a 
%> vector from coordinate frame t to the coordinate frame b, given a
%> vector of Euler angles.
%>
%> @param[out]  R      Rotation matrix.
%> @param[in]   ang    Euler angles [roll,pitch,heading]
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R=Rt2b(ang)

% roll
cr=cos(ang(1));
sr=sin(ang(1));

% pitch 
cp=cos(ang(2));
sp=sin(ang(2));

% yaw
cy=cos(ang(3));
sy=sin(ang(3));

R=[cy*cp sy*cp -sp; 
    -sy*cr+cy*sp*sr cy*cr+sy*sp*sr cp*sr; 
    sy*sr+cy*sp*cr -cy*sr+sy*sp*cr cp*cr];

end