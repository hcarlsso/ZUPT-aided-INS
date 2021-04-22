%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%> @file view_data.m  
%>
%> @brief Script for plotting the data from the zero-velocity aided inertial
%> navigations system.
%>
%> @authors Isaac Skog, John-Olof Nilsson
%> @copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Close all windows
close all

%% Generate a vector with the time scale
N=size(u,2);
t=0:simdata.Ts:(N-1)*simdata.Ts;


%% Plot the IMU data
figure(1)
clf
subplot(2,1,1)
plot(t,u(1:3,:)')
xlabel('time [s]')
ylabel('Specific force [m/s^2]')
title('Specific force (accelerometer) measurements')
legend('x-axis','y-axis','z-axis')
box on
grid on

subplot(2,1,2)
plot(t,u(4:6,:)'*180/pi)
xlabel('time [s]')
ylabel('Angular rate  [deg/s]')
title('Angular rate measurements')
legend('x-axis','y-axis','z-axis')
box on
grid on


%% Plot the trajectory in the horizontal plane
figure(2)
clf
plot(x_h(2,:),x_h(1,:))
hold
plot(x_h(2,1),x_h(1,1),'rs')
title('Trajectory')
legend('Trajectory','Start point')
xlabel('x [m]')
ylabel('y [m]')
axis equal
grid on
box on


%% Plot the height profile, the speed and when ZUPTs were applied

figure(3)
clf
subplot(3,1,1)
plot(t,-x_h(3,:))
title('Heigth')
xlabel('time [s]')
ylabel('z [m]')
grid on
box on


subplot(3,1,2)
plot(t,sqrt(sum(x_h(4:6,:).^2)))
title('Speed')
xlabel('time [s]')
ylabel('|v| [m/s]')
grid on
box on

subplot(3,1,3)
stem(t,zupt)
title('Zupt applied')
xlabel('time [s]')
ylabel('on/off')
grid on
box on


%% Plot the attitude

figure(4)
clf
plot(t,unwrap(x_h(7:9,:)')*180/pi)
title('Attitude')
xlabel('time [s]')
ylabel('Angle [deg]')
legend('Roll','Pitch','Yaw')
grid on
box on


%% Plot the diagonal elements of the filter covariance matrices as a
%% function of time

figure(5)
clf

subplot(3,1,1)
plot(t,sqrt(cov_h(1:3,:))')
title('Position covariance')
ylabel('sqrt(cov) [m]')
xlabel('time [s]')
legend('x-axis', 'y-axis','z-axis')
grid on
box on


subplot(3,1,2)
plot(t,sqrt(cov_h(4:6,:))')
title('Velocity covariance')
ylabel('sqrt(cov) [m/s]')
xlabel('time [s]')
legend('x-axis', 'y-axis','z-axis')
grid on
box on

subplot(3,1,3)
plot(t,sqrt(cov_h(7:9,:))'*180/pi)
title('Heading covariance')
ylabel('sqrt(cov) [deg]')
xlabel('time [s]')
legend('Roll', 'Pitch','Yaw')
grid on
box on


figure(6)
clf
plot(t,T)
hold on;
plot(t(zupt),T(zupt),'k.')
legend('Test statistics','ZUPT detected')
xlabel('time [s]')
ylabel('Test statistics')
title('Test statistics vs time')
grid on;
box on;


%% If the filter also estimates the sensor biases and/or the scalefactor,
%% plot these now

if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    %    Both scale and bias errors included
    figure(7)
    clf
    subplot(2,1,1)
    plot(t,x_h(10:12,:)')
    legend('x-axis','y-axis','z-axis')
    title('Accelerometer bias errors')
    xlabel('time [s]')
    ylabel('Bias [m/s^2]')
    grid on
    box on
    
    subplot(2,1,2)
    plot(t,x_h(13:15,:)'*180/pi)
    legend('x-axis','y-axis','z-axis')
    title('Gyroscope bias errors')
    xlabel('time [s]')
    ylabel('Bias [deg/s]')
    box on
    grid on
    
    figure(8)
    clf
    subplot(2,1,1)
    plot(t,x_h(16:18,:)')
    legend('x-axis','y-axis','z-axis')
    title('Accelerometer scale factor errors')
    xlabel('time [s]')
    box on
    grid on
    
    subplot(2,1,2)
    plot(t,x_h(19:21,:)')
    legend('x-axis','y-axis','z-axis')
    title('Gyroscope scale factor errors')
    xlabel('time [s]')
    box on
    grid on
    
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    %    Scale errors included
    figure(7)
    clf
    subplot(2,1,1)
    plot(t,x_h(10:12,:)')
    legend('x-axis','y-axis','z-axis')
    title('Accelerometer scale factor errors')
    xlabel('time [s]')
    box on
    grid on
    
    subplot(2,1,2)
    plot(t,x_h(13:15,:)'*180/pi)
    legend('x-axis','y-axis','z-axis')
    title('Gyroscope scale factor errors')
    xlabel('time [s]')
    box on
    grid on
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    %    Bias errors included
    figure(7)
    clf
    subplot(2,1,1)
    plot(t,x_h(10:12,:)')
    legend('x-axis','y-axis','z-axis')
    title('Accelerometer bias errors')
    xlabel('time [s]')
    ylabel('Bias [m/s^2]')
    grid on
    box on
    
    subplot(2,1,2)
    plot(t,x_h(13:15,:)'*180/pi)
    legend('x-axis','y-axis','z-axis')
    title('Gyroscope bias errors')
    xlabel('time [s]')
    ylabel('Bias [deg/s]')
    box on
    grid on
end








