% %------- section - 1: Interfacing IMU6050 sensor with Arduino Mega250 -------
% clc;
% clear all;
% % a = arduino(); %Update the name of communication port
% a = arduino('COM3', 'Mega2560', 'Libraries', 'I2C');
% fs = 20; % Sample Rate in Hz   
% imu = mpu6050(a,'SampleRate',fs,'OutputFormat','matrix'); 
% %%
% %------- section - 2: Reading the realtime data from IMU6050 sensor -------
% decim = 1;
% duration = 5; % seconds
% fs = 20;         % Hz
% N = duration*fs;
% i=0;
% 
% accelR=[];
% gyroR=[];
% 
% openExample('shared_fusion_arduinoio/EstimateOrientationUsingInertialSensorFusionAndMPU9250Example')
% viewer = HelperOrientationViewer('Title',{'Visualization of Orientation'})
% 
% while i<N
%     
%     [accelReadings, gyroReadings] = read(imu)
%     i=i+1;
%     accelR = [accelR;accelReadings]
%     gyroR = [gyroR;gyroReadings]
%     
%     fuse = imufilter('SampleRate',fs,'DecimationFactor',decim);
%     orientation = fuse(accelR,gyroR);
%     
%     % 3D figure or Sensor
%     for j = numel(orientation)
%        viewer(orientation(j));
%     end
%     
%     orientationEuler = eulerd(orientation,'ZYX','frame');
% end
% 
% N=N*10;
% timeVector = (0:(N-1))/fs;
% figure
% subplot(2,1,1)
% plot(timeVector,accelR)
% legend('X-axis','Y-axis','Z-axis')
% ylabel('Acceleration (m/s^2)')
% title('Accelerometer Readings')
% 
% subplot(2,1,2)
% plot(timeVector,gyroR)
% legend('X-axis','Y-axis','Z-axis')
% ylabel('Angular Velocity (rad/s')
% xlabel('Time (s)')
% title('Gyroscope Readings')
% 
% 
% %3D curve
% figure
% plot3(orientationEuler(:,1),orientationEuler(:,2),orientationEuler(:,3))
% legend('Z-axis','Y-axis','X-axis')
% xlabel('Time (s)')
% ylabel('Rotation (degrees)')
% title('Estimated Orientation')
% 





%------- section - 1: Interfacing IMU6050 sensor with Arduino Mega250 -------
clc;
clear all;
g=-9.8;
% a = arduino(); %Update the name of communication port
a = arduino('COM5', 'Mega2560', 'Libraries', 'I2C');
fs = 20; % Sample Rate in Hz   
imu = mpu6050(a,'SampleRate',fs,'OutputFormat','matrix'); 
%%
%------- section - 2: Reading the realtime data from IMU6050 sensor -------
decim = 1;
duration = 1; % seconds
fs = 20;         % Hz
N = duration*fs;
i1=0;
t=(0:((N*10)-1)).'/fs;
accelR=[];
gyroR=[];

openExample('shared_fusion_arduinoio/EstimateOrientationUsingInertialSensorFusionAndMPU9250Example')
viewer_imuf = HelperOrientationViewer('Title',{'Visualization of imuf_Orientation'})
viewer_CF = HelperOrientationViewer('Title',{'Visualization of CF_Orientation'})

orientation_CF = zeros(N*10,1,'quaternion');
angles_prev=zeros(10,3);
while i1<N
    alpha=0.98;

    
    [accelReadings, gyroReadings] = read(imu);
    i1=i1+1;
    accelR = [accelR;accelReadings];
    gyroR = [gyroR;gyroReadings];

  
   accel_norm=norm(accelReadings(1,:));
   error=abs(accel_norm-abs(g))/abs(g);
%    if error<=0.1
%        alpha=alpha;
%    end
%    if error>0.2
%        alpha=0;
%    end
%    if error>0.1 && error<=0.2
%        alpha=alpha*(-10*error+2);
%    end
       



    gyro_pitch=gyroReadings(:,2)*(1/fs)+angles_prev(1,1);
    gyro_roll=gyroReadings(:,1)*(1/fs)+angles_prev(1,2);
    gyro_yaw=gyroReadings(:,3)*(1/fs)+angles_prev(1,3);
    R_gyro=eul2rotm([gyro_yaw(1,1),gyro_pitch(1,1),gyro_roll(1,1)]);
    
    g_gyro=R_gyro*[0,0,g]';
    %accelReadings=0.5*accelReadings+0.5*g_gyro';
        
    %accel_pitch=-(asin(accelReadings(:,1)/g));
  
    
    accel_pitch=-atan2(accelReadings(:,1),sqrt(accelReadings(:,2).^2 +accelReadings(:,3).^2));
    accel_roll=atan2(accelReadings(:,2),accelReadings(:,3));  %% use this for full 180 deg roll
  %accel_roll=(asin(accelReadings(:,2)./(g*cos(accel_pitch)))); %%%[-90,90]
    
   

    
    
    
    accel_pitch_roll=0.98*[accel_pitch,0.98*accel_roll/0.98];
    gyro_pitch_roll=(0.02)*[gyro_pitch,0.02*gyro_roll/0.02];
    res_ag=accel_pitch_roll+gyro_pitch_roll;
    disp(res_ag)
   
    R=[cos(res_ag(1,1)) sin(res_ag(1,1))*sin(res_ag(1,2)) sin(res_ag(1,1))*cos(res_ag(1,2)); 0 cos(res_ag(1,2)) -sin(res_ag(1,2));
       -sin(res_ag(1,1)) sin(res_ag(1,2))*cos(res_ag(1,1)) cos(res_ag(1,1))*cos(res_ag(1,2))];
    %L_b_m=R*magReadings';
%     mag_yaw=atan2(L_b_m(2,:),L_b_m(1,:));
%     mag_yaw=mag_yaw';
%     
%     res_mg=0.98*mag_yaw+0.02*gyro_yaw;
    
    res=[res_ag,res_ag,zeros(10,1)];
    disp(res);

    angle=res;
     angle_prev=angle;
     
     
     
    A=zeros(size(res,1),1);
    B=zeros(size(res,1),1);
    C=zeros(size(res,1),1);
    D=zeros(size(res,1),1);
    
    for i=1:size(res,1)
        v=res(i,:);
        q1=[cos(v(1,3)/2),0,0,sin(v(1,3)/2)];
        q2=[cos(v(1,1)/2),0,sin(v(1,1)/2),0];
        q3=[cos(v(1,2)/2),sin(v(1,2)/2),0,0];
        qmul=quatmultiply(q1,quatmultiply(q2,q3));
        A(i,1)=qmul(1,1);
        B(i,1)=qmul(1,2);
        C(i,1)=qmul(1,3);
        D(i,1)=qmul(1,4);
        
    end
  
    q=quaternion(A,B,C,D);
    
    % euler 321: yaw(z)-pitch(y)-roll(x)
    
    orientation_CF(1+10*(i1-1):i1*10)=(q);
  
    %orientationDefault(i) = aFilter(accelBody,gyroBody);
%     orientation_gt=orientationNED(i);
%     orientation_est=orientationDefault(i);
    
    
    
    
    fuse = imufilter('SampleRate',fs,'DecimationFactor',decim);
    orientation = fuse(accelR,gyroR);
  
    % 3D figure or Sensor
    for j =1: 10%numel(orientation)
       %viewer_imuf(orientation(j));
       viewer_CF(q(j));
    end
    
    orientationEuler = eulerd(orientation,'ZYX','frame');
end

N=N*10;
timeVector = (0:(N-1))/fs;
% figure
% subplot(2,1,1)
% plot(timeVector,accelR)
% legend('X-axis','Y-axis','Z-axis')
% ylabel('Acceleration (m/s^2)')
% title('Accelerometer Readings')
% 
% subplot(2,1,2)
% plot(timeVector,gyroR)
% legend('X-axis','Y-axis','Z-axis')
% ylabel('Angular Velocity (rad/s')
% xlabel('Time (s)')
% title('Gyroscope Readings')


%3D curve
figure
plot3(orientationEuler(:,1),orientationEuler(:,2),orientationEuler(:,3))
legend('Z-axis','Y-axis','X-axis')
xlabel('Time (s)')
ylabel('Rotation (degrees)')
title('Estimated Orientation')


euler_orientation_CF=eulerd(orientation_CF,'ZYX','frame');

figure(5)
plot3(euler_orientation_CF(:,1),euler_orientation_CF(:,2),euler_orientation_CF(:,3))
legend('Z-axis','Y-axis','X-axis')
xlabel('Time (s)')
ylabel('Rotation (degrees)')
title('Estimated Orientation')
disp(size(orientationEuler(:,1:2)));
disp((eulerd(orientation_CF,'ZYX','frame')));

figure(3)
plot(timeVector,orientationEuler(:,[2 3]));
xlabel('Time(s)')
ylabel('Rotation(deg)')
title('orientation ---IMUFILTER');
legend('Y-axis(PITCH)','X-axis(ROLL)');
timeVector = (0:(N-1))/fs;
figure(4)
plot(timeVector,euler_orientation_CF(:,[1 3]));
xlabel('Time(s)')
ylabel('Rotation(deg)')
title('orientation ---complementary filter');
legend('Y-axis(PITCH)','X-axis(ROLL)');
figure
subplot(2,1,1)
plot(timeVector,accelR)
legend('X-axis','Y-axis','Z-axis')
ylabel('Acceleration (m/s^2)')
title('Accelerometer Readings')

subplot(2,1,2)
plot(timeVector,gyroR)
legend('X-axis','Y-axis','Z-axis')
ylabel('Angular Velocity (rad/s')
xlabel('Time (s)')
title('Gyroscope Readings')
