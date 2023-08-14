%------- section - 1: Interfacing IMU6050 sensor with Arduino Mega250 -------
clc;
clear all;
g=9.8;
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

orientation_KF = zeros(N*10,1,'quaternion');
angles_prev=zeros(10,3);

%%%%%%%%%%%%%%%%%%%%% Iniialization for kalman filter %%%%%%%%%%%%%%%%%%%%%%%%%%%%
o=[0;0;0]; %orientation=o=[roll;pitch;yaw]
gyro_offset=[0;-0.09;0];
lin_accel=[0;0;0];

x_pre=zeros((N+1)*10,9);
x_up=zeros((N+1)*10,9);
x0_est=[0.01*ones(10,3),0.0*ones(10,3),0.0*ones(10,3)];
x_up(1:10,:)=x0_est;
P_up=diag([0.01^2;0.01^2;0.01^2;0^2;0^2;0^2;0^2;0^2;0^2])*100;
Q=1*diag([0.000006092348396;0.000006092348396;0.000006092348396;0.000076154354947;0.000076154354947;0.000076154354947;
    0.009623610000000;0.009623610000000;0.009623610000000]);
R=1*diag([0.0099074550003;0.0099074550003;0.0099074550003]);

% Q=1*diag([0.1^2/10000;0.1^2/10000;0.1^2/10000;0.00000304;0.00000304;0.00000304;0.03;0.03;0.03]);
% R=1*diag([0.2^2;0.2^2;0.2^2]);

dt=1/fs;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



while i1<N
    

    
    [accelReadings, gyroReadings] = read(imu);
    i1=i1+1;
    accelR = [accelR;accelReadings];
    gyroR = [gyroR;gyroReadings];
    
    q_mat=zeros(10,4);
    Aq=zeros(10,1);
    Bq=zeros(10,1);
    Cq=zeros(10,1);
    Dq=zeros(10,1);
    
    for r=1:10
        w=gyroReadings(r,:)';
%          R=eul2rotm([o(3,1),o(2,1),o(1,1)]);
%          w=R'*w;
%          
        
       norm_w=norm(w);
       phi=zeros(9,9);
       pred=phi*x_up(((i1-1)*10)+r,:)';
        
        x_pre((i1*10)+r,:)=pred';
        P_pre=phi*P_up*phi'+Q;
        
        del_o=(w-gyro_offset)*dt; %gyro_offset is always subtracted to compensate for the drift
        o=o+del_o;
        R=eul2rotm([o(3,1),o(2,1),o(1,1)]);
        g_gyro=R*[0;0;g];
        
        g_accel=accelReadings(r,:)'-lin_accel;
        

        z=g_gyro-g_accel;
        
        gx=g_gyro(1,1);
        gy=g_gyro(2,1);
        gz=g_gyro(3,1);
        H=[ 0 , gz, -gy, 0, -dt*gz, dt*gy, 1, 0, 0;
            -gz, 0, gx, dt*gz, 0, -dt*gx, 0, 1, 0;
            gy, -gx, 0, -dt*gy, dt*gx, 0, 0, 0, 1];
        z_hat=H*pred;
        y=z-z_hat;
        K=P_pre*(H')*inv(H*P_pre*H'+R);
        update=pred+K*y;
        
        x_up((i1*10)+r,:)=(update');
        P_up=(eye(9)-K*H)*P_pre;
                
        o=o+update(1:3);
        decay=0.5;
        lin_accel=decay*lin_accel-update(7:9); % lin_accel model is ak=c*ak-1+w where w=noise c=decay
        gyro_offset=gyro_offset-update(4:6);
        
        
        q1=[cos(0*o(3,1)/2),0,0,sin(0*o(3,1)/2)];
        q2=[cos(o(2,1)/2),0,sin(o(2,1)/2),0];
        q3=[cos(o(1,1)/2),sin(o(1,1)/2),0,0];
        qmul=quatmultiply(q1,quatmultiply(q2,q3));
        q_mat(r,:)=[qmul(1),qmul(2),qmul(3),qmul(4)];
         Aq(r,1)=qmul(1,1);
        Bq(r,1)=qmul(1,2);
        Cq(r,1)=qmul(1,3);
        Dq(r,1)=qmul(1,4);
        
        

                      
    end
    
    orientation_KF(1+10*(i1-1):i1*10)=quaternion(Aq,Bq,Cq,Dq);
  
    %orientationDefault(i) = aFilter(accelBody,gyroBody);
%     orientation_gt=orientationNED(i);
%     orientation_est=orientationDefault(i);
    
    
    q_anim=quaternion(Aq,Bq,Cq,Dq);
    
    fuse = imufilter('SampleRate',fs,'DecimationFactor',decim);
    orientation = fuse(accelR,gyroR);
  
    % 3D figure or Sensor
    for j =1: 10%numel(orientation)
       %viewer_imuf(orientation(j));
       viewer_CF(q_anim(j));
    end
    
    orientationEuler = eulerd(orientation,'ZYX','frame');
end
orientation_KF=eulerd(orientation_KF,'ZYX','frame');
disp((x_up*180/pi));
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


% %3D curve
% figure
% plot3(orientationEuler(:,1),orientationEuler(:,2),orientationEuler(:,3))
% legend('Z-axis','Y-axis','X-axis')
% xlabel('Time (s)')
% ylabel('Rotation (degrees)')
% title('Estimated Orientation')
% 
% 
% euler_orientation_CF=eulerd(orientation_CF,'ZYX','frame');
% 
% figure(5)
% plot3(euler_orientation_CF(:,1),euler_orientation_CF(:,2),euler_orientation_CF(:,3))
% legend('Z-axis','Y-axis','X-axis')
% xlabel('Time (s)')
% ylabel('Rotation (degrees)')
% title('Estimated Orientation')


figure(3)
plot(timeVector,orientationEuler(:,[2 3]));
xlabel('Time(s)')
ylabel('Rotation(deg)')
title('orientation ---IMUFILTER');
legend('Y-axis(PITCH)','X-axis(ROLL)');
timeVector = (0:(N-1))/fs;
figure(4)
plot(timeVector,orientation_KF(:,[2 3]));
xlabel('Time(s)')
ylabel('Rotation(deg)')
title('orientation ---KALMAN filter');
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
