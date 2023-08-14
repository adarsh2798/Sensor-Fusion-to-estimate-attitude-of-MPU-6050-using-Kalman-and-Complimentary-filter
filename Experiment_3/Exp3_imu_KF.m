





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

orientation_CF = zeros(N*10,1,'quaternion');
angles_prev=zeros(10,3);

%%%%%%%%%%%%%%%%%%%%% KF variables%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_pre=zeros((N+1)*10,1,'quaternion');
x_up=zeros((N+1)*10,1,'quaternion');
x0_est=quaternion(1.0*ones(10,1),0.005*ones(10,1),0.005*ones(10,1),0.00*ones(10,1));
x_up(1:10,1)=x0_est;
% P_up=diag([0^2;0.005^2;0.005^2;0.005^2]);
% Q=diag([0.0001^2;0.0001^2;0.0001^2;0.0001^2]);
% R=diag([0.9^2;0.4^2;0.7^2]/1000);

P_up=diag([0.0^2;0.005^2;0.005^2;0.00^2])*100;
Q=diag([0.1^2;0.1^2;0.1^2;0.1^2])/(1);
R=diag([0.2^2;0.2^2;0.2^2])/(100000);

dt=1/fs;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 disp(quat2eul(x_up((i1*10)+1:(i1*10)+10))*180/pi);


while i1<N
    

    
    [accelReadings, gyroReadings] = read(imu);
    i1=i1+1;
    accelR = [accelR;accelReadings];
    gyroR = [gyroR;gyroReadings];
    
    for r=1:10
        w=gyroReadings(r,:);
        
        w_q_ned=quatmultiply(quatmultiply(quatinv(compact(normalize(x_up(((i1-1)*10)+r)))),[0,w]),compact(normalize(x_up(((i1-1)*10)+r))));
        w=w_q_ned(:,2:4);
        w(1,3)=0;
        norm_w=norm(w);
        a=0.5*dt*norm_w;
         phi=[cos(a),-w(1,1)*sin(a)/norm_w,-w(1,2)*sin(a)/norm_w,-w(1,3)*sin(a)/norm_w;
              w(1,1)*sin(a)/norm_w,cos(a),-w(1,3)*sin(a)/norm_w,w(1,2)*sin(a)/norm_w;
              w(1,2)*sin(a)/norm_w,w(1,3)*sin(a)/norm_w,cos(a),-w(1,1)*sin(a)/norm_w;
              w(1,3)*sin(a)/norm_w,-w(1,2)*sin(a)/norm_w,w(1,1)*sin(a)/norm_w,cos(a)];
%         phi=[cos(a),w(1,1)*sin(a)/norm_w,w(1,2)*sin(a)/norm_w,w(1,3)*sin(a)/norm_w;
%               -w(1,1)*sin(a)/norm_w,cos(a),w(1,3)*sin(a)/norm_w,-w(1,2)*sin(a)/norm_w;
%               -w(1,2)*sin(a)/norm_w,-w(1,3)*sin(a)/norm_w,cos(a),w(1,1)*sin(a)/norm_w;
%               -w(1,3)*sin(a)/norm_w,w(1,2)*sin(a)/norm_w,-w(1,1)*sin(a)/norm_w,cos(a)];
        
        
        
        q1=[cos(a),w(1,1)*sin(a)/norm_w,w(1,2)*sin(a)/norm_w,w(1,3)*sin(a)/norm_w];
        omega_mat=[0 ,-w(1,1),-w(1,2),-w(1,3);
                   w(1,1),0,w(1,3),-w(1,2);
                   w(1,2),-w(1,3),0,w(1,1);
                   w(1,3),w(1,2),-w(1,1),0];
        %pred=quatmultiply(q1,compact(x_up(((i1-1)*10)+r)))';
        pred=phi*compact(x_up(((i1-1)*10)+r))';
        
%         pred=expm(omega_mat*dt/2)*compact(x_up(((i1-1)*10)+r))';
        x_pre((i1*10)+r)=quaternion(pred');
        P_pre=phi*P_up*phi'+Q;
        H=[
           2*g*pred(3,1),2*g*pred(4,1),2*g*pred(1,1),2*g*pred(2,1);
           -2*g*pred(2,1),-2*g*pred(1,1),2*g*pred(4,1),2*g*pred(3,1);
           2*g*pred(1,1),-2*g*pred(2,1),-2*g*pred(3,1),2*g*pred(4,1)];
       H=H/(norm(pred)^2);
%        H=[
%            2*g*pred(3,1),2*g*pred(4,1),-2*g*pred(1,1),-2*g*pred(2,1);
%            -2*g*pred(2,1),2*g*pred(1,1),2*g*pred(4,1),-2*g*pred(3,1);
%            2*g*pred(1,1),-2*g*pred(2,1),2*g*pred(3,1),2*g*pred(4,1)];
        z=[accelReadings(r,:)]';
        z_hat=H*pred;
        y=z-z_hat;
        K=P_pre*(H')*inv(H*P_pre*H'+R);
        update=pred+K*y;
        
        x_up((i1*10)+r)=quaternion(update');
        P_up=(eye(4)-K*H)*P_pre;

       
disp(update);
                      
    end
    retrieved_euler=quat2eul(x_up((i1*10)+1:(i1*10)+10));
    ypr=zeros(10,3);
    ypr(:,2)=retrieved_euler(:,2);
    ypr(:,3)=retrieved_euler(:,3);
    new_quat=eul2quat(ypr);
    orientation_CF(1+10*(i1-1):i1*10)=quaternion(new_quat(:,1),new_quat(:,2),new_quat(:,3),new_quat(:,4));
  
    %orientationDefault(i) = aFilter(accelBody,gyroBody);
%     orientation_gt=orientationNED(i);
%     orientation_est=orientationDefault(i);
    disp(quat2eul(x_up((i1*10)+1:(i1*10)+10))*180/pi);
    
    q_anim=quaternion(new_quat(:,1),new_quat(:,2),new_quat(:,3),new_quat(:,4));
    
    fuse = imufilter('SampleRate',fs,'DecimationFactor',decim);
    orientation = fuse(accelR,gyroR);
  
    % 3D figure or Sensor
    for j =1: 10%numel(orientation)
       %viewer_imuf(orientation(j));
       viewer_CF(q_anim(j));
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


figure(3)
plot(timeVector,orientationEuler(:,[2 3]));
xlabel('Time(s)')
ylabel('Rotation(deg)')
title('orientation ---IMUFILTER');
legend('Y-axis(PITCH)','X-axis(ROLL)');
timeVector = (0:(N-1))/fs;
figure(4)
plot(timeVector,euler_orientation_CF(:,[2 3]));
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
