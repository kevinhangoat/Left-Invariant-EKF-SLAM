clc; clear all; close all;
load('IMU_data.mat'); load('GPS_data.mat');
len = length([IMU_data]);
IMUData = [IMU_data.Time; IMU_data.dt; ...
           IMU_data.accelX; IMU_data.accelY; IMU_data.accelZ;  ...
           IMU_data.omegaX; IMU_data.omegaY; IMU_data.omegaZ]';
GPSData = [GPS_data.Time; GPS_data.dt; GPS_data.X; GPS_data.Y; GPS_data.Z]';
    
run(IMUData, GPSData, 0.3, 'InEKF')

