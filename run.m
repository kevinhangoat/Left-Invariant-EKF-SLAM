function varargout = run(IMUData, GPSData, pauseLen, filter_name)
addpath([cd, filesep, 'lib'])

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------

initialStateMean = [0 0 0 0 0 0 0 0 0]'; %position, velocity, heading in 3D
initialStateCov = eye(9);

% Motion noise (in odometry space, see Table 5.5, p.134 in book).
alphas = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
beta = deg2rad(5);

persistent numSteps;

numSteps = size(IMUData, 1);
global FIELDINFO;
FIELDINFO = getfieldinfo;

results = zeros(2,numSteps);
sys = system_initialization(alphas, beta);

filter = filter_initialization(sys, initialStateMean, initialStateCov, filter_name);

load('Ground_truth.mat');
figure
plot(Ground_truth(:, 1), Ground_truth(:, 2), 'b.')
hold on
for k = 1:numSteps
    %=================================================
    % data available to your filter at this time step
    %=================================================
    dt = IMUData(k, 2);
    ak = IMUData(k, 3:5); % angular acceleration
    wk = IMUData(k, 6:8); % acceleration
    Y = GPSData(k, 3:5);
    %=================================================
    % data *not* available to your filter, i.e., known
    % only by the simulator, useful for making error plots
    %=================================================

    
    
    
    

    %=================================================
    % graphics
    %=================================================
    
    
    
    
    
    
    
    %=================================================
    % update the filter based upon the
    % IMU and GPS data
    %=================================================
    
    switch filter_name
        % Only InEKF for this project
        case "InEKF"
            %CHANGE=========== use IMU data
            filter.prediction(wk, ak, dt)
            filter.correction(Y);
            results(1:2,k) = filter.mu(1:2, 5);
          
    end
    plot(results(1,k), results(2,k), 'r.')
    hold on  
    legend('Ground truth','IMU-GPS estimation')
    xlabel('x(m)')
    ylabel('y(m)')
    axis equal
    
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end
end


end


