close all; clear all; clc;

%% Static Analysis
static_data = readtable('sensors_data_2024-12-10_20-18-11_static_15s_1000_sample_freq');

%% Extract and preprocess data variables
static_times = static_data.("Time");
static_A1 = static_data.("AccelZ1"); %MPU6050 -- will be blue
static_A2 = static_data.("AccelZ2"); %ADXL    -- red
static_Ts = mean(diff(static_times)); %av time sample

%% Plot Raw Accel
% figure;
% plot(times, static_A1, "b-", 'DisplayName', 'MPU6050');
% hold on;
% plot(times, static_A2, "r-", 'DisplayName', 'ADXL345');

% %% interpolation (for ADXL testing)
% for i=2:length(static_times)-1
%     if abs(static_A2(i)) > 4
%         if abs(static_A2(i+1)) > 4
%             static_A2(i) = static_A2(i-1);
%         else
%             static_A2(i) = (static_A2(i+1)+static_A2(i-1))/2;
%         end
%     end
% end

%% Plot interpolated ADXL accel
% figure;
% plot(times, static_A2, "g--", 'DisplayName', 'ADXL345interpo');
% ylabel('Acceleration [m/s^2]')
%     xlabel('Time elapsed [s]')
%     legend("show");


%% calculate variances and mean offsets
MPU_static_var = var(static_A1);
ADXL_static_var = var(static_A2);
MPU_static_mean = mean(static_A1);
ADXL_static_mean = mean(static_A2);
disp(MPU_static_var);
disp(ADXL_static_var);



%% end of static data analysis, begin moving data analysis
%%%%
%%%%
%%%%
%%%%
%%%%
%%%%

%% Moving Data Initialization

% Load data from the file
data = readtable('sensors_data_2024-12-10_23-47-56_oscillating_15s_1000_sample_freq.csv');
% data = readtable('sensors_data_2024-12-11_00-23-57_other_oscillator_15s_1000_sample_freq.csv');

%% Extract and preprocess data variables
times = data.("Time");
A1 = data.("AccelZ1"); %MPU6050 -- will be blue
A2 = data.("AccelZ2"); %ADXL    -- red
Ts = mean(diff(times)); %av time sample

%% Plot Raw Accel
figure;
plot(times, A1, "b-", 'DisplayName', 'MPU6050');
hold on;
plot(times, A2, "r-", 'DisplayName', 'ADXL345');
title('Raw Sensor Readings');
ylabel('m/s^2');

% %% interpolation (for ADXL testing)
% for i=2:length(times)-1
%     if abs(A2(i)) > 4
%         if abs(A2(i+1)) > 4
%             A2(i) = A2(i-1);
%         else
%             A2(i) = (A2(i+1)+A2(i-1))/2;
%         end
%     end
% end

% %% Plot interpolated ADXL accel
% figure;
% plot(times, A2, "g--", 'DisplayName', 'ADXL345interpo');
% ylabel('Acceleration [m/s^2]')
%     xlabel('Time elapsed [s]')
%     legend("show");


%% calculate variances and remove mean offsets


% this is more accurate, but kind of cheating if you
% are supposed to be "predicting"
A1C = A1 - mean(A1); 
A2C = A2 - mean(A2);
% A1C = A1 - MPU_static_mean; % using the mean from the moving data (above) seems to give
% much better results
% A2C = A2 - ADXL_static_mean;
var_1 = var(A1);
var_2 = var(A2);

disp(var_1);
disp(var_2);
% disp(mean(A1));
% disp(mean(A2));



%% Define measurements that will be used for estimators
measurements = zeros(2, length(times));
measurements(1,:) = A1C;
measurements(2,:) = A2C;

%% Plot measurements after all conditioning (Much nicer!)
figure;
subplot(2,1,1);
plot(times, measurements(1,:), "b"); hold on;
title('Conditioned Accel MPU');
ylabel('m/s^2');

subplot(2,1,2);
plot(times, measurements(2,:), "r"); hold on;
title('Conditioned Accel ADXL');
ylabel('m/s^2');





% End of Data conditioning, begin EKF
%%%%%%%%
%%%%%%%%
%%%%%%%%
%%%%%%%%
%%%%%%%%
%%%%%%%%

% Extended Kalman Filter for Linearly Oscillating Mass

% Parameters
% T = 1/1000;                % Time step
T = Ts;
% num_steps = 5000;         % Number of steps
num_steps = length(times);
% sigma_w = [1e-4, 1e-4];  % Process noise variances
sigma_w = 0; % no process noise
% sigma_v = [0.05, .1];   % Measurement noise variances (different for each sensor)
sigma_v = [var_1, var_2]; 

%% Initial State [position; velocity; frequency]
x_0 = [0; 0; 4];        % Initial guess: position=1, velocity=0, frequency=2*pi rad/s
% EKF is mildly sensitive to x_0, would not recommend touching this
P = diag([10, 10, 10]); % Initial state covariance
% could play with this, it seems to either work fine or entirely break it.
% if it ain't broke, don't fix it

% State Transition and Measurement Models
f = @(x) [x(1) + T*x(2); 
          x(2) - T*x(3)^2*x(1);
          x(3)];  
    % State dynamics function (phi, or A matrix)
    % this must be a FUNCTION and not a matrix because it's nonlinear
    % position = pos + dt * vel
    % vel = vel - dt * freq^2 * pos;
    % freq = freq

h = @(x) [-x(3)^2 * x(1);   % both equal because both read same quantity
          -x(3)^2 * x(1)];  % Measurements: 2 accelerometer readings
    % -(radians/sec)^2 * position = acceleration
    
% Jacobians of f and h
F_jac = @(x) [1, T, 0;
              -T*x(3)^2, 1, -2*T*x(3)*x(1);
              0, 0, 1];

H_jac = @(x) [-x(3)^2, 0, -2*x(3)*x(1);
              -x(3)^2, 0, -2*x(3)*x(1)];

% Covariance Matrices
% Q = diag(sigma_w.^2);                % Process noise covariance
Q = zeros(3);
R = diag(sigma_v.^2);                % Measurement noise covariance

% Storage for Results
x_est = zeros(3, num_steps);         % State estimates
x_est(:,1) = x_0;
P_est = zeros(3, 3, num_steps);

x = x_0; % current state estimate is initialized

for k = 1:num_steps
    % EKF Prediction Step
    x_pred = f(x);                   % Nonlinear state prediction
    F = F_jac(x);                    % Linearized state transition matrix

    P_pred = F * P * F' + Q;         % Predicted covariance
    
    % EKF Update Step
    H = H_jac(x_pred);               % Linearized measurement matrix
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman Gain
    y = measurements(:,k) - h(x_pred); % Innovation
    x = x_pred + K * y;              % State update
    P = (eye(3) - K * H) * P_pred;   % Covariance update
    
    % Store Results
    x_est(:,k) = x;
    P_est(:, :, k) = P;
    disp(P);
end

% Plot Results
figure;
subplot(4,1,1);
plot(times, x_est(1,:), 'b', 'LineWidth', 1.5); hold on;
title('Position (m)');
ylabel('Position');

subplot(4,1,2);
plot(times, x_est(2,:), 'b', 'LineWidth', 1.5); hold on;
title('Velocity (m/s)');
ylabel('Velocity');

subplot(4,1,3);
plot(times, x_est(3,:), 'b', 'LineWidth', 1.5); hold on;
title('Frequency (rad/s)');
ylabel('Frequency');
xlabel('Time (s)');

subplot(4,1,4);
plot(times, measurements(1,:), "b"); hold on;
plot(times, measurements(2,:), "r"); hold on;
title('Raw Accel');
ylabel('Raw Accel');


% uncertainty plotting
%%%%%%%%
%%%%%%%%
% Assume P_est is a 3x3xN array storing P at each time step
N = size(P_est, 3); % Number of time steps
sigma_x = zeros(1, N); % Position uncertainty
sigma_v = zeros(1, N); % Velocity uncertainty
sigma_w = zeros(1, N); % Frequency uncertainty

for k = 1:N
    P = P_est(:, :, k); % Extract P at time step k
    sigma_x(k) = sqrt(P(1, 1)); % Position uncertainty
    sigma_v(k) = sqrt(P(2, 2)); % Velocity uncertainty
    sigma_w(k) = sqrt(P(3, 3)); % Frequency uncertainty
end

% Plot uncertainties over time
time = 1:N; % Replace with actual time vector if available
figure;
plot(time, sigma_x, 'r', 'LineWidth', 1.5); hold on;
plot(time, sigma_v, 'g', 'LineWidth', 1.5);
plot(time, sigma_w, 'b', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Standard Deviation (Uncertainty)');
title('EKF State Uncertainties Over Time');
legend('Position Uncertainty', 'Velocity Uncertainty', 'Frequency Uncertainty');
grid on;

%%%%%%%%
%%%%%%%%




% End of EKF, Begin Weighted Average
%%%%%%%%
%%%%%%%%
%%%%%%%%
%%%%%%%%
%%%%%%%%
%%%%%%%%

W = ADXL_static_var/(MPU_static_var + ADXL_static_var); 
A_weighted = measurements(1,:) * (W) + measurements(2,:) * (1-W);

x_est_2 = zeros(2, num_steps);
for k = 1:(num_steps-1)
    x_est_2(:,k+1) = [
        x_est_2(1,k) + Ts*x_est_2(2,k),
        x_est_2(2,k) + Ts*A_weighted(k)
        ];
end

% Plot Results
figure;
subplot(3,1,1);
plot(times, x_est_2(1,:), 'b', 'LineWidth', 1.5); hold on;
title('Position (m)');
ylabel('Position');

subplot(3,1,2);
plot(times, x_est_2(2,:), 'b', 'LineWidth', 1.5); hold on;
title('Velocity (m/s)');
ylabel('Velocity');

subplot(3,1,3);
plot(times, measurements(1,:), "b"); hold on;
plot(times, measurements(2,:), "r"); hold on;
title('Raw Accel');
ylabel('Raw Accel');


% uncertainty plotting, but for weighted avg model
%%%%%%
%%%%%%

% Initialize uncertainties
sigma_pos = zeros(1, num_steps);
sigma_vel = zeros(1, num_steps);

% Static variances
sigma_ADXL = sqrt(ADXL_static_var); % Standard deviation of ADXL
sigma_MPU = sqrt(MPU_static_var);   % Standard deviation of MPU

% Weighted average variance
W = ADXL_static_var / (MPU_static_var + ADXL_static_var);
sigma_weighted = sqrt(W^2 * ADXL_static_var + (1-W)^2 * MPU_static_var);

% Initial uncertainties (assuming starting uncertainty is zero)
sigma_pos(1) = 0;
sigma_vel(1) = 0;

% Propagate uncertainties through the system
for k = 1:(num_steps-1)
    % Update position and velocity uncertainties
    sigma_pos(k+1) = sqrt(sigma_pos(k)^2 + Ts^2 * sigma_vel(k)^2);
    sigma_vel(k+1) = sqrt(sigma_vel(k)^2 + Ts^2 * sigma_weighted^2);
end

% Plot Results
figure;
subplot(3,1,1);
plot(times, x_est_2(1,:), 'b', 'LineWidth', 1.5); hold on;
title('Position (m)');
ylabel('Position');
grid on;

subplot(3,1,2);
plot(times, x_est_2(2,:), 'b', 'LineWidth', 1.5); hold on;
% title('Velocity (m/s)');
ylabel('Velocity');
grid on;

subplot(3,1,3);
plot(times, sigma_pos, 'r', 'LineWidth', 1.5); hold on;
plot(times, sigma_vel, 'g', 'LineWidth', 1.5);
title('Uncertainty in Position and Velocity');
ylabel('Standard Deviation');
legend('Position Uncertainty', 'Velocity Uncertainty');
grid on;
