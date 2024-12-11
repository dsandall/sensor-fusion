%% Data Initialization
close all; clear all; clc;
% load("/MATLAB Drive/MobileSensorData/sensorlog_20240827_141539.mat");


% Get list of sensor data files
files = dir('../sensors_data*.csv'); 

% Find the most recent file and get its path
%[mostRecentFile, mostRecentFilePath] = getMostRecentFile(files);

% Load data from the file
% data = readtable(mostRecentFilePath);
data = readtable('C:\Users\zakth\OneDrive\Documents\MATLAB_files\Stochastic\sensor_data\Final_static_o.csv');


% Extract and preprocess data variables
[times, accelZ1, accelZ2] = extractDataVariables(data);

t = times;
len = length(t);
Ts = mean(diff(t)); %av time sample
A1 = accelZ1; %MPU6050 -- will be blue
A2 = accelZ2; %ADXL    -- red
figure;
plot(t, A1, "b-", 'DisplayName', 'MPU6050');
    hold on;
    plot(t, A2, "r-", 'DisplayName', 'ADXL345');

    

    %% interpolation (for ADXL testing)
for i=2:len
    if abs(A2(i)) > 4
        if abs(A2(i+1)) > 4
            A2(i) = A2(i-1);
        else
            A2(i) = (A2(i+1)+A2(i-1))/2;
        end
    end
end
X1=A1-mean(A1);
X2=A2-mean(A2);
plot(t, A2, "g--", 'DisplayName', 'ADXL345interpo');
ylabel('Acceleration [m/s^2]')
    xlabel('Time elapsed [s]')
    legend("show");

%% convert to position!!!

% mean_1 = mean(A1); %syntax
% mean_2 = mean(A2);
% 
% 
% X1 = zeros(len,1); X2 = X1; V1=X1;V2=X1;
% for i=2:len     %starts w/ ICs=0
%     V1(i) = V1(i-1)+(A1(i-1)+A1(i)-2*mean_1)/2*(t(i)-t(i-1));
%     V2(i) = V2(i-1)+(A2(i-1)+A2(i)-2*mean_2)/2*(t(i)-t(i-1));
% 
%     X1(i) = X1(i-1)+(V1(i-1)+V1(i))/2*(t(i)-t(i-1));
%     X2(i) = X2(i-1)+(V2(i-1)+V2(i))/2*(t(i)-t(i-1));
% end
% figure;
% plot(t, X1, "b-", 'DisplayName', 'MPU6050 pos');
%     hold on;
%     plot(t, X2, "r-", 'DisplayName', 'ADXL345 pos');
%     plot(t, V1, "b--", 'DisplayName', 'MPU6050 velo');
%     plot(t, V2, "r--", 'DisplayName', 'ADXL345 velo');
%     ylabel('position [m]')
%     xlabel('Time elapsed [s]')
%     legend("show");






%% Autocorrelations

NL=len-2;%NumLags

%numlags is # of data points for autocorr
ACFTb1 = autocorr(X1, NumLags=NL);
ACFTb2 = autocorr(X2, NumLags=NL);   %Is there another way to do autocorr?
lags=0:NL;
figure;
plot(lags*Ts*100,ACFTb1, 'b', 'DisplayName', 'MPU6050 Rx');
    xlabel('Autocorrelation Delay [ms]')
    ylabel('Autocorrelation')
    title('Autocorrelation of Accelerometer outputs')

hold on;
plot(lags*Ts*100,ACFTb2, 'r', 'DisplayName', 'ADXL345 Rx');
legend('show')


%manual Autocorr?? with V(t)??



%% PSD 

%[pxx,f] = periodogram(X1,[],[],1/Ts);
% Units of periodogram are (sampled units)^2/Hz, so (m/s/s)^2/Hz
    %xlabel('Analog frequency [Hz]')
    %ylabel('PSD estimate [dB_(m/s^2)^2/Hz]')
%plot(f,db(pxx,'power')); %this plots it but downsampled, will ignore

%periodogram
% Use all data (instead of pow2)
figure;
[pxxnfft1,fnfft1] = periodogram(X1,[],length(X1),1/Ts);
[pxxnfft2,fnfft2] = periodogram(X2,[],length(X2),1/Ts);
plot(2*pi*fnfft1,db(pxxnfft1,'power'), 'b-', 'DisplayName','MPU default');
    xlabel('Analog frequency [Hz]')
    ylabel('PSD estimate [dB_(m/s^2)^2/Hz]')
hold on;
% Use Hamming window
[pxxhamm1,fhamm1] = periodogram(X1,hamming(length(X1)),length(X1),1/Ts);
[pxxhamm2,fhamm2] = periodogram(X2,hamming(length(X2)),length(X2),1/Ts);

w1 = 2*pi*fhamm1;
w2 = 2*pi*fhamm2;
plot(fhamm1,db(pxxhamm1,'power'), 'Color', '[0.3, 0.4, 0.6]', 'LineStyle', '--', 'DisplayName', 'MPU Hamming')
legend('show');

figure;
    plot(fnfft2,db(pxxnfft2,'power'), 'r-', 'DisplayName','ADXL default');
        xlabel('Analog frequency [Hz]')
        ylabel('PSD estimate [dB_(m/s^2)^2/Hz]')
        hold on;
    plot(fhamm2,db(pxxhamm2,'power'), 'Color', '[0.6, 0.3, 0.4]', 'LineStyle', '--', 'DisplayName', 'ADXL Hamming')
        legend('show');
%% How does PERIODOGRAM relate to FFT? //need to convert to dB
Xf1 = fft(X1);
N1 = length(X1);
PSD1 = Xf1.*conj(Xf1);

Xf2 = fft(X2);    %I thought PSD was integral of Autocorrelation???
N2 = length(X2);
PSD2 = Xf2.*conj(Xf2);

figure;
    plot(fnfft1,db(pxxnfft1, "power"), "b-", "DisplayName","Periodogram PSD MPU"); hold on;
    plot(fnfft1,db(PSD1(1:length(fnfft1))/N1/(1/Ts), "power"), 'Color', '[0.3, 0.4, 0.6]', 'LineStyle', '--',"DisplayName", "FFT MPU");
    grid on;
    legend('show');
%xlim([0,0.5])

figure;
    plot(fnfft2,db(pxxnfft2, "power"), "r-", "DisplayName","Periodogram PSD ADXL"); hold on;
    plot(fnfft2,db(PSD2(1:length(fnfft2))/N2/(1/Ts), "power"), 'Color', '[0.6, 0.3, 0.4]', 'LineStyle', '--', "DisplayName", "FFT ADXL");
    grid on;
    legend('show');


%% Cross-Correlation??
cor = corrcoef([X1 X2]);


%% Statistics about data

%means:

%Ensemble mean? or is this just time statistics?
mean1 = mean(X1); %syntax
mean2 = mean(X2);
fprintf('mean (MPU) [m/s^2]: %.2f \n', mean1);
fprintf('mean (ADXL) [m/s^2]: %.2f \n', mean2);

db_accel_0hz_nfft1 = db(pxxnfft1(1), 'power');  % dB value at 0 Hz
db_accel_0hz_hamm1 = db(pxxhamm1(1), 'power');  % dB value at 0 Hz
fprintf('PSD(0) No filter for MPU (mean): %.2f\n', sqrt(pxxnfft1(1))/2);
fprintf('PSD(0) Hamming filter for MPU (mean): %.2f\n', sqrt(pxxhamm1(1))/2);

db_accel_0hz_nfft2 = db(pxxnfft2(1), 'power');  % dB value at 0 Hz
db_accel_0hz_hamm2 = db(pxxhamm2(1), 'power');  % dB value at 0 Hz
fprintf('PSD(0) No filter for ADXL (mean): %.2f\n', sqrt(pxxnfft2(1))/2);
fprintf('PSD(0) Hamming filter for ADXL (mean): %.2f\n', sqrt(pxxhamm2(1))/2);


%is Rx0 =MSV or MSV+mu^2

%vars:

R1 = xcorr(X1, 'none'); 
centerIndex = ceil(length(R1) / 2); % Index of the autocorrelation at t = 0
R_t01 = R1(centerIndex)/(centerIndex);
R2 = xcorr(X2, 'none'); 
centerIndex = ceil(length(R2) / 2); % Index of the autocorrelation at t = 0
R_t02 = R2(centerIndex)/(centerIndex);

MSV1 = mean(X1.^2);
MSV2 = mean(X2.^2);
fprintf('calc MSV of MPU (Var): %.2f \n', MSV1);
fprintf('calc MSV of ADXL (Var): %.2f \n', MSV2);

intS1 = sum(fnfft1(2)*pxxnfft1); %need to find total integral of PSD
intS2 = sum(fnfft2(2)*pxxnfft2);
fprintf('Integral of PSD MSV of MPU (Var): %.2f \n', intS1);
fprintf('Integral of PSD MSV of ADXL (Var): %.2f \n', intS2);
%Ergodicity? Would need to model X(t) equation???

intS1 = sum(fnfft1(2)*pxxnfft1); %need to find total integral of PSD
intS2 = sum(fnfft2(2)*pxxnfft2);
fprintf('Integral of PSD MSV of MPU (Var): %.2f \n', intS1);
fprintf('Integral of PSD MSV of ADXL (Var): %.2f \n', intS2);
col1 = [mean1;sqrt(pxxnfft1(1))/2;sqrt(pxxhamm1(1))/2;0;MSV1;R_t01;0;MSV1-mean1^2;intS1];
col2 = [mean2;sqrt(pxxnfft2(1))/2;sqrt(pxxhamm2(1))/2;0;MSV2;R_t02;0;MSV2-mean2^2;intS2];

%% find mf line of best fit
% figure;
% 
% plot(w1,db(pxxhamm1,'power'), 'b-', 'DisplayName','MPU Hamming');
% hold on;
% plot(w2,db(pxxhamm2,'power'), 'r-', 'DisplayName','ADXL Hamming');
% xlabel('Analog frequency w [rad/s]')
% ylabel('PSD estimate [dB_(m/s^2)^2/rad/s]')
% legend('show');
% 
% s=1i*w1;
% 
% pxxhamm1db = db(pxxhamm1,'power');
% 
% % Define the 2nd-order Gauss-Markov PSD model in s-domain
% gauss_markov_2nd = @(b, s) b(1) ./ ((1 + (s ./ b(2)).^2) .* (1 + (s ./ b(3)).^2));
% 
% % Initial guesses for [P0, omega_c1, omega_c2]
% initial_guess_2nd = [max(pxxhamm1db), 2 * pi * 10, 2 * pi * 20]; % Adjust as needed
% 
% % Fit the data
% fit_params_2nd = lsqcurvefit(gauss_markov_2nd, initial_guess_2nd, s, pxxhamm1db);
% 
% % Extract parameters
% P0_2nd_s = fit_params_2nd(1);
% omega_c1_2nd_s = fit_params_2nd(2);
% omega_c2_2nd_s = fit_params_2nd(3);
% 
% % Generate fitted PSD
% psd_fitted_2nd = gauss_markov_2nd(fit_params_2nd, s);
% 
% 
% figure;
% plot(s,pxxhamm1db, 'b-', 'DisplayName','MPU Hamming');
% hold on;
% plot(s,psd_fitted_2nd, 'r-', 'DisplayName','fit');


%% --- Helper functions ---

function [mostRecentFile, mostRecentFilePath] = getMostRecentFile(files)
    % Find timestamps of all files
    timestamps = NaT(length(files), 1);
    for i = 1:length(files)
        tokens = regexp(files(i).name, 'sensors_data_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2}).*', 'tokens');
        if ~isempty(tokens)
            dateStr = tokens{1}{1};
            timeStr = strrep(tokens{1}{2}, '-', ':');
            timestamps(i) = datetime([dateStr ' ' timeStr], 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
        end
    end

    % Find the most recent timestamp
    [~, mostRecentIdx] = max(timestamps);
    mostRecentFile = files(mostRecentIdx).name;
    mostRecentFilePath = fullfile(files(mostRecentIdx).folder, mostRecentFile);
end

function timestamp = extractTimestamp(file)
    % Extract the timestamp from a single filename
    tokens = regexp(file, 'sensors_data_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2}).*', 'tokens');
    if ~isempty(tokens)
        dateStr = tokens{1}{1};
        timeStr = strrep(tokens{1}{2}, '-', ':');
        timestamp = datetime([dateStr ' ' timeStr], 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
    else
        timestamp = NaT; % Return Not-a-Time if format does not match
    end
end

function [time, accelZ1, accelZ2] = extractDataVariables(data)
    % Extract relevant variables from the data table
    time = data.("Time");
    accelZ1 = data.("AccelZ1");
    accelZ2 = data.("AccelZ2");
end

function sampleFrequency = calculateTimeMetrics(time)
    % Calculate and display sampling frequency and time interval metrics
    timeDiffs = diff(time);
    sampleFrequency = 1 / mean(timeDiffs);
    sampleStdDev = std(timeDiffs);
    fprintf('Sample frequency: %.2f Hz\n', sampleFrequency);
    fprintf('Average time interval: %.6f seconds\n', mean(timeDiffs));
    fprintf('Time interval standard deviation: %.6f seconds\n', sampleStdDev);
end

function truncatedData = detrendData(accelData, time)
    global earth_G;
    firstIndex = find(time <= 0.2, 1, 'last');

    % Detrend data based on the median of the first 0.2 seconds
    medianAccel = median(accelData(1:firstIndex));
    fprintf('Median Accel Z during first 0.2 seconds: %.4f m/s^2\n', medianAccel);

    % remove median during calibration data period
    detrendedData = accelData - medianAccel; 

    % interpolate outliers
    filteredData = filloutliers(detrendedData, 'linear', 'movmedian', 55);

    % finally, truncate it
    truncatedData = max(-.5*earth_G, min(filteredData, .5*earth_G));

end

function plotData(time, data1, data2, maxSampleT, plotTitle, yLabel)
    % Generalized plotting function
    figure;
    plot(time, data1, 'b', 'DisplayName', 'MPU6050');
    hold on;
    plot(time, data2, 'r', 'DisplayName', 'ADXL345');
    xlabel('Time (s)');
    ylabel(yLabel);
    title([plotTitle, ' over Time for MPU6050 and ADXL345']);
    legend;
    xlim([0 maxSampleT]);
    ylim([-2.5 2.5]);
end

function [filteredData1, filteredData2] = applyLowPassFilter(data1, data2, time, cutoffFreq)
    % Apply a low-pass filter to data1 and data2
    % Input: cutoffFreq - low-pass filter cutoff frequency in Hz
    
    % Calculate sample frequency
    timeDiffs = diff(time);
    sampleFrequency = 1 / mean(timeDiffs);
    
    % Apply the low-pass filter to both datasets
    filteredData1 = lowpass(data1, cutoffFreq, sampleFrequency);
    filteredData2 = lowpass(data2, cutoffFreq, sampleFrequency);
end

