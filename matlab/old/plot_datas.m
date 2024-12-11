global earth_G;
earth_G = 9.8;

LOW_PASS_CUTOFF = 1.5;

% Get list of sensor data files
files = dir('../sensors_data*.csv'); 

% Find the most recent file and get its path
[mostRecentFile, mostRecentFilePath] = getMostRecentFile(files);

% Display the most recent file path
disp(['Most recent file: ', mostRecentFilePath]);

% Extract timestamp of the most recent file
mostRecentTimestamp = extractTimestamp(mostRecentFile);
disp(['Timestamp of most recent file: ', datestr(mostRecentTimestamp)]);

% Load data from the file
data = readtable(mostRecentFilePath);

% Extract and preprocess data variables
[times, accelZ1, accelZ2] = extractDataVariables(data);

% Calculate sampling frequency and time interval metrics
calculateTimeMetrics(times);

% Detrend and filter acceleration data
accelZ1_detrended = detrendData(accelZ1, times);
accelZ2_detrended = detrendData(accelZ2, times);
disp(['mean mpu over all samples', string(mean(accelZ1))]);
disp(['mean adxl over all samples', string(mean(accelZ2))]);

% Plot acceleration and velocity data
MAX_SAMPLE_T = 4.4;
plotData(times, accelZ1, accelZ2, MAX_SAMPLE_T, 'Acceleration Z', 'm/s^2');
plotData(times, cumtrapz(times, accelZ1), cumtrapz(times, accelZ2), MAX_SAMPLE_T, 'Velocity Z', 'm/s');

% Plot detrended data
plotData(times, accelZ1_detrended, accelZ2_detrended, MAX_SAMPLE_T, 'Detrended Acceleration Z', 'm/s^2');
plotData(times, cumtrapz(times, accelZ1_detrended), cumtrapz(times, accelZ2_detrended), MAX_SAMPLE_T, 'Detrended Velocity Z', 'm/s');

% % Apply a low-pass filter to the detrended data
% [accelZ1_filtered, accelZ2_filtered] = applyLowPassFilter(accelZ1_detrended, accelZ2_detrended, times, LOW_PASS_CUTOFF);
% plotData(times, accelZ1_filtered, accelZ2_filtered, MAX_SAMPLE_T, 'Low-Pass Filtered Acceleration Z', 'm/s^2');

% --- Helper functions ---

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

function calculateTimeMetrics(time)
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
