% Implement 1D CFAR using lagging cells on the given noise and target scenario.
clc;
clear all;
% Close and delete all currently open figures
close all;

% Generate Noisy Signal

% Specify the parameters of a signal with a sampling frequency of 1 kHz 
% and a signal duration of 1.5 seconds.

Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector

% Form a signal containing a 50 Hz sinusoid of amplitude 0.7 and a 120 Hz 
% sinusoid of amplitude 1.

S = 0.7*sin(2*pi*50*t) + sin(2*pi*120*t);

% Corrupt the signal with zero-mean white noise with a variance of 4
X = S + 2*randn(size(t));

X_cfar = abs(X);

% Data_points
Ns = 1500;  % let it be the same as the length of the signal

%Targets location. Assigning bin 100, 200, 300, and 700 as Targets
%  with the amplitudes of 16, 18, 27, 22.
X_cfar([100 ,200, 300, 700])=[16 18 27 22];

% plot the output
figure(1);
tiledlayout(2,1)
nexttile
plot(X_cfar)

% Apply CFAR to detect the targets by filtering the noise.

T = 12; G = 4;

% Offset : Adding room above noise threshold for desired SNR
offset=5; % Changed to 5 instead of 3

% More efficient
% Create convolution kernel that performs averaging of training
% blocks except for guard and centre
f = ones(2*(T + G) + 1, 1);
f(T + G : T + 2*G + 1) = 0;
f = f / sum(f);

% Perform convolution to perform averaging of each block to compute
% thresholds
threshold_cfar = conv(S, f, 'same') * offset;

% Copy signal over and any values in the signal less than the threshold,
% set to 0
signal_cfar = S;
signal_cfar(S < threshold_cfar) = 0;

% Plot - note that we don't need to circular shift anymore as the
% convolution with 'same' handles that for us
% Also got rid of the ridiculous use of cells - not needed
figure;
plot(signal_cfar,'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(S);
hold on, plot(threshold_cfar,'r--','LineWidth',2)
hold on, plot(signal_cfar,'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')