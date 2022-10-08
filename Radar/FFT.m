%% FFT
clc;
clear all;

Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector

% TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
% For sin wave adding http://spiff.rit.edu/classes/phys207/lectures/beats/add_beats.html
% signal = A*sin(2*pi*f*t)
s1 = 0.7 * sin(2 * pi * 77 * t);
s2 = 2 * sin(2 * pi * 43 * t);

S = s1 + s2;

% Corrupt the signal with noise 
X = S + 2*randn(size(t));

% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
plot(1000*t(1:50) ,X(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% TODO : Compute the Fourier transform of the signal. 
S_fft = fft(X);
% TODO : Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
P2 = abs(S_fft/L);
P1 = P2(1:L/2+1);

% Plotting
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')


%% 2d FFT
% Implement a second FFT along the second dimension to determine the 
% doppler frequency shift.

% First we need to generate a 2D signal
% Convert 1D signal X to 2D using reshape

% while reshaping a 1D signal to 2D we need to ensure that dimensions match
% length(X) = M*N

% let
M = length(X)/50;
N = length(X)/30;

X_2d = reshape(X, [M, N]);

figure(2);
tiledlayout(1,2)

nexttile
imagesc(X_2d)


% TODO: Compute the 2-D Fourier transform of the data. 
Y_2d = fft2(X_2d)

% Shift the zero-frequency component to the center of the output and plot the resulting 
% matrix, which is the same size as X_2d.
Y_2dshifted = fftshift(Y_2d)
Y_2dshifted = abs(Y_2dshifted)

nexttile
% TODO: plot here
imagesc(Y_2dshifted)