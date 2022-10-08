clc;
clear all;

% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz

% TODO: Calculate the wavelength
lambda = c / frequency;

% TODO: Define the doppler shifts in Hz using the information from above 
% TODO: Calculate the velocity of the targets  fd = 2*vr/lambda
doppler_frequency_shift = [3e3, -4.5e3, 11e3, -3e3];
v = doppler_frequency_shift * lambda/2;

% TODO: Display results
disp(v)

%From the workspace above, 
%estimate the location of the targets.
%If all the targets were at 200 m range
%ahead of the ego (radar) vehicle
%and the velocity of the ego vehicle
%is 5 m/s. Then in next 5 seconds 
%which of the targets would be 
%closest to the ego vehicle. 
%Letes give targets a tag [A B C D]
init_d = 200;
ego_v = 5;
A = init_d;
B = init_d;
C = init_d;
D = init_d;
delta_t = 1;
for i=1:5
    A = A - v(1) * delta_t;
    B = B - v(2) * delta_t;
    C = C - v(3) * delta_t;
    D = D - v(4) * delta_t;
end
res = [A, B, C, D];
disp(res)