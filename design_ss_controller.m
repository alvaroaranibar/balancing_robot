clc
close all
clear all

g = 9.8067;             % m/s^2 - Acceleration due to gravity.

% Model parameters based on V2 Prototype.

% --- Parameters of Geared Motor  ---
% From datasheet of 35:1 motor
Tstall = 0.028;       % N-m - stall torque (2.8 kg/cm).
RPMnoload = 170;        % No-load RPM at Vin=12V.
wnl = (RPMnoload * 2 * pi) / 60;     % No load gearbox output shaft rotation velocity in rad/sec.
Inl = 0.06;             % No load current into motor, from measurement.  //Datasheet parecido a medida = 0.06795 A @12.14 V
Vin = 12;                % Volt. Input voltage to armature during test.
Istall = 1.3;           % Amp, stall armature current.
Ng = 35;               % Gear ratio.
Dg = 0.00001;           % kgm/rad/sec - Viscous loss of gear.
La = 0;                 % Assume armature inductance is 0.

Ra = Vin / Istall;                        % Armature resistance.
Kt = (Tstall / (Vin * Ng)) * Ra;          % Armature torque constant.
Kb = (Vin - Ra * Inl) / (wnl * Ng);       % Armature back EMF constant.

% --- Parameters of Motor Driver  ---
Kmd = 1;             % Voltage gain of motor driver.

% Estimando masa de estator y rotor
mmcr = 0.11;   %  kg - Masa del motor completo (110g) 
mcr = 0.03;  % kg - Masa de la caja reductora estimada (30g)
me = 0.65*(mmcr-mcr);  % kg - Masa del estador estimada 65%
mr = 0.35*(mmcr-mcr);  % kg - Masa del rotor estimada 35%
mll = 0.035;         % kg - masa de la llanta pequeña (35g)

% --- Parameters of Wheel Assembly ---
R = 0.034;              % m - Radius of wheel, 70mm 3D printed wheels.
mw = mr+mll+mll;            % kg - Mass of wheel assembly: llanta y rotor, se suma otra masa de rueda por tornillo no medido
Jw = 0.5 * mw * R^2;    % kgm^2 - Moment of inertia of wheel referenced to the center (axle).
                        % This is approximate only, assuming uniform mass distribution.

% --- Parameters of Robot Body ---
mb = 0.562 + mcr + me;            % kg - Mass of body: cuerpo, estator y caja reductora
lb = 0.056155;             % m - Distance of center of mass to wheel axle.
Jb = 0.1 * mb * lb^2;   % kgm^2 - Moment of inertia of body referenced to the center of mass.
                        % This is approximate only.

% Approximate constants for motor, gear box, and wheel.
% See the derivation, V3, 31 March 2017.
Cm1 = Ng * Kt / Ra;
Cm2 = Cm1 * Kb;
M = mb + 2 * (mw + Jw / R^2) - (mb^2 * lb^2) / (mb * lb^2 + Jb);
J = mb * lb^2 + Jb - (mb^2 * lb^2) / (mb + 2 * (mw + Jw / R^2));
C1 = (1 / M) * ((mb^2 * lb^2 * g) / (Jb + mb * lb^2));
C2 = (2 / M) * ((1 / R) + (mb * lb) / (Jb + mb * lb^2));
C3 = (mb * lb * g) / J;
C4 = (2 / (J * R)) * (R + (mb * lb) / (mb + 2 * (mw + Jw / R^2)));

% The elements of the state-transition (A) matrix.

a22 = -(C2 * Cm2) / (R * Ng);
a24 = C2 * Cm2;
a42 = (C4 * Cm2) / (R * Ng);
a44 = -C4 * Cm2;

% From the A matrix.
A = [0,   1,   0,     0;
     0,  a22,  -C1,   a24;
     0,  0,    0,     1;
     0,  a42,  C3,   a44]

% The elements of B vector.
b21 = C2 * Cm1 * Kmd;
b41 = -C4 * Cm1 * Kmd;
% Form the B vector.
B = [0; b21; 0; b41]

%% %% POLOS DESEADOS
C = [1 0 0 0];

%Dinámica deseada del servosistema digital
Tes=2.5; 
Mp = 0.15;
sigma=4/Tes;

wd=-pi*sigma/log(Mp);

far = -log(Mp)/sqrt(pi^2+log(Mp)^2);
wn = sigma/far;
fi = acos(far);
Tr = exp(fi/tan(fi))/wn

T = Tr/10

%% Modelo discreto y con integrador
FI = eye(4)*T + A*T^2/2 + A^2*T^3/6 + A^3*T^4/24;
G = eye(4) + A*FI
H = FI*B

GN = [G    zeros(4,1)
      -C*G   1       ]
HN = [H
     -C*H]

%%Controlabilidad
CoN = [HN  GN*HN  GN^2*HN GN^3*HN GN^4*HN];
det_CoN = det(CoN)

%%
%Polos en Laplace
s1=-sigma+wd*i;
s2=-sigma-wd*i;
s3=-10*sigma;
s4=-10*sigma;
s5=-10*sigma;
%Polos en plano Z
z1 = exp(s1*T)
z2 = exp(s2*T)
z3 = exp(s3*T)
z4 = exp(s4*T)
z5 = exp(s5*T)

z1_mod = abs(z1)

KN = acker(GN,HN,[z1  z2  z3 z4 z5])
K1 = -KN(5)
K2 = KN(1:4)

