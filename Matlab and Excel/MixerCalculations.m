close all;

Water_Density=1; % in Kg/m^3
Diameter=0.076; %T200 Diameter in m
mass=10; % in kg
g=9.81; % in m/s^2

l=0.75;
w=1; % fake dimensions: replace with thruster rectangle size
h=0.5;


C_T=1.5; % Coefficient of Thrust: variable
C_Q=15;  % Coefficient of Torque: variable

syms x y yaw a b c d real;

%Thruster Positions Relative to Center of Mass

% Robot: CW=clockwise, CC=counter clockwise
%
%  (CC)T2 _____ T1(CW)  ]
%        |front|        ]
%  (CW)T3|  +  |T6(CC)  }length=l
%        |_____|        ]
%  (CC)T4       T5(CW)  ]
%
%       |_______|
%         width=w


P1=[l/2;w/2;0];
P2=[l/2;-w/2;0];
P3=[0;-w/2;0];
P4=[-l/2;w/2;0];
P5=[-l/2;-w/2;0];
P6=[0;w/2;0];

%Rotor Thrust Directions (unit vectors)
TDir1=[sqrt(2)/2;-sqrt(2)/2;0];
TDir2=[sqrt(2)/2;sqrt(2)/2;0];
TDir3=[0;0;1];
TDir4=[sqrt(2)/2;sqrt(2)/2;0];
TDir5=[sqrt(2)/2;-sqrt(2)/2;0];
TDir6=[0;0;1];

T1=@(n) n^2*sign(n)*TDir1*Water_Density*C_T*Diameter^4;
T2=@(n) n^2*sign(n)*TDir2*Water_Density*C_T*Diameter^4;
T3=@(n) n^2*sign(n)*TDir3*Water_Density*C_T*Diameter^4;
T4=@(n) -n^2*sign(n)*TDir4*Water_Density*C_T*Diameter^4;
T5=@(n) -n^2*sign(n)*TDir5*Water_Density*C_T*Diameter^4;
T6=@(n) -n^2*sign(n)*TDir6*Water_Density*C_T*Diameter^4;

% CC Propeller= -Q, CW Propeller=+Q
Q1=@(n) n^2*sign(n)*TDir1*Water_Density*C_Q*Diameter^5;
Q2=@(n) -n^2*sign(n)*TDir2*Water_Density*C_Q*Diameter^5;
Q3=@(n) n^2*sign(n)*TDir3*Water_Density*C_Q*Diameter^5;
Q4=@(n) -n^2*sign(n)*TDir4*Water_Density*C_Q*Diameter^5;
Q5=@(n) n^2*sign(n)*TDir5*Water_Density*C_Q*Diameter^5;
Q6=@(n) -n^2*sign(n)*TDir6*Water_Density*C_Q*Diameter^5;

M1=@(n) cross(P1,T1(n))+Q1(n);
M2=@(n) cross(P2,T2(n))+Q2(n);
M3=@(n) cross(P3,T3(n))+Q3(n);
M4=@(n) cross(P4,T4(n))+Q4(n);
M5=@(n) cross(P5,T5(n))+Q5(n);
M6=@(n) cross(P6,T6(n))+Q6(n);

syms n1 n2 n3 n4 n5 n6;

backMixer=[T1(1),T2(1),T3(1),T4(1),T5(1),T6(1); ...
        M1(1),M2(1),M3(1),M4(1),M5(1),M6(1)]


Mixer=nthroot((inv(backMixer).*sqrt(abs(inv(backMixer)))),3)










