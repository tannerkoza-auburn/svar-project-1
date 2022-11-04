%%
clear
clc
close all

%% Part 1
% Problem 1
syms M m b  g I l F x dotx ddotx phi dotphi ddotphi

A = [(M+m) m*l*-cos(phi);
    m*l*-cos(phi) (I + m*l^2)];
B = [(F + m*l*dotphi^2*-sin(phi) - b*dotx);
    -m*g*l*-sin(phi)];

%% Problem 2 & 3

states = simplify(A\B);

%% Problem 4

syms z_1 z_2 z_3 z_4

x = z_1;
dotx = z_2;
phi = z_3;
dotphi = z_4;

zstates = subs(states);

z_1 = 0;
z_2 = 0;
z_3 = 0;
z_4 = 0;

zeqstates = subs(zstates);

M=0.5;
g=9.81;
m=0.2;
b=0.1;
l=0.3;
I=0.006;
a1=M*l^2*m+I*M+I*m;

A=[0 , 1 , 0 , 0; ... 
   0 , (-I*b-b*l^2*m)/a1 , (m^2*g*l^2)/a1 , 0; ...
   0 , 0 , 0 , 1; ...
   0 , (-b*l*m)/a1 , (M*g*l*m+m^2*g*l)/a1 , 0];
B=[0 ; (I+l^2*m)/a1 ; 0 ; (l*m)/a1];
C=[1 , 0 , 0 , 0; 0 , 0 , 0 , 0];
D=[0 ; 0];

%%part 2
%problem 1
[V,s]=eig(A);
%problem 4
CO=ctrb(A,B);
rnk_C=rank(CO);
rnk_s=rank([s-A,B]);
%problem 5
OB=obsv(A,C);
rnk_d=rank([s-A;C]);
rnk_O=rank(OB);
%problem 6
M=minreal(ss(A,B,C,D));
%problem 7
[N,D] = ss2tf(A,B,C,D);

%%Part 3
%problem 1
p=[roots([1 0.8 16]),roots([1 0.8 4])];

k=place(A,B,p-1);


%problem 2 
[V2,s2]=eig(A-B*k);

%problem 3
k_NL=place(A,B,p-1.5);

%% Part 4
%problem 1


%% Simulink 
% Parameters
linewidth = 2;
fontsize = 14;

% Simualtion Parameters
dt = 0.002; %integration step,
tf = 10; %final time, sec

% INTIAL CONDITONS
x_initial = [0.5 ; 0 ; deg2rad(30) ; 0];

%% Set Controller
% Set the Controller:
   % 1 = State-feedback linear model
   % 2 = .... so on
Controller = 1;

if Controller == 1
    K = k_NL;
else
    K = k_NL;
end


% Set Model
% Set the model to use: 1 = linear, 2 = nonlinear.
Model = 2;

% Set Desired State
% Set the desired state: 1 = Regulation, 2 = Setpoint Tracking.
Desired = 1;

if Desired == 1
    xd = [0 , 0 , 0 , 0];
else
    xd = [0 , 0 , 0 , 0];
end

% Run Simulation
[t,~,x,u1] = sim('P1_Sim_Simulink');
%   x is the state vector of [x; x_dot; phi; phi_dot],  4x1 vector

x1 = x(:,1);% Cart x position plot
phi = x(:,3);
theta = rad2deg((phi+pi()))-180;


%% Plots
plot(t,x1)
title('X vs Time')
xlabel('Time (s)')
ylabel('X (m)')
figure
plot(t,theta)
title('Theta vs Time')
xlabel('Time(s)')
ylabel('Theta(deg)')
figure
plot(t,u1)
title('Input vs Time')
ylabel('Force (N)')
xlabel('Time (s)')


