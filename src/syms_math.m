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
[N,D] = ss2tf(A,B,C,D)

%%Part 3

