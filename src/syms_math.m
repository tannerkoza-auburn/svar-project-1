%%
clear
clc
close all

%% Problem 1
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

zeqstates = subs(zstates)