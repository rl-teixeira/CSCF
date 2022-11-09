%% 
clear all, close all, clc

%% loads
load drone_model.mat

%% setup

A= drone_ss_d.A;
B= drone_ss_d.B;
C= drone_ss_d.C;
D= drone_ss_d.D;

%
P= 1;
Q= 1;
R= 1;
N= 10;

%LQT matrices
[Fb, Gb, Qb, Rb, F, G, H]= GetBatchYMatrices(A, B, C, P, Q, R);

%final cost matrices and MPC gains
Rt= Gb'*Qb*Gb + Rb;
St= Gb'*Qb;
Ky= Rt^-1*St;
K= Ky*Fb;

%% simulation





