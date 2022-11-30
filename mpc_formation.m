%% 
clear all, close all, clc

set(0,'defaultTextInterpreter','latex');
set(0,'defaultLegendInterpreter','latex');
%% loads
load drone_model.mat
A_model= drone_ss_d.A;
B_model= drone_ss_d.B;
%C= drone_ss_d.C;
C_model= [1 0];
D_model= drone_ss_d.D;

%% Centralized

%player 1 model
A1= A_model;
B11= B_model; B12= [0 0]';
C1= C_model;
D1= D_model;

%player 2 model
A2= A_model;
B22= B_model; B21= [0 0]';
C2= C_model;
D2= D_model;

%joint model
A= blkdiag(A1,A2);
B= [B11,B12;B21,B22];
C= blkdiag(C1,C2);
D= 0;n

Ref= [-1*ones(200,1); 1*ones(200,1); -1*ones(200,1); 1*ones(200,1); ones(N,1)]';


%batch
Pi= 1;
Qi= 1;
Ri= 1;
N= 10;

%centralized
P= blkdiag(Pi,Pi);
Q= blkdiag(Qi,Qi);
R= blkdiag(Ri,Ri);
[F, G, Qb, Rb, H]= GetBatchXMatrices(A, B, C, N, P, Q, R);
Fb= H*F;
Gb= H*G;
Rt= Gb'*Qb*Gb + Rb;
St= Gb'*Qb;
Ky= Rt^-1*St;
K= Ky*Fb;

%decentralized
%player 1
[F1, G1, Qb1, Rb1, H1]= GetBatchXMatrices(A1, B11, C1, N, Pi, Qi, Ri);
Fb1= H1*F1;
Gb1= H1*G1;
Rt1= Gb1'*Qb1*Gb1 + Rb1;
St1= Gb1'*Qb1;
Ky1= Rt1^-1*St1;
K1= Ky1*Fb1;
%player 2
[F2, G2, Qb2, Rb2, H2]= GetBatchXMatrices(A2, B22, C2, N, Pi, Qi, Ri);
Fb2= H2*F2;
Gb2= H2*G2;
Rt2= Gb2'*Qb2*Gb2 + Rb2;
St2= Gb2'*Qb2;
Ky2= Rt2^-1*St2;
K2= Ky2*Fb2;













