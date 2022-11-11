%% 
clear all, close all, clc

%% loads
load drone_model.mat

%% setup

A= drone_ss_d.A;
B= drone_ss_d.B;
C= drone_ss_d.C;
D= drone_ss_d.D;

Ref= [-1*ones(200,1) 1*ones(200,1) -1*ones(200,1) 1*ones(200,1)];
nk= length(Ref);
TRef= 1:nk;
nu= size(B,2);
nx= size(B,1);
ny= size(C,1);


% Cost functional matrices
P= 1%*eye(2);
Q= 1%*eye(2);
R= 1%*eye(2);
N= 5;

%LQT matrices
[F, G, Qb, Rb, H]= GetBatchXMatrices(A, B, C, N, P, Q, R);
Fb= F*H;
Gb= F*H;
%final cost matrices and MPC gains
Rt= Gb'*Qb*Gb + Rb;
St= Gb'*Qb;
Ky= Rt^-1*St;
K= Ky*Fb;

%% simulation

x0= [0 0]';
X(:,1)=x0;

for k= 1:nk-1

    ref= Ref(:,k:k+N);
    Uopt(:,:,k)= reshape( (-K*X(:,k) + Ky*ref), nu, N);
    U(:,k)= Uopt(:,1,k);
    X(:,k+1)= A*X(:,k) + B*U(:,k);
    Y(:,k+1)= C*X(:,k);
    
end


%% plots

figure(101)
plot(X(1,:),X(2,:))
xlabel('X1 - $P_{10}$')
ylabel('X2 - $V_1$')
title('Phase Plot')

figure(102)
plot(TRef, Ref, 'r')
hold on
plot(Tref, X(1,:),'b')
plot(TRef, X(2,:),'g')
hold off
xlabel('Samples')
title('State evolution')









