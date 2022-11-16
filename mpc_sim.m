%% 
clear all, close all, clc

set(0,'defaultTextInterpreter','latex');
set(0,'defaultLegendInterpreter','latex');

%% loads
load drone_model.mat
A= drone_ss_d.A;
B= drone_ss_d.B;
%C= drone_ss_d.C;
C= [1 0];
D= drone_ss_d.D;

%% Unconstrained MPC

% Cost functional matrices
P= 1;
Q= 1;
R= 1;
N= 10;

Ref= [-1*ones(200,1); 1*ones(200,1); -1*ones(200,1); 1*ones(200,1); ones(N,1)]';
nk= length(Ref);
TRef= 1:nk;
nu= size(B,2);
nx= size(B,1);
ny= size(C,1);

%batch matrices
[F, G, Qb, Rb, H]= GetBatchXMatrices(A, B, C, N, P, Q, R);
Fb= H*F;
Gb= H*G;
Rt= Gb'*Qb*Gb + Rb;
St= Gb'*Qb;
%MPC gains
Ky= Rt^-1*St;
K= Ky*Fb;

%simulation
x0= [0 0]';
X(:,1)=x0;

for k= 1:nk-N-1

    ref= Ref(:,k:k+N)';
    Uopt(:,:,k)= reshape( (-K*X(:,k) + Ky*ref), nu, N);
    U(:,k)= Uopt(:,1,k);
    X(:,k+1)= A*X(:,k) + B*U(:,k);
    Y(:,k+1)= C*X(:,k);
    
end


% plots
figure(101)
plot(X(1,:),X(2,:))
xlabel('X1 - $P_{10}$')
ylabel('X2 - $V_1$')
title('Phase Plot')

figure(102)
plot(TRef*Ts, Ref, 'r')
hold on
plot(TRef(1:nk-N)*Ts, X(1,:),'b')
plot(TRef(1:nk-N)*Ts, X(2,:),'g')
hold off
xlabel('Time ($s$)')
title('State evolution')
legend('Ref','X1 - $P_{10}$','X2 - $V_1$','FontSize',10)


%% Constrained MPC

clear U Uopt X x0 Y

x0= [0 0]';

%batch matrices
[F, G, Qb, Rb, H]= GetBatchXMatrices(A, B, C, N, P, Q, R);
Fb= H*F;
Gb= H*G;
Rt= Gb'*Qb*Gb + Rb;
St= Gb'*Qb;

%constrain matrices
u_max= 30*(pi/180);
u_min= -u_max;
y_min= 0.2;
y_max= 5;
Mu= [-1;1];
My= [-Gb; Gb];
M= [Mu; My];
wu= [-u_min; u_max];
wy= [-y_min + Fb*x0; y_max - Fb*x0];
w= [wu; wy];

%simulation






