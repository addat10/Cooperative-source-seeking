% This script designs an LQR controller for the quadrotor model
clear
clc
%% Define the model of the quadrocopter
g = 9.81;   % Gravity constant
m = 0.640;  %  Mass of the Quadrocopter
Ts=0.01;
A = [ 0  1  0  0  0  0  0  0  0  0  0  0   ;
      0  0  0  0  0  0  0  0 -g  0  0  0   ;
      0  0  0  1  0  0  0  0  0  0  0  0   ;
      0  0  0  0  0  0  0  0  0  0  g  0   ;
      0  0  0  0  0  1  0  0  0  0  0  0   ;
      0  0  0  0  0  0  0  0  0  0  0  0   ;
      0  0  0  0  0  0  0  1  0  0  0  0   ;
      0  0  0  0  0  0  0  0  0  0  0  0   ;
      0  0  0  0  0  0  0  0  0  1  0  0   ;
      0  0  0  0  0  0  0  0  0  0  0  0   ;
      0  0  0  0  0  0  0  0  0  0  0  1   ;
      0  0  0  0  0  0  0  0  0  0  0  0 ] ;

% Note the transpose at the end of B
B = [ 0  0  0  0  0 1/m 0  0  0  0  0  0   ;
      0  0  0  0  0  0  0  1  0  0  0  0   ;
      0  0  0  0  0  0  0  0  0  1  0  0   ;
      0  0  0  0  0  0  0  0  0  0  0  1 ]';
C = eye(12);  
D = zeros(12,4);
P = ss(A,B,C,D);
ref_dim=6; % Position and velocity references
%% Design a tracking controller
% Hinf design
%G_quad_wrapped = hinf_design_2DOF_four_block(P,ref_dim);
Pd=c2d(P,Ts);
[Fb,Ff] = lqr_design_with_static_FF(P);
save('Vehicles\quadrotor_LQR','m','Ts','Fb','Ff');
%% helper functions
function [Fb,Ff] = lqr_design_with_static_FF(P)
% This function designs an LQR controller
    [A,B]=ssdata(P);
    %LQR design    
    [nx,nu]=size(B);    
    Q=eye(nx);
    R=0.01*eye(nu);
    N=zeros(nx,nu);    
    [Fb,~,~] = lqr(A,B,Q,R);
    A_cl=(A-B*Fb);
    %% Obtain feedforward gain
    % z_des and zdot_des are fixed to zero at the moment
    C_ff=zeros(4,12);
    C_ff(1,1)=1;C_ff(2,3)=1;C_ff(3,2)=1;C_ff(4,4)=1;
    H1=-C_ff*inv(A_cl)*B;
    Ff=pinv(H1)*eye(4);               
end