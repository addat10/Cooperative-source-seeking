%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script performs a stability analysis for source seeking dynamics
% with scalar fields characterized by a general static IQC (with the aim 
% of flocking analysis) and is used to verify stability of flocking
% dynamics for different values of kd

  
clear all
clc
tol=1e-4; % tolerance for the strict LMIs
%% Define the second order generic vehicle dynamics
dim=2;
kd=5; % friction/damping of the generic vehicle
kp=1;
% Define the flocking filter dynamics as kp/(s^2+kd.s)
A_genv=[zeros(dim),eye(dim);zeros(dim,dim),-kd*eye(dim)];
B_genv=[zeros(dim,dim);-kp*eye(dim)];

%% Local vehicle tracking closed loop with inputs q,p and output position
% Quadrotor example
addpath(genpath('..\vehicles\quadrotor'))
G_veh=get_quad_G_cl();

%% Characterize the IQC for the field f

% Incremental norm bound on f without strong convexity for flocking
% interaction dynamics (small gain IQC)
L=3;
M0=[L^2,0;0,-1];
M=kron(M0,eye(dim));

% % Strongly convex with Lipschtiz gradients for analyzing convex
% interactions
% m=1;
% L=2;
% M0=[-2*m*L,(m+L);m+L,-2];
% M=kron(M0,eye(dim));

%% Define the generalized plant for the IQC analysis
n=size(G_veh.A,1);
A_G=[G_veh.A,            G_veh.B;...
     zeros(2*dim,n),     A_genv];
B_G=[zeros(n,dim);B_genv];
C_G=[G_veh.C,  zeros(dim),   zeros(dim)];
C1=[zeros(dim,n),   eye(dim),   zeros(dim)];
C2=[-G_veh.C,           eye(dim),   zeros(dim)];

A0=A_G;
B0=[B_G, -B_G];
C10=[C1;zeros(dim,n+2*dim)];
C20=[C2;zeros(dim,n+2*dim)];
D10=[zeros(dim,2*dim);
     eye(dim),       zeros(dim)];
D20=[zeros(dim,2*dim);
     zeros(dim),     eye(dim)];
G0=ss(A0,B0,[C10;C20],[D10;D20]);


%% Run the Stability analysis
% Get projection matrices for converting a non-strict LMI into a strict LMI
% over a subspace. The coulumns of T1 form the kernel of LMI and thus
% T1'.LMI.T1=0 and the strict constraint T2'.LMI.T2<0 is imposed.
[T1,T2]=get_proj_mats(G_veh.A,G_veh.B(:,1:dim));
[status,vars]=verify_stab_flocking(dim,tol,T2,A0,B0,C10,C20,D10,D20,M,M);
%% Verification of feasibility post analysis
LMI_obj=get_LMI_flocking(A0,B0,C10,C20,D10,D20,dim,M,M);
LMIp=LMI_obj(vars.X,vars.lambda1,vars.lambda2,tol,vars.mu_var);
LMI1=T1'*LMIp*T1;
LMI2=T2'*LMIp*T2;
if status==1
    max(eig(LMI2))<-tol % verify that this is less than 0
    abs(max(eig(LMI1+LMI1')))<1e-12% verify that this is zero
end

%% Helper functions
function [status,variables]=verify_stab_flocking(dim,tol,T2,A0,B0,C10,C20,D10,D20,M1,M2)
 % This function runs the stab analysis LMI with cvx
 
    n=size(A0,1)-2*dim;
    status=false;
    cvx_begin sdp 
    cvx_precision best
    variable lambda1
    variable lambda2
    variable mu_var
    variable Q(n,n) symmetric
    
    
    X=[eye(n),inv(A0(1:n,1:n))*A0(1:n,n+1:n+dim),zeros(n,dim)]'*Q*[eye(n),inv(A0(1:n,1:n))*A0(1:n,n+1:n+dim),zeros(n,dim)]+blkdiag(zeros(n+dim),eye(dim));

    % Define the LMIs
    L1=[A0'*X + X*A0,         X*B0;
        (X*B0)',             zeros(size(B0,2))];
    L2=[C10';D10']*lambda1*M1*[C10,D10]+[C20';D20']*lambda2*M2*[C20,D20];
    L3=blkdiag(zeros(n),zeros(dim),eye(dim),zeros(dim),zeros(dim));
    L4=zeros(n+4*dim);
    L4(n+dim+1:n+2*dim,n+3*dim+1:n+4*dim)=eye(dim);    
    L4=(L4+L4');

    
    minimize 1; 
    subject to:
    X>=0;

    Q>=tol*eye(n);

    LL=T2'*(L1+L2+tol*L3+mu_var*L4)*T2;
    LL+LL'<=-2*tol*eye(size(T2,2));
    lambda1==0;
    mu_var>=0;
    lambda2>=0;
    cvx_end 
    
    variables=struct;
    variables.X=X;
    variables.Q=Q;
    variables.lambda1=lambda1;
    variables.lambda2=lambda2;
    variables.mu_var=mu_var;
    variables.lambda2=lambda2;
    
    if strcmp('Solved',cvx_status)
        status=true;
    end
end
function LMI_obj=get_LMI_flocking(A0,B0,C10,C20,D10,D20,dim,M1,M2)
    % Return the LMI object as an implicit function of LMI variables
    n=size(A0,1)-2*dim;
    % Define the LMIs
    L1=@(X) [A0'*X + X*A0,         X*B0;
             (X*B0)',             zeros(size(B0,2))];
    L2=@(lambda1,lambda2)[C10';D10']*lambda1*M1*[C10,D10]+[C20';D20']*lambda2*M2*[C20,D20];
    L3=blkdiag(zeros(n),zeros(dim),eye(dim),zeros(dim),zeros(dim));
    L4=zeros(n+4*dim);
    L4(n+dim+1:n+2*dim,n+3*dim+1:n+4*dim)=eye(dim);    
    L4=(L4+L4');
    LMI_obj=@(X,lambda1,lambda2,tol,mu)(L1(X)+L2(lambda1,lambda2)+tol*L3+mu*L4);
end
function [T1,T2]=get_proj_mats(A,B)
% Get projection matrices for converting a non-strict LMI into a strict LMI
% over a subspace. The coulumns of T1 form the kernel of LMI and thus
% T1'.LMI.T1=0 and the strict constraint T2'.LMI.T2<0 is imposed.
[n,dim]=size(B);
T11=[-inv(A)*B;eye(dim);zeros(dim);zeros(dim);zeros(dim)];
T12=[zeros(n,dim);zeros(dim);zeros(dim);eye(dim);zeros(dim)];
T1=[T11,T12];
T1=orth(T1); % orthonormal basis for the range of T1
T2=null(T1'); % orthonormal basis for the kernel of T1', i.e., R(T1) perp 
end