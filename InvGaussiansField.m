% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>

classdef InvGaussiansField
    %field_inv_gaussians 
    % This object defines an underlying scalar field as a sum of inverted
    % Gaussian functions at different centers with heterogeneous variances
    % and scaling.
    % Could define a Baseclass called field to this class to generalize to
    % other fields
   properties
      name              % Name of the field
      dim               % spatial dimensions agents live in
      no_centers        % No of minima in the field
      centers           % Locations of minima
      Sigmas            % Co-variance matrices at each center
      scales            % scaling at each center
   end
   methods
        function obj = InvGaussiansField(d,no_centers,fcenter,frange,fvar,fscale)
            % Constructor function for setting the properties
            obj.name       = 'Sum_inv_gaussians';
            obj.dim        = d;
            obj.no_centers = no_centers; % No of Gaussians
            obj.centers    = fcenter+frange*(-0.5+rand(d,no_centers));
            obj.Sigmas     = kron(ones(1,no_centers),fvar*eye(d)); 
            obj.scales     = fscale*ones(1,no_centers);
        end
        function field_eval=get_field_value_at(obj,z)
            % This function outputs the value of the field at the 
            % location z
            field_eval=zeros(1,size(z,2));            
            for i=1:obj.no_centers
                source_i=obj.centers(:,i);
                Sigma_i=obj.Sigmas(:,(i-1)*obj.dim+1:i*obj.dim);
                scale_i=obj.scales(i);
                e=(z-source_i);
                field_eval = field_eval +scale_i*exp(-0.5*diag(e'*inv(Sigma_i)*e)');
            end
            field_eval=-1*field_eval;
        end
        function grad = get_true_gradient(obj,q_agent) 
            %GET.GRAD_ESTIMATE 
            %   This function asks for a gradient at the q_agent location. 
            %   Can be thought of as a gradient measurement            
            grad=zeros(obj.dim,1);            
            for i=1:obj.no_centers
                source_i=obj.centers(:,i);
                Sigma_i=obj.Sigmas(:,(i-1)*obj.dim+1:i*obj.dim);
                scale_i=obj.scales(i);
                e=(q_agent-source_i);
                grad = grad +scale_i*exp(-0.5*e'*inv(Sigma_i)*e)*inv(Sigma_i)*e;
            end
        end
        % Should also define a get_true_hessian function later
        function [X,Y,Z]=data_for_contour(obj,lim)
            % This function creates mesh grid data for plotting contours based on the
            % underlying source field
            no_minima=obj.no_centers;
            d=obj.dim;
            XX = lim(1,1):1:lim(1,2);
            YY = lim(2,1):1:lim(2,2);    
            [X,Y] = meshgrid(XX,YY); 
            Z=zeros(size(X));
            for i=1:no_minima
                Source_i=obj.centers(:,i);
                Sigma_i=obj.Sigmas(1:d,(i-1)*d+1:i*d);
                scale_i=obj.scales(i);
                Ex=[XX-Source_i(1)];
                Ey=[YY-Source_i(2)];
                Z_temp=zeros(size(X));
                for m=1:size(YY,2)
                    for n=1:size(XX,2)
                        e=[Ex(n);Ey(m)];
                        Z_temp(m,n)=scale_i*exp(-0.5*e'*inv(Sigma_i)*e);
                    end
                end
                Z=Z+Z_temp;
            end 
        end
   end
end