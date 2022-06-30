% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>

classdef QuadraticField
    % Underlying quadratic scalar field 
    % This object defines an underlying scalar field as a Quadratic field    
    % Could define a Baseclass called field to this class to generalize to
    % other fields
   properties
      name              % Name of the field
      dim               % spatial dimensions agents live in      
      center            % Locations of minima
      Q                 % Hessian
      scale             % scaling at each center
   end
   methods
        function obj = QuadraticField(fcenter,Q,fscale)
            % Constructor function for setting the properties
            obj.name       = 'Quadratic field';
            obj.dim        = size(Q,1);            
            obj.center     = fcenter;
            obj.Q          = Q; 
            obj.scale      = fscale;
        end
        function field_eval=get_field_value_at(obj,z)
            % This function outputs the value of the field at the 
            % location z
            e=(z-obj.center);
            field_eval = obj.scale*(-0.5*diag(e'*obj.Q*e)');
            field_eval=-1*field_eval;
        end
        function grad = get_true_gradient(obj,q_agent)            
            %   This function asks for a gradient at the q_agent location. 
            %   Can be thought of as a gradient measurement 
            e=(q_agent-obj.center);
            grad = obj.scale*obj.Q*e;
        end
        function hess = get_true_hessian(obj,q_agent) 
            %   This function asks for a hessian at the q_agent location. 
            %   Can be thought of as hessian measurement             
            hess = obj.Q;
        end        
        function [X,Y,Z]=data_for_contour(obj,lim)
            % This function creates mesh grid data for plotting contours based on the
            % underlying source field
            XX = lim(1,1):1:lim(1,2);
            YY = lim(2,1):1:lim(2,2);    
            [X,Y] = meshgrid(XX,YY); 
            Z=zeros(size(X));
            Ex=[XX-obj.center(1)];
            Ey=[YY-obj.center(2)];
            Z_temp=zeros(size(X));
            for m=1:size(YY,2)
                for n=1:size(XX,2)
                    e=[Ex(n);Ey(m)];
                    Z_temp(m,n)=obj.scale*(-0.5*e'*obj.Q*e);
                end
            end
                Z=Z+Z_temp;
%              end 
        end
   end
end