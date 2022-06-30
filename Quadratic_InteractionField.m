% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>

classdef Quadratic_InteractionField
    %Quadratic interaction field between agents 
    %Like springs connecting agents
   properties(Constant)
        epsilon         = 1; % Used to define the sigma_norm
        da              = 7;   % Equilibrium distance(or desired distance) to neighbours
        h               = 0.9; % Bump function goes to zero after h
   end
   methods
        function obj = Quadratic_InteractionField()
            % Constructor function for Initializing interaction field 
        end
        function [force,a]=get_interaction_force(obj,dist_euclidean)
            % This function takes in the Euclidean distance and outputs the
            % interaction force based on the field parameters and the
            % adjacency element governing the topology.
            
            % Compute sigma distance for a given euclidean distance
              [dist, grad] = sigma_norm(dist_euclidean,obj.epsilon);
            % Compute the interaction force between for a given distance
              force=(dist-obj.da)*grad;              
            % Compute the adjacency element for velocity alignement
              a = 1;   % All agents communicate with all other
        end
        function psi_eval=look_up_psi_a(obj,z)
            psi_eval=0.5*(vecnorm(z,2,1)-obj.da).^2;
        end        
   end
end