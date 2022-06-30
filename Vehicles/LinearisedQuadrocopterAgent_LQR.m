% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef LinearisedQuadrocopterAgent_LQR < LinearisedQuadrocopter
    %LINEARISEDQUADROCOPTERAGENT Agent.    
    
    properties(GetAccess = private, SetAccess = immutable)
        controller % Discrete-time LTI controller
    end
   
    methods
        function obj = LinearisedQuadrocopterAgent_LQR(id,initialPos)
            %LINEARISEDQUADROCOPTERAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent to the given initial position.
            
            data = load('quadrotor_LQR');            
            % Initialize quadrotor model
            initialVel = zeros(size(initialPos));
            obj@LinearisedQuadrocopter(id, data.Ts, data.m, initialPos, initialVel);
            
            % Assemble controller
             obj.controller.Fb=data.Fb;
             obj.controller.Ff=data.Ff;
        end
        function step(obj,ref)            
            % Since the equilibirum state is not the origin, compute the
            % error in the state separately and provide the controller with
            % output and state error.            
          
            % Evaluate agent dynamics
            u     = -obj.controller.Fb*obj.state+obj.controller.Ff*[ref([1:2,4:5])];
            obj.move(u);
        end
    end
end