%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the flocking example presented in
% Fig.9 in the above paper.
%---------------------------------------------------------------------------------------------------
% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>
%                   Christian Hespe <christian.hespe@tuhh.de>

% addpath(genpath('mas-simulation/lib'))
addpath(genpath('./.'))

% Clear workspace to increase repeatability
clc
clear

% Reset the Matlab profiler
profile clear

c_fric_vec=[3,4,5,6];
for n_sim=1:length(c_fric_vec)
    % Seed the pseudo random number generator. This is required if we want
    % reproducible simulation, e. g. for profiling the code.
    rng(56);


    % Select Vehicle Model
    % 1-> Linearized Quadrocopter
    % 2-> Dynamic Unicycle
    % 3-> HippoCampus underwater vehicle
    veh=1;

    % Flag to enable exporting a video from the simulation results
    saveVideo = false;

    %% Network parameters
    agentCount = 3;   % Number of agents in the network
    dimension  = 2;    % Dimension of the space the agents move in
    dT         = 0.01; % Size of the simulation time steps [s]
    steps      = 20000;  % number of steps
    Tf         = dT*steps; %Final Time
    %% Initialize the network

    % Type of communication, 1->ideal, 2->Bernoulli
    netType    = 1;

    switch netType
        case 1
            range   = inf;      % Range of the radio communication
            Network = IdealNetwork(agentCount, dT, dimension, range);
        case 2
            range     = 10;    % Range of the radio communication
            pTransmit = 0.95; % Probability of successful transmission
            Network   = BernoulliNetwork(agentCount, dT, dimension, range, pTransmit);
    end
    %% Initialize the External field

    % Type of external field, 1->inverted Gaussians, 2->Quadratic
    fieldType=2;

    switch fieldType
        case 1 % Inverted Gaussian field
            % Field Parameters
            no_centers=5;
            fcenter=50;
            frange=50;
            fvar=10e1;
            fscale=50;
            conc_Field=InvGaussiansField(dimension,no_centers,fcenter,frange,fvar,fscale);
        case 2
            fcenter=[60;30];
            Q=eye(2);
            fscale=1;
            conc_Field=QuadraticField(fcenter,Q,fscale);
    end
    %% Initialize the agent sensor

    % Type of agent sensor, 1->single measurement, 2-> multiple measurements
    sensorType=1;

    switch sensorType
        case 1  % single exact measurement at the agent pos
            N               =1;    % Number of measurements around the agent pos
            sensor_range    =2;     % sensor range around the agent pos
            noise_bound     =0;    % apriori known bound on measurement noise
            C_reg           =0;  % L1 regularization tuning.(0-> no regularization)
            field_sensor    =AgentFieldSensor(sensor_range,N,noise_bound,conc_Field,C_reg);
        case 2 % multiple noisy measurements at and near the agent pos
            N               =4;    % Number of measurements around the agent pos
            sensor_range    =1;     % sensor range around the agent pos
            noise_bound     =0;    % apriori known peak bound (linf) on measurement noise
            C_reg           =0;  % L1 regularization tuning.(0-> no regularization)
            field_sensor    =AgentFieldSensor(sensor_range,N,noise_bound,conc_Field,C_reg);
    end
    %% Initialize the interaction field

    % Type of interaction field, 1->Olfati Saber's field, 2->to be implemented (quadratic)
    interac_fieldType=2;

    switch interac_fieldType
        case 1 % Olfati Saber interaction field
            interac_field=OS_InteractionField();
        case 2
            interac_field=Quadratic_InteractionField();
    end


    %% Initialize Architecture
    % Initialize the architecture
    % 1-> coupled(transmit vehicle pos and vel) or 
    % 2-> decoupled(transmit virtual pos and vel) architecture
    arch=1; 
    % Set the friction between virtual particle and environment via c_fric and
    % mass of the virtual particle mass. 
    switch(arch)
        % The coupled architecture (case 1), the outer loop needs to be considerably slower. 
        % So set a high value for mass.
        case 1
            switch(veh)
                case 1 % Linearized quadrocopter
                    c_fric = c_fric_vec(n_sim); mass = 1;
                case 2 % Dynamic unicycle
                    c_fric = c_fric_vec(n_sim); mass = 10;
                case 3 % Hippocanmpus underwater robot
                    c_fric = c_fric_vec(n_sim); mass = 20;
            end
        case 2
            switch(veh)
                case 1 % Linearized quadrocopter
                    c_fric = 1; mass = 1;
                case 2 % Dynamic unicycle
                    c_fric = 0.2; mass = 0.7;
                case 3 % Hippocanmpus underwater robot
                    c_fric = 2; mass = 2;
            end
    end

    %% Initialize the agents
    % To avoid Matlab initializing the array of agents without including the
    % required constructor arguments, we first construct a cell array of agents
    % and later convert this to a standard Matlab array.
    Agents = cell(agentCount, 1);
    for i = 1:length(Agents)
        % Randomly place the agents in the square [0,100]^2
        pos = 50 + 30 * (rand(dimension, 1) - 0.5);
        % Randomly assign the agent velocites in [-5,5]^2
        vel = 0 * (rand(dimension, 1) - 0.5);    
        id=Network.getId();    
        % Initialize Vehicle model and architecture type 
        switch(veh)
        case 1
            Vehicle = LinearisedQuadrocopterAgent_LQR(id, [pos;0]); % concatinated pos with 0 for the z position
        case 2
            Vehicle = DynamicUnicycleAgent(id, [pos]);
        case 3
            Vehicle = HippoCampusAgent(id, [pos;0]); % concatinated pos with 0 for the z position
        end        
        % Initiallize an agent with the generated initial conditions  
        Agents{i} = FlockingVehicleAgent(id, dT, pos, vel, conc_Field, interac_field,field_sensor,Vehicle,arch,mass,c_fric);
    end
    Agents = [Agents{:}];

    %% Perform simulation

    % Save start time of the simulation. We want to periodically print the
    % progress of the simulation.
    lastprint = posixtime(datetime('now'));

    % Preallocate storage for simulation results
    leech = DataLeech(Agents, steps, 'position', 'velocity','position_vir', 'velocity_vir','Vehicle_state');

    % Initialize remaining values for the simulation
    t = 0;

    % Data structure for Energy analysis
    Energy=struct;
    Energy.KE=zeros(1,steps);
    Energy.Vfield=zeros(1,steps);
    Energy.V=zeros(1,steps);



    tic
    % profile on

    for k = 1:steps
        t = (k-1) * dT;    
        % Perform a single time step for each agent. This includes evaluating
        % the dynamic equations and the flocking protocol as well as sending
        % its own position and velocity to the other agents as a message.
        for agent = Agents        
            agent.step()
            Network.updateAgent(agent)
        end

        % Process sent messages in the network
        recvMessages = Network.process();

        % Save current position of all agents
        leech.save(t)
        curr_positions=[Agents.position]; % used for dist calculation later

        for agent = Agents
            % Distribute transmitted messages among the agents
            agent.receive(t, recvMessages{agent.id});

            % Calculate kinetic and source field energies
            Energy.KE(1,k)=Energy.KE(1,k)+0.5*norm(agent.velocity,2)^2;
            Energy.Vfield(1,k)=Energy.Vfield(1,k)+conc_Field.get_field_value_at(agent.position);        

            % Calculate the distance from the agent to all other agents
            dist = vecnorm(agent.position - squeeze(curr_positions)); 
            Energy.V(1,k)=Energy.V(1,k)+0.5*sum(interac_field.look_up_psi_a(dist'));        
        end

        % Print progress every 2 seconds
        if posixtime(datetime('now')) - lastprint >= 1
            lastprint = posixtime(datetime('now'));
            fprintf('Simulation %3.5g%% finished\n', 100*t/Tf)
        end    
    end
    % profile viewer
    toc

    %% Analyze energy in the system
    % post_process(dT,steps,Energy)

    %% Resample data for plotting
    % Due to the multi-rate support, the sampling will not always be uniform.
    % Therefore, we need to resample the data. The resampling is oriented on
    % the set parameters of the video, that may be produced

    TVideo = 15; % Desired duration of the video [s]
    FPS    = 30; % Framerate of the video [Hz]

    % Calculate the required sampling time to meet the expectations
    dTAnimate = Tf / (TVideo * FPS);

    % Resample the data. The function uses a ZOH resampling approach 
    [t_sampled, sampled] = leech.resample(dTAnimate);
    
    % Compute boundary
    x_pos  = squeeze(leech.data.position(:,1,:));
    y_pos  = squeeze(leech.data.position(:,2,:));

    x_pos_vir  = squeeze(leech.data.position_vir(:,1,:));
    y_pos_vir  = squeeze(leech.data.position_vir(:,2,:));
    
    
    % Compute contour lines
    bounds = @(x) [min(min(x)), max(max(x))];
    lim     = [ bounds(x_pos); bounds(y_pos) ];
    [X,Y,Z] = conc_Field.data_for_contour(lim);
    
     save(['./data/traj_veh_',int2str(veh),'_c_fric_',int2str(c_fric_vec(n_sim))],'t_sampled','sampled','x_pos','y_pos','x_pos_vir','y_pos_vir','X','Y','Z','lim','veh')
end
%%
% plot_trajectories
%% Animate simulation results
figure()




% Open video file with mp4 encoding
if saveVideo
    video = VideoWriter('source_seeking', 'MPEG-4');
    video.FrameRate = FPS;
    open(video)
else
    video = [];
end


for k = 1:length(t_sampled)    
    contour(X,Y,Z)
    hold on
    pos = squeeze(sampled.position(k,:,:));
    pos_vir = squeeze(sampled.position_vir(k,:,:));
	if veh==2 || veh==3
        switch(veh)
            case 2
                phi = squeeze(sampled.Vehicle_state(k,4,:))';  
            case 3
                phi = squeeze(sampled.Vehicle_state(k,6,:))';  
        end		  
	cycledir = [cos(phi); sin(phi)];
    
    switch(veh)
        case 2% Unicycle controller design uses handle with d=0.2
            cg=pos - 0.2 * cycledir;
            front = cg + 0.25 * cycledir;
            back  = cg - 0.25 * cycledir;
        case 3% Hippocampus controller design doesn't use a handle d=0
            % Note that the controller is designed to move them backwards.
            % So, the velocities will most of the times be negative
            front = pos +  0.2 * cycledir;
            back  = pos -  0.1 * cycledir;
    end
			
	line([front(1,:); back(1,:)], [front(2,:); back(2,:)], 'Color', 'k', 'LineWidth', 2)
    end
    scatter(pos(1,:), pos(2,:))
    scatter(pos_vir(1,:), pos_vir(2,:),'*')
    hold off

    xlim(bounds(x_pos));
    ylim(bounds(y_pos));     
    drawnow limitrate

    if ~isempty(video)
        % Save figure as video frame
        frame = getframe(gcf);
        writeVideo(video, frame);
    end     
end

if ~isempty(video)
    close(video)
    video = [];
end
