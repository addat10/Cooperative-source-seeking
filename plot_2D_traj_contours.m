clear
clc
%%

% Vehicle 2: Dynamic Unicycle
data=load(['./data/traj_veh_3_c_fric_12']);

figure()
plot_contour_2d_traj(data.t_sampled,data.X,data.Y,data.Z,data.lim,data.sampled,data.veh,data.x_pos,data.y_pos,data.x_pos_vir,data.y_pos_vir);

%% User defined functions
function []=plot_contour_2d_traj(t_sampled,X,Y,Z,lim,sampled,veh,x_pos,y_pos,x_pos_vir,y_pos_vir)

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

%         line([front(1,:); back(1,:)], [front(2,:); back(2,:)], 'Color', 'k', 'LineWidth', 2)
        end
        N=size(pos,2);
        for agent=1:N
%             color_gen=rand(3,1);
            plot(pos(1,agent),pos(2,agent),'.','color',[agent/N;0;agent/N], 'LineWidth', 0.1)      
        end
%         scatter(pos(1,:), pos(2,:),'color',[1,0,0;0,1,0;0,0,1])
%         scatter(pos_vir(1,:), pos_vir(2,:),'*')
%         hold off

        xlim(lim(1,:));
        ylim(lim(2,:));     
        drawnow limitrate
    end

end