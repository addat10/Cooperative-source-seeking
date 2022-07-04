clear
clc
%%

% Vehicle 2: Dynamic Unicycle
data=load(['./data/traj_veh_3_c_fric_1']);

figure()
rng(100)
plot_contour_2d_traj(data.t_sampled,data.X,data.Y,data.Z,data.lim,data.sampled,data.veh,data.x_pos,data.y_pos,data.x_pos_vir,data.y_pos_vir);

%% User defined functions
function []=plot_contour_2d_traj(t_sampled,X,Y,Z,lim,sampled,veh,x_pos,y_pos,x_pos_vir,y_pos_vir)
    N=size(sampled.position,3);
    color_gen=rand(3,N);
    for k = 1:length(t_sampled)-1    
        contour(X,Y,Z)
        hold on
        pos = squeeze(sampled.position(k:k+1,:,:));
        pos_vir = squeeze(sampled.position_vir(k,:,:));
        
        for agent=1:N
%             color_gen=rand(3,1);
            if k==1
            plot(pos(1,1,agent),pos(1,2,agent),'o','color',color_gen(:,agent), 'LineWidth', 1,'MarkerSize',15)    
            end
            plot(pos(:,1,agent),pos(:,2,agent),'color',color_gen(:,agent), 'LineWidth', 1)
            if k==length(t_sampled)-1
            plot(pos(end,1,agent),pos(end,2,agent),'X','color',color_gen(:,agent), 'LineWidth', 1,'MarkerSize',15)    
            end
        end
        
        xlim(lim(1,:));
        ylim(lim(2,:));     
        drawnow limitrate
    end
    

end