% Generate data
if ~exist('veh','var')
%     % Vehicle 1: Quadrotor
%     veh=1;
%     c_fric_vec=[2,3,4,5];
    
    % Vehicle 3: Hippocampus
    veh=3;
    c_fric_vec=[3,6,9,12];
    
end
data2=load(['./data/traj_veh_',int2str(veh),'_c_fric_',int2str(c_fric_vec(1))],'t_sampled','sampled');
data3=load(['./data/traj_veh_',int2str(veh),'_c_fric_',int2str(c_fric_vec(2))],'t_sampled','sampled');    
data4=load(['./data/traj_veh_',int2str(veh),'_c_fric_',int2str(c_fric_vec(3))],'t_sampled','sampled');
data5=load(['./data/traj_veh_',int2str(veh),'_c_fric_',int2str(c_fric_vec(4))],'t_sampled','sampled');    

figure()
subplot(2,2,1)
plot_traj(data2.t_sampled,data2.sampled)
title(['kd=',int2str(c_fric_vec(1))])
subplot(2,2,2)
plot_traj(data3.t_sampled,data3.sampled)
title(['kd=',int2str(c_fric_vec(2))])
subplot(2,2,3)
plot_traj(data4.t_sampled,data4.sampled)
title(['kd=',int2str(c_fric_vec(3))])
subplot(2,2,4)
plot_traj(data5.t_sampled,data5.sampled)
title(['kd=',int2str(c_fric_vec(4))])

%% Use defined functions

function []=plot_traj(t_sampled,sampled)
    xx=squeeze(sampled.position(:,1,:));
    yy=squeeze(sampled.position(:,2,:)); 
    plot(t_sampled,xx(:,1),'r')
    hold on
    plot(t_sampled,xx(:,2),'g')
    plot(t_sampled,xx(:,3),'b')
    plot(t_sampled,yy(:,1),'r--')
    plot(t_sampled,yy(:,2),'g--')
    plot(t_sampled,yy(:,3),'b--')
    xlabel('time')
end
