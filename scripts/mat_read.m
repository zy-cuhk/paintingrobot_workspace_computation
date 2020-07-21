clc,clear, close all;

% data=load('data4.mat');
% workspace_points=data.workspace_points;
% 
% num=1;
% for i=1:1:size(workspace_points,1)
%     if workspace_points(i,1)>=0 || workspace_points(i,2)>=0
%         x(num,1)=workspace_points(i,1);
%         y(num,1)=workspace_points(i,2);
%         z(num,1)=workspace_points(i,3);
%         c(num,1)=sqrt(workspace_points(i,4));
%         num=num+1;
%     end
% end
% 
% cmax=max(c);
% for i=1:1:size(c,1)
%     c(i,1)=c(i,1)/cmax;
% end
% 
% cmax=max(c)
% figure
% scatter3(x,y,z,15,c,'.')
% 
% axis equal;
% xlabel("x axis")
% ylabel("y axis")
% zlabel("z axis")
% view(-30,10)
% h=colorbar;
% set(get(h,'label'),'string','the manipulability value')
% title("the workspace of painting robot")
% 
% hold on;
% robot = importrobot('ur5.urdf')
% show(robot)

% figure;
% hold on;
robot=importrobot('/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_workspace_computation/scripts/ur5_1.urdf');
show(robot);