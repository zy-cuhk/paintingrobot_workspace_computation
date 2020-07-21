data=load('data1.mat');
workspace_points=data.workspace_points;
x=workspace_points(:,1);
y=workspace_points(:,2);
z=workspace_points(:,3);
figure
scatter3(x,y,z,'.')
view(-30,10)