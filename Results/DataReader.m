close all
clear all

% Taking last file from joint_data folder
fileList = dir("/home/vandalsnike/catkin_ws7/Results/JointData");
fileList = fileList(~[fileList.isdir]);
[~, idx] = max([fileList.datenum]);

% Reading data
M = readmatrix('trajectory.csv');
M2 = readmatrix('trajectory_done.csv');
bag=rosbag(fileList(idx).name);

% Component over time
figure(1)
sgtitle("Components Over Time")
subplot(4,3,[1 2 3])
plot(M(:,1),M(:,2),'r','LineWidth',1.4)
hold on
plot(M2(:,1),M2(:,2),'b-.','LineWidth',1.4)
legend('Required','Executed')
xlabel("Time (s)")
ylabel("X (m)")
subplot(4,3,[4 5 6])
plot(M(:,1),M(:,4),'r','LineWidth',1.4)
hold on
plot(M2(:,1),M2(:,4),'b-.','LineWidth',1.4)
legend('Required','Executed')
xlabel("Time (s)")
ylabel("Z (m)")
subplot(4,3,[7 8 9])
plot(M(:,1),M(:,3),'r','LineWidth',1.4)
hold on
plot(M2(:,1),M2(:,3),'b-.','LineWidth',1.4)
legend('Required','Executed')
xlabel("Time (s)")
ylabel("Y (m)")

quat_req = [M(:,8), M(:,5:7)];  
quat_exe = [M2(:,8), M2(:,5:7)];  

euler_req = zeros(size(M(:, 5:8), 1), 3);
euler_exe = zeros(size(M2(:, 5:8), 1), 3);

for i = 1:size(M(:, 5:8), 1)
    q = quat_req(i, :);
    euler_req(i, :) = rad2deg(quat2eul(q,'XYZ'));
end

for i = 1:size(M2(:, 5:8), 1)
    q2 = quat_exe(i, :);
    euler_exe(i, :) = rad2deg(quat2eul(q2,'XYZ'));
end

subplot(4,3,10);
plot(M(:, 1),  euler_req(:, 1), 'b', 'DisplayName', 'Required','LineWidth',1.4); 
hold on;
plot(M2(:, 1), abs(euler_exe(:, 1)), 'r--', 'DisplayName', 'Executed','LineWidth',1.2);
title('Roll over time');
xlabel('Time (s)'); 
ylabel('Roll (degree)');
legend('show', 'Location', 'southeast');
grid on;

subplot(4,3,11);
plot(M(:, 1), euler_req(:, 2), 'b', 'DisplayName', 'Required','LineWidth',1.4); 
hold on;
plot(M2(:, 1),euler_exe(:, 2), 'r--', 'DisplayName', 'Executed','LineWidth',1.2);
title('Pitch over time');
xlabel('Time (s)'); 
ylabel('Pitch (degree)');
legend('show', 'Location', 'southeast');
grid on;

subplot(4,3,12);
plot(M(:, 1), euler_req(:, 3), 'b', 'DisplayName', 'Required','LineWidth',1.4); 
hold on;
plot(M2(:, 1), euler_exe(:, 3), 'r--', 'DisplayName', 'Executed','LineWidth',1.2);
title('Yaw over time');
xlabel('Time (s)'); 
ylabel('Yaw (degree)');
legend('show', 'Location', 'southeast');
grid on;




% Trajectory on XZ plane
figure(2)
plot(M(:,2),M(:,4),'r','LineWidth',1.4)
axis equal
hold on
plot(M2(:,2),M2(:,4),'b-.','LineWidth',0.9)
title("Trajectory Data: Trefoil Knot in XZ Plane")
scatter(M2(1,2),M2(1,4),'m*')
scatter(M2(end,2),M2(end,4),'g*')
legend('Required','Executed','Start point','End point','Location','best')
axis equal
xlabel("X (m)")
ylabel("Z (m)")



% Reading message from .bag file
msgs = readMessages(bag);

joints_velocities=zeros(size(msgs,1),8);
joints_positions=zeros(size(msgs,1),8);

for i=1:length(msgs)
    time=bag.MessageList{i,1};
    positions=msgs{i,1}.Position(1:7);
    velocities=msgs{i,1}.Velocity(1:7);

    joints_positions(i,:)=[time positions'];
    joints_velocities(i,:)=[time velocities'];

end


% Joint data over time
figure(3)
subplot(2,1,1)
plot(joints_positions(:,1), joints_positions(:,2:end), 'LineWidth', 1.3)
title("Joint position")
legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7')
xlabel('Time (s)')
hold on
smoothed_velocities = movmean(joints_velocities(:,2:end), 200);
subplot(2,1,2)
plot(joints_velocities(:,1), smoothed_velocities, 'LineWidth', 1.3)  
title("Joint velocity (smoothed)")
legend('joint1','joint2','joint3','joint4','joint5','joint6','joint7')
xlabel('Time (s)')


% Trajectory on 3D space
figure
plot3(M(:,2),M(:,3),M(:,4),'r')
hold on
plot3(M2(:,2),M2(:,3),M2(:,4),'b--')

