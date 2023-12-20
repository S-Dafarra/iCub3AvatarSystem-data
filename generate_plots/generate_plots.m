close all;
clear all;

addpath('./export_fig')


%% Rimini demo
filename = 'robot_logger_device_2022_06_16_15_38_03.mat';
destination = 'data/Rimini/';
url = "https://github.com/ami-iit/icub3-avatar-system-paper-data/raw/main/Rimini/robot_logger_device_2022_06_16_15_38_03.mat";
fullpath = fullfile(destination, filename);
if ~isfile(fullpath)
    if ~exist(destination, 'dir')
        mkdir(destination)
    end
    websave(fullpath, url, weboptions('Timeout', 60));
    if ~isfile(fullpath)
        error(["Failed to download ", filename, ". Download manually from ", url, " in the ", destination, " folder and try again"])
    end
end
load(fullpath)

pathToFigures = './generated/Rimini/';

if ~exist(pathToFigures, 'dir')
    mkdir(pathToFigures)
end

initial_point_walking = 1600;
final_point_walking = 2300;

domain = [0, 40];

com_desired = squeeze(robot_logger_device.walking.com.position.desired.data(:, initial_point_walking : final_point_walking));
com_measured = squeeze(robot_logger_device.walking.com.position.measured.data(:, initial_point_walking : final_point_walking));
com_time = squeeze(robot_logger_device.walking.com.position.desired.timestamps(initial_point_walking : final_point_walking)); 

initial_time = com_time(1);

com_time = com_time - initial_time;

figure('Renderer', 'painters', 'Position', [10 10 1170 650])

plot3DSignalDesiredMeasured(com_time, com_desired',  com_measured', ...
                                                       domain, [-0.1, 0.8], 'Position (m)');
fileName = [pathToFigures, 'comTacking.pdf'];
export_fig('-transparent', fileName)



zmp_desired = squeeze(robot_logger_device.walking.zmp.desired_planner.data(:, initial_point_walking : final_point_walking));
zmp_measured = squeeze(robot_logger_device.walking.zmp.measured.data(:, initial_point_walking : final_point_walking));


figure('Renderer', 'painters', 'Position', [10 10 1170 650])

plot2DSignalDesiredMeasured(com_time, zmp_desired',  zmp_measured', ...
                                                       domain, [-0.2, 0.8], 'Position (m)');
fileName = [pathToFigures, 'zmpTacking.pdf'];
export_fig('-transparent', fileName)

force_time = squeeze(robot_logger_device.cartesian_wrenches.right_arm_wrench_client.timestamps);

force_time = force_time - initial_time;

initial_point_forces= find(force_time > 0, 1);
final_point_forces = find(force_time >= com_time(end), 1);

left_arm_force = squeeze(robot_logger_device.cartesian_wrenches.left_arm_wrench_client.data(3, initial_point_forces : final_point_forces));
right_arm_force = squeeze(robot_logger_device.cartesian_wrenches.right_arm_wrench_client.data(3, initial_point_forces : final_point_forces));
force_time = force_time(initial_point_forces : final_point_forces);


figure('Renderer', 'painters', 'Position', [10 10 1170 650])

plot (force_time, left_arm_force');

hold on

plot (force_time, right_arm_force');

plot_aesthetic('', 'Time (s)',  'Force (N)', '', 'Normal Force on Left Arm', 'Normal Force on Right Arm')
fileName = [pathToFigures, 'armsForces.pdf'];
export_fig('-transparent', fileName)

%% Cleanup
clear all

%% Semifinals (Vase)
pathToFigures = './generated/Semifinals/';

if ~exist(pathToFigures, 'dir')
    mkdir(pathToFigures)
end

filename = 'robot_logger_device_2022_03_21_16_14_41.mat';
destination = 'data/Semifinals/';
url = "https://github.com/ami-iit/icub3-avatar-system-paper-data/raw/main/Semifinals/robot_logger_device_2022_03_21_16_14_41.mat";
fullpath = fullfile(destination, filename);
if ~isfile(fullpath)
    if ~exist(destination, 'dir')
        mkdir(destination)
    end
    websave(fullpath, url, weboptions('Timeout', 60));
    if ~isfile(fullpath)
        error(["Failed to download ", filename, ". Download manually from ", url, " in the ", destination, " folder and try again"])
    end
end
load(fullpath)

initial_point_forces = 44000;
final_point_forces = 47000;

left_arm_force = squeeze(robot_logger_device.cartesian_wrenches.left_arm_wrench_client.data(3, initial_point_forces : final_point_forces));
right_arm_force = squeeze(robot_logger_device.cartesian_wrenches.right_arm_wrench_client.data(3, initial_point_forces : final_point_forces));
force_time = squeeze(robot_logger_device.cartesian_wrenches.right_arm_wrench_client.timestamps(initial_point_forces : final_point_forces));
initial_time = force_time(1);
force_time = force_time - initial_time;
final_time = force_time(end);

figure('Renderer', 'painters', 'Position', [10 10 1170 650])

plot (force_time, left_arm_force');

hold on

plot (force_time, right_arm_force');
xlim([0, 30])
plot_aesthetic('', 'Time (s)',  'Force (N)', '', 'Normal Force on Left Arm', 'Normal Force on Right Arm')
fileName = [pathToFigures, 'vaseLifting.pdf'];
export_fig('-transparent', fileName)

%% Cleanup
clear all

%% Semifinals (Walking)
pathToFigures = './generated/Semifinals/';

if ~exist(pathToFigures, 'dir')
    mkdir(pathToFigures)
end

filename = 'robot_logger_device_2022_03_21_16_28_38.mat';
destination = 'data/Semifinals/';
url = "https://github.com/ami-iit/icub3-avatar-system-paper-data/raw/main/Semifinals/robot_logger_device_2022_03_21_16_28_38.mat";
fullpath = fullfile(destination, filename);
if ~isfile(fullpath)
    if ~exist(destination, 'dir')
        mkdir(destination)
    end
    websave(fullpath, url, weboptions('Timeout', 60));
    if ~isfile(fullpath)
        error(["Failed to download ", filename, ". Download manually from ", url, " in the ", destination, " folder and try again"])
    end
end
load(fullpath)

initial_point_walking = 6950;
final_point_walking = 10250;

domain = [0, 54];

com_desired = squeeze(robot_logger_device.walking.com.position.desired.data(:, initial_point_walking : final_point_walking));
com_measured = squeeze(robot_logger_device.walking.com.position.measured.data(:, initial_point_walking : final_point_walking));
com_time = squeeze(robot_logger_device.walking.com.position.desired.timestamps(initial_point_walking : final_point_walking)); 

initial_time = com_time(1);

com_time = com_time - initial_time;

figure('Renderer', 'painters', 'Position', [10 10 1170 650])

plot3DSignalDesiredMeasured(com_time, com_desired',  com_measured', ...
                                                       domain, [-2.5, 0.8], 'Position (m)');
fileName = [pathToFigures, 'comTacking.pdf'];
export_fig('-transparent', fileName)

%% Cleanup
clear all

%% Semifinals (Puzzle)
pathToFigures = './generated/Semifinals/';

if ~exist(pathToFigures, 'dir')
    mkdir(pathToFigures)
end

filename = 'robot_logger_device_2022_03_21_14_56_33.mat';
destination = 'data/Semifinals/';
url = "https://github.com/ami-iit/icub3-avatar-system-paper-data/raw/main/Semifinals/robot_logger_device_2022_03_21_14_56_33.mat";
fullpath = fullfile(destination, filename);
if ~isfile(fullpath)
    if ~exist(destination, 'dir')
        mkdir(destination)
    end
    websave(fullpath, url, weboptions('Timeout', 60));
    if ~isfile(fullpath)
        error(["Failed to download ", filename, ". Download manually from ", url, " in the ", destination, " folder and try again"])
    end
end
load(fullpath)

 [modelPath,~,~] = fileparts(mfilename('fullpath'));
 modelPath = [modelPath, '/'];
 fileName='iCubGenova09.urdf';

 consideredJoints={
                         'neck_pitch'; 'neck_roll'; 'neck_yaw';
                         'torso_pitch'; 'torso_roll'; 'torso_yaw';
                         'l_shoulder_pitch'; 'l_shoulder_roll'; 'l_shoulder_yaw'; 'l_elbow'; 'l_wrist_prosup'; 'l_wrist_pitch'; 'l_wrist_yaw';
                         'r_shoulder_pitch'; 'r_shoulder_roll'; 'r_shoulder_yaw'; 'r_elbow'; 'r_wrist_prosup'; 'r_wrist_pitch'; 'r_wrist_yaw';
                         'l_hip_pitch'; 'l_hip_roll'; 'l_hip_yaw'; 'l_knee'; 'l_ankle_pitch'; 'l_ankle_roll';
                         'r_hip_pitch'; 'r_hip_roll'; 'r_hip_yaw'; 'r_knee'; 'r_ankle_pitch'; 'r_ankle_roll'
    };

 packageFile = fullfile('+iDynTreeWrappers', 'loadReducedModel.m');
if exist(packageFile, 'file') ~= 2
    if ~exist('./robotology-matlab', 'dir')
        disp('Installing iDynTreeWrappers.');
        install_robotology_packages 
    end
    robotology_setup %It does not work on Ubuntu 20.04. See https://github.com/robotology/idyntree/issues/1084
end

% Main variable of iDyntreeWrappers used for many things including updating
% robot position and getting world to frame transforms
KinDynModel = iDynTreeWrappers.loadReducedModel(consideredJoints,'root_link',modelPath,fileName,false);

% create vector of positions
joints_positions=zeros(KinDynModel.NDOF,1);

% add a world to base mainly to avoid overlap of coordinate frame and robot
world_H_base=[1,0,0,0;
                            0,1,0,0;
                            0,0,1,0;
                            0,0,0,1];

initial_point_joints = 4000;
final_point_joints = 32500;

retargeting_joints = squeeze(robot_logger_device.walking.joints_state.positions.retargeting.data(1:length(consideredJoints), initial_point_joints : final_point_joints));
measured_joints = squeeze(robot_logger_device.walking.joints_state.positions.measured.data(1:length(consideredJoints), initial_point_joints : final_point_joints));
joints_time = squeeze(robot_logger_device.walking.joints_state.positions.retargeting.timestamps(initial_point_joints : final_point_joints));
initial_time = joints_time(1);
joints_time = joints_time - initial_time;
final_time = joints_time(end);

left_hand_cartesian_retargeting  = zeros(3, length(joints_time));
right_hand_cartesian_retargeting  = zeros(3, length(joints_time), 1);

left_hand_cartesian_measured  = zeros(3, length(joints_time), 1);
right_hand_cartesian_measured  = zeros(3, length(joints_time), 1);

h = waitbar(0,'Initializing waitbar...');

for i = 1 : length(joints_time)
    perc = i/length(joints_time);
    waitbar(perc, h, 'Generating cartesian data')

    iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,retargeting_joints(:, i),zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);
    l_hand_transform = iDynTreeWrappers.getWorldTransform(KinDynModel, 'l_hand');
    left_hand_cartesian_retargeting(:, i) = l_hand_transform(1:3, 4);

    r_hand_transform = iDynTreeWrappers.getWorldTransform(KinDynModel, 'r_hand');
    right_hand_cartesian_retargeting(:, i) = r_hand_transform(1:3, 4);

    iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,measured_joints(:, i),zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);
    l_hand_transform = iDynTreeWrappers.getWorldTransform(KinDynModel, 'l_hand');
    left_hand_cartesian_measured(:, i) = l_hand_transform(1:3, 4);

    r_hand_transform = iDynTreeWrappers.getWorldTransform(KinDynModel, 'r_hand');
    right_hand_cartesian_measured(:, i) = r_hand_transform(1:3, 4);

end
close(h)

figure('Renderer', 'painters', 'Position', [10 10 1170 650])
plot3DSignalDesiredMeasured(joints_time, left_hand_cartesian_retargeting',  left_hand_cartesian_measured', [0 300], [-0.6, 0.6], 'Left Hand Position (m)');
fileName = [pathToFigures, 'leftHandTracking.pdf'];
export_fig('-transparent', fileName)

figure('Renderer', 'painters', 'Position', [10 10 1170 650])
plot3DSignalDesiredMeasured(joints_time, right_hand_cartesian_retargeting',  right_hand_cartesian_measured', [0 300], [-0.6, 0.6], 'Right Hand Position (m)');
fileName = [pathToFigures, 'rightHandTracking.pdf'];
export_fig('-transparent', fileName)

zoom_init_s = 130;
zoom_end_s = 160;

zoom_init_index = find(joints_time >= zoom_init_s, 1);
zoom_end_index = find(joints_time >= zoom_end_s, 1);
figure('Renderer', 'painters', 'Position', [10 10 1170 650])
plot3DSignalDesiredMeasured(joints_time(zoom_init_index:zoom_end_index), left_hand_cartesian_retargeting(:, zoom_init_index:zoom_end_index)',  left_hand_cartesian_measured(:, zoom_init_index:zoom_end_index)', [zoom_init_s zoom_end_s], [0.0, 0.6], 'Left Hand Position (m)');
fileName = [pathToFigures, 'leftHandTrackingZoomed.pdf'];
export_fig('-transparent', fileName)
