close all;
clear;
clc;

%% Load Trajectories
load TestTrajectory.mat


%% Activate/Deactivate Robots

Robot1Active = true;                %true or false
Robot2Active = true;

%% Init Target Variables

%Define Target Point Robot 1
TargetPoint1 = rob1path(1, :);

%Define Target Point Robot 2
TargetPoint2 = rob2path(1, :);

%Define Magnet Orientation Robot 1
MagOrient1 = rob1Angle(1, :);               %[Rx, Ry Rz]

%Define Magnet orientation Robot 2
MagOrient2 = rob2Angle(1, :);

%% Constants

% Define unit vectors in local magnet frame (assuming the magnet's north pole points along the local Z-axis)
localMagnetDirection = [0; 0; 1];

%Magentic moment of the EPM (Magnitude)
mu_EPM = 970.1; %from some old code, where does it come from?
%mu_EPM = 0.00166; %N52 = 1.48T. Square cylinder, d=100mm,L=100mm mu = M*Volume = 1.48*pi*0.05^2*0.1

% Permeability of free space
mu0 = 4*pi*1e-7; 

% Grid
[x, y, z] = meshgrid(linspace(-0.4, 0.4, 25), linspace(-0.4, 0.4, 25), linspace(-0.42, 1, 25));

%Robot joint limits
jointLimits = [-170,170;
               -90,120; % Elbow Up
               -170,170;
               -120,120;
               -170,170;
               -120,120;
               -175,175];

%% Define Robot Parameters

%Robot 1
if Robot1Active

    %Load robot 1
    robot1 = importrobot('urdf/kuka_iiwa_1.urdf','DataFormat','row');

    %Create initial guess
    initialguess1 = robot1.homeConfiguration;
   
    %Create Inverse Kinematic solver
    IK1 = generalizedInverseKinematics('RigidBodyTree', robot1,'ConstraintInputs', {'position','aiming','joint','orientation'});

end

if Robot2Active

    %Load robot 2
    robot2 = importrobot('urdf/kuka_iiwa_2.urdf','DataFormat','row');

    %Create initial guess
    initialguess2 = robot2.homeConfiguration;

    %Create Inverse Kinematic solver
    IK2 = generalizedInverseKinematics('RigidBodyTree', robot2,'ConstraintInputs', {'position','aiming','joint','orientation'});

end

%% Load Trajectory Data

%load trajectories into workspace
load TestTrajectory.mat;

% Number of steps in the trajectory
numSteps = size(rob1Angle, 1);

%% Setup Animation

% Create a figure for the animation
hFig = figure;

% Set the figure to fullscreen
set(hFig, 'units', 'normalized', 'outerposition', [0 0 1 1]);

% Set up the axes for the 3D plot
ax = axes('Parent', hFig);
hold(ax, 'on');
axis(ax, 'equal');
axis(ax, [-1 1 -1 1 -1 1]);  %[-1 1 -1 1 -0.42 1]; [-0.2 0.2 -0.2 0.2 -0.2 0.2]
grid(ax, 'on');
xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');
title(ax, 'Robot Trajectories and Magnetic Field Visualization');

% Set the view to isometric
view(ax, 45, 45);

%% Setup Loop for animation

% Loop through each step in the trajectory
for step = 1:numSteps

    % Update target points and magnet orientations
    TargetPoint1 = rob1path(step, :);
    TargetPoint2 = rob2path(step, :);
    MagOrient1 = rob1Angle(step, :);
    MagOrient2 = rob2Angle(step, :);

    %Update initial Guess Perform IK to get joint pos
    if step>1
        initialguess1 = Angles1;
        initialguess2 = Angles2;
    end

    

    %Update Constraints and solve
    if Robot1Active

        %Define Position Constraints
        posConstraint1 = constraintPositionTarget('magnet_center_link');
        posConstraint1.TargetPosition = TargetPoint1;
    
        %Define Aiming Constraint
        aimConstraint1 = constraintAiming('magnet_center_link');
        aimConstraint1.TargetPoint = TargetPoint1;
    
        %Define Oirentation Constraints
        oirConstraint1 = constraintOrientationTarget('magnet_center_link');
        oirConstraint1.OrientationTolerance = deg2rad(5);
        oirConstraint1.TargetOrientation = eul2quat(MagOrient1);
    
        %Define Constraints
        jointConstraint1 = constraintJointBounds(robot1);
        jointConstraint1.Bounds = deg2rad(jointLimits);

        %Inverse kinematics (optimisation routine)
        [Angles1,success1] = IK1(initialguess1, posConstraint1, aimConstraint1, jointConstraint1, oirConstraint1);
    end

    %Update Constraints and solve
    if Robot2Active

        %Define Position Constraints
        posConstraint2 = constraintPositionTarget('magnet_center_link');
        posConstraint2.TargetPosition = TargetPoint2;
    
        %Define Aiming Constraint
        aimConstraint2 = constraintAiming('magnet_center_link');
        aimConstraint2.TargetPoint = TargetPoint2;
    
        %Define Oirentation Constraints
        oirConstraint2 = constraintOrientationTarget('magnet_center_link');
        oirConstraint2.OrientationTolerance = deg2rad(5);
        oirConstraint2.TargetOrientation = eul2quat(MagOrient2);
    
        %Define Constraints
        jointConstraint2 = constraintJointBounds(robot2);
        jointConstraint2.Bounds = deg2rad(jointLimits);

        %Inverse kinematics (optimisation routine)
        [Angles2,success2] = IK2(initialguess2, posConstraint2, aimConstraint2, jointConstraint2, oirConstraint2);
    end

    %% Robot End Effector Magnetic Fields

    %initialise Bx, By, Bz
    Bx1 = x.*0; By1 = x.*0; Bz1 = x.*0; 
    Bx2 = x.*0; By2 = x.*0; Bz2 = x.*0; 
    
    %Robot 1
    if Robot1Active
        
        % Compute transformation matrix for robots from base to end effector
        transformMatrix1 = getTransform(robot1, Angles1, 'magnet_center_link', 'base_link');
    
        % Extract rotation matrices from transformation matrices
        R1 = transformMatrix1(1:3, 1:3);
    
        % Get x,y,z position Magnet of Robot 1
        Mag1_x = transformMatrix1(1,4);
        Mag1_y = transformMatrix1(2,4);
        Mag1_z = transformMatrix1(3,4);
    
        % Calculate magnetic moment vectors
        m1 = mu_EPM * R1 * localMagnetDirection;
    
        % Calculate field components for the first EPM (robot1)
        x1 = x - Mag1_x;
        y1 = y - Mag1_y;
        z1 = z - Mag1_z;
        r1 = sqrt(x1.^2 + y1.^2 + z1.^2);
        rx1 = x1./r1; ry1 = y1./r1; rz1 = z1./r1;
        
        Bx1 = mu0/(4*pi) * (3*(m1(1)*rx1 + m1(2)*ry1 + m1(3)*rz1).*rx1 - m1(1))./r1.^3;
        By1 = mu0/(4*pi) * (3*(m1(1)*rx1 + m1(2)*ry1 + m1(3)*rz1).*ry1 - m1(2))./r1.^3;
        Bz1 = mu0/(4*pi) * (3*(m1(1)*rx1 + m1(2)*ry1 + m1(3)*rz1).*rz1 - m1(3))./r1.^3;
    
        % Remove singularities
        threshold = 0.1;
        Bx1(r1<threshold) = NaN; By1(r1<threshold) = NaN; Bz1(r1<threshold) = NaN;
    end
    
    %Robot 2
    if Robot2Active
    
        % Compute transformation matrix for robots from base to end effector
        transformMatrix2 = getTransform(robot2, Angles2, 'magnet_center_link', 'base_link');
    
        % Extract rotation matrices from transformation matrices
        R2 = transformMatrix2(1:3, 1:3);
    
        % Get x,y,z position Magnet of Robot 2
        Mag2_x = transformMatrix2(1,4);
        Mag2_y = transformMatrix2(2,4);
        Mag2_z = transformMatrix2(3,4);
    
        % Calculate magnetic moment vectors
        m2 = mu_EPM * R2 * localMagnetDirection;
    
        % Calculate field components for the second EPM (robot2)
        x2 = x - Mag2_x;
        y2 = y - Mag2_y;
        z2 = z - Mag2_z;
        r2 = sqrt(x2.^2 + y2.^2 + z2.^2);
        rx2 = x2./r2; ry2 = y2./r2; rz2 = z2./r2;
        
        Bx2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*rx2 - m2(1))./r2.^3;
        By2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*ry2 - m2(2))./r2.^3;
        Bz2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*rz2 - m2(3))./r2.^3;
    
        % Remove singularities
        threshold = 0.1;
        Bx2(r2<threshold) = NaN; By2(r2<threshold) = NaN; Bz2(r2<threshold) = NaN;
    end
    
    
    % Sum the magnetic fields from all dipoles
    Bx_total = Bx1 + Bx2;
    By_total = By1 + By2;
    Bz_total = Bz1 + Bz2;

    % Clear previous robots and fields for the new frame (optional)
    cla(ax);

    if Robot1Active
        show(robot1,Angles1,'Frames','off');
        hold on
        addOrientationArrows(transformMatrix1);
        plot3(Mag1_x, Mag1_y, Mag1_z, 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r'); % First dipole position
    end
    if Robot2Active
        show(robot2,Angles2,"Frames","off");
        hold on
        addOrientationArrows(transformMatrix2);
        plot3(Mag2_x, Mag2_y, Mag2_z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Second dipole position
    end
    hold on
    quiver3(x, y, z, Bx_total, By_total, Bz_total);
    %axis([-1 1 -1 1 -0.42 1]);


    % Pause to control the speed of animation
    pause(0.1);

    % Capture frames for a video
    frames(step) = getframe(hFig);
end

% Save the animation as a video file
video = VideoWriter('robot_trajectory2.avi');
video.FrameRate = 10;
open(video);
writeVideo(video, frames);
close(video);














