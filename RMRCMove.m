%% Generalized RMRC function for any robot model
function [qMatrix]=RMRCMove(targetPose, robotModel)
    steps = 50;    % Number of steps in trajectory
    deltaT = 0.05; % Discrete time step

    % Get current joint positions and end-effector pose
    currentQ = robotModel.model.getpos();          % Get current joint state
    currentPose = robotModel.model.fkine(currentQ).T; % Current end-effector pose (4x4 matrix)

    % Extract the position from the current pose
    x1 = currentPose(1:3,4);  % Current position of the end-effector

    % Define the target position from the target pose
    x2 = targetPose(1:3, 4);

    % Create interpolation in 3D space for the position
    s = lspb(0, 1, steps);  % Linear trajectory scalar
    x = zeros(3, steps);
    for i = 1:steps
        x(:, i) = x1 * (1 - s(i)) + x2 * s(i);  % Interpolate positions
    end

    % Initialize joint angles matrix for RMRC
    qMatrix = nan(steps, robotModel.model.n);  % n is the number of DOF for the robot

    % Use the current joint configuration as the starting point
    qMatrix(1, :) = currentQ;

    % Loop through steps for RMRC
    for i = 1:steps-1
        % Calculate the velocity at the current step
        xdot = (x(:, i+1) - x(:, i)) / deltaT;

        % Compute the Jacobian at the current joint state
        J = robotModel.model.jacob0(qMatrix(i, :));

        % Only consider the linear part of the Jacobian (3x6) for position control
        Jv = J(1:3, :);  % Linear velocity part

        % Solve for joint velocities using inverse Jacobian
        qdot = pinv(Jv) * xdot;  % Use pseudo-inverse in case of singularities

        % Update the next joint configuration
        qMatrix(i+1, :) = qMatrix(i, :) + deltaT * qdot';

    
       
    end
end
