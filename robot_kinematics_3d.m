function kin = robot_kinematics_3d(q, dq, param)
%ROBOT_KINEMATICS_3D
% Kinematics of a 3-DOF robot consisting of:
%
%   q1 : Absolute angle of link 1 in the local radial-Z plane
%   q2 : Absolute angle of link 2 in the local radial-Z plane
%   q3 : Root yaw angle about the world Z axis
%
% Coordinate system:
%   X, Y : Horizontal world coordinates
%   Z    : Vertical world coordinate
%
% Input:
%   q:
%       3x1 joint-angle vector [rad]
%
%   dq:
%       3x1 joint-velocity vector [rad/s]
%
%   param:
%       Robot parameter structure
%
% Output:
%   kin:
%       Structure containing positions, velocities,
%       and Jacobians of the second joint and end effector.

    %% Input check

    if numel(q) ~= 3
        error('q must contain three joint angles.');
    end

    if numel(dq) ~= 3
        error('dq must contain three joint velocities.');
    end

    q = q(:);
    dq = dq(:);

    %% Joint states

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);

    %% Forward kinematics

    [joint_position, ee_position] = ...
        forward_kinematics_3d( ...
            q1, ...
            q2, ...
            q3, ...
            param);

    %% Jacobians

    J_joint = ...
        joint_jacobian_3d( ...
            q1, ...
            q3, ...
            param);

    J_ee = ...
        ee_jacobian_3d( ...
            q1, ...
            q2, ...
            q3, ...
            param);

    %% Cartesian velocities

    joint_velocity = ...
        J_joint * dq;

    ee_velocity = ...
        J_ee * dq;

    %% Output

    kin.joint.position = ...
        joint_position;

    kin.joint.velocity = ...
        joint_velocity;

    kin.joint.J = ...
        J_joint;

    kin.ee.position = ...
        ee_position;

    kin.ee.velocity = ...
        ee_velocity;

    kin.ee.J = ...
        J_ee;

end


function [joint_position, ee_position] = ...
    forward_kinematics_3d(q1, q2, q3, param)
%FORWARD_KINEMATICS_3D
% Calculate the second-joint and end-effector positions
% in the world X-Y-Z coordinate system.

    %% Position in the local radial-Z plane

    rho_joint = ...
        param.l1 * cos(q1);

    z_joint = ...
        param.l1 * sin(q1);

    rho_ee = ...
        rho_joint ...
        + param.l2 * cos(q2);

    z_ee = ...
        z_joint ...
        + param.l2 * sin(q2);

    %% Rotation about the world Z axis

    joint_position = [
        rho_joint * cos(q3);
        rho_joint * sin(q3);
        z_joint
    ];

    ee_position = [
        rho_ee * cos(q3);
        rho_ee * sin(q3);
        z_ee
    ];

end


function J_joint = ...
    joint_jacobian_3d(q1, q3, param)
%JOINT_JACOBIAN_3D
% Jacobian of the second-joint position.
%
% Column order:
%   1 : Partial derivative with respect to q1
%   2 : Partial derivative with respect to q2
%   3 : Partial derivative with respect to q3

    rho_joint = ...
        param.l1 * cos(q1);

    J_joint = [
        -param.l1 * sin(q1) * cos(q3), ...
         0, ...
        -rho_joint * sin(q3);

        -param.l1 * sin(q1) * sin(q3), ...
         0, ...
         rho_joint * cos(q3);

         param.l1 * cos(q1), ...
         0, ...
         0
    ];

end


function J_ee = ...
    ee_jacobian_3d(q1, q2, q3, param)
%EE_JACOBIAN_3D
% Jacobian of the end-effector position.
%
% Column order:
%   1 : Partial derivative with respect to q1
%   2 : Partial derivative with respect to q2
%   3 : Partial derivative with respect to q3

    rho_ee = ...
        param.l1 * cos(q1) ...
        + param.l2 * cos(q2);

    J_ee = [
        -param.l1 * sin(q1) * cos(q3), ...
        -param.l2 * sin(q2) * cos(q3), ...
        -rho_ee * sin(q3);

        -param.l1 * sin(q1) * sin(q3), ...
        -param.l2 * sin(q2) * sin(q3), ...
         rho_ee * cos(q3);

         param.l1 * cos(q1), ...
         param.l2 * cos(q2), ...
         0
    ];

end