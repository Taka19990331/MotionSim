function kin = robot_kinematics_2d(q, dq, param)
%ROBOT_KINEMATICS_2D
% Kinematics of the planar 2-link robot.
%
% Joint definition:
%   q1 : Absolute angle of link 1 from the +X axis
%   q2 : Absolute angle of link 2 from the +X axis
%
% Coordinate system:
%   X : Horizontal direction
%   Z : Vertical direction
%
% Input:
%   q:
%       2x1 joint-angle vector [rad]
%
%   dq:
%       2x1 joint-velocity vector [rad/s]
%
%   param:
%       Robot parameter structure
%
% Output:
%   kin:
%       Structure containing positions, velocities,
%       and Jacobians of the second joint and end effector.

    %% Input check

    if numel(q) ~= 2
        error('q must contain two joint angles.');
    end

    if numel(dq) ~= 2
        error('dq must contain two joint velocities.');
    end

    q = q(:);
    dq = dq(:);

    %% Joint states

    q1 = q(1);
    q2 = q(2);

    %% Forward kinematics

    [joint_position, ee_position] = ...
        forward_kinematics_2d( ...
            q1, ...
            q2, ...
            param);

    %% Jacobians

    J_joint = ...
        joint_jacobian_2d( ...
            q1, ...
            param);

    J_ee = ...
        ee_jacobian_2d( ...
            q1, ...
            q2, ...
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
    forward_kinematics_2d(q1, q2, param)
%FORWARD_KINEMATICS_2D
% Calculate the second-joint and end-effector positions
% in the planar X-Z coordinate system.

    joint_position = [
        param.l1 * cos(q1);
        param.l1 * sin(q1)
    ];

    ee_position = ...
        joint_position ...
        + [
            param.l2 * cos(q2);
            param.l2 * sin(q2)
        ];

end


function J_joint = ...
    joint_jacobian_2d(q1, param)
%JOINT_JACOBIAN_2D
% Jacobian of the second-joint position.
%
% Position:
%   x = l1*cos(q1)
%   z = l1*sin(q1)

    J_joint = [
        -param.l1 * sin(q1), 0;
         param.l1 * cos(q1), 0
    ];

end


function J_ee = ...
    ee_jacobian_2d(q1, q2, param)
%EE_JACOBIAN_2D
% Jacobian of the end-effector position.
%
% Position:
%   x = l1*cos(q1) + l2*cos(q2)
%   z = l1*sin(q1) + l2*sin(q2)

    J_ee = [
        -param.l1 * sin(q1), ...
        -param.l2 * sin(q2);

         param.l1 * cos(q1), ...
         param.l2 * cos(q2)
    ];

end