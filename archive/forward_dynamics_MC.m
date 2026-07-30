function dx = forward_dynamics_MC(t, x, param)
%FORWARD_DYNAMICS_MC Forward dynamics with inertia and Coriolis terms.
%
% State:
%   x = [q1; q2; dq1; dq2]
%
% Equation of motion:
%   M(q) * ddq + c(q,dq) = tau

    %% State variables
    q  = x(1:2);
    dq = x(3:4);

    %% Robot dynamics
    M = mass_matrix(q, param);
    c = coriolis_torque(q, dq, param);

    %% Applied joint torque
    tau = applied_joint_torque(t);

    %% Joint acceleration
    ddq = M \ (tau - c);

    %% State derivative
    dx = [
        dq;
        ddq
    ];

end


function tau = applied_joint_torque(t)
%APPLIED_JOINT_TORQUE Joint torque used in this test.
%
% Joint 1:
%   1 Nm from 0.5 s to 1.0 s
%
% Joint 2:
%   No applied torque

    if t >= 0.5 && t < 1.0
        tau = [
            1.0;
            0.0
        ];
    else
        tau = [
            0.0;
            0.0
        ];
    end

end