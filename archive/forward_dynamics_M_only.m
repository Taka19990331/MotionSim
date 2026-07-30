function dx = forward_dynamics_M_only(t, x, param)

    %% State
    q  = x(1:2);
    dq = x(3:4);

    %% Mass matrix
    M = mass_matrix(q, param);

    %% Applied joint torque
    tau = applied_joint_torque(t);

    %% Forward dynamics
    ddq = M \ tau;

    %% State derivative
    dx = [
        dq;
        ddq
    ];

end


function tau = applied_joint_torque(t)

    % Apply torque from 0.5 s to 1.0 s
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