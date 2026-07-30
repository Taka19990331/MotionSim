function dx = forward_dynamics_MCG(~, x, param)
%FORWARD_DYNAMICS_MCG Forward dynamics with M, C, and G.
%
% State:
%   x = [q1; q2; dq1; dq2]
%
% Equation of motion:
%   M(q)*ddq + c(q,dq) + G(q) = tau
%
% For this free-fall test:
%   tau = [0; 0]

    %% State variables
    q  = x(1:2);
    dq = x(3:4);

    %% Robot dynamics
    M = mass_matrix(q, param);
    c = coriolis_torque(q, dq, param);
    G = gravity_torque(q, param);

    %% Applied joint torque
    tau = [
        0;
        0
    ];

    %% Joint acceleration
    ddq = M \ (tau - c - G);

    %% State derivative
    dx = [
        dq;
        ddq
    ];

end