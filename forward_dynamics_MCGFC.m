function dx = forward_dynamics_MCGFC(~, x, param)
%FORWARD_DYNAMICS_MCGFC
% Forward dynamics with:
%   M : inertia
%   C : Coriolis and centrifugal effect
%   G : gravity
%   F : joint friction
%   C : surface contact
%
% Equation:
%   M(q)*ddq
%   + c(q,dq)
%   + G(q)
%   + tau_fric(dq)
%   = tau_command + tau_contact

    %% State

    q  = x(1:2);
    dq = x(3:4);

    %% Dynamics terms

    M = mass_matrix(q, param);

    c = coriolis_torque( ...
        q, dq, param);

    G = gravity_torque( ...
        q, param);

    tau_fric = friction_torque( ...
        dq, param);

    %% Surface contact

    [~, tau_contact] = ...
        surface_contact_force( ...
            q, dq, param);

    %% Command torque

    tau_command = [
        0;
        0
    ];

    %% Forward dynamics

    ddq = M \ ( ...
        tau_command ...
        + tau_contact ...
        - c ...
        - G ...
        - tau_fric);

    %% State derivative

    dx = [
        dq;
        ddq
    ];

end