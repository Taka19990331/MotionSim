function param = robot_parameters()

    %% Geometry [m]
    param.l1 = 0.4;
    param.l2 = 0.4;

    %% Complete moving-link masses [kg]
    % Each mass should include all components rigidly moving with the link:
    % plates, brackets, shafts, bearings, motor housing, etc.
    param.m1 = 1.0;
    param.m2 = 1.0;

    %% Uniform-link approximation
    % Temporary approximation until COM and inertia are obtained from CAD.
    param.r1 = param.l1 / 2;
    param.r2 = param.l2 / 2;

    % Inertia about each link COM [kg*m^2]
    param.I1 = (1/12) * param.m1 * param.l1^2;
    param.I2 = (1/12) * param.m2 * param.l2^2;

    %% Motor and gearbox specifications
    param.reduction_ratio = 8;

    % Catalog inertia:
    % 1232.6191 g*cm^2
    %
    % Unit conversion:
    % 1 g*cm^2 = 1e-7 kg*m^2
    param.J_motor_spec = 1232.6191e-7;

    %% Reflected motor inertia at joint output [kg*m^2]
    % Assumption:
    % The catalog inertia is motor-side inertia.
    %
    % Reflected inertia:
    % J_ref = N^2 * J_motor
    param.JM1_ref = ...
        param.reduction_ratio^2 * param.J_motor_spec;

    param.JM2_ref = ...
        param.reduction_ratio^2 * param.J_motor_spec;

    %% Gravity [m/s^2]
    param.g = 9.80665;
    
    %% Joint friction parameters
    % Viscous friction coefficient [Nm*s/rad]
    param.B1 = 0.05;
    param.B2 = 0.05;
    
    % Coulomb-like backdrive resistance [Nm]
    %
    % Catalog reference:
    % Backdrive torque = 0.75 Nm
    param.tau_c1 = 0.75;
    param.tau_c2 = 0.75;
    
    % Smoothing velocity for Coulomb friction [rad/s]
    %
    % Smaller values make the model closer to sign(dq),
    % but also make the differential equation stiffer.
    param.v_s = 0.05;

end
