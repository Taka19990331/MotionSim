function param = robot_parameters_all()
%ROBOT_PARAMETERS_ALL
% Common robot parameters for both 2-DOF and 3-DOF simulations.
%
% Joint definition:
%   q1 : Absolute angle of link 1 in the local radial-Z plane
%   q2 : Absolute angle of link 2 in the local radial-Z plane
%   q3 : Root yaw angle about the world Z axis
%
% Cartesian control-coordinate definition:
%
%   p = [
%       X;
%       Y;
%       Z
%   ]
%
% The 2-DOF model uses q1 and q2.
% The 3-DOF model uses q1, q2, and q3.

    %% Model information

    param.n_dof_2d = 2;
    param.n_dof_3d = 3;

    %% Geometry [m]

    param.l1 = 0.4;
    param.l2 = 0.4;

    %% Complete moving-link masses [kg]
    %
    % Each mass should include all components rigidly moving
    % with the corresponding link:
    %
    %   plates
    %   brackets
    %   shafts
    %   bearings
    %   motor housings
    %   other attached components

    param.m1 = 1.0;
    param.m2 = 1.0;

    %% Center-of-mass positions [m]
    %
    % Temporary uniform-link approximation.
    % Replace these values with CAD-derived COM positions later.

    param.r1 = ...
        param.l1 / 2;

    param.r2 = ...
        param.l2 / 2;

    %% Link inertia about each center of mass [kg*m^2]
    %
    % Temporary slender uniform-link approximation:
    %
    %   I = (1/12) * m * l^2
    %
    % These are the moments of inertia for rotation perpendicular
    % to the longitudinal axis of each link.

    param.I1 = ...
        (1 / 12) ...
        * param.m1 ...
        * param.l1^2;

    param.I2 = ...
        (1 / 12) ...
        * param.m2 ...
        * param.l2^2;

    %% Root-yaw structural inertia [kg*m^2]
    %
    % This value represents inertia belonging directly to the root
    % yaw assembly, excluding the configuration-dependent inertia
    % of links 1 and 2.
    %
    % Replace this temporary value with a CAD-derived value.

    param.I_yaw_base = 0.001;

    %% Motor and gearbox specifications

    param.reduction_ratio = [
        8;
        8;
        8
    ];

    % Catalog motor-side rotor inertia:
    %
    %   1232.6191 g*cm^2
    %
    % Unit conversion:
    %
    %   1 g*cm^2 = 1e-7 kg*m^2

    param.J_motor_spec = ...
        1232.6191e-7;

    %% Reflected motor inertia at joint output [kg*m^2]
    %
    % Reflected inertia:
    %
    %   J_ref = N^2 * J_motor

    param.JM_ref = ...
        param.reduction_ratio.^2 ...
        * param.J_motor_spec;

    % Named values retained for readability and compatibility.

    param.JM1_ref = ...
        param.JM_ref(1);

    param.JM2_ref = ...
        param.JM_ref(2);

    param.JM3_ref = ...
        param.JM_ref(3);

    %% Gravity [m/s^2]

    param.g = 9.80665;

    %% Joint friction parameters

    % Viscous friction coefficient [Nm*s/rad]

    param.B = [
        0.05;
        0.05;
        0.05
    ];

    % Smooth Coulomb-like resistance [Nm]
    %
    % Catalog reference:
    %
    %   Backdrive torque = 0.75 Nm

    param.tau_c = [
        0.75;
        0.75;
        0.75
    ];

    % Smoothing velocity for Coulomb friction [rad/s]
    %
    % Smaller values make tanh(dq / v_s) closer to sign(dq),
    % but also make the differential equation numerically stiffer.

    param.v_s = 0.05;

    % Named values retained for readability and compatibility.

    param.B1 = ...
        param.B(1);

    param.B2 = ...
        param.B(2);

    param.B3 = ...
        param.B(3);

    param.tau_c1 = ...
        param.tau_c(1);

    param.tau_c2 = ...
        param.tau_c(2);

    param.tau_c3 = ...
        param.tau_c(3);

    %% Cartesian task-space KD control
    %
    % Cartesian coordinate order:
    %
    %   p = [
    %       X;
    %       Y;
    %       Z
    %   ]
    %
    % Control law:
    %
    %   F_control
    %       = K * (p_ref - p)
    %       - D * dp
    %
    % Joint torque:
    %
    %   tau_control
    %       = J_ee.' * F_control
    %
    % X and Y are controlled relatively stiffly so that
    % the end effector behaves approximately like it is
    % constrained to a vertical slider.
    %
    % Z is intentionally softer so that its response to
    % the sinusoidal external force can be observed.
    %
    % The Cartesian position reference is calculated from
    % the initial posture in main_MCGFC_3d.

    param.control.enabled = true;

    % Cartesian stiffness matrix [N/m]
    %
    % Coordinate order:
    %   X, Y, Z

    param.control.K = diag([
        2000;
        2000;
         300
    ]);

    % Cartesian damping matrix [N*s/m]
    %
    % Coordinate order:
    %   X, Y, Z

    param.control.D = diag([
        100;
        100;
        30
    ]);

    % The reference position will be assigned in the main script
    % after calculating the initial end-effector position.

    param.control.position_reference = [
        NaN;
        NaN;
        NaN
    ];

    %% External sinusoidal force
    %
    % External Cartesian force:
    %
    %   F_external = [
    %       0;
    %       0;
    %       F_external_z
    %   ]
    %
    % where:
    %
    %   F_external_z
    %       = amplitude
    %       * sin( ...
    %           2*pi*frequency*(t - start_time) ...
    %           + phase)
    %
    % The force is zero before start_time.

    param.external_force.enabled = true;

    % Force amplitude [N]

    param.external_force.amplitude = 50.0;

    % Force frequency [Hz]

    param.external_force.frequency = 0.5;

    % Initial phase [rad]

    param.external_force.phase = 0;

    % Time at which the sinusoidal force starts [s]

    param.external_force.start_time = 0.0;

    %% Surface-contact parameters

    % Contact is disabled during the present
    % Cartesian KD-control test.

    param.contact.enabled = false;

    % Horizontal floor height in world coordinates [m]

    param.z_floor = -0.01;

    % Normal contact stiffness [N/m]

    param.k_contact = 3000;

    % Normal contact damping [N*s/m]

    param.d_contact = 40;

    %% Numerical contact settings

    % The surface may push the robot but must not pull it.

    param.contact_force_min = 0;

    %% Visualization settings

    param.animation.playback_speed = 1.0;
    param.animation.frame_rate = 30;
    param.animation.force_scale = 0.01;

end