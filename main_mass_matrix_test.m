clear;
close all;
clc;

param = robot_parameters();

%% Test posture
q = [
    deg2rad(120);
    deg2rad(60)
];

%% Calculate mass matrix
[M, detail] = mass_matrix(q, param);

disp('M_links [kg*m^2]'); %[output:0e4f661c]
disp(detail.M_links); %[output:33b7d78c]

disp('M_motor [kg*m^2]'); %[output:93bb4525]
disp(detail.M_motor); %[output:4e101238]

disp('M_total [kg*m^2]'); %[output:64cf35dc]
disp(M); %[output:649d4a92]

%% Symmetry check
symmetry_error = norm(M - M.', 'fro');

disp('Symmetry error'); %[output:47d15b30]
disp(symmetry_error); %[output:335cb0e8]

%% Positive-definite check
eigenvalues = eig(M);

disp('Eigenvalues'); %[output:06505424]
disp(eigenvalues); %[output:23a340a8]

if all(eigenvalues > 0) %[output:group:5e1781dc]
    disp('M is positive definite at the test posture.'); %[output:89ea4177]
else
    warning('M is not positive definite.');
end %[output:group:5e1781dc]

%% Check over all relative link angles
q1_range = deg2rad(0:5:180);
q2_range = deg2rad(0:5:180);

minimum_eigenvalue = inf;
maximum_eigenvalue = -inf;
maximum_symmetry_error = 0;

worst_q = [NaN; NaN];

for q1 = q1_range
    for q2 = q2_range

        q_test = [q1; q2];
        M_test = mass_matrix(q_test, param);

        eig_test = eig(M_test);

        if min(eig_test) < minimum_eigenvalue
            minimum_eigenvalue = min(eig_test);
            worst_q = q_test;
        end

        maximum_eigenvalue = max( ...
            maximum_eigenvalue, ...
            max(eig_test));

        maximum_symmetry_error = max( ...
            maximum_symmetry_error, ...
            norm(M_test - M_test.', 'fro'));
    end
end

disp('Minimum eigenvalue over tested postures'); %[output:318edbdf]
disp(minimum_eigenvalue); %[output:6ed2a8cd]

disp('Maximum eigenvalue over tested postures'); %[output:82b9dc27]
disp(maximum_eigenvalue); %[output:1847bc04]

disp('Posture at minimum eigenvalue [deg]'); %[output:1aaf9b25]
disp(rad2deg(worst_q)); %[output:7d020ac2]

disp('Maximum symmetry error'); %[output:776edbcf]
disp(maximum_symmetry_error); %[output:413cd4fd]

if minimum_eigenvalue > 0 %[output:group:45d6ba54]
    disp('M is positive definite over all tested postures.'); %[output:9d4bc1df]
else
    warning('M becomes non-positive-definite.');
end %[output:group:45d6ba54]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:0e4f661c]
%   data: {"dataType":"text","outputData":{"text":"M_links [kg*m^2]\n","truncated":false}}
%---
%[output:33b7d78c]
%   data: {"dataType":"text","outputData":{"text":"    0.2133    0.0400\n    0.0400    0.0533\n\n","truncated":false}}
%---
%[output:93bb4525]
%   data: {"dataType":"text","outputData":{"text":"M_motor [kg*m^2]\n","truncated":false}}
%---
%[output:4e101238]
%   data: {"dataType":"text","outputData":{"text":"    0.0079         0\n         0    0.0079\n\n","truncated":false}}
%---
%[output:64cf35dc]
%   data: {"dataType":"text","outputData":{"text":"M_total [kg*m^2]\n","truncated":false}}
%---
%[output:649d4a92]
%   data: {"dataType":"text","outputData":{"text":"    0.2212    0.0400\n    0.0400    0.0612\n\n","truncated":false}}
%---
%[output:47d15b30]
%   data: {"dataType":"text","outputData":{"text":"Symmetry error\n","truncated":false}}
%---
%[output:335cb0e8]
%   data: {"dataType":"text","outputData":{"text":"     0\n\n","truncated":false}}
%---
%[output:06505424]
%   data: {"dataType":"text","outputData":{"text":"Eigenvalues\n","truncated":false}}
%---
%[output:23a340a8]
%   data: {"dataType":"text","outputData":{"text":"    0.0518\n    0.2307\n\n","truncated":false}}
%---
%[output:89ea4177]
%   data: {"dataType":"text","outputData":{"text":"M is positive definite at the test posture.\n","truncated":false}}
%---
%[output:318edbdf]
%   data: {"dataType":"text","outputData":{"text":"Minimum eigenvalue over tested postures\n","truncated":false}}
%---
%[output:6ed2a8cd]
%   data: {"dataType":"text","outputData":{"text":"    0.0281\n\n","truncated":false}}
%---
%[output:82b9dc27]
%   data: {"dataType":"text","outputData":{"text":"Maximum eigenvalue over tested postures\n","truncated":false}}
%---
%[output:1847bc04]
%   data: {"dataType":"text","outputData":{"text":"    0.2544\n\n","truncated":false}}
%---
%[output:1aaf9b25]
%   data: {"dataType":"text","outputData":{"text":"Posture at minimum eigenvalue [deg]\n","truncated":false}}
%---
%[output:7d020ac2]
%   data: {"dataType":"text","outputData":{"text":"     0\n     0\n\n","truncated":false}}
%---
%[output:776edbcf]
%   data: {"dataType":"text","outputData":{"text":"Maximum symmetry error\n","truncated":false}}
%---
%[output:413cd4fd]
%   data: {"dataType":"text","outputData":{"text":"     0\n\n","truncated":false}}
%---
%[output:9d4bc1df]
%   data: {"dataType":"text","outputData":{"text":"M is positive definite over all tested postures.\n","truncated":false}}
%---
