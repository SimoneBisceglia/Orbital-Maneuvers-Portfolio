% =========================================================================
% ADCS / GNC SIMULATION: MASTER INITIALIZATION SCRIPT
% IDEAL BASELINE - NADIR TO OFF-NADIR SLEW
%
% Quaternion convention:
%   q_BI = attitude quaternion representing BODY w.r.t. ECI
%   q_BL = attitude quaternion representing BODY w.r.t. LVLH
%   q_LI = attitude quaternion representing LVLH w.r.t. ECI
%
% Angular velocity convention:
%   omega = angular velocity of BODY w.r.t. ECI,
%           expressed in BODY coordinates [rad/s]
% =========================================================================

clc;
clear;
close all;

disp('--- ADCS / GNC SIMULATOR INITIALIZATION ---');


%% ========================================================================
% 1. SIMULATION PARAMETERS
% =========================================================================

T_sim  = 600.0;       % Total simulation duration [s]
dt_sim = 0.1;         % Simulation / integration step [s]


%% ========================================================================
% 2. ENVIRONMENTAL AND ORBITAL PARAMETERS
%    Circular LEO orbit at 800 km altitude
% =========================================================================

% Earth constants
mu_E    = 398600.4418;      % Earth's gravitational parameter [km^3/s^2]
R_E     = 6371.0;           % Earth's mean radius [km]

% Orbit parameters
h_orbit = 800.0;            % Orbit altitude [km]
r_orbit = R_E + h_orbit;    % Orbital radius [km]

% Circular orbital angular velocity magnitude
omega_orb_mag = sqrt(mu_E / r_orbit^3);   % [rad/s]

% Orbital angular velocity expressed in LVLH coordinates
%
% LVLH convention:
%   X = radial direction
%   Y = along-track direction
%   Z = orbit normal
%
% Here the chosen convention gives the orbital rate around the LVLH Y axis.
omega_orb_LVLH = [0;
                 -omega_orb_mag;
                  0];


%% ========================================================================
% 3. SPACECRAFT RIGID-BODY PARAMETERS
% =========================================================================

% Principal moments of inertia [kg*m^2]
% Approximate microsatellite model
I_xx = 15.0;
I_yy = 15.0;
I_zz = 10.0;

% Inertia tensor
I_tensor = diag([I_xx, I_yy, I_zz]);

% Inverse inertia tensor
I_inv = inv(I_tensor);


%% ========================================================================
% 4. INITIAL ATTITUDE AND ANGULAR VELOCITY
% =========================================================================
%
% Initial attitude:
%   Body is perfectly aligned with LVLH (NADIR pointing)
%
% Initial Euler angles are defined as:
%   Roll  = rotation around X
%   Pitch = rotation around Y
%   Yaw   = rotation around Z
%
% Euler sequence:
%   3-2-1 (Yaw-Pitch-Roll)
% =========================================================================

%% 4.1 Initial Euler angles w.r.t. LVLH

roll_init  = deg2rad(0.0);
pitch_init = deg2rad(0.0);
yaw_init   = deg2rad(0.0);


%% 4.2 Initial BODY-to-LVLH quaternion q_BL_init

cy0 = cos(yaw_init   / 2);
sy0 = sin(yaw_init   / 2);

cp0 = cos(pitch_init / 2);
sp0 = sin(pitch_init / 2);

cr0 = cos(roll_init  / 2);
sr0 = sin(roll_init  / 2);

% Hamilton quaternion, scalar first:
% q = [qw; qx; qy; qz]

q_Body_LVLH_init = [ ...
    cr0*cp0*cy0 + sr0*sp0*sy0;
    sr0*cp0*cy0 - cr0*sp0*sy0;
    cr0*sp0*cy0 + sr0*cp0*sy0;
    cr0*cp0*sy0 - sr0*sp0*cy0 ];

% Normalize
q_Body_LVLH_init = q_Body_LVLH_init / norm(q_Body_LVLH_init);


%% 4.3 Initial LVLH-to-ECI quaternion q_LI_init
%
% This defines the initial orientation of the LVLH frame relative to ECI.
%
% q_LI = quaternion representing LVLH w.r.t. ECI.

q_LVLH_ECI_init = [sqrt(2)/2;
                   0;
                   sqrt(2)/2;
                   0];

q_LVLH_ECI_init = q_LVLH_ECI_init / norm(q_LVLH_ECI_init);


%% 4.4 Initial absolute quaternion q_BI_init
%
% BODY w.r.t. ECI:
%
%       q_BI = q_BL * q_LI
%
% Therefore:
%       q_BI_init = q_Body_LVLH_init * q_LVLH_ECI_init
%

q_init = [ ...
    q_Body_LVLH_init(1)*q_LVLH_ECI_init(1) ...
  - q_Body_LVLH_init(2)*q_LVLH_ECI_init(2) ...
  - q_Body_LVLH_init(3)*q_LVLH_ECI_init(3) ...
  - q_Body_LVLH_init(4)*q_LVLH_ECI_init(4);

    q_Body_LVLH_init(1)*q_LVLH_ECI_init(2) ...
  + q_Body_LVLH_init(2)*q_LVLH_ECI_init(1) ...
  + q_Body_LVLH_init(3)*q_LVLH_ECI_init(4) ...
  - q_Body_LVLH_init(4)*q_LVLH_ECI_init(3);

    q_Body_LVLH_init(1)*q_LVLH_ECI_init(3) ...
  - q_Body_LVLH_init(2)*q_LVLH_ECI_init(4) ...
  + q_Body_LVLH_init(3)*q_LVLH_ECI_init(1) ...
  + q_Body_LVLH_init(4)*q_LVLH_ECI_init(2);

    q_Body_LVLH_init(1)*q_LVLH_ECI_init(4) ...
  + q_Body_LVLH_init(2)*q_LVLH_ECI_init(3) ...
  - q_Body_LVLH_init(3)*q_LVLH_ECI_init(2) ...
  + q_Body_LVLH_init(4)*q_LVLH_ECI_init(1) ];

% Normalize
q_init = q_init / norm(q_init);


%% 4.5 Initial relative angular velocity
%
% Since the spacecraft is initially aligned with LVLH and is assumed
% to be perfectly tracking the LVLH frame:
%
%       omega_rel = 0
%

omega_rel_init = [0;
                  0;
                  0];


%% 4.6 Initial DCM: LVLH -> BODY
%
% C_BL transforms a vector expressed in LVLH coordinates into BODY
% coordinates.
%

C_BL_init = [ ...
    cp0*cy0,                         cp0*sy0,                        -sp0;
    sr0*sp0*cy0 - cr0*sy0,           sr0*sp0*sy0 + cr0*cy0,           sr0*cp0;
    cr0*sp0*cy0 + sr0*sy0,           cr0*sp0*sy0 - sr0*cy0,           cr0*cp0 ];


%% 4.7 Initial BODY angular velocity w.r.t. ECI
%
% The spacecraft is initially tracking the LVLH frame.
%
% Therefore:
%
%       omega_BI = C_BL * omega_LI
%
% where omega_LI is the orbital angular velocity expressed in LVLH.
%

omega_init = C_BL_init * omega_orb_LVLH;


%% ========================================================================
% 5. SENSOR PARAMETERS
% =========================================================================
%
% These parameters are currently placeholders for future sensor models.
% =========================================================================

gyro_bias_true = [ ...
     0.001;
    -0.002;
     0.0015 ];              % True gyro bias [rad/s]

gyro_arw = 1e-4;            % Angular Random Walk [rad/s/sqrt(Hz)]

st_noise_sigma = 1e-3;      % Attitude sensor noise [rad]


%% ========================================================================
% 6. PD CONTROLLER GAINS
% =========================================================================
%
% Desired closed-loop second-order dynamics:
%
%       wn   = natural frequency
%       zeta = damping ratio
%
% Standard rigid-body PD gains:
%
%       Kp = I * wn^2
%       Kd = 2*zeta*I*wn
%
% For zeta = 1, the system is critically damped.
% =========================================================================

omega_n = 0.1;       % Desired natural frequency [rad/s]
zeta    = 1.0;       % Damping ratio

Kp = I_tensor * omega_n^2;

Kd = 2 * zeta * I_tensor * omega_n;

% Force diagonal matrices
Kp = diag(diag(Kp));
Kd = diag(diag(Kd));


%% ========================================================================
% 7. FINAL TARGET ATTITUDE
% =========================================================================
%
% Final target is defined relative to LVLH:
%
%   Roll  = +30 deg
%   Pitch = +45 deg
%   Yaw   = +20 deg
%
% After the maneuver the spacecraft keeps this fixed attitude relative
% to LVLH, therefore the absolute BODY attitude continues to rotate
% with the orbit.
% =========================================================================


%% 7.1 Final target Euler angles

roll_f  = deg2rad(30.0);
pitch_f = deg2rad(45.0);
yaw_f   = deg2rad(20.0);


%% 7.2 Final BODY-to-LVLH quaternion q_BL_final

cy_f = cos(yaw_f   / 2);
sy_f = sin(yaw_f   / 2);

cp_f = cos(pitch_f / 2);
sp_f = sin(pitch_f / 2);

cr_f = cos(roll_f  / 2);
sr_f = sin(roll_f  / 2);

q_Body_LVLH_final = [ ...
    cr_f*cp_f*cy_f + sr_f*sp_f*sy_f;
    sr_f*cp_f*cy_f - cr_f*sp_f*sy_f;
    cr_f*sp_f*cy_f + sr_f*cp_f*sy_f;
    cr_f*cp_f*sy_f - sr_f*sp_f*cy_f ];

% Normalize
q_Body_LVLH_final = q_Body_LVLH_final / norm(q_Body_LVLH_final);


%% ========================================================================
% 7.3 Final LVLH-to-ECI quaternion
% =========================================================================
%
% The LVLH frame rotates with the orbit.
%
% Orbital rotation after T_sim:
%
%       theta_orb = -omega_orb_mag*T_sim
%
% The orbital rotation is around the LVLH Y axis.
% =========================================================================

theta_orb_final = -omega_orb_mag * T_sim;

q_orb_rot_final = [ ...
    cos(theta_orb_final/2);
    0;
    sin(theta_orb_final/2);
    0 ];

q_orb_rot_final = q_orb_rot_final / norm(q_orb_rot_final);


%% 7.4 Final LVLH w.r.t. ECI quaternion
%
% q_LI_final = q_orb_rot_final * q_LI_init
%

q_LI_final = [ ...
    q_orb_rot_final(1)*q_LVLH_ECI_init(1) ...
  - q_orb_rot_final(2)*q_LVLH_ECI_init(2) ...
  - q_orb_rot_final(3)*q_LVLH_ECI_init(3) ...
  - q_orb_rot_final(4)*q_LVLH_ECI_init(4);

    q_orb_rot_final(1)*q_LVLH_ECI_init(2) ...
  + q_orb_rot_final(2)*q_LVLH_ECI_init(1) ...
  + q_orb_rot_final(3)*q_LVLH_ECI_init(4) ...
  - q_orb_rot_final(4)*q_LVLH_ECI_init(3);

    q_orb_rot_final(1)*q_LVLH_ECI_init(3) ...
  - q_orb_rot_final(2)*q_LVLH_ECI_init(4) ...
  + q_orb_rot_final(3)*q_LVLH_ECI_init(1) ...
  + q_orb_rot_final(4)*q_LVLH_ECI_init(2);

    q_orb_rot_final(1)*q_LVLH_ECI_init(4) ...
  + q_orb_rot_final(2)*q_LVLH_ECI_init(3) ...
  - q_orb_rot_final(3)*q_LVLH_ECI_init(2) ...
  + q_orb_rot_final(4)*q_LVLH_ECI_init(1) ];

% Normalize
q_LI_final = q_LI_final / norm(q_LI_final);


%% ========================================================================
% 7.5 Final absolute BODY-to-ECI quaternion q_BI_final
% =========================================================================
%
% IMPORTANT:
%
%       q_BI_final = q_BL_final * q_LI_final
%
% This quaternion represents:
%
%       BODY w.r.t. ECI
%
% It is therefore the quaternion that should be compared with the
% quaternion coming from the spacecraft attitude dynamics.
%

q_final_expected = [ ...
    q_Body_LVLH_final(1)*q_LI_final(1) ...
  - q_Body_LVLH_final(2)*q_LI_final(2) ...
  - q_Body_LVLH_final(3)*q_LI_final(3) ...
  - q_Body_LVLH_final(4)*q_LI_final(4);

    q_Body_LVLH_final(1)*q_LI_final(2) ...
  + q_Body_LVLH_final(2)*q_LI_final(1) ...
  + q_Body_LVLH_final(3)*q_LI_final(4) ...
  - q_Body_LVLH_final(4)*q_LI_final(3);

    q_Body_LVLH_final(1)*q_LI_final(3) ...
  - q_Body_LVLH_final(2)*q_LI_final(4) ...
  + q_Body_LVLH_final(3)*q_LI_final(1) ...
  + q_Body_LVLH_final(4)*q_LI_final(2);

    q_Body_LVLH_final(1)*q_LI_final(4) ...
  + q_Body_LVLH_final(2)*q_LI_final(3) ...
  - q_Body_LVLH_final(3)*q_LI_final(2) ...
  + q_Body_LVLH_final(4)*q_LI_final(1) ];

% Normalize
q_final_expected = q_final_expected / norm(q_final_expected);


%% ========================================================================
% 7.6 FINAL RELATIVE ANGULAR VELOCITY
% =========================================================================
%
% Once the maneuver is completed, the spacecraft keeps a CONSTANT
% orientation relative to LVLH.
%
% Therefore:
%
%       omega_rel_final = 0
%

omega_rel_final = [0;
                   0;
                   0];


%% ========================================================================
% 7.7 Final DCM: LVLH -> BODY
% =========================================================================

C_BL_final = [ ...
    cp_f*cy_f,                         cp_f*sy_f,                        -sp_f;
    sr_f*sp_f*cy_f - cr_f*sy_f,        sr_f*sp_f*sy_f + cr_f*cy_f,       sr_f*cp_f;
    cr_f*sp_f*cy_f + sr_f*sy_f,        cr_f*sp_f*sy_f - sr_f*cy_f,       cr_f*cp_f ];


%% ========================================================================
% 7.8 FINAL BODY ANGULAR VELOCITY w.r.t. ECI
% =========================================================================
%
% Since the spacecraft remains fixed relative to LVLH:
%
%       omega_BI = C_BL * omega_LI
%
% where:
%
%       omega_LI = [0; -omega_orb_mag; 0]
%
% The result is expressed in BODY coordinates.
%

omega_final_expected = C_BL_final * omega_orb_LVLH;


%% ========================================================================
% 8. ADDITIONAL VALIDATION VARIABLES
% =========================================================================

% Quaternion norm checks
q_init_norm          = norm(q_init);
q_final_expected_norm = norm(q_final_expected);

% Initial and final angular velocity magnitude
omega_init_mag = norm(omega_init);
omega_final_mag = norm(omega_final_expected);


%% ========================================================================
% 9. CONSOLE OUTPUT
% =========================================================================

disp(' ');
disp('==============================================================');
disp('              MISSION KINEMATICS SUMMARY');
disp('==============================================================');

disp(' ');
disp('[1] ORBIT PARAMETERS');
fprintf('Orbital radius              : %.3f km\n', r_orbit);
fprintf('Orbital angular velocity    : %.8f rad/s\n', omega_orb_mag);
fprintf('Orbital period              : %.3f s\n', 2*pi/omega_orb_mag);

disp('--------------------------------------------------------------');

disp('[2] INITIAL STATE - t = 0 s');

disp('Initial Euler angles [deg]:');
fprintf('Roll  = %.3f deg\n', rad2deg(roll_init));
fprintf('Pitch = %.3f deg\n', rad2deg(pitch_init));
fprintf('Yaw   = %.3f deg\n', rad2deg(yaw_init));

disp(' ');
disp('Initial q_BL = BODY w.r.t. LVLH:');
disp(q_Body_LVLH_init);

disp('Initial q_BI = BODY w.r.t. ECI:');
disp(q_init);

disp('Initial relative angular velocity [rad/s]:');
disp(omega_rel_init);

disp('Initial BODY angular velocity w.r.t. ECI [rad/s]:');
disp(omega_init);

disp('--------------------------------------------------------------');

disp('[3] FINAL TARGET STATE - t = T_sim');

disp('Final target Euler angles [deg]:');
fprintf('Roll  = %.3f deg\n', rad2deg(roll_f));
fprintf('Pitch = %.3f deg\n', rad2deg(pitch_f));
fprintf('Yaw   = %.3f deg\n', rad2deg(yaw_f));

disp(' ');
disp('Final q_BL = BODY w.r.t. LVLH:');
disp(q_Body_LVLH_final);

disp('Final q_LI = LVLH w.r.t. ECI:');
disp(q_LI_final);

disp('Final q_BI = BODY w.r.t. ECI:');
disp(q_final_expected);

disp('Final relative angular velocity [rad/s]:');
disp(omega_rel_final);

disp('Final BODY angular velocity w.r.t. ECI [rad/s]:');
disp(omega_final_expected);

disp('--------------------------------------------------------------');

disp('[4] VALIDATION');

fprintf('Initial quaternion norm = %.12f\n', q_init_norm);
fprintf('Final quaternion norm   = %.12f\n', q_final_expected_norm);

fprintf('Initial |omega_BI|      = %.8f rad/s\n', omega_init_mag);
fprintf('Final |omega_BI|        = %.8f rad/s\n', omega_final_mag);

disp(' ');
disp('==============================================================');
disp('Parameters successfully loaded into the MATLAB Workspace.');
disp('Ready to run the Simulink model.');
disp('==============================================================');
