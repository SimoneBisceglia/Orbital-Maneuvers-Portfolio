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
% LVLH convention used in this simulation (Right-Handed):
%   X = forward / along-track direction (+v)
%   Y = opposite to orbit normal direction (-h)
%   Z = nadir direction (-r)
%
% Here the chosen convention gives the orbital rate around the LVLH Y axis.
omega_orb_LVLH = [0;
                 -omega_orb_mag;
                  0];

%% ========================================================================
% 3. SPACECRAFT RIGID-BODY PARAMETERS
% =========================================================================
% Principal moments of inertia [kg*m^2]
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
% Initial attitude: Perfectly aligned with LVLH (NADIR pointing)
% Euler sequence: 3-2-1 (Yaw-Pitch-Roll)
% =========================================================================
%% 4.1 Initial Euler angles w.r.t. LVLH
roll_init  = deg2rad(0.0);
pitch_init = deg2rad(0.0);
yaw_init   = deg2rad(0.0);

%% 4.2 Initial BODY-to-LVLH quaternion q_BL_init (Half angles)
cy0 = cos(yaw_init   / 2);
sy0 = sin(yaw_init   / 2);
cp0 = cos(pitch_init / 2);
sp0 = sin(pitch_init / 2);
cr0 = cos(roll_init  / 2);
sr0 = sin(roll_init  / 2);

% Hamilton quaternion, scalar first:
q_Body_LVLH_init = [ ...
    cr0*cp0*cy0 + sr0*sp0*sy0;
    sr0*cp0*cy0 - cr0*sp0*sy0;
    cr0*sp0*cy0 + sr0*cp0*sy0;
    cr0*cp0*sy0 - sr0*sp0*cy0 ];
q_Body_LVLH_init = q_Body_LVLH_init / norm(q_Body_LVLH_init);

%% 4.3 Initial LVLH-to-ECI quaternion q_LI_init
q_LVLH_ECI_init = [sqrt(2)/2;
                   0;
                   -sqrt(2)/2;
                   0];
q_LVLH_ECI_init = q_LVLH_ECI_init / norm(q_LVLH_ECI_init);

%% 4.4 Initial absolute quaternion q_BI_init (Manual composition)
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
q_init = q_init / norm(q_init);

%% 4.5 Initial relative angular velocity
omega_rel_init = [0; 0; 0];

%% 4.6 Initial DCM: LVLH -> BODY (Full angles)
cy0_mat = cos(yaw_init);
sy0_mat = sin(yaw_init);
cp0_mat = cos(pitch_init);
sp0_mat = sin(pitch_init);
cr0_mat = cos(roll_init);
sr0_mat = sin(roll_init);

C_BL_init = [ ...
    cp0_mat*cy0_mat,                                   cp0_mat*sy0_mat,                                  -sp0_mat;
    sr0_mat*sp0_mat*cy0_mat - cr0_mat*sy0_mat,         sr0_mat*sp0_mat*sy0_mat + cr0_mat*cy0_mat,         sr0_mat*cp0_mat;
    cr0_mat*sp0_mat*cy0_mat + sr0_mat*sy0_mat,         cr0_mat*sp0_mat*sy0_mat - sr0_mat*cy0_mat,         cr0_mat*cp0_mat ];

%% 4.7 Initial BODY angular velocity w.r.t. ECI
omega_init = C_BL_init * omega_orb_LVLH;

%% ========================================================================
% 5. SENSOR PARAMETERS
% =========================================================================
gyro_bias_true = [ 0.001; -0.002; 0.0015 ];
gyro_arw = 1e-4;
st_noise_sigma = 1e-3;

%% ========================================================================
% 6. PD CONTROLLER GAINS
% =========================================================================
omega_n = 0.1;
zeta    = 1.0;
Kp = I_tensor * omega_n^2;
Kd = 2 * zeta * I_tensor * omega_n;
Kp = diag(diag(Kp));
Kd = diag(diag(Kd));

%% ========================================================================
% 7. FINAL TARGET ATTITUDE
% =========================================================================
%% 7.1 Final target Euler angles
roll_f  = deg2rad(30.0);
pitch_f = deg2rad(45.0);
yaw_f   = deg2rad(20.0);

%% 7.2 Final BODY-to-LVLH quaternion q_BL_final (Half angles)
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
q_Body_LVLH_final = q_Body_LVLH_final / norm(q_Body_LVLH_final);

%% 7.3 Final LVLH-to-ECI quaternion
theta_orb_final = -omega_orb_mag * T_sim;
q_orb_rot_final = [ ...
    cos(theta_orb_final/2);
    0;
    sin(theta_orb_final/2);
    0 ];
q_orb_rot_final = q_orb_rot_final / norm(q_orb_rot_final);

%% 7.4 Final LVLH w.r.t. ECI quaternion
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
q_LI_final = q_LI_final / norm(q_LI_final);

%% 7.5 Final absolute BODY-to-ECI quaternion q_BI_final (Manual composition)
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
q_final_expected = q_final_expected / norm(q_final_expected);

%% 7.6 FINAL RELATIVE ANGULAR VELOCITY
omega_rel_final = [0; 0; 0];

%% 7.7 Final DCM: LVLH -> BODY (Full angles)
cy_f_mat = cos(yaw_f);
sy_f_mat = sin(yaw_f);
cp_f_mat = cos(pitch_f);
sp_f_mat = sin(pitch_f);
cr_f_mat = cos(roll_f);
sr_f_mat = sin(roll_f);

C_BL_final = [ ...
    cp_f_mat*cy_f_mat,                                  cp_f_mat*sy_f_mat,                                 -sp_f_mat;
    sr_f_mat*sp_f_mat*cy_f_mat - cr_f_mat*sy_f_mat,     sr_f_mat*sp_f_mat*sy_f_mat + cr_f_mat*cy_f_mat,     sr_f_mat*cp_f_mat;
    cr_f_mat*sp_f_mat*cy_f_mat + sr_f_mat*sy_f_mat,     cr_f_mat*sp_f_mat*sy_f_mat - sr_f_mat*cy_f_mat,     cr_f_mat*cp_f_mat ];

%% 7.8 FINAL BODY ANGULAR VELOCITY w.r.t. ECI
omega_final_expected = C_BL_final * omega_orb_LVLH;

%% ========================================================================
% 8. ADDITIONAL VALIDATION VARIABLES
% =========================================================================
q_init_norm          = norm(q_init);
q_final_expected_norm = norm(q_final_expected);
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
