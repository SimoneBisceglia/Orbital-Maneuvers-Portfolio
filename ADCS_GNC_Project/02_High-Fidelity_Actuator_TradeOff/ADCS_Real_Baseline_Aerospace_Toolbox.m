% =========================================================================
% ADCS / GNC SIMULATION: MASTER INITIALIZATION SCRIPT
% REAL BASELINE - NADIR TO OFF-NADIR SLEW
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
% =========================================================================
mu_E    = 398600.4418;      
R_E     = 6371.0;           
h_orbit = 800.0;            
r_orbit = R_E + h_orbit;    
omega_orb_mag = sqrt(mu_E / r_orbit^3);   

% LVLH convention used in this simulation (Right-Handed):
%   X = forward / along-track direction (+v)
%   Y = opposite to orbit normal direction (-h)
%   Z = nadir direction (-r)
omega_orb_LVLH = [0;
                 -omega_orb_mag;
                  0];

%% ========================================================================
% 3. SPACECRAFT RIGID-BODY PARAMETERS
% =========================================================================
I_xx = 15.0;
I_yy = 15.0;
I_zz = 10.0;
I_tensor = diag([I_xx, I_yy, I_zz]);
I_inv = inv(I_tensor);

%% ========================================================================
% 4. INITIAL ATTITUDE AND ANGULAR VELOCITY
% =========================================================================
%% 4.1 Initial Euler angles w.r.t. LVLH
roll_init  = deg2rad(0.0);
pitch_init = deg2rad(0.0);
yaw_init   = deg2rad(0.0);

%% 4.2 Initial DCM and Quaternion: LVLH -> BODY
cy0_mat = cos(yaw_init);   sy0_mat = sin(yaw_init);
cp0_mat = cos(pitch_init); sp0_mat = sin(pitch_init);
cr0_mat = cos(roll_init);  sr0_mat = sin(roll_init);

C_BL_init = [ ...
    cp0_mat*cy0_mat,                                   cp0_mat*sy0_mat,                                  -sp0_mat;
    sr0_mat*sp0_mat*cy0_mat - cr0_mat*sy0_mat,         sr0_mat*sp0_mat*sy0_mat + cr0_mat*cy0_mat,         sr0_mat*cp0_mat;
    cr0_mat*sp0_mat*cy0_mat + sr0_mat*sy0_mat,         cr0_mat*sp0_mat*sy0_mat - sr0_mat*cy0_mat,         cr0_mat*cp0_mat ];

q_Body_LVLH_init = dcm2quat(C_BL_init).';

%% 4.3 Initial LVLH-to-ECI quaternion q_LI_init (Derived from physical axes)
%
% At t = 0, we define the LVLH frame basis vectors in ECI coordinates:
%   X_LVLH = +Z_ECI       (velocity / along-track)
%   Y_LVLH = +Y_ECI       (-orbit normal / -h)
%   Z_LVLH = -X_ECI       (nadir / towards Earth)

X_LVLH_ECI = [ 0;  0;  1];
Y_LVLH_ECI = [ 0;  1;  0];
Z_LVLH_ECI = [-1;  0;  0];

% The matrix transforming from LVLH to ECI (C_IL) has these vectors as columns:
C_IL_init = [X_LVLH_ECI, Y_LVLH_ECI, Z_LVLH_ECI];

% The matrix transforming from ECI to LVLH (C_LI) is its transpose:
C_LI_init = C_IL_init.';

% Extract the initial LVLH w.r.t ECI quaternion
q_LVLH_ECI_init = dcm2quat(C_LI_init).';
q_LVLH_ECI_init = q_LVLH_ECI_init / norm(q_LVLH_ECI_init);

%% 4.4 Initial absolute BODY-to-ECI Quaternion (MATLAB Toolbox Extraction)
% Order reversed to match the manual Simulink Hamilton product convention (q_BL * q_LI)
C_BI_for_quat_init = C_LI_init * C_BL_init;
q_init = dcm2quat(C_BI_for_quat_init).';
q_init = q_init / norm(q_init);

%% 4.5 Initial angular velocities
omega_rel_init = [0; 0; 0];
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
Kp = diag(diag(I_tensor * omega_n^2));
Kd = diag(diag(2 * zeta * I_tensor * omega_n));

%% ========================================================================
% 7. FINAL TARGET ATTITUDE
% =========================================================================
%% 7.1 Final target Euler angles
roll_f  = deg2rad(30.0);
pitch_f = deg2rad(45.0);
yaw_f   = deg2rad(20.0);

%% 7.2 Final DCM and Quaternion: LVLH -> BODY
cy_f_mat = cos(yaw_f);   sy_f_mat = sin(yaw_f);
cp_f_mat = cos(pitch_f); sp_f_mat = sin(pitch_f);
cr_f_mat = cos(roll_f);  sr_f_mat = sin(roll_f);

C_BL_final = [ ...
    cp_f_mat*cy_f_mat,                                  cp_f_mat*sy_f_mat,                                 -sp_f_mat;
    sr_f_mat*sp_f_mat*cy_f_mat - cr_f_mat*sy_f_mat,     sr_f_mat*sp_f_mat*sy_f_mat + cr_f_mat*cy_f_mat,     sr_f_mat*cp_f_mat;
    cr_f_mat*sp_f_mat*cy_f_mat + sr_f_mat*sy_f_mat,     cr_f_mat*sp_f_mat*sy_f_mat - sr_f_mat*cy_f_mat,     cr_f_mat*cp_f_mat ];

q_Body_LVLH_final = dcm2quat(C_BL_final).';

%% 7.3 Final DCM and Quaternion: ECI -> LVLH
theta_orb_final = -omega_orb_mag * T_sim;
q_orb_rot_final = [cos(theta_orb_final/2); 0; sin(theta_orb_final/2); 0];
q_orb_rot_final = q_orb_rot_final / norm(q_orb_rot_final);

C_orb_final = quat2dcm(q_orb_rot_final.');
C_LI_final = C_orb_final * C_LI_init;
q_LI_final = dcm2quat(C_LI_final).';

%% 7.4 Final absolute BODY-to-ECI Quaternion (MATLAB Toolbox Extraction)
% Order reversed to match the manual Simulink Hamilton product convention (q_BL * q_LI)
C_BI_for_quat_final = C_LI_final * C_BL_final;
q_final_expected = dcm2quat(C_BI_for_quat_final).';

%% 7.5 Final angular velocities
omega_rel_final = [0; 0; 0];
omega_final_expected = C_BL_final * omega_orb_LVLH;

%% ========================================================================
% 8. ENVIRONMENTAL DISTURBANCES
% =========================================================================
rho_atm = 1e-13;            
Cd = 2.2;                   
Area_drag = 1.5;            
V_orb = sqrt(mu_E*1e9 / (r_orbit*1e3)); 
cp_cg_offset_drag = [0.05; 0.02; -0.03]; 
P_solar = 4.56e-6;          
Cr = 1.5;                   
Area_srp = 2.0;             
cp_cg_offset_srp = [0.0; 0.1; 0.05]; 

%% ========================================================================
% 9. SENSOR MODELS (NAVIGATION NOISE)
% =========================================================================
st_noise_sigma = 5e-4;      
gyro_noise_sigma = 1e-5;    
gyro_bias_true = [1e-4; -2e-4; 1.5e-4]; 

%% ========================================================================
% 10. ACTUATOR MODELS & BUDGET PARAMETERS
% =========================================================================
% Reaction Wheels
rw_max_torque = 0.05;       
rw_max_momentum = 1.5;      
rw_power_W_per_Nm = 50.0;   
% Magnetorquers
mtq_max_dipole = 15.0;      
M_earth_dipole = 7.96e15;   
mtq_power_W_per_Am2 = 0.5;  
% Thrusters
thruster_force = 1.0;       
thruster_arm = 0.5;         
thruster_max_torque = thruster_force * thruster_arm; 
thruster_Isp = 65.0;        
g0 = 9.80665;               
thruster_min_pulse = 0.05;  

%% ========================================================================
% 11. PRE-CALCULATIONS FOR VALIDATION OUTPUT
% =========================================================================
q_init_norm           = norm(q_init);
q_final_expected_norm = norm(q_final_expected);
omega_init_mag        = norm(omega_init);
omega_final_mag       = norm(omega_final_expected);

%% ========================================================================
% 12. CONSOLE OUTPUT (BASELINE + HIGH-FIDELITY REPORT)
% =========================================================================
disp(' ');
disp('==============================================================');
disp('          MISSION KINEMATICS & GNC HIGH-FIDELITY SUMMARY');
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
disp('Initial q_BI = BODY w.r.t. ECI:');
disp(q_init);
disp('Initial BODY angular velocity w.r.t. ECI [rad/s]:');
disp(omega_init);
disp('--------------------------------------------------------------');
disp('[3] FINAL TARGET STATE - t = T_sim');
disp('Final target Euler angles [deg]:');
fprintf('Roll  = %.3f deg\n', rad2deg(roll_f));
fprintf('Pitch = %.3f deg\n', rad2deg(pitch_f));
fprintf('Yaw   = %.3f deg\n', rad2deg(yaw_f));
disp('Final q_BI = BODY w.r.t. ECI:');
disp(q_final_expected);
disp('Final BODY angular velocity w.r.t. ECI [rad/s]:');
disp(omega_final_expected);
disp('--------------------------------------------------------------');
disp('[4] KINEMATICS VALIDATION');
fprintf('Initial quaternion norm = %.12f\n', q_init_norm);
fprintf('Final quaternion norm   = %.12f\n', q_final_expected_norm);
fprintf('Initial |omega_BI|      = %.8f rad/s\n', omega_init_mag);
fprintf('Final |omega_BI|        = %.8f rad/s\n', omega_final_mag);
disp('--------------------------------------------------------------');
disp('[5] ENVIRONMENTAL DISTURBANCES');
fprintf('Atmospheric Density (rho)   : %g kg/m^3\n', rho_atm);
fprintf('Drag Area & Cd              : %.2f m^2, Cd = %.1f\n', Area_drag, Cd);
fprintf('SRP Area & Reflectivity     : %.2f m^2, Cr = %.1f\n', Area_srp, Cr);
disp('--------------------------------------------------------------');
disp('[6] SENSOR NOISE PROFILE');
fprintf('Star Tracker Sigma          : %g rad\n', st_noise_sigma);
fprintf('Gyroscope Sigma             : %g rad/s\n', gyro_noise_sigma);
fprintf('Gyroscope Bias [X,Y,Z]      : [%g, %g, %g] rad/s\n', gyro_bias_true(1), gyro_bias_true(2), gyro_bias_true(3));
disp('--------------------------------------------------------------');
disp('[7] ACTUATOR CAPABILITIES (TRADE-OFF SETUP)');
fprintf('Reaction Wheels Max Torque  : %.3f Nm (Cost: %.1f W/Nm)\n', rw_max_torque, rw_power_W_per_Nm);
fprintf('Magnetorquers Max Dipole    : %.1f Am^2 (Cost: %.1f W/Am^2)\n', mtq_max_dipole, mtq_power_W_per_Am2);
fprintf('Thrusters Force & Isp       : %.1f N, %.1f s\n', thruster_force, thruster_Isp);
disp(' ');
disp('==============================================================');
disp('Go to Simulink.');
disp('==============================================================');