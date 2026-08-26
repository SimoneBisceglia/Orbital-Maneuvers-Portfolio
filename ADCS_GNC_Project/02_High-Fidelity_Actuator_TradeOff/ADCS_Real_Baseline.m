% =========================================================================
% ADCS / GNC SIMULATION: HIGH-FIDELITY SETUP (REAL ACTUATORS & ENVIRONMENT)
% =========================================================================
clc;
clear;
close all;
disp('--- ADCS HIGH-FIDELITY SIMULATOR INITIALIZATION ---');

%% ========================================================================
% 1. SIMULATION PARAMETERS
% =========================================================================
T_sim  = 600.0;       % Total simulation duration [s]
dt_sim = 0.1;         % Simulation / integration step [s]

%% ========================================================================
% 2. ENVIRONMENTAL AND ORBITAL PARAMETERS
% =========================================================================
mu_E    = 398600.4418;      % Earth's gravitational parameter [km^3/s^2]
R_E     = 6371.0;           % Earth's mean radius [km]
h_orbit = 800.0;            % Orbit altitude [km]
r_orbit = R_E + h_orbit;    % Orbital radius [km]
omega_orb_mag = sqrt(mu_E / r_orbit^3);   % Circular orbital angular velocity [rad/s]
omega_orb_LVLH = [0; -omega_orb_mag; 0];  % Expressed in LVLH

%% ========================================================================
% 3. SPACECRAFT RIGID-BODY PARAMETERS
% =========================================================================
I_xx = 15.0; I_yy = 15.0; I_zz = 10.0;
I_tensor = diag([I_xx, I_yy, I_zz]);
I_inv = inv(I_tensor);

%% ========================================================================
% 4. INITIAL ATTITUDE AND ANGULAR VELOCITY (Nadir Pointing)
% =========================================================================
roll_init = 0; pitch_init = 0; yaw_init = 0;
cy0 = cos(yaw_init / 2);   sy0 = sin(yaw_init / 2);
cp0 = cos(pitch_init / 2); sp0 = sin(pitch_init / 2);
cr0 = cos(roll_init / 2);  sr0 = sin(roll_init / 2);

q_Body_LVLH_init = [ ...
    cr0*cp0*cy0 + sr0*sp0*sy0;
    sr0*cp0*cy0 - cr0*sp0*sy0;
    cr0*sp0*cy0 + sr0*cp0*sy0;
    cr0*cp0*sy0 - sr0*sp0*cy0 ];
q_Body_LVLH_init = q_Body_LVLH_init / norm(q_Body_LVLH_init);

q_LVLH_ECI_init = [sqrt(2)/2; 0; sqrt(2)/2; 0];
q_LVLH_ECI_init = q_LVLH_ECI_init / norm(q_LVLH_ECI_init);

q_init = [ ...
    q_Body_LVLH_init(1)*q_LVLH_ECI_init(1) - q_Body_LVLH_init(2)*q_LVLH_ECI_init(2) - q_Body_LVLH_init(3)*q_LVLH_ECI_init(3) - q_Body_LVLH_init(4)*q_LVLH_ECI_init(4);
    q_Body_LVLH_init(1)*q_LVLH_ECI_init(2) + q_Body_LVLH_init(2)*q_LVLH_ECI_init(1) + q_Body_LVLH_init(3)*q_LVLH_ECI_init(4) - q_Body_LVLH_init(4)*q_LVLH_ECI_init(3);
    q_Body_LVLH_init(1)*q_LVLH_ECI_init(3) - q_Body_LVLH_init(2)*q_LVLH_ECI_init(4) + q_Body_LVLH_init(3)*q_LVLH_ECI_init(1) + q_Body_LVLH_init(4)*q_LVLH_ECI_init(2);
    q_Body_LVLH_init(1)*q_LVLH_ECI_init(4) + q_Body_LVLH_init(2)*q_LVLH_ECI_init(3) - q_Body_LVLH_init(3)*q_LVLH_ECI_init(2) + q_Body_LVLH_init(4)*q_LVLH_ECI_init(1) ];
q_init = q_init / norm(q_init);

omega_rel_init = [0; 0; 0];
C_BL_init = [ ...
    cp0*cy0, cp0*sy0, -sp0;
    sr0*sp0*cy0 - cr0*sy0, sr0*sp0*sy0 + cr0*cy0, sr0*cp0;
    cr0*sp0*cy0 + sr0*sy0, cr0*sp0*sy0 - sr0*cy0, cr0*cp0 ];
omega_init = C_BL_init * omega_orb_LVLH;

%% ========================================================================
% 5. PD CONTROLLER GAINS
% =========================================================================
omega_n = 0.1;       
zeta    = 1.0;       
Kp = diag(diag(I_tensor * omega_n^2));
Kd = diag(diag(2 * zeta * I_tensor * omega_n));

%% ========================================================================
% 6. FINAL TARGET ATTITUDE (Off-Nadir)
% =========================================================================
roll_f  = deg2rad(30.0); pitch_f = deg2rad(45.0); yaw_f   = deg2rad(20.0);
cy_f = cos(yaw_f / 2);   sy_f = sin(yaw_f / 2);
cp_f = cos(pitch_f / 2); sp_f = sin(pitch_f / 2);
cr_f = cos(roll_f / 2);  sr_f = sin(roll_f / 2);

q_Body_LVLH_final = [ ...
    cr_f*cp_f*cy_f + sr_f*sp_f*sy_f;
    sr_f*cp_f*cy_f - cr_f*sp_f*sy_f;
    cr_f*sp_f*cy_f + sr_f*cp_f*sy_f;
    cr_f*cp_f*sy_f - sr_f*sp_f*cy_f ];
q_Body_LVLH_final = q_Body_LVLH_final / norm(q_Body_LVLH_final);

theta_orb_final = -omega_orb_mag * T_sim;
q_orb_rot_final = [cos(theta_orb_final/2); 0; sin(theta_orb_final/2); 0];
q_orb_rot_final = q_orb_rot_final / norm(q_orb_rot_final);

q_LI_final = [ ...
    q_orb_rot_final(1)*q_LVLH_ECI_init(1) - q_orb_rot_final(2)*q_LVLH_ECI_init(2) - q_orb_rot_final(3)*q_LVLH_ECI_init(3) - q_orb_rot_final(4)*q_LVLH_ECI_init(4);
    q_orb_rot_final(1)*q_LVLH_ECI_init(2) + q_orb_rot_final(2)*q_LVLH_ECI_init(1) + q_orb_rot_final(3)*q_LVLH_ECI_init(4) - q_orb_rot_final(4)*q_LVLH_ECI_init(3);
    q_orb_rot_final(1)*q_LVLH_ECI_init(3) - q_orb_rot_final(2)*q_LVLH_ECI_init(4) + q_orb_rot_final(3)*q_LVLH_ECI_init(1) + q_orb_rot_final(4)*q_LVLH_ECI_init(2);
    q_orb_rot_final(1)*q_LVLH_ECI_init(4) + q_orb_rot_final(2)*q_LVLH_ECI_init(3) - q_orb_rot_final(3)*q_LVLH_ECI_init(2) + q_orb_rot_final(4)*q_LVLH_ECI_init(1) ];
q_LI_final = q_LI_final / norm(q_LI_final);

q_final_expected = [ ...
    q_Body_LVLH_final(1)*q_LI_final(1) - q_Body_LVLH_final(2)*q_LI_final(2) - q_Body_LVLH_final(3)*q_LI_final(3) - q_Body_LVLH_final(4)*q_LI_final(4);
    q_Body_LVLH_final(1)*q_LI_final(2) + q_Body_LVLH_final(2)*q_LI_final(1) + q_Body_LVLH_final(3)*q_LI_final(4) - q_Body_LVLH_final(4)*q_LI_final(3);
    q_Body_LVLH_final(1)*q_LI_final(3) - q_Body_LVLH_final(2)*q_LI_final(4) + q_Body_LVLH_final(3)*q_LI_final(1) + q_Body_LVLH_final(4)*q_LI_final(2);
    q_Body_LVLH_final(1)*q_LI_final(4) + q_Body_LVLH_final(2)*q_LI_final(3) - q_Body_LVLH_final(3)*q_LI_final(2) + q_Body_LVLH_final(4)*q_LI_final(1) ];
q_final_expected = q_final_expected / norm(q_final_expected);

omega_rel_final = [0; 0; 0];
C_BL_final = [ ...
    cp_f*cy_f, cp_f*sy_f, -sp_f;
    sr_f*sp_f*cy_f - cr_f*sy_f, sr_f*sp_f*sy_f + cr_f*cy_f, sr_f*cp_f;
    cr_f*sp_f*cy_f + sr_f*sy_f, cr_f*sp_f*sy_f - sr_f*cy_f, cr_f*cp_f ];
omega_final_expected = C_BL_final * omega_orb_LVLH;

%% ========================================================================
% 7. ENVIRONMENTAL DISTURBANCES
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
% 8. SENSOR MODELS (NAVIGATION NOISE)
% =========================================================================
st_noise_sigma = 5e-4;      
gyro_noise_sigma = 1e-5;    
gyro_bias_true = [1e-4; -2e-4; 1.5e-4]; 

%% ========================================================================
% 9. ACTUATOR MODELS & BUDGET PARAMETERS
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
% 10. PRE-CALCULATIONS FOR VALIDATION OUTPUT
% =========================================================================
q_init_norm           = norm(q_init);
q_final_expected_norm = norm(q_final_expected);
omega_init_mag        = norm(omega_init);
omega_final_mag       = norm(omega_final_expected);

%% ========================================================================
% 11. CONSOLE OUTPUT (BASELINE + HIGH-FIDELITY REPORT)
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