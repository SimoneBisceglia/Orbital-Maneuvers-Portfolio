%% ========================================================================
%  MASTER SCRIPT: ORBITAL MANEUVERS & PLANE CHANGE (UNPERTURBED & PERTURBED)
%  ========================================================================
clc; clear; close all;

%% 1. CONSTANTS AND INPUT DATA
% Physical Constants
mu = 398600.4418;               % Earth gravitational parameter [km^3/s^2]
R_Earth = 6371.0;               % Earth mean radius [km]
J2 = 1.08263e-3;                % J2 zonal harmonic perturbation constant

% Initial Orbit Parameters (Parking Orbit)
h = 709;                        % Altitude [km]
R1 = R_Earth + h;               % Initial orbit radius [km]
Vc = sqrt(mu/R1);               % Circular velocity [km/s]

inc1_deg = 50.0;                % Initial inclination [deg]
RAAN1_deg = 170.0;              % Initial RAAN [deg]

% Final Orbit Parameters (Target - e.g., Landsat 8)
inc2_deg = 98.2;                % Target inclination [deg]
RAAN2_deg = 175.012;            % Target RAAN [deg]

% Conversions to radians
inc1 = deg2rad(inc1_deg);
inc2 = deg2rad(inc2_deg);
DeltaOmega = deg2rad(RAAN2_deg - RAAN1_deg);

%% 2. PLANE CHANGE GEOMETRY (Spherical Triangle)
% Total plane change angle (theta)
theta_rad = acos(cos(inc1)*cos(inc2) + sin(inc1)*sin(inc2)*cos(DeltaOmega));
theta_deg = rad2deg(theta_rad);

% Arguments of latitude for intersection nodes
u1_rad = acos((-cos(inc2) + cos(theta_rad)*cos(inc1))/(sin(theta_rad)*sin(inc1)));
u2_rad = acos((cos(inc1) - cos(theta_rad)*cos(inc2))/(sin(theta_rad)*sin(inc2)));

% Wait time to reach the geometric node
TauWait_Secs = u1_rad * sqrt(R1^3/mu);
TauWait_Days = TauWait_Secs / 86400;

fprintf('========================================================\n');
fprintf('  1. PLANE CHANGE GEOMETRY (IDEAL)\n');
fprintf('========================================================\n');
fprintf('Plane change angle (Theta): %.4f deg\n', theta_deg);
fprintf('Maneuver node argument of latitude (u1):  %.4f deg\n', rad2deg(u1_rad));
fprintf('Node wait time (Tau_wait):  %.4f sec (%.4f days)\n\n', TauWait_Secs, TauWait_Days);


%% 3. STRATEGY 1: SINGLE IMPULSE MANEUVER
DeltaV_1Imp = 2 * Vc * sin(theta_rad/2);

% VNB components for GMAT (Backward thrust to change direction + Lateral thrust)
DV_1I_V = Vc * cos(theta_rad) - Vc;
DV_1I_N = Vc * sin(theta_rad);

fprintf('========================================================\n');
fprintf('  2. STRATEGY 1-IMPULSE\n');
fprintf('========================================================\n');
fprintf('Total Delta V:       %.4f km/s\n', DeltaV_1Imp);
fprintf('-> V Component:      %.4f km/s\n', DV_1I_V);
fprintf('-> N Component:      %.4f km/s\n\n', DV_1I_N);


%% 4. STRATEGY 2: RESTRICTED 3-IMPULSE MANEUVER
% Objective Function Definition (Total Delta V as a function of R_apogee)
Vtot_R3I = @(Rb) 2*( sqrt(mu*(2/R1 - 2/(R1+Rb))) - sqrt(mu/R1) ) + ...
                 2*sqrt(mu*(2/Rb - 2/(R1+Rb))) * sin(theta_rad/2);

% Optimization (Minimum search)
Rb_R3I = fminbnd(Vtot_R3I, 1.01*R1, 40*R1);
DeltaVtot_R3I = Vtot_R3I(Rb_R3I);

% Optimal velocities and times calculation
atr_R3I = (R1 + Rb_R3I)/2;
Vp_R3I = sqrt(mu*(2/R1 - 1/atr_R3I));  % Velocity at transfer ellipse perigee
Va_R3I = sqrt(mu*(2/Rb_R3I - 1/atr_R3I)); % Velocity at transfer ellipse apogee

TauMan_R3I_Days = pi*sqrt(atr_R3I^3/mu)/86400; % Perigee -> Apogee Time

% Single Delta Vs (Magnitudes)
DV1_R3I = Vp_R3I - Vc;                              % Apogee raising
DV2_R3I = 2 * Va_R3I * sin(theta_rad/2);            % Plane change at apogee
DV3_R3I = DV1_R3I;                                  % Final circularization (Symmetric)

% VNB Components
DV1_R3I_V = DV1_R3I;           DV1_R3I_N = 0;
DV2_R3I_V = Va_R3I*(cos(theta_rad)-1); DV2_R3I_N = -Va_R3I*sin(theta_rad);
DV3_R3I_V = -DV3_R3I;          DV3_R3I_N = 0; 

fprintf('========================================================\n');
fprintf('  3. STRATEGY RESTRICTED 3-IMPULSE\n');
fprintf('========================================================\n');
fprintf('Apogee Radius (Rb): %.2f km\n', Rb_R3I);
fprintf('Total Delta V:         %.4f km/s\n', DeltaVtot_R3I);
fprintf('  - Burn 1 (V, N):     [%.4f,  %.4f] km/s\n', DV1_R3I_V, DV1_R3I_N);
fprintf('  - Burn 2 (V, N):     [%.4f, %.4f] km/s\n', DV2_R3I_V, DV2_R3I_N);
fprintf('  - Burn 3 (V, N):     [%.4f,  %.4f] km/s\n\n', DV3_R3I_V, DV3_R3I_N);


%% 5. STRATEGY 3: GENERAL 3-IMPULSE MANEUVER
% Grid Search Initialization
N_Rb = 500; N_alpha1 = 500;
Rb_vec = linspace(1.01*R1, 40*R1, N_Rb);
alpha1_vec = linspace(0, theta_rad/2, N_alpha1);
DV_grid = NaN(N_alpha1, N_Rb);

% Delta V Matrix Calculation
for i = 1:N_Rb
    Rb_val = Rb_vec(i);
    atr_val = (R1 + Rb_val)/2;
    Vp_val = sqrt(mu*(2/R1 - 1/atr_val));
    Va_val = sqrt(mu*(2/Rb_val - 1/atr_val));
    
    for j = 1:N_alpha1
        a1 = alpha1_vec(j);
        a2 = theta_rad - 2*a1;
        if a2 >= 0
            dv1 = sqrt(Vc^2 + Vp_val^2 - 2*Vc*Vp_val*cos(a1));
            dv2 = 2 * Va_val * sin(a2/2);
            DV_grid(j, i) = 2*dv1 + dv2;
        end
    end
end

% Global Minimum Extraction
[DeltaVtot_G3I, idx] = min(DV_grid(:));
[j_opt, i_opt] = ind2sub(size(DV_grid), idx);

Rb_G3I = Rb_vec(i_opt);
alpha1_G3I = alpha1_vec(j_opt);
alpha2_G3I = theta_rad - 2*alpha1_G3I;

% Optimal Parameters
atr_G3I = (R1 + Rb_G3I)/2;
TauMan_G3I_Days = pi*sqrt(atr_G3I^3/mu)/86400;
Vp_G3I = sqrt(mu*(2/R1 - 1/atr_G3I));
Va_G3I = sqrt(mu*(2/Rb_G3I - 1/atr_G3I));

% VNB Components (Vector Law of Cosines)
DV1_G3I_V = Vp_G3I*cos(alpha1_G3I) - Vc;
DV1_G3I_N = Vp_G3I*sin(alpha1_G3I);

DV2_G3I_V = Va_G3I*(cos(alpha2_G3I) - 1);
DV2_G3I_N = -Va_G3I*sin(alpha2_G3I);

DV3_G3I_V = Vc*cos(alpha1_G3I) - Vp_G3I;
DV3_G3I_N = -Vc*sin(alpha1_G3I);

fprintf('========================================================\n');
fprintf('  4. STRATEGY GENERAL 3-IMPULSE\n');
fprintf('========================================================\n');
fprintf('Apogee Radius (Rb): %.2f km\n', Rb_G3I);
fprintf('Alpha 1 (Perigee):     %.4f deg\n', rad2deg(alpha1_G3I));
fprintf('Alpha 2 (Apogee):      %.4f deg\n', rad2deg(alpha2_G3I));
fprintf('Total Delta V:         %.4f km/s\n', DeltaVtot_G3I);
fprintf('  - Burn 1 (V, N):     [%.4f,  %.4f] km/s\n', DV1_G3I_V, DV1_G3I_N);
fprintf('  - Burn 2 (V, N):     [%.4f, %.4f] km/s\n', DV2_G3I_V, DV2_G3I_N);
fprintf('  - Burn 3 (V, N):     [%.4f, %.4f] km/s\n\n', DV3_G3I_V, DV3_G3I_N);


%% 6. J2 PERTURBATION EFFECT AND TARGETING
% The Earth's oblateness causes the orbital plane to precess. If the maneuver
% takes hours/days, the target RAAN will have "drifted".

% Target orbit RAAN derivative (deg/s and deg/day)
RAANdot_rad_s = -(3/2)*J2*(R_Earth/R1)^2 * sqrt(mu/R1^3) * cos(inc2);
RAANdot_deg_day = RAANdot_rad_s * (180/pi) * 86400;

% Total Flight Times Calculation (Node wait + 2 ellipse branches)
TimeOfFlight_R3I_Days = TauWait_Days + 2 * TauMan_R3I_Days;
TimeOfFlight_G3I_Days = TauWait_Days + 2 * TauMan_G3I_Days;

% Calculation of the "New Target" RAAN (Drift compensation)
% In GMAT you will target this value to land on the correct orbit at the end.
RAAN_target_R3I = mod(RAAN2_deg + RAANdot_deg_day * TimeOfFlight_R3I_Days, 360);
RAAN_target_G3I = mod(RAAN2_deg + RAANdot_deg_day * TimeOfFlight_G3I_Days, 360);

fprintf('========================================================\n');
fprintf('  5. PERTURBATIONS (J2 EFFECT) & GMAT TARGETS\n');
fprintf('========================================================\n');
fprintf('Target RAAN Drift:     %.4f deg/day\n\n', RAANdot_deg_day);

fprintf('For Restricted 3-Impulse in GMAT:\n');
fprintf('  - Flight Time:       %.4f days\n', TimeOfFlight_R3I_Days);
fprintf('  - Target RAAN:       %.4f deg\n\n', RAAN_target_R3I);

fprintf('For General 3-Impulse in GMAT:\n');
fprintf('  - Flight Time:       %.4f days\n', TimeOfFlight_G3I_Days);
fprintf('  - Target RAAN:       %.4f deg\n', RAAN_target_G3I);
fprintf('========================================================\n');