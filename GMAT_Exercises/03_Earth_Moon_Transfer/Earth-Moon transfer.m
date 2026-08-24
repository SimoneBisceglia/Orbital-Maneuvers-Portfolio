%% ========================================================================
%  ORBITAL MANEUVERS: EARTH - MOON TRANSFER (PATCHED CONICS)
%  ========================================================================
clc; clear; close all;

%% 1. CONSTANTS AND INPUT DATA
% Earth Physical Constants
mu_E = 398600.4418;             % Earth gravitational parameter [km^3/s^2]
R_E = 6371.0;                   % Earth mean radius [km]

% Moon Physical Constants
mu_M = 4902.8000;               % Moon gravitational parameter [km^3/s^2]
R_M = 1737.4;                   % Moon mean radius [km]
D_EM = 384400.0;                % Mean Earth-Moon distance [km]

% Initial Orbit Parameters (LEO - Low Earth Orbit)
h_LEO = 300;                    % Earth parking altitude [km]
r1 = R_E + h_LEO;               % Initial orbit radius [km]
v_LEO = sqrt(mu_E / r1);        % LEO circular velocity [km/s]

% Final Orbit Parameters (LLO - Low Lunar Orbit)
h_LLO = 100;                    % Lunar parking altitude [km]
r2 = R_M + h_LLO;               % Target lunar orbit radius [km]
v_LLO = sqrt(mu_M / r2);        % LLO circular velocity [km/s]

fprintf('========================================================\n');
fprintf('  1. PARKING ORBITS PARAMETERS\n');
fprintf('========================================================\n');
fprintf('LEO Velocity (Earth): %.4f km/s\n', v_LEO);
fprintf('LLO Velocity (Moon):  %.4f km/s\n\n', v_LLO);

%% 2. PHASE 1: TRANS-LUNAR INJECTION (TLI)
% We want a transfer ellipse departing from LEO and reaching the Moon.
% The apogee radius of the ellipse equals the Earth-Moon distance.
r_apogeo_tx = D_EM;
a_tx = (r1 + r_apogeo_tx) / 2;  % Semi-major axis of the transfer orbit

% Velocity calculation on the ellipse (Vis-viva equation)
v_tx_perigeo = sqrt(mu_E * (2/r1 - 1/a_tx));
v_tx_apogeo = sqrt(mu_E * (2/r_apogeo_tx - 1/a_tx));

% Departure maneuver Delta-V
DeltaV_TLI = v_tx_perigeo - v_LEO;

% Time of Flight (One-way, hence half orbital period)
TOF_sec = pi * sqrt(a_tx^3 / mu_E);
TOF_days = TOF_sec / 86400;

fprintf('========================================================\n');
fprintf('  2. EARTH DEPARTURE (TRANS-LUNAR INJECTION)\n');
fprintf('========================================================\n');
fprintf('Required perigee velocity: %.4f km/s\n', v_tx_perigeo);
fprintf('TLI Delta-V (Burn 1):      %.4f km/s\n', DeltaV_TLI);
fprintf('Time of Flight (TOF):      %.2f days\n\n', TOF_days);

%% 3. PHASE 2: MOON ARRIVAL AND LUNAR ORBIT INSERTION (LOI)
% Assume the Moon is on a circular orbit around the Earth
v_Moon = sqrt(mu_E / D_EM);     % Moon orbital velocity [km/s]

% When arriving at the transfer ellipse apogee, we are slower than the Moon.
% The Moon "catches up" and over-takes us. The relative velocity is:
v_inf = v_Moon - v_tx_apogeo;   % Hyperbolic excess velocity (V_infinity) [km/s]

% Now we enter the Lunar reference frame.
% We arrive via a hyperbola with V_inf at infinity and perilune radius r2 (100km altitude).
% Energy conservation for the lunar hyperbola:
v_iperbole_perilenio = sqrt(v_inf^2 + 2*mu_M / r2);

% Delta-V for capture (braking from hyperbolic trajectory to circular orbit)
DeltaV_LOI = v_iperbole_perilenio - v_LLO;

fprintf('========================================================\n');
fprintf('  3. MOON ARRIVAL (LUNAR ORBIT INSERTION)\n');
fprintf('========================================================\n');
fprintf('Moon Velocity:                   %.4f km/s\n', v_Moon);
fprintf('Hyperbolic Excess (V_inf):       %.4f km/s\n', v_inf);
fprintf('Hyperbola velocity at perilune:  %.4f km/s\n', v_iperbole_perilenio);
fprintf('LOI Delta-V (Burn 2 - Braking):  %.4f km/s\n\n', DeltaV_LOI);

%% 4. FINAL RESULTS
DeltaV_Tot = DeltaV_TLI + DeltaV_LOI;

fprintf('========================================================\n');
fprintf('  TOTAL DELTA-V BUDGET \n');
fprintf('========================================================\n');
fprintf('Departure Delta-V:  %.4f km/s\n', DeltaV_TLI);
fprintf('Capture Delta-V:    %.4f km/s\n', DeltaV_LOI);
fprintf('TOTAL DELTA-V:      %.4f km/s\n', DeltaV_Tot);
fprintf('========================================================\n');
