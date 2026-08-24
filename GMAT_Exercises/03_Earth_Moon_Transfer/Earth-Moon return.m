% =========================================================================
% ORBITAL MANEUVERS: COMPLETE EARTH - MOON - EARTH MISSION
% =========================================================================
clc; clear; close all;

%% 1. CONSTANTS AND INPUT DATA
mu_E = 398600.4418;             % Earth gravitational parameter [km^3/s^2]
R_E = 6371.0;                   % Earth mean radius [km]
mu_M = 4902.8000;               % Moon gravitational parameter [km^3/s^2]
R_M = 1737.4;                   % Moon mean radius [km]
D_EM = 384400.0;                % Mean Earth-Moon distance [km]

% Parking Orbits
h_LEO = 300;                    
r1 = R_E + h_LEO;               
v_LEO = sqrt(mu_E / r1);        

h_LLO = 100;                    
r2 = R_M + h_LLO;               
v_LLO = sqrt(mu_M / r2);        

fprintf('========================================================\n');
fprintf('  1. CIRCULAR PARKING ORBITS VELOCITIES\n');
fprintf('========================================================\n');
fprintf('Earth Orbit (LEO) at %d km:      %.4f km/s\n', h_LEO, v_LEO);
fprintf('Lunar Orbit (LLO)    at %d km:      %.4f km/s\n\n', h_LLO, v_LLO);

%% 2. PHASE 1: TRANS-LUNAR INJECTION (TLI)
r_apogeo_tx = D_EM;
a_tx = (r1 + r_apogeo_tx) / 2;  
v_tx_perigeo = sqrt(mu_E * (2/r1 - 1/a_tx));
v_tx_apogeo = sqrt(mu_E * (2/r_apogeo_tx - 1/a_tx));
DeltaV_TLI = v_tx_perigeo - v_LEO;

fprintf('========================================================\n');
fprintf('  2. OUTBOUND PHASE (TLI TRANSFER ELLIPSE)\n');
fprintf('========================================================\n');
fprintf('Perigee Velocity (Earth Departure):   %.4f km/s\n', v_tx_perigeo);
fprintf('Apogee Velocity (Moon Arrival):       %.4f km/s\n', v_tx_apogeo);
fprintf('-> Delta-V TLI (Burn 1):              %.4f km/s\n\n', DeltaV_TLI);

%% 3. PHASE 2: LUNAR ORBIT INSERTION (LOI)
v_Moon = sqrt(mu_E / D_EM); 
v_inf_in = v_Moon - v_tx_apogeo; 
v_iperbole_arr = sqrt(v_inf_in^2 + 2*mu_M / r2);
DeltaV_LOI = v_iperbole_arr - v_LLO;

fprintf('========================================================\n');
fprintf('  3. CAPTURE PHASE (LUNAR SPHERE OF INFLUENCE)\n');
fprintf('========================================================\n');
fprintf('Moon Orbital Velocity:                %.4f km/s\n', v_Moon);
fprintf('Inbound Hyperbolic Excess (v_inf):    %.4f km/s\n', v_inf_in);
fprintf('Hyperbola Velocity at Perilune:       %.4f km/s\n', v_iperbole_arr);
fprintf('-> Delta-V LOI (Burn 2 - Braking):    %.4f km/s\n\n', DeltaV_LOI);

%% 4. PHASE 3: TRANS-EARTH INJECTION (TEI) AND RE-ENTRY
% Set Entry Interface altitude to 122 km
h_entry = 122; 
r_perigeo_ritorno = R_E + h_entry;
a_return = (r_perigeo_ritorno + D_EM) / 2;
v_ret_apogeo = sqrt(mu_E * (2/D_EM - 1/a_return));
v_inf_out = v_Moon - v_ret_apogeo; 
v_iperbole_part = sqrt(v_inf_out^2 + 2*mu_M / r2);
DeltaV_TEI = v_iperbole_part - v_LLO;

% ATMOSPHERIC RE-ENTRY VELOCITY CALCULATION
v_ret_perigeo = sqrt(mu_E * (2/r_perigeo_ritorno - 1/a_return));

fprintf('========================================================\n');
fprintf('  4. RETURN PHASE (TEI RETURN ELLIPSE)\n');
fprintf('========================================================\n');
fprintf('Outbound Hyperbolic Excess (v_inf):   %.4f km/s\n', v_inf_out);
fprintf('Hyperbola Velocity at Perilune:       %.4f km/s\n', v_iperbole_part);
fprintf('Apogee Velocity (Moon Departure):     %.4f km/s\n', v_ret_apogeo);
fprintf('-> Delta-V TEI (Burn 3 - Escape):     %.4f km/s\n', DeltaV_TEI);
fprintf('Perigee Velocity (Earth Re-entry):    %.4f km/s\n\n', v_ret_perigeo);

%% 5. TOTAL BUDGET
DeltaV_Tot = DeltaV_TLI + DeltaV_LOI + DeltaV_TEI;

fprintf('========================================================\n');
fprintf('  TOTAL MISSION DELTA-V BUDGET \n');
fprintf('========================================================\n');
fprintf('TOTAL MISSION DELTA-V:                %.4f km/s\n', DeltaV_Tot);
fprintf('========================================================\n');
