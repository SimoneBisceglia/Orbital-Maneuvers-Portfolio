% =========================================================================
% ORBIT MAINTENANCE ANALYTICAL CALCULATION (LEO & GEO)
% =========================================================================
clear; clc; close all;

mu = 398600.4418; % Gravitational parameter [km^3/s^2]
Re = 6378.1363;   % Earth radius [km]
deg2rad = pi/180;

disp('--- STATION KEEPING DELTA-V CALCULATION ---');

%% 1. DRAG MAKE-UP MANEUVER (LEO)
h_leo = 400;                 
a_nom_leo = Re + h_leo;      
V_nom_leo = sqrt(mu/a_nom_leo); 

% The spacecraft decays down to (a_nom - 5 km). We raise it back to (a_nom + 5 km).
delta_a_tol = 5; 
delta_a_tot = 2 * delta_a_tol;

% Approximated Hohmann transfer
dV_drag_tot = V_nom_leo * (delta_a_tot / a_nom_leo);
dV_drag_1 = dV_drag_tot / 2; % Tangential (V) to raise apogee
dV_drag_2 = dV_drag_tot / 2; % Tangential (V) to circularize

fprintf('\n[LEO] Drag Make-up (Hohmann Jump from %g km to %g km altitude):\n', h_leo-delta_a_tol, h_leo+delta_a_tol);
fprintf('Delta-V Burn 1 (Tangential, V): +%.5f km/s\n', dV_drag_1);
fprintf('Delta-V Burn 2 (Tangential, V): +%.5f km/s\n', dV_drag_2);

%% 2. NORTH-SOUTH STATION KEEPING (GEO - Inclination Control)
a_geo = 42164;               
V_geo = sqrt(mu/a_geo);      

delta_i_deg = 0.43; % Maximum inclination drift
delta_i_rad = delta_i_deg * deg2rad;

% Pure plane change variation
dV_NS = 2 * V_geo * sin(delta_i_rad / 2);

fprintf('\n[GEO] North-South Station Keeping (Inclination correction of %g degrees):\n', delta_i_deg);
fprintf('Delta-V Burn (Normal, N): +/-%.5f km/s\n', dV_NS);

%% 3. EAST-WEST STATION KEEPING (GEO - Longitude Drift Control)
delta_a_ew = 2; % Temporary SMA shift to induce corrective drift

dV_EW_1 = V_geo * (delta_a_ew / (2 * a_geo)); 

fprintf('\n[GEO] East-West Station Keeping (SMA shift of %g km for drift correction):\n', delta_a_ew);
fprintf('Delta-V Trigger Burn (Tangential, V): %.5f km/s\n', dV_EW_1);
fprintf('Delta-V Arrest Burn (Tangential, V):  %.5f km/s\n', -dV_EW_1);
disp('-----------------------------------------');