% =========================================================================
% ORBIT MAINTENANCE ANALYTICAL CALCULATION (LEO & GEO)
% =========================================================================
clear; clc; close all;

mu = 398600.4418; % Parametro gravitazionale [km^3/s^2]
Re = 6378.1363;   % Raggio terrestre [km]
deg2rad = pi/180;

disp('--- CALCOLO DELTA-V PER STATION KEEPING ---');

%% 1. DRAG MAKE-UP MANEUVER (LEO)
h_leo = 400;                 
a_nom_leo = Re + h_leo;      
V_nom_leo = sqrt(mu/a_nom_leo); 

% Il satellite decade fino a (a_nom - 5 km). Lo riportiamo a (a_nom + 5 km).
delta_a_tol = 5; 
delta_a_tot = 2 * delta_a_tol;

% Trasferimento di Hohmann approssimato
dV_drag_tot = V_nom_leo * (delta_a_tot / a_nom_leo);
dV_drag_1 = dV_drag_tot / 2; % Tangenziale (V) per alzare l'apogeo
dV_drag_2 = dV_drag_tot / 2; % Tangenziale (V) per circolarizzare

fprintf('\n[LEO] Drag Make-up (Hohmann Jump da %g km a %g km alt):\n', h_leo-delta_a_tol, h_leo+delta_a_tol);
fprintf('Delta-V Impulso 1 (Tangenziale, V): +%.5f km/s\n', dV_drag_1);
fprintf('Delta-V Impulso 2 (Tangenziale, V): +%.5f km/s\n', dV_drag_2);

%% 2. NORTH-SOUTH STATION KEEPING (GEO - Controllo Inclinazione)
a_geo = 42164;               
V_geo = sqrt(mu/a_geo);      

delta_i_deg = 0.43; % Deriva massima dell'inclinazione
delta_i_rad = delta_i_deg * deg2rad;

% Variazione di piano puro
dV_NS = 2 * V_geo * sin(delta_i_rad / 2);

fprintf('\n[GEO] North-South Station Keeping (Correzione Inclinazione di %g gradi):\n', delta_i_deg);
fprintf('Delta-V Impulso (Normale, N): +/-%.5f km/s\n', dV_NS);

%% 3. EAST-WEST STATION KEEPING (GEO - Controllo Deriva in Longitudine)
delta_a_ew = 2; % Shift temporaneo di SMA per indurre la deriva correttiva

dV_EW_1 = V_geo * (delta_a_ew / (2 * a_geo)); 

fprintf('\n[GEO] East-West Station Keeping (Shift SMA di %g km per deriva):\n', delta_a_ew);
fprintf('Delta-V Impulso Innesco (Tangenziale, V): %.5f km/s\n', dV_EW_1);
fprintf('Delta-V Impulso Arresto (Tangenziale, V): %.5f km/s\n', -dV_EW_1);
disp('-----------------------------------------');