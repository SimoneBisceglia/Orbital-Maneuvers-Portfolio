%% ========================================================================
%  ORBITAL MANEUVERS: MISSIONE COMPLETA TERRA - LUNA - TERRA
%  ========================================================================
clc; clear; close all;

%% 1. COSTANTI E DATI DI INPUT
mu_E = 398600.4418;             % Parametro gravitazionale Terrestre [km^3/s^2]
R_E = 6371.0;                   % Raggio medio terrestre [km]
mu_M = 4902.8000;               % Parametro gravitazionale Lunare [km^3/s^2]
R_M = 1737.4;                   % Raggio medio lunare [km]
D_EM = 384400.0;                % Distanza media Terra-Luna [km]

% Orbite di Parcheggio
h_LEO = 300;                    
r1 = R_E + h_LEO;               
v_LEO = sqrt(mu_E / r1);        

h_LLO = 100;                    
r2 = R_M + h_LLO;               
v_LLO = sqrt(mu_M / r2);        

fprintf('========================================================\n');
fprintf('  1. VELOCITA NELLE ORBITE DI PARCHEGGIO CIRCOLARI\n');
fprintf('========================================================\n');
fprintf('Orbita Terrestre (LEO) a %d km:      %.4f km/s\n', h_LEO, v_LEO);
fprintf('Orbita Lunare    (LLO) a %d km:      %.4f km/s\n\n', h_LLO, v_LLO);

%% 2. FASE 1: TRANS-LUNAR INJECTION (TLI)
r_apogeo_tx = D_EM;
a_tx = (r1 + r_apogeo_tx) / 2;  
v_tx_perigeo = sqrt(mu_E * (2/r1 - 1/a_tx));
v_tx_apogeo = sqrt(mu_E * (2/r_apogeo_tx - 1/a_tx));
DeltaV_TLI = v_tx_perigeo - v_LEO;

fprintf('========================================================\n');
fprintf('  2. FASE DI ANDATA (ELLISSE DI TRASFERIMENTO TLI)\n');
fprintf('========================================================\n');
fprintf('Velocita al Perigeo (Partenza Terra): %.4f km/s\n', v_tx_perigeo);
fprintf('Velocita all''Apogeo (Arrivo Luna):   %.4f km/s\n', v_tx_apogeo);
fprintf('-> Delta V TLI (Burn 1):              %.4f km/s\n\n', DeltaV_TLI);

%% 3. FASE 2: LUNAR ORBIT INSERTION (LOI)
v_Moon = sqrt(mu_E / D_EM); 
v_inf_in = v_Moon - v_tx_apogeo; 
v_iperbole_arr = sqrt(v_inf_in^2 + 2*mu_M / r2);
DeltaV_LOI = v_iperbole_arr - v_LLO;

fprintf('========================================================\n');
fprintf('  3. FASE DI CATTURA (SFERA D''INFLUENZA LUNARE)\n');
fprintf('========================================================\n');
fprintf('Velocita orbitale della Luna:         %.4f km/s\n', v_Moon);
fprintf('Eccesso Iperbolico arrivo (v_inf):    %.4f km/s\n', v_inf_in);
fprintf('Vel. sull''Iperbole al Perilenio:     %.4f km/s\n', v_iperbole_arr);
fprintf('-> Delta V LOI (Burn 2 - Frenata):    %.4f km/s\n\n', DeltaV_LOI);

%% 4. FASE 3: TRANS-EARTH INJECTION (TEI) E RIENTRO
% Impostiamo l'Entry Interface a 122 km
h_entry = 122; 
r_perigeo_ritorno = R_E + h_entry;
a_return = (r_perigeo_ritorno + D_EM) / 2;

v_ret_apogeo = sqrt(mu_E * (2/D_EM - 1/a_return));
v_inf_out = v_Moon - v_ret_apogeo; 
v_iperbole_part = sqrt(v_inf_out^2 + 2*mu_M / r2);
DeltaV_TEI = v_iperbole_part - v_LLO;

% ECCO IL CALCOLO DELLA VELOCITÀ DI RIENTRO NELL'ATMOSFERA
v_ret_perigeo = sqrt(mu_E * (2/r_perigeo_ritorno - 1/a_return));

fprintf('========================================================\n');
fprintf('  4. FASE DI RITORNO (ELLISSE DI RIENTRO TEI)\n');
fprintf('========================================================\n');
fprintf('Eccesso Iperbolico partenza (v_inf):  %.4f km/s\n', v_inf_out);
fprintf('Vel. sull''Iperbole al Perilenio:     %.4f km/s\n', v_iperbole_part);
fprintf('Velocita all''Apogeo (Partenza Luna): %.4f km/s\n', v_ret_apogeo);
fprintf('-> Delta V TEI (Burn 3 - Scappa):     %.4f km/s\n', DeltaV_TEI);
fprintf('Velocita al Perigeo (Rientro Terra):  %.4f km/s\n\n', v_ret_perigeo);

%% 5. BUDGET TOTALE
DeltaV_Tot = DeltaV_TLI + DeltaV_LOI + DeltaV_TEI;
fprintf('========================================================\n');
fprintf('  BUDGET DELTA-V MISSIONE COMPLETA \n');
fprintf('========================================================\n');
fprintf('DELTA V TOTALE DELLA MISSIONE:        %.4f km/s\n', DeltaV_Tot);
fprintf('========================================================\n');