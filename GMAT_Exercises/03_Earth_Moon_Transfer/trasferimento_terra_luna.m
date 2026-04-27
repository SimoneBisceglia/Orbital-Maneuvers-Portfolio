%% ========================================================================
%  ORBITAL MANEUVERS: TRASFERIMENTO TERRA - LUNA (PATCHED CONICS)
%  ========================================================================
clc; clear; close all;

%% 1. COSTANTI E DATI DI INPUT
% Costanti Fisiche Terra
mu_E = 398600.4418;             % Parametro gravitazionale Terrestre [km^3/s^2]
R_E = 6371.0;                   % Raggio medio terrestre [km]

% Costanti Fisiche Luna
mu_M = 4902.8000;               % Parametro gravitazionale Lunare [km^3/s^2]
R_M = 1737.4;                   % Raggio medio lunare [km]
D_EM = 384400.0;                % Distanza media Terra-Luna [km]

% Parametri Orbita Iniziale (LEO - Low Earth Orbit)
h_LEO = 300;                    % Quota di parcheggio terrestre [km]
r1 = R_E + h_LEO;               % Raggio orbita iniziale [km]
v_LEO = sqrt(mu_E / r1);        % Velocità circolare LEO [km/s]

% Parametri Orbita Finale (LLO - Low Lunar Orbit)
h_LLO = 100;                    % Quota di parcheggio lunare [km]
r2 = R_M + h_LLO;               % Raggio orbita lunare target [km]
v_LLO = sqrt(mu_M / r2);        % Velocità circolare LLO [km/s]

fprintf('========================================================\n');
fprintf('  1. PARAMETRI DELLE ORBITE DI PARCHEGGIO\n');
fprintf('========================================================\n');
fprintf('Velocita in LEO (Terra): %.4f km/s\n', v_LEO);
fprintf('Velocita in LLO (Luna):  %.4f km/s\n\n', v_LLO);

%% 2. FASE 1: TRANS-LUNAR INJECTION (TLI)
% Vogliamo un'ellisse di trasferimento che parta dalla LEO e arrivi alla Luna.
% Il raggio di apogeo dell'ellisse è pari alla distanza Terra-Luna.
r_apogeo_tx = D_EM;
a_tx = (r1 + r_apogeo_tx) / 2;  % Semiasse maggiore del trasferimento

% Calcolo velocità sull'ellisse (Equazione della vis-viva)
v_tx_perigeo = sqrt(mu_E * (2/r1 - 1/a_tx));
v_tx_apogeo = sqrt(mu_E * (2/r_apogeo_tx - 1/a_tx));

% Delta V della manovra di partenza
DeltaV_TLI = v_tx_perigeo - v_LEO;

% Tempo di volo (Solo andata, quindi metà periodo orbitale)
TOF_sec = pi * sqrt(a_tx^3 / mu_E);
TOF_days = TOF_sec / 86400;

fprintf('========================================================\n');
fprintf('  2. PARTENZA DALLA TERRA (TRANS-LUNAR INJECTION)\n');
fprintf('========================================================\n');
fprintf('Velocita necessaria al perigeo: %.4f km/s\n', v_tx_perigeo);
fprintf('Delta V TLI (Burn 1):           %.4f km/s\n', DeltaV_TLI);
fprintf('Tempo di Volo (TOF):            %.2f giorni\n\n', TOF_days);

%% 3. FASE 2: ARRIVO E LUNAR ORBIT INSERTION (LOI)
% Assumiamo che la Luna sia su un'orbita circolare attorno alla Terra
v_Moon = sqrt(mu_E / D_EM); % Velocità orbitale della Luna [km/s]

% Quando arriviamo all'apogeo dell'ellisse, andiamo più lenti della Luna.
% La Luna ci "tampona" e ci sorpassa. La velocità relativa tra noi e la Luna è:
v_inf = v_Moon - v_tx_apogeo; % Eccesso iperbolico (V_infinity) [km/s]

% Ora entriamo nel sistema di riferimento Lunare.
% Arriviamo con una iperbole che ha V_inf all'infinito e raggio di pericentro r2 (quota 100km)
% Conservazione dell'energia per l'iperbole lunare:
v_iperbole_perilenio = sqrt(v_inf^2 + 2*mu_M / r2);

% Delta V per la cattura (frenata da traiettoria iperbolica a circolare)
DeltaV_LOI = v_iperbole_perilenio - v_LLO;

fprintf('========================================================\n');
fprintf('  3. ARRIVO ALLA LUNA (LUNAR ORBIT INSERTION)\n');
fprintf('========================================================\n');
fprintf('Velocita della Luna:              %.4f km/s\n', v_Moon);
fprintf('Eccesso Iperbolico (V_inf):       %.4f km/s\n', v_inf);
fprintf('Vel. Iperbole al perilenio (r2):  %.4f km/s\n', v_iperbole_perilenio);
fprintf('Delta V LOI (Burn 2 - Frenata):   %.4f km/s\n\n', DeltaV_LOI);

%% 4. RISULTATI FINALI
DeltaV_Tot = DeltaV_TLI + DeltaV_LOI;

fprintf('========================================================\n');
fprintf('  BUDGET DELTA-V TOTALE \n');
fprintf('========================================================\n');
fprintf('Delta V Partenza:  %.4f km/s\n', DeltaV_TLI);
fprintf('Delta V Cattura:   %.4f km/s\n', DeltaV_LOI);
fprintf('DELTA V TOTALE:    %.4f km/s\n', DeltaV_Tot);
fprintf('========================================================\n');