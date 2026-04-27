%% ========================================================================
%  MASTER SCRIPT: ORBITAL MANEUVERS & PLANE CHANGE (UNPERTURBED & PERTURBED)
%  ========================================================================
clc; clear; close all;

%% 1. COSTANTI E DATI DI INPUT
% Costanti Fisiche
mu = 398600.4418;               % Parametro gravitazionale Terrestre [km^3/s^2]
R_Earth = 6371.0;               % Raggio medio terrestre [km]
J2 = 1.08263e-3;                % Costante di perturbazione armonica zonale J2

% Parametri Orbita Iniziale (Parking Orbit)
h = 709;                        % Quota [km]
R1 = R_Earth + h;               % Raggio orbita iniziale [km]
Vc = sqrt(mu/R1);               % Velocità circolare [km/s]

inc1_deg = 50.0;                % Inclinazione iniziale [deg]
RAAN1_deg = 170.0;              % RAAN iniziale [deg]

% Parametri Orbita Finale (Target - es. Landsat 8)
inc2_deg = 98.2;                % Inclinazione target [deg]
RAAN2_deg = 175.012;            % RAAN target [deg]

% Conversioni in radianti
inc1 = deg2rad(inc1_deg);
inc2 = deg2rad(inc2_deg);
DeltaOmega = deg2rad(RAAN2_deg - RAAN1_deg);

%% 2. GEOMETRIA DEL CAMBIO DI PIANO (Triangolo Sferico)
% Angolo totale di cambio di piano (theta)
theta_rad = acos(cos(inc1)*cos(inc2) + sin(inc1)*sin(inc2)*cos(DeltaOmega));
theta_deg = rad2deg(theta_rad);

% Argomenti di latitudine per i nodi di intersezione
u1_rad = acos((-cos(inc2) + cos(theta_rad)*cos(inc1))/(sin(theta_rad)*sin(inc1)));
u2_rad = acos((cos(inc1) - cos(theta_rad)*cos(inc2))/(sin(theta_rad)*sin(inc2)));

% Tempo di attesa per raggiungere il nodo geometrico
TauWait_Secs = u1_rad * sqrt(R1^3/mu);
TauWait_Days = TauWait_Secs / 86400;

fprintf('========================================================\n');
fprintf('  1. GEOMETRIA DEL CAMBIO DI PIANO (IDEALE)\n');
fprintf('========================================================\n');
fprintf('Angolo di cambio piano (Theta): %.4f deg\n', theta_deg);
fprintf('Anomalia nodo di manovra (u1):  %.4f deg\n', rad2deg(u1_rad));
fprintf('Tempo attesa nodo (Tau_wait):   %.4f sec (%.4f giorni)\n\n', TauWait_Secs, TauWait_Days);


%% 3. STRATEGIA 1: SINGLE IMPULSE MANEUVER
DeltaV_1Imp = 2 * Vc * sin(theta_rad/2);

% Componenti VNB per GMAT (Spinta all'indietro per variare direzione + Spinta laterale)
DV_1I_V = Vc * cos(theta_rad) - Vc;
DV_1I_N = Vc * sin(theta_rad);

fprintf('========================================================\n');
fprintf('  2. STRATEGIA 1-IMPULSE\n');
fprintf('========================================================\n');
fprintf('Delta V Totale:      %.4f km/s\n', DeltaV_1Imp);
fprintf('-> Componente V:     %.4f km/s\n', DV_1I_V);
fprintf('-> Componente N:     %.4f km/s\n\n', DV_1I_N);


%% 4. STRATEGIA 2: RESTRICTED 3-IMPULSE MANEUVER
% Definizione Funzione Obiettivo (Delta V totale in funzione di R_apogeo)
Vtot_R3I = @(Rb) 2*( sqrt(mu*(2/R1 - 2/(R1+Rb))) - sqrt(mu/R1) ) + ...
                 2*sqrt(mu*(2/Rb - 2/(R1+Rb))) * sin(theta_rad/2);

% Ottimizzazione (Ricerca del minimo)
Rb_R3I = fminbnd(Vtot_R3I, 1.01*R1, 40*R1);
DeltaVtot_R3I = Vtot_R3I(Rb_R3I);

% Calcolo velocità e tempi ottimali
atr_R3I = (R1 + Rb_R3I)/2;
Vp_R3I = sqrt(mu*(2/R1 - 1/atr_R3I));  % Velocità al perigeo dell'ellisse di trasf.
Va_R3I = sqrt(mu*(2/Rb_R3I - 1/atr_R3I)); % Velocità all'apogeo dell'ellisse di trasf.

TauMan_R3I_Days = pi*sqrt(atr_R3I^3/mu)/86400; % Tempo Perigeo -> Apogeo

% Delta V Singoli (Moduli)
DV1_R3I = Vp_R3I - Vc;                              % Innalzamento apogeo
DV2_R3I = 2 * Va_R3I * sin(theta_rad/2);            % Cambio piano all'apogeo
DV3_R3I = DV1_R3I;                                  % Circolarizzazione finale (Simmetrico)

% Componenti VNB
DV1_R3I_V = DV1_R3I;           DV1_R3I_N = 0;
DV2_R3I_V = Va_R3I*(cos(theta_rad)-1); DV2_R3I_N = -Va_R3I*sin(theta_rad);
DV3_R3I_V = -DV3_R3I;          DV3_R3I_N = 0; 

fprintf('========================================================\n');
fprintf('  3. STRATEGIA RESTRICTED 3-IMPULSE\n');
fprintf('========================================================\n');
fprintf('Raggio di Apogeo (Rb): %.2f km\n', Rb_R3I);
fprintf('Delta V Totale:        %.4f km/s\n', DeltaVtot_R3I);
fprintf('  - Burn 1 (V, N):     [%.4f,  %.4f] km/s\n', DV1_R3I_V, DV1_R3I_N);
fprintf('  - Burn 2 (V, N):     [%.4f, %.4f] km/s\n', DV2_R3I_V, DV2_R3I_N);
fprintf('  - Burn 3 (V, N):     [%.4f,  %.4f] km/s\n\n', DV3_R3I_V, DV3_R3I_N);


%% 5. STRATEGIA 3: GENERAL 3-IMPULSE MANEUVER
% Inizializzazione Grid Search
N_Rb = 500; N_alpha1 = 500;
Rb_vec = linspace(1.01*R1, 40*R1, N_Rb);
alpha1_vec = linspace(0, theta_rad/2, N_alpha1);
DV_grid = NaN(N_alpha1, N_Rb);

% Calcolo Matrice dei Delta V
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

% Estrazione del Minimo Globale
[DeltaVtot_G3I, idx] = min(DV_grid(:));
[j_opt, i_opt] = ind2sub(size(DV_grid), idx);

Rb_G3I = Rb_vec(i_opt);
alpha1_G3I = alpha1_vec(j_opt);
alpha2_G3I = theta_rad - 2*alpha1_G3I;

% Parametri Ottimali
atr_G3I = (R1 + Rb_G3I)/2;
TauMan_G3I_Days = pi*sqrt(atr_G3I^3/mu)/86400;
Vp_G3I = sqrt(mu*(2/R1 - 1/atr_G3I));
Va_G3I = sqrt(mu*(2/Rb_G3I - 1/atr_G3I));

% Componenti VNB (Teorema di Carnot vettoriale)
DV1_G3I_V = Vp_G3I*cos(alpha1_G3I) - Vc;
DV1_G3I_N = Vp_G3I*sin(alpha1_G3I);

DV2_G3I_V = Va_G3I*(cos(alpha2_G3I) - 1);
DV2_G3I_N = -Va_G3I*sin(alpha2_G3I);

DV3_G3I_V = Vc*cos(alpha1_G3I) - Vp_G3I;
DV3_G3I_N = -Vc*sin(alpha1_G3I);

fprintf('========================================================\n');
fprintf('  4. STRATEGIA GENERAL 3-IMPULSE\n');
fprintf('========================================================\n');
fprintf('Raggio di Apogeo (Rb): %.2f km\n', Rb_G3I);
fprintf('Alpha 1 (Perigeo):     %.4f deg\n', rad2deg(alpha1_G3I));
fprintf('Alpha 2 (Apogeo):      %.4f deg\n', rad2deg(alpha2_G3I));
fprintf('Delta V Totale:        %.4f km/s\n', DeltaVtot_G3I);
fprintf('  - Burn 1 (V, N):     [%.4f,  %.4f] km/s\n', DV1_G3I_V, DV1_G3I_N);
fprintf('  - Burn 2 (V, N):     [%.4f, %.4f] km/s\n', DV2_G3I_V, DV2_G3I_N);
fprintf('  - Burn 3 (V, N):     [%.4f, %.4f] km/s\n\n', DV3_G3I_V, DV3_G3I_N);


%% 6. EFFETTO PERTURBATIVO J2 E TARGETING
% La Terra schiacciata fa ruotare il piano orbitale. Se ci mettiamo ore/giorni
% a fare la manovra, il RAAN target si sarà "spostato".

% Derivata del RAAN dell'orbita target (deg/s e deg/giorno)
RAANdot_rad_s = -(3/2)*J2*(R_Earth/R1)^2 * sqrt(mu/R1^3) * cos(inc2);
RAANdot_deg_day = RAANdot_rad_s * (180/pi) * 86400;

% Calcolo dei Tempi di Volo Totali (Attesa del nodo + 2 rami di ellisse)
TimeOfFlight_R3I_Days = TauWait_Days + 2 * TauMan_R3I_Days;
TimeOfFlight_G3I_Days = TauWait_Days + 2 * TauMan_G3I_Days;

% Calcolo del "Nuovo Target" RAAN (Kompensazione deriva)
% In GMAT punterai a questo valore per atterrare sull'orbita corretta alla fine.
RAAN_target_R3I = mod(RAAN2_deg + RAANdot_deg_day * TimeOfFlight_R3I_Days, 360);
RAAN_target_G3I = mod(RAAN2_deg + RAANdot_deg_day * TimeOfFlight_G3I_Days, 360);

fprintf('========================================================\n');
fprintf('  5. PERTURBAZIONI (EFFETTO J2) & GMAT TARGETS\n');
fprintf('========================================================\n');
fprintf('Deriva RAAN Target:    %.4f deg/giorno\n\n', RAANdot_deg_day);

fprintf('Per la Restricted 3-Impulse in GMAT:\n');
fprintf('  - Tempo di Volo:     %.4f giorni\n', TimeOfFlight_R3I_Days);
fprintf('  - RAAN da puntare:   %.4f deg\n\n', RAAN_target_R3I);

fprintf('Per la General 3-Impulse in GMAT:\n');
fprintf('  - Tempo di Volo:     %.4f giorni\n', TimeOfFlight_G3I_Days);
fprintf('  - RAAN da puntare:   %.4f deg\n', RAAN_target_G3I);
fprintf('========================================================\n');