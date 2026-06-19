%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Université de Technologie de Compiègne (UTC)                       %
%% Département Génie Informatique (GI)                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mini-projet (SY15)                                                 %
%% "Planification et commande des robots mobiles"                     %
%% Théorème de stabilité de Lyapunov et méthode des cycles-limites    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dernière modification le 12/05/2020                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ModeleRobotUnicycle

global L d r 
global AccelerationLineaireMax AccelerationAngulaireMax
global Rrobot

% Paramètres géométriques du robot mobile
L = 1.2;     % Empattement (en mètre)
d = Rrobot;  % Largeur des essieux (en mètre) pour que ça colle à la définition du robot circulaire
r = 0.2;     % Rayon des roues (en mètre)

AccelerationLineaireMax = 1;
AccelerationAngulaireMax = 1;
