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

function CommandeReelle = CommandeEvitement(Donnees)
%%Donnees = [ThetaTilde; ThetaC_p];

ThetaTilde = Donnees(1);
ThetaC_p = Donnees(2);

%Vmax = 1; %1m/s -> 3.6km/h
Vmax = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Premier type de commande 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kp = 15; 
V = Vmax;
W = 0.2; %A changer

ValeurFonctionLyapunov = 0.5*rad2deg(ThetaTilde)^2/10; 

CommandeReelle = [V, W, ValeurFonctionLyapunov];
