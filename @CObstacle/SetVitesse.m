%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Polytech'Clermont-Ferrand                                          %    
%% Département Génie Electrique (GE3)                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TP Robotique mobile (AURO13)                                       %
%% Partie "Commande du robot et évitement d’obstacles"                %
%% Théorème de stabilité de Lyapunov et méthode des cycles-limites    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dernière modification le 14/12/2009                                %   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function SetVitesse(Obstacle, tVitesse)
%% Affectation de la vitesse de la classe CObstacle

%%Une vitesse est compposée d'un module |v| [m/s] et d'une orientation en
%%orientaion [rad entre -pi et pi]
Obstacle.Vitesse = tVitesse; %%tVitesse = [Amplitude Orientation]