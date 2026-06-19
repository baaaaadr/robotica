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
 
function [PositionsObstacles, CentreMasseObstacleEnglobant, RayonObstacleEnglobant]  = GenerationPositionsAleatoiresObstacles(NombreObstacles, Intervalle)
%Intervalle=[Xmin Xmax Ymin Ymax] -> C'est l'intervalle de recherche
 
Matrice = rand(NombreObstacles, 2);
Xmin = Intervalle(1);
Xmax = Intervalle(2);
Ymin = Intervalle(3);
Ymax = Intervalle(4);
xmin=min(Matrice(:,1));
xmax=max(Matrice(:,1));
ymin=min(Matrice(:,2));
ymax=max(Matrice(:,2));
B1 = (Xmin*xmax-Xmax*xmin)/(xmax-xmin);
A1 = (Xmax-B1)/xmax;
B2 = (Ymin*ymax-Ymax*ymin)/(ymax-ymin);
A2 = (Ymax-B2)/ymax;
 
Matrice(:,1) = A1*Matrice(:,1)+B1; 
Matrice(:,2) = A2*Matrice(:,2)+B2;
 
%Pour classer l'indice des obstacles par ordre croissant selon les x
Matrice(:,1)=sort(Matrice(:,1),1);
 
PositionsObstacles = Matrice;
 
%% Obtention du centre de masse des obstacles
CentreMasseObstacleEnglobant = [mean(Matrice(:,1)), mean(Matrice(:,2))];
 
%% Obtention de la position de l'obstacle le plus éloigné
DistanceObstaclePlusEloigne = DistanceEuclidienne(CentreMasseObstacleEnglobant,  PositionsObstacles(1,:));
for i=2:NombreObstacles
 DistanceObstacle = DistanceEuclidienne(CentreMasseObstacleEnglobant,  PositionsObstacles(i,:));
 if DistanceObstacle > DistanceObstaclePlusEloigne   
    DistanceObstaclePlusEloigne = DistanceObstacle;
 end
end
RayonObstacleEnglobant = DistanceObstaclePlusEloigne;
 
%%DessinEnveloppeObstacle_EtMargeSecurite(CentreMasseObstacleEnglobant(1),CentreMasseObstacleEnglobant(2),RayonObstacleEnglobant, 0, 1);


