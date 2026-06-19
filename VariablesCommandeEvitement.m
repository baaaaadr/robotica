%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Universit� de Technologie de Compi�gne (UTC)                       %
%% D�partement G�nie Informatique (GI)                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mini-projet (SY15)                                                 %
%% "Planification et commande des robots mobiles"                     %
%% Th�or�me de stabilit� de Lyapunov et m�thode des cycles-limites    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Derni�re modification le 12/05/2020                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Donnees = VariablesCommandeEvitement(Data, IndiceObstaclePlusProche, DistanceRobotObstaclePlusProche, SensEvitementObstacle)
%Donnees = [Ecart; ThetaTilde; ThetaC_p]; %Ecart Robot-Cible,
%ThetaTilde=Theta_C-Theta; "SensEvitementObstacle" param�tre pour forcer ou pas le sens de l'evitement d'obstacle 

%%%%%%%%%%%%
global Xd Yd
global Obstacle
global RayonCycleLimite Mu
global IndiceObstaclePlusProche_Old

%Etat actuel du Robot Mobile
Xreel = Data(1);
Yreel = Data(2);
Theta = Data(3);

%%D�termination de la position relative du robot par rapport � l'obstacle (car le cycle limite est donn� par rapport
%%au centre de l'obstacle et non au centre du rep�re (0,0)
PositionObstacle = GetPosition(Obstacle(IndiceObstaclePlusProche));
Xrelatif = Xreel - PositionObstacle(1); %X robot-obstacle
Yrelatif = Yreel - PositionObstacle(2); %Y robot-obstacle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%M�thode pour le changenement de rep�res
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Y_D_O = Yd - PositionObstacle(2); %Diff�rence D : Point d�sir�e (target) et Yobst
X_D_O = Xd - PositionObstacle(1);

Alpha = atan2(Y_D_O, X_D_O);

%Calcul de la matrice de passage du rep�re obtstacle (R_O) au rep�re absolu (R_A)
%%Eq. (12) article [1] :
T_O_A = inv ([cos(Alpha), -sin(Alpha), 0, PositionObstacle(1);
              sin(Alpha),  cos(Alpha), 0, PositionObstacle(2);
              0,           0,          1, 0;
              0,           0,          0, 1]);

CoordonneeRepereObstacle = T_O_A * [Xreel; Yreel; 0; 1];

X_Prime = CoordonneeRepereObstacle(1); %Attention le "prime" ici ne veut pas dire d�riv�e
Y_Prime = CoordonneeRepereObstacle(2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Obtention du Rayon du cycle-limite � suivre
if (X_Prime <= 0) %Alors Attraction vers le cycle limite 
    RayonCycleLimite=GetRv(Obstacle(IndiceObstaclePlusProche))-0.3;
else %P�riode de l'extraction du cycle limite 
    RayonCycleLimite=RayonCycleLimite+0.03; %Pour que le changement de rayon soit en douceur. 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Partie pour forcer ou pas le sens de rotation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if SensEvitementObstacle ~= 0 %=> Qu'il faut forcer le sens de rotation
    Y_Prime = SensEvitementObstacle;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Calcul de l'orientation "ThetaC" � faire suivre par le robot pour suivre le cycle limite 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Mu = 0.2;
%%Eq. (5)/(6) article [1] combinees via sign(Y_Prime) (cf. Algorithm 2 ligne 12)
X_dot =  sign(Y_Prime) * Yrelatif + (Mu * Xrelatif * ((RayonCycleLimite^2) - (Xrelatif^2) - (Yrelatif^2)));
Y_dot = -sign(Y_Prime) * Xrelatif + (Mu * Yrelatif * ((RayonCycleLimite^2) - (Xrelatif^2) - (Yrelatif^2)));
ThetaC = atan2(Y_dot, X_dot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Proc�dure pour r�cup�rer ThetaC_p
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X0 = [X_dot, Y_dot];
[t,Xc] = ode23('EquationDiff_Tourbillon',[0, 0.2],X0);
ThetaC_p = atan2((Xc(2,2)-Xc(1,2)), (Xc(2,1)-Xc(1,1)));

%%%%
ThetaTilde=SoustractionAnglesAtan2(ThetaC, Theta);

%Param�tres de commande
Donnees = [ThetaTilde; ThetaC_p; sign(Y_Prime)];