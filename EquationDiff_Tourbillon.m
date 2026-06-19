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

function XP = EquationDiff_Tourbillon(t, Vecteur)
% XP = systBO(t,X)
% Cette fonction retourne le vecteur Xpoint du systeme

global RayonCycleLimite Mu

X  = Vecteur(1);
Y  = Vecteur(2);

Xp = Y + Mu*X*(RayonCycleLimite^2 - X^2 - Y^2);
Yp = -X + Mu*Y*(RayonCycleLimite^2 - X^2 - Y^2);

XP = [Xp;
      Yp ];
