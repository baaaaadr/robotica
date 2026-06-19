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
 
function CommandeReelle = CommandeAttraction(Donnees)
%Donnees = [Ecart; ThetaTilde; Ex; Ey; ThetaReel];
Ecart = Donnees(1);
ThetaTilde = Donnees(2);
Ex = Donnees(3);
Ey = Donnees(4);
ThetaReel = Donnees(5);
 
%Vmax = 1; %1m/s -> 3.6km/h
Vmax = 0.5;
 
if (gt(Ecart,0.2)) % Pour appliquer cette commande uniquement quand le robot n'est pas tout pr�s de la cible
    %%La position du point effectif (dispos� sur le robot) � asservir sur la consigne
    l1 = 0.4; %Selon l'axe x
    l2 = 0;   %Selon l'axe y 
    K1 = 0.1; 
    K2 = 0.1; 
    V1 =  K1*Ex;
    V2 =  K2*Ey;
    %%
    %%Matrice cinematique du point Pt deporte de (l1, l2) (eq. 2 article [1])
    M = [cos(ThetaReel), -l1*sin(ThetaReel) - l2*cos(ThetaReel);
         sin(ThetaReel),  l1*cos(ThetaReel) - l2*sin(ThetaReel)];
    %%Inversion : (v, w) = M^-1 * (V1, V2) (loi Lyapunov, eq. 3 article [1])
    Commande = M \ [V1; V2];
    %%
    V = Vmax;
    W = Commande(2);
else
    V = 0;
    W = 0;
end
 
ValeurFonctionLyapunov = 0.5*(Ex^2+Ey^2);
CommandeReelle = [V, W, ValeurFonctionLyapunov];