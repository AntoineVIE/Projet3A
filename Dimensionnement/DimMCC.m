f = 2
rho = 1000
h_ang = 65e-3
l_ang = 25e-3

p_piece = 18e-3
L = p_piece*16
S_ang = h_ang*L

demi_grand_axe  = L
demi_petit_axe   = 2*L/pi

dx = demi_petit_axe 
dy = abs(L-sqrt((1-dx^2/(4*L^2/pi^2))*L^2))
d_theta = atan2(dy,dx) %Angle entre l'origine et extrémité queue
dl = d_theta*sqrt((demi_grand_axe^2+...
    demi_petit_axe^2)/2) %Longueur déplacement queue
%par rapport à l'origine

v_eau_deplace = dl*S_ang
Cx = 1.55 %Surdimensionnement du coef de trainée
F = (1/2)*rho*(f*dx)^2*L*h_ang %Force de trainée
Cmot = F*L %Calcul du couple moteur nécessaire

%%
dx_reel = 10e-2 %déplacement horizontal maximal imposé
dy_reel = L-sqrt((1-dx_reel^2/(4*L^2/pi^2))*L^2)
R_enrouleur = 2.5e-2
phi_mot = dy_reel/R_enrouleur

2*f*phi_mot*60/(2*pi)

%%
phi_reel = 1
R_enrouleur = 2.5-2
dy_reel = R_enrouleur*phi_reel
dx_reel = sqrt((1 - (L-dy_reel)^2/L^2)*4*L^2/pi^2)