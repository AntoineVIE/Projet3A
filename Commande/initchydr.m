% initchydr.m
%
% Ce programme iniialise les param�tres du mod�le de la centrale hydro�lectrique


% Mod�le et param�tres de la centrale
% ===================================
Ta=10; Tw=1;
TI=0.3; kw=0.5;
[a,b,c,d]=linmod('chydr_s');
centrale=ss(a,b,c,d);
G=tf(centrale); zpk(G)
[ztbo,pbo,k] = ss2zp(a,b,c,d,1)
usat=100;			% saturation ouverture vanne

