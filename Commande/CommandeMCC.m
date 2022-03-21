%% Commande MCC continu

%Parametres
R = 2.0;
L = 0.5;
Km = sqrt(0.4407);
Kb = sqrt(0.4407);
Kf = Km*Kb;
J = 0.08;

% Espace d'état 
A = [   0       1           0;
        0       -Kf/J       1/J;
        0       -Km*Kb/L    -R/L;]
B = [0; 0; Km/L];
Bd = [0; 1/J; 0;];
C = [1 0 0];
D = 0


%Commandable / Observable
C0 = ctrb(A,B);
Obs = obsv(A,C);

rank_co = rank(C0) %Commandable ok
rank_obs = rank(Obs) %Observable ok

%Calcul de la fonction de transfert
s = tf('s')

G = tf(ss(A,B,C,D))

poles_g = pole(G)
zeros_g = zero(G)

%% Commande par retout d'état

Tc = 0.02
K = acker(A,B,[-1/Tc,-1/Tc, -1/Tc])
Kc = -inv(C*inv(A-B*K)*B)
Kd = Kc*(-C*inv(A-B*K)*Bd)
sys_bf = ss(A-B*K,B*Kc,C,0);

step(sys_bf)

%% Observateur
T0 = .05
L = acker(A',[1 1 0]',[-1/T0,-1/T0,-1/T0])'

%% Reconstruction de perturbation
Ad = 0; Bd = [0; 1/J; 0;]; Cd = 1; Dd = 0;

Ao = [A Bd*Cd;[0 0 0] Ad];
Bo = [B;0];
Co = [C 0];
Bdo = [0;1/J;0;0]
T0 = 1e-1
La = acker(Ao',[1 1 0 0]',[-1/T0,-1/T0, -1/T0,-1/T0])'

%% Commande LQI
Aa = [A zeros(3,1); -C 0]
Ba = [B;0];

Tc = 9e-2;

Q = (1/Tc)*inv(gramt(Aa,Ba,Tc))
R = 1
[Klqi,S,E] = lqi(ss(A,B,C,0),Q,R)
K_lqi = Klqi(1:3)
Ki = -Klqi(4)
Kc_lqi = -inv(C*inv(A-B*K_lqi)*B)
Kd_lqi = Kc_lqi*(-C*inv(A-B*K_lqi)*Bd)
%sys = ss([A-B*K_lqi,B*Ki;-C 0],[0;0;1],[C 0],0)
%step(sys)



%% Vector of input values
Vect = []
i = 0
for t=0:0.5:10
    t
    i = i+1
    if mod(i,2) == 0
        Vect(i) = -1
    else
        Vect(i) = 1
    end

end

Vect2 = -Vect

%% Discrete system
%CNA Bloqueur d'ordre 0
Ac = A; Bc = B; Cc = C; Dc = 0;

Te = 0.05;
sys_d = c2d(ss(A,B,C,D),Te,'zoh') %Système discret 

syspc = d2c(sys_d,'tustin') %Système pseudo-continu méthode tustin

%% LQI commande pseudo-continuous system

Aa_pc = [syspc.a zeros(3,1); -syspc.c 0]
Ba_pc = [syspc.b;0];

Tc = 0.5;

Q = (1/Tc)*inv(gramt(Aa_pc,Ba_pc,Tc))
R = 1
[Klqi_pc,S,E] = lqi(syspc,Q,R)
K_lqi_pc = Klqi_pc(1:3)
Ki_d = -Klqi_pc(4)

K_d_lqi = inv((2/Te)+K_lqi_pc*sys_d.b)*K_lqi_pc*(sys_d.a+eye(3))

%% Observateur discret avec reconstruction de pertrubation
Ad_d = 0; Bd_d = [0; 1/J; 0;]; 
Cd_d = 1; Dd_d = 0;

Ao = [sys_d.a Bd_d*Cd_d;[0 0 0] Ad_d];
Bo = [sys_d.b;0];
Co = [sys_d.c 0];
Bdo = [0;1/J;0;0]
T0 = 10
La = acker(Ao',[1 1 0 0]',[-1/T0,-1/T0, -1/T0,-1/T0])'


%% RST
%Conception d'un régulateur
Tc = 0.2; Tf = 1e-1;
[Bmat,Amat] = tfdata(G,'v')
A_barre =conv(Amat,[1 0]);

%polynome de commande

Crst = ppcom(Bmat,Amat,Tc)

%polynome de filtrage 

Frst = ppfil(Bmat,A_barre,Tf)

%bezout

[Srst,Rrst] = bezout(Bmat,A_barre,conv(Crst,Frst))
Srst = conv(Srst,[1,0])
%F = conv(F,[0,1])
Trst = (1/8.207)*Crst(4)*Frst

Krst=tf(Rrst,Srst)



%% RST Discret
%Conception d'un régulateur
Tc = 0.3; Tf = 0.3
%Parametres


[Bmat,Amat] = tfdata(G,'v')
A_barre =conv(Amat,[1 0]);

%polynome de commande

Crst = ppcom(Bmat,Amat,Tc)

%polynome de filtrage 

Frst = ppfil(Bmat,A_barre,Tf)

%bezout

[Srst,Rrst] = bezout(Bmat,A_barre,conv(Crst,Frst))
Srst = conv(Srst,[1,0])
%F = conv(F,[0,1])
Trst = (1/8.207)*Crst(4)*Frst

%From continuous to discrete
ConsPol = c2d(tf(Trst,Frst),Te,'tustin')
SatPol = c2d(tf(Frst-Srst,Frst),Te,'tustin')
RetPol = c2d(tf(Rrst,Frst),Te,'tustin')
Process = c2d(tf(Bmat,Amat),Te,'tustin')

[ConsPolNum,ConsPolDen] = tfdata(ConsPol,'v')
[SatPolNum,SatPolDen] = tfdata(SatPol,'v')
[RetPolNum,RetPolDen] = tfdata(RetPol,'v')
[ProcessPolNum,ProcessPolDen] = tfdata(Process,'v')

%%
%% RST Discret
%Conception d'un régulateur
Tc = 1e-2; Tf = 1e-3; Te = 1e-3
%Parametres
kt = 1.1459
ke = 1.1459
R = 2
L = 0.25
J = 0.4

G2 = tf(kt,[0 L*J R*J kt*ke]) 

[Bmat,Amat] = tfdata(G2,'v')
A_barre =conv(Amat,[1 0]);

%polynome de commande

Crst = ppcom(Bmat,Amat,Tc)

%polynome de filtrage 

Frst = ppfil(Bmat,A_barre,Tf)

%bezout

[Srst,Rrst] = bezout(Bmat,A_barre,conv(Crst,Frst))
Srst = conv(Srst,[1,0])
%F = conv(F,[0,1])
Trst = Crst(3)*Frst

%From continuous to discrete
ConsPol = c2d(tf(Trst,Frst),Te,'tustin')
SatPol = c2d(tf(Frst-Srst,Frst),Te,'tustin')
RetPol = c2d(tf(Rrst,Frst),Te,'tustin')
Process = c2d(tf(Bmat,Amat),Te,'tustin')

[ConsPolNum,ConsPolDen] = tfdata(ConsPol,'v')
[SatPolNum,SatPolDen] = tfdata(SatPol,'v')
[RetPolNum,RetPolDen] = tfdata(RetPol,'v')
[ProcessPolNum,ProcessPolDen] = tfdata(Process,'v')