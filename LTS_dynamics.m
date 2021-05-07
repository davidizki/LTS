function [dx,g_neq] = LTS_dynamics(x,u,p,t,data)
%% LTS_dynamics
% AUTHOR:
% David Izquierdo
% 
% FUNCTION: 
% Compute the derivatives of the state variables and the valus of the path constraints
% 
% SYNTAX:  
% [dx] = Dynamics(x,u,p,t,data)	(Dynamics Only)
% [dx,g_eq] = Dynamics(x,u,p,t,data)   (Dynamics and Eqaulity Path Constraints)
% [dx,g_neq] = Dynamics(x,u,p,t,data)   (Dynamics and Inqaulity Path Constraints)
% [dx,g_eq,g_neq] = Dynamics(x,u,p,t,data)   (Dynamics, Equality and Ineqaulity Path Constraints)
% 
% INPUTS:
% x: state vector
% u: input
% p: parameter
% t: time
% data: structured variable containing the values of additional data used inside the function
% 
% OUTPUTS:
% dx: time derivative of x
% g_eq: constraint function for equality constraints
% g_neq: constraint function for inequality constraints
%
% STRUCTURE:
% 0. UNFOLD INPUTS
% 1. PREPARE VARIABLES
% 2. EOMs - EQUATIONS OF MOTION
% 3. CONSTRAINTS
% 4. OUTPUTS

%% 0. UNFOLD INPUTS
auxdata = data.auxdata;

s = x(:,1); % "s" is the TIME and "t" is the CENTRELINE POSITION (to keep the convention of "t" -> integration variable)
n = x(:,2);
xi = x(:,3);
u = x(:,4);
v = x(:,5);
omega = x(:,6);

delta = u(:,1);
theta_t = u(:,2);
theta_b = u(:,3);

%% 1. PREPARE VARIABLES
% 1.1 Calculate forces at the tyres
% Weight distribution
% Static
F.fr.static = auxdata.W.*auxdata.b/auxdata.w/2;
F.fl.static = F.fr.static;
F.rr.static = auxdata.W.*auxdata.a/auxdata.w/2;
F.rl.static = F.rr.static;

% Tractive
% Fxr_t = theta_t.*auxdata.Teng_max.*auxdata.R;

% Braking
% 

% Tyres
mu_xmax = interp1(x,v,xq,'linear','extrap');

% 1.2 Calculate aerodynamic forces
Faz = 1/2.*auxdata.rho.*auxdata.ClA.*(u.^2 + v.^2);
Fax = -1/2.*auxdata.rho.*auxdata.CdA.*(u.^2 + v.^2);

% Temp=auxdata.Ts+H*auxdata.dTdH;
% pressure=auxdata.ps*(Temp./auxdata.Ts).^(-auxdata.g/auxdata.dTdH/auxdata.R);
% rho=auxdata.rhos*(Temp./auxdata.Ts).^(-(auxdata.g/auxdata.dTdH/auxdata.R+1));
% [ V_cas ] = CAS2TAS( auxdata.kappa, pressure, rho, auxdata.ps, auxdata.rhos, V_tas );
% 
% ap=auxdata.a1p.*V_cas.^2+auxdata.a2p.*V_cas+auxdata.a3p;
% bp=auxdata.b1p.*V_cas.^2+auxdata.b2p.*V_cas+auxdata.b3p;
% cp=auxdata.c1p.*V_cas.^2+auxdata.c2p.*V_cas+auxdata.c3p;
% Pmax=ap.*H.^2+bp.*H+cp;
% 
% P=(Pmax-auxdata.Pidle).*throttle+auxdata.Pidle;
% J=60*V_tas/auxdata.nprop/auxdata.Dprop;
% ita=-0.13289*J.^6+1.2536*J.^5-4.8906*J.^4+10.146*J.^3-11.918*J.^2+7.6740*J-1.3452;
% Thrust=2*745.6*ita.*P./V_tas;
% 
% % Calculate aerodynamic forces
% cl=auxdata.clalpha.*(alpha-auxdata.alpha0)*180/pi;
% L=0.5.*cl.*rho.*V_tas.^2.*auxdata.S;
% cd=auxdata.cd0+auxdata.k_cd*cl.^2;
% Drag=0.5.*cd.*rho.*V_tas.^2.*auxdata.S;

%% 2. EOMs - EQUATIONS OF MOTION
n_dot = (1 - n.*auxdata.trackInterp.C(t)).*tan(alpha);
% TAS_dot=(Thrust-Drag-W.*sin(gamma)).*auxdata.g./W;
% gamma_dot=(L.*cos(phi)+Thrust*sin(auxdata.alphat)-W.*cos(gamma))*auxdata.g./W./V_tas;
% H_dot=V_tas.*sin(gamma);
% x_dot=V_tas.*cos(gamma).*cos(chi);
% y_dot=V_tas.*cos(gamma).*sin(chi);
% chi_dot=L.*sin(phi)./cos(gamma)*auxdata.g./W./V_tas;
% W_dot= calcWeight(V_cas,H,throttle,auxdata.FFModel);

%% 3. CONSTRAINTS
c1 = n + auxdata.track_N; % cannot be lower than zero
c2 = -n + auxdata.track_N; % cannot be lower than zero
% c1=(npos-data.auxdata.obs_npos_1).^2+(epos-data.auxdata.obs_epos_1).^2-data.auxdata.obs_r_1.^2;
% c2=(npos-data.auxdata.obs_npos_2).^2+(epos-data.auxdata.obs_epos_2).^2-data.auxdata.obs_r_2.^2;
% c3=(npos-data.auxdata.obs_npos_3).^2+(epos-data.auxdata.obs_epos_3).^2-data.auxdata.obs_r_3.^2;
% c4=(npos-data.auxdata.obs_npos_4).^2+(epos-data.auxdata.obs_epos_4).^2-data.auxdata.obs_r_4.^2;
% c5=(npos-data.auxdata.obs_npos_5).^2+(epos-data.auxdata.obs_epos_5).^2-data.auxdata.obs_r_5.^2;
% c6=(npos-data.auxdata.obs_npos_6).^2+(epos-data.auxdata.obs_epos_6).^2-data.auxdata.obs_r_6.^2;
% c7=(npos-data.auxdata.obs_npos_7).^2+(epos-data.auxdata.obs_epos_7).^2-data.auxdata.obs_r_7.^2;
% c8=(npos-data.auxdata.obs_npos_8).^2+(epos-data.auxdata.obs_epos_8).^2-data.auxdata.obs_r_8.^2;
% c9=(npos-data.auxdata.obs_npos_9).^2+(epos-data.auxdata.obs_epos_9).^2-data.auxdata.obs_r_9.^2;
% c10=(npos-data.auxdata.obs_npos_10).^2+(epos-data.auxdata.obs_epos_10).^2-data.auxdata.obs_r_10.^2;

%% 4. OUTPUTS
dx = [n_dot];
g_neq=[c1,c2];
% dx = [H_dot, x_dot, y_dot, TAS_dot, gamma_dot, chi_dot, W_dot];
% g_neq=[c1,c2,c3,c4,c5,c6,c7,c8,c9,c10]/1e04;
