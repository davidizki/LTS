function [dx,g_eq,g_neq] = LTS_dynamics(x,u,p,t,data)
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
u_controls = u; % to avoid naming conflict
refsize = ones(size(x(:,1)));

s = x(:,1); % "s" is the TIME and "t" is the CENTRELINE POSITION (to keep the convention of "t" -> integration variable)
n = x(:,2);
xi = x(:,3);
u = x(:,4);
v = x(:,5);
omega = x(:,6);

delta = u_controls(:,1);
% kappas are unfolded later on

%% 1.a TYRES NORMAL FORCES
% NOTATION: 1-2-3-4 == fr-fl-rr-rl
% 1.a.i Static weight distribution
tyres.Fz_static(:,1) = auxdata.W.*auxdata.b/auxdata.w/2.*refsize;
tyres.Fz_static(:,2) = tyres.Fz_static(:,1);
tyres.Fz_static(:,3) = auxdata.W.*auxdata.a/auxdata.w/2.*refsize;
tyres.Fz_static(:,4) = tyres.Fz_static(:,3);

% 1.a.ii Load transfers

% 1.a.iii Calculate aerodynamic forces
Df = 1/2.*auxdata.rho.*auxdata.ClA.*(u.^2 + v.^2);
D = -1/2.*auxdata.rho.*auxdata.CdA.*(u.^2 + v.^2);

tyres.Fz_aero(:,1) = Df.*auxdata.bA/auxdata.w/2.*refsize;
tyres.Fz_aero(:,2) = tyres.Fz_aero(:,1);
tyres.Fz_aero(:,3) = Df.*auxdata.aA/auxdata.w/2.*refsize;
tyres.Fz_aero(:,4) = tyres.Fz_aero(:,3);

% Collect all forces
tyres.Fz = tyres.Fz_static + tyres.Fz_aero;

%% 1.b TYRES FRICTION FORCES
% Tyres
% Maximum coefficients
tyres.mux_max = auxdata.mux_max_interp(tyres.Fz);
tyres.muy_max = auxdata.muy_max_interp(tyres.Fz);
tyres.kappa_max = auxdata.kappa_max_interp(tyres.Fz);
tyres.alpha_max = auxdata.alpha_max_interp(tyres.Fz);

% Slip ratios -> Directly from controls
tyres.kappa(:,1) = u_controls(:,2);
tyres.kappa(:,2) = u_controls(:,3);
tyres.kappa(:,3) = u_controls(:,4);
tyres.kappa(:,4) = u_controls(:,5);

% Slip angles
tyres.alpha(:,1) = atan2(sin(delta).*(omega.*auxdata.wf - u) + cos(delta).*(omega.*auxdata.a + v),...
    cos(delta).*(u - omega.*auxdata.wf) + sin(delta).*(omega.*auxdata.a + v));
tyres.alpha(:,2) = atan2(cos(delta).*(omega.*auxdata.a + v) - sin(delta).*(omega.*auxdata.wf + u),...
    cos(delta).*(u + omega.*auxdata.wf) + sin(delta).*(omega.*auxdata.a + v));
tyres.alpha(:,3) = atan2(v - omega.*auxdata.b, u - omega*auxdata.wr);
tyres.alpha(:,4) = atan2(v - omega.*auxdata.b, u + omega*auxdata.wr);
tyres.alpha = rad2deg(tyres.alpha);

% Spin speeds (from slip ratios and kinematics)
tyres.spin(:,1) = -(tyres.kappa(:,1) + 1).*(cos(delta).*(u - omega.*auxdata.wf) + sin(delta).*(v + omega.*auxdata.a))/auxdata.R;
tyres.spin(:,2) = -(tyres.kappa(:,2) + 1).*(cos(delta).*(u + omega.*auxdata.wf) + sin(delta).*(v + omega.*auxdata.a))/auxdata.R;
tyres.spin(:,3) = -(tyres.kappa(:,3) + 1).*(u - omega.*auxdata.wr)/auxdata.R;
tyres.spin(:,4) = -(tyres.kappa(:,4) + 1).*(u + omega.*auxdata.wr)/auxdata.R;

% Normalized slip angles and slip ratios
tyres.kappa_n = tyres.kappa./tyres.kappa_max;
tyres.alpha_n = tyres.alpha./tyres.alpha_max;

tyres.rho = sqrt(tyres.kappa_n.^2 + tyres.alpha_n.^2);

% Friction coefficients
tyres.mux = tyres.mux_max.*sin(auxdata.Qx.*atan2(pi.*tyres.rho,2.*atan(auxdata.Qx)));
tyres.muy = tyres.muy_max.*sin(auxdata.Qy.*atan2(pi.*tyres.rho,2.*atan(auxdata.Qy)));

% Friction forces
tyres.Fx = tyres.mux.*tyres.Fz.*tyres.kappa_n./tyres.rho;
tyres.Fy = tyres.muy.*tyres.Fz.*tyres.alpha_n./tyres.rho;

tyres.Fx(isnan(tyres.Fx)) = 0; % to avoid errors when rho=0 at a given tyre
tyres.Fy(isnan(tyres.Fy)) = 0;

% Total forces in car body-fixed reference frame
tyres.Fx_tot = cos(delta).*(tyres.Fx(:,1) + tyres.Fx(:,2)) - sin(delta).*(tyres.Fy(:,1) + tyres.Fy(:,2)) + ...
    tyres.Fx(:,3) + tyres.Fx(:,4) + D;
tyres.Fy_tot = cos(delta).*(tyres.Fy(:,1) + tyres.Fy(:,2)) + sin(delta).*(tyres.Fx(:,1) + tyres.Fx(:,2)) + ...
    tyres.Fy(:,3) + tyres.Fy(:,4);

% Tractive and braking forces are fixed by kappas control
% Tractive forces
% Fxr_t = theta_t.*auxdata.Teng_max./auxdata.R;
% Braking forces
% 

%% 2. EOMs - EQUATIONS OF MOTION
% t_dot = (u.*cos(xi) - v.*sin(xi))./(1 - n.*auxdata.trackInterp.C(t)); % == 1/Sf == dt/ds (== ds/dt in Perantoni notation)
Sf = (1 - n.*auxdata.trackInterp.C(t))./(u.*cos(xi) - v.*sin(xi));
n_dot = u.*sin(xi) + v.*cos(xi);
xi_dot = omega - auxdata.trackInterp.C(t)./Sf;
u_dot = omega.*v + tyres.Fx_tot/auxdata.M;
v_dot = -omega.*u + tyres.Fy_tot/auxdata.M;
Iz_omega_dot = auxdata.a.*(cos(delta).*(tyres.Fy(:,1) + tyres.Fy(:,2)) + sin(delta).*(tyres.Fx(:,1) + tyres.Fx(:,2))) + ...
    auxdata.wf.*(sin(delta).*tyres.Fy(:,1) - cos(delta).*tyres.Fx(:,1)) + ...
    auxdata.wf.*(-sin(delta).*tyres.Fy(:,2) + cos(delta).*tyres.Fx(:,2)) + ...
    -auxdata.wr.*tyres.Fx(:,3) + auxdata.wr.*tyres.Fx(:,4) - auxdata.b.*(tyres.Fy(:,3) + tyres.Fy(:,4));
omega_dot = Iz_omega_dot./auxdata.Iz;

dsdt = 1.*refsize;
dndt = n_dot.*Sf; % derivatives with respect to centreline coordinate (dnds in Perantoni notation)
dxidt = xi_dot.*Sf;
dudt = u_dot.*Sf;
dvdt = v_dot.*Sf;
domegadt = omega_dot.*Sf;


%% 3. CONSTRAINTS
% c1 = n + auxdata.track_N; % cannot be lower than zero % ENFORCED VIA n VARIABLE BOUNDS
% c2 = -n + auxdata.track_N; % cannot be lower than zero % ENFORCED VIA n VARIABLE BOUNDS

g1 = tyres.Fx(:,3) - tyres.Fx(:,4); % rear longitudinal forces must be the same -both if braking or if accelerating
g2 = tyres.Fx(:,1) - tyres.Fx(:,2); % front longitudinal forces

h1 = tyres.Fx(:,3) + tyres.Fx(:,4) - auxdata.Teng_max/auxdata.R; % tyres forces - engine max force <= 0 (tyres <= engine max)
h2 = -(tyres.Fx(:,1) + tyres.Fx(:,2)) - auxdata.Fbrk_max.*auxdata.kbb; % (front) tyres - brakes <= 0 (tyres <= brakes)
h3 = -(tyres.Fx(:,3) + tyres.Fx(:,4)) - auxdata.Fbrk_max.*(1-auxdata.kbb); % (rear) tyres - brakes <= 0 (tyres <= brakes)

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
dx = [dsdt dndt dxidt dudt dvdt domegadt];
g_eq = [0 0].*refsize; % [0 0].*refsize; // [g1 g2];
g_neq = [0 0 0].*refsize; % [0 0 0].*refsize; // % [h1 h2 h3];
