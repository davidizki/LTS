function [dx] = LTS_dynamics(x,u,p,t,data)
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
% xi = x(:,3);
u = x(:,3);
v = x(:,4);
% omega = x(:,6);

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

% % % tyres.Fz_aero(:,1) = Df.*auxdata.bA/auxdata.w/2.*refsize;
% % % tyres.Fz_aero(:,2) = tyres.Fz_aero(:,1);
% % % tyres.Fz_aero(:,3) = Df.*auxdata.aA/auxdata.w/2.*refsize;
% % % tyres.Fz_aero(:,4) = tyres.Fz_aero(:,3);
% % % 
% % % % Collect all forces
% % % tyres.Fz = tyres.Fz_static + tyres.Fz_aero;
% % % 
% % % %% 1.b TYRES FRICTION FORCES
% % % % Tyres
% % % % Maximum coefficients
% % % tyres.mux_max = auxdata.mux_max_interp(tyres.Fz);
% % % tyres.muy_max = auxdata.muy_max_interp(tyres.Fz);
% % % tyres.kappa_max = auxdata.kappa_max_interp(tyres.Fz);
% % % tyres.alpha_max = auxdata.alpha_max_interp(tyres.Fz);
% % % 
% % % % Slip ratios -> Directly from controls
% % % tyres.kappa(:,1) = u_controls(:,2);
% % % tyres.kappa(:,2) = u_controls(:,3);
% % % tyres.kappa(:,3) = u_controls(:,4);
% % % tyres.kappa(:,4) = u_controls(:,5);
% % % 
% % % % Slip angles
% % % tyres.alpha(:,1) = atan2(sin(delta).*(omega.*auxdata.wf - u) + cos(delta).*(omega.*auxdata.a + v),...
% % %     cos(delta).*(u - omega.*auxdata.wf) + sin(delta).*(omega.*auxdata.a + v));
% % % tyres.alpha(:,2) = atan2(cos(delta).*(omega.*auxdata.a + v) - sin(delta).*(omega.*auxdata.wf + u),...
% % %     cos(delta).*(u + omega.*auxdata.wf) + sin(delta).*(omega.*auxdata.a + v));
% % % tyres.alpha(:,3) = atan2(v - omega.*auxdata.b, u - omega*auxdata.wr);
% % % tyres.alpha(:,4) = atan2(v - omega.*auxdata.b, u + omega*auxdata.wr);
% % % tyres.alpha = rad2deg(tyres.alpha);
% % % 
% % % % Spin speeds (from slip ratios and kinematics)
% % % tyres.spin(:,1) = -(tyres.kappa(:,1) + 1).*(cos(delta).*(u - omega.*auxdata.wf) + sin(delta).*(v + omega.*auxdata.a))/auxdata.R;
% % % tyres.spin(:,2) = -(tyres.kappa(:,2) + 1).*(cos(delta).*(u + omega.*auxdata.wf) + sin(delta).*(v + omega.*auxdata.a))/auxdata.R;
% % % tyres.spin(:,3) = -(tyres.kappa(:,3) + 1).*(u - omega.*auxdata.wr)/auxdata.R;
% % % tyres.spin(:,4) = -(tyres.kappa(:,4) + 1).*(u + omega.*auxdata.wr)/auxdata.R;
% % % 
% % % % Normalized slip angles and slip ratios
% % % tyres.kappa_n = tyres.kappa./tyres.kappa_max;
% % % tyres.alpha_n = tyres.alpha./tyres.alpha_max;
% % % 
% % % tyres.rho = sqrt(tyres.kappa_n.^2 + tyres.alpha_n.^2);
% % % 
% % % % Friction coefficients
% % % tyres.mux = tyres.mux_max.*sin(auxdata.Qx.*atan2(pi.*tyres.rho,2.*atan(auxdata.Qx)));
% % % tyres.muy = tyres.muy_max.*sin(auxdata.Qy.*atan2(pi.*tyres.rho,2.*atan(auxdata.Qy)));
% % % 
% % % % Friction forces
% % % tyres.Fx = tyres.mux.*tyres.Fz.*tyres.kappa_n./tyres.rho;
% % % tyres.Fy = tyres.muy.*tyres.Fz.*tyres.alpha_n./tyres.rho;
% % % 
% % % tyres.Fx(isnan(tyres.Fx)) = 0; % to avoid errors when rho=0 at a given tyre
% % % tyres.Fy(isnan(tyres.Fy)) = 0;
% % % 
% % % % Total forces in car body-fixed reference frame
% % % tyres.Fx_tot = cos(delta).*(tyres.Fx(:,1) + tyres.Fx(:,2)) - sin(delta).*(tyres.Fy(:,1) + tyres.Fy(:,2)) + ...
% % %     tyres.Fx(:,3) + tyres.Fx(:,4) + D;
% % % tyres.Fy_tot = cos(delta).*(tyres.Fy(:,1) + tyres.Fy(:,2)) + sin(delta).*(tyres.Fx(:,1) + tyres.Fx(:,2)) + ...
% % %     tyres.Fy(:,3) + tyres.Fy(:,4);
% % % 
% % % % Tractive and braking forces are fixed by kappas control
% % % % Tractive forces
% % % % Fxr_t = theta_t.*auxdata.Teng_max./auxdata.R;
% % % % Braking forces
% % % % 

% REDUCED MODEL
tyres.Fy_tot = cos(delta).*(auxdata.W + Df).*auxdata.mux_2.*6;
tyres.Fx_tot = u_controls(:,2)./1e5;

% Fx = min(max,asked) >> Smooth approximation of min() function
% tyres.Fx_tot = -log(exp(-tyres.Fx_tot) + exp(-auxdata.Teng_max.*u./auxdata.R));
tyres.Fx_tot = ((tyres.Fx_tot.^(-50))+((auxdata.Teng_max.*u./auxdata.R).^(-50))).^(-1/50);

%% 2. EOMs - EQUATIONS OF MOTION
% t_dot = (u.*cos(xi) - v.*sin(xi))./(1 - n.*auxdata.trackInterp.C(t)); % == 1/Sf == dt/ds (== ds/dt in Perantoni notation)

% Sf = (1 - n.*auxdata.trackInterp.C(t))./(u.*cos(xi) - v.*sin(xi));
% n_dot = u.*sin(xi) + v.*cos(xi);
% % xi_dot = omega - auxdata.trackInterp.C(t)./Sf;
% u_dot = omega.*v + tyres.Fx_tot/auxdata.M;
% v_dot = -omega.*u + tyres.Fy_tot/auxdata.M;
% Iz_omega_dot = auxdata.a.*(cos(delta).*(tyres.Fy(:,1) + tyres.Fy(:,2)) + sin(delta).*(tyres.Fx(:,1) + tyres.Fx(:,2))) + ...
%     auxdata.wf.*(sin(delta).*tyres.Fy(:,1) - cos(delta).*tyres.Fx(:,1)) + ...
%     auxdata.wf.*(-sin(delta).*tyres.Fy(:,2) + cos(delta).*tyres.Fx(:,2)) + ...
%     -auxdata.wr.*tyres.Fx(:,3) + auxdata.wr.*tyres.Fx(:,4) - auxdata.b.*(tyres.Fy(:,3) + tyres.Fy(:,4));
% omega_dot = Iz_omega_dot./auxdata.Iz;

Sf = (1 - n.*auxdata.trackInterp.C(t))./u;
n_dot = v;
u_dot = tyres.Fx_tot/auxdata.M;
v_dot = tyres.Fy_tot/auxdata.M;

dsdt = 1.*refsize;
dndt = n_dot.*Sf; % derivatives with respect to centreline coordinate (dnds in Perantoni notation)
% dxidt = xi_dot.*Sf;
dudt = u_dot.*Sf;
dvdt = v_dot.*Sf;
% domegadt = omega_dot.*Sf;


%% 3. CONSTRAINTS
% h1 = tyres.Fx_tot - auxdata.Teng_max.*u./auxdata.R;
% h2 = -tyres.Fx_tot - auxdata.Fbrk_max;

%% 4. OUTPUTS
dx = [dsdt dndt dudt dvdt];
% g_eq = [0 0].*refsize; % [0 0].*refsize; // [g1 g2];
% g_neq = [0.*refsize 0.*refsize 0.*refsize]; % [0 0 0].*refsize; // % [h1 h2 h3];
