function [problem,guess] = LTS
%% LTS
% AUTHOR:
% David Izquierdo
% 
% FUNCTION: 
% Formulate the OCP problem
% 
% INPUTS:
% (no inputs)
%
% OUTPUTS:
% problem: Structure with information on the optimal control problem
% guess: Guess for state, control and multipliers
%
% STRUCTURE:
% 0. SETUP
% 1. LOAD MODELS & CONSTANTS
%     2.a TIME INFORMATION
%     2.b INITIAL CONDITIONS DATA
%     2.c BOUNDS DATA
%     2.d TERMINAL CONDITIONS DATA
% 3. STATES DATA
% 4. CONTROLS DATA
% 5. PARAMETERS DATA
% 6. CONSTRAINTS
% 7. OUTPUT FOLDING

%% 0. SETUP
% Plant model name, used for Adigator
InternalDynamics=@LTS_dynamics;
SimDynamics=@LTS_dynamics; % the same function instead of separated @LTS_Dynamics_Sim;

% Analytic derivative files (optional)
problem.analyticDeriv.gradCost=[];
problem.analyticDeriv.hessianLagrangian=[];
problem.analyticDeriv.jacConst=[];

% Settings file
problem.settings=@LTS_settings;

%% 1. LOAD MODELS & CONSTANTS
% Track model
auxdata.track_name = "Track03"; % Name of the folder (and the .txt file)
auxdata.track_N = 2; % [m] -note that minimum TOTAL track width in FSG is 3 m-
[auxdata.trackInterp, auxdata.trackData] = LTS_trackGenerator(auxdata.track_name,auxdata.track_N);

% % Thrust model
% x=[0 0.5 1];
% y=[0 0.48 1];
% FFModel=pchip(x,y);

% Constants
auxdata.V = 20; % [m/s]
% auxdata.g=9.81;
% auxdata.ktomps=0.514444;
% auxdata.ftom=0.3048;
% auxdata.R=287.15;
% auxdata.ps=101325;
% auxdata.rhos=1.225;
% auxdata.Ts=288.15;
% auxdata.dTdH=-0.0065;
% auxdata.nprop=1020;
% auxdata.Dprop=3.66;
% auxdata.kappa=1.4;
% auxdata.alpha0=-2.32*pi/180;
% auxdata.clalpha=0.095;
% auxdata.k_cd=0.033;
% auxdata.cd0=0.0233;
% auxdata.S=70;
% auxdata.alphat=0*pi/180;
% auxdata.FFModel=FFModel;

% % Look up tables
% auxdata.a1p=1.5456*10^-9;
% auxdata.a2p=-3.1176*10^-7;
% auxdata.a3p=1.9477*10^-5;
% auxdata.b1p=-2.0930*10^-5;
% auxdata.b2p=4.1670*10^-3;
% auxdata.b3p=-0.40739;
% auxdata.c1p=0.088835;
% auxdata.c2p=-12.855;
% auxdata.c3p=3077.7;
% auxdata.Pidle=75;

%% 2.a TIME INFORMATION
t0 = 0;
tf = auxdata.trackData.s(end);

% Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max=0;
guess.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=tf; % total distance is known
problem.time.tf_max=tf; 
guess.tf=tf;
% problem.time.tf_min=20;     
% problem.time.tf_max=200; 
% guess.tf=50;

% problem.time.tf_min=7000;     
% problem.time.tf_max=10000; 
% guess.tf=8500;

%% 2.b INITIAL CONDITIONS DATA
n0 = 0;

% x0 = trackInterp.x(1);
% y0 = trackInterp.y(1);
% alpha0 = 0;

% W0=18000*auxdata.g;
% H0=1600*auxdata.ftom;
% T0=auxdata.Ts+H0*auxdata.dTdH;
% p0=auxdata.ps*(T0/auxdata.Ts)^(-auxdata.g/auxdata.dTdH/auxdata.R);
% rho0=auxdata.rhos*(T0/auxdata.Ts)^(-(auxdata.g/auxdata.dTdH/auxdata.R+1));
% Vtas0=CAS2TAS( auxdata.kappa, p0, rho0, auxdata.ps, auxdata.rhos, 190*auxdata.ktomps );
% x0=0;y0=0;
% alpha0=5*pi/180;
% chi0=0;
% gamma0=8*pi/180;

%% 2.c BOUNDS DATA
n_max = auxdata.track_N; % [m] n_min = -n_max
alpha_max = pi/10; % - 1e-2; % alpha_min = -alpha_max

% W_min=(18000-4000)*auxdata.g;
% H_min=H0;H_max=25000*auxdata.ftom;
% T_Hmax=auxdata.Ts+H_max*auxdata.dTdH;
% p_Hmax=auxdata.ps*(T_Hmax/auxdata.Ts)^(-auxdata.g/auxdata.dTdH/auxdata.R);
% rho_Hmax=auxdata.rhos*(T_Hmax/auxdata.Ts)^(-(auxdata.g/auxdata.dTdH/auxdata.R+1));
% Vtas_max=CAS2TAS( auxdata.kappa, p_Hmax, rho_Hmax, auxdata.ps, auxdata.rhos, 227*auxdata.ktomps );
% Vtas_min=Vtas0;
% alpha_max=10*pi/180;
% gamma_max=60*pi/180;
% throttle_max=1;throttle_min=0;
% chi_max=pi;
% chi_min=-pi;
% phi_0=0;
% phi_max=45*pi/180;
% x_max=1200000;
% y_max=1200000;

%% 2.d TERMINAL CONDITIONS DATA
nf = 0;

% xf = trackInterp.x(end);
% yf = trackInterp.y(end);
% alphaf = 0;

% Hf=2000*auxdata.ftom;
% T_Hf=auxdata.Ts+Hf*auxdata.dTdH;
% p_Hf=auxdata.ps*(T_Hf/auxdata.Ts)^(-auxdata.g/auxdata.dTdH/auxdata.R);
% rho_Hf=auxdata.rhos*(T_Hf/auxdata.Ts)^(-(auxdata.g/auxdata.dTdH/auxdata.R+1));
% Vtasf=CAS2TAS( auxdata.kappa, p_Hf, rho_Hf, auxdata.ps, auxdata.rhos, 190*auxdata.ktomps );
% xf=800000;
% yf=900000;
% chif=-3/4*pi;
% gammaf=-3*pi/180;

%% 3. STATES DATA:
% 3.1 ICs (& ICs bounds)
% 3.2 Bounds
% 3.3 Rate bounds
% 3.4 State error tol
% 3.5 State bounds error tol
% 3.6 TCs ((& TCs bounds)
% 3.7 Guesses

% 3.1 Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0=[]; % empty -> free
problem.states.x0l=[-n_max];
problem.states.x0u=[n_max];
% problem.states.x0=[H0 x0 y0 Vtas0 gamma0 chi0 W0];
% problem.states.x0l=[H0 x0 y0 Vtas0 gamma0 chi0 W0];
% problem.states.x0u=[H0 x0 y0 Vtas0 gamma0 chi0 W0];

% 3.2 State bounds. xl=< x <=xu
problem.states.xl=[-inf];
problem.states.xu=[inf];
% problem.states.xl=[H_min x0 y0 Vtas_min -gamma_max chi_min W_min]; 
% problem.states.xu=[H_max x_max y_max Vtas_max gamma_max chi_max W0]; 

% 3.3 State rate bounds. xrl=< x <=xru
problem.states.xrl=[-inf]; 
problem.states.xru=[inf]; 
% problem.states.xrl=[-inf -inf -inf -inf -inf -inf -inf]; 
% problem.states.xru=[inf inf inf inf inf inf inf]; 

% 3.4 State error bounds - Integration errors
problem.states.xErrorTol_local=[auxdata.track_N/100];
problem.states.xErrorTol_integral=[auxdata.track_N/100];
% problem.states.xErrorTol_local=[1 1 1 0.5 deg2rad(5) deg2rad(5) 0.1*auxdata.g];
% problem.states.xErrorTol_integral=[1 1 1 0.5 deg2rad(5) deg2rad(5) 0.1*auxdata.g];

% 3.5 State constraint error bounds - By how much can the variable bounds be violated?
problem.states.xConstraintTol=[auxdata.track_N/100];
problem.states.xrConstraintTol=[auxdata.track_N/100];
% problem.states.xConstraintTol=[1 1 1 0.5 deg2rad(5) deg2rad(5) 0.1*auxdata.g];
% problem.states.xrConstraintTol=[1 1 1 0.5 deg2rad(5) deg2rad(5) 0.1*auxdata.g];

% 3.6 Terminal state bounds. xfl=< xf <=xfu
problem.states.xf=[]; % empty -> free
problem.states.xfl=[-n_max];
problem.states.xfu=[n_max];
% problem.states.xfl=[Hf xf yf Vtasf gammaf chif W_min]; 
% problem.states.xfu=[Hf xf yf Vtasf gammaf chif W0];

% 3.7 Guess the state trajectories with [x0 xf]
guess.time(:,1) = [t0:1:tf];
guess.states(:,1)=zeros(size(guess.time));
% guess.states(:,1)=[H_max H_max];
% guess.states(:,2)=[x0 xf];
% guess.states(:,3)=[y0 yf];
% guess.states(:,4)=[Vtas_max Vtas_max];
% guess.states(:,5)=[gamma0 gammaf];
% guess.states(:,6)=[chi0 chif];
% guess.states(:,7)=[W0 (18000-2000)*auxdata.g];

%% 4. CONTROLS DATA
% 4.0 # control actions
% 4.1 ICs (& ICs bounds)
% 4.2 Bounds
% 4.3 Rate bounds
% (NO) 4.4 Control error tol: the input is directly specified (it is not integrated) -> No integration error like in the states.
% 4.5 Control bounds error tol
% (NO) 4.6 TCs ((& TCs bounds): not sure whether the code is able to handle them. Anyways, it is not usual to need them.
% 4.7 Guesses

% 4.0 Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       

% 4.1 Initial conditions
problem.inputs.u0=[]; % empty -> free
problem.inputs.u0l=[-alpha_max];
problem.inputs.u0u=[alpha_max];
% problem.inputs.u0l=[-alpha_max -phi_max throttle_min];
% problem.inputs.u0u=[alpha_max phi_max throttle_max];

% 4.2 Input bounds
problem.inputs.ul=[-alpha_max];
problem.inputs.uu=[alpha_max];
% problem.inputs.ul=[-alpha_max -phi_max throttle_min];
% problem.inputs.uu=[alpha_max phi_max throttle_max];

% 4.3 Input rate bounds
problem.inputs.url=[-inf]; % check carefully
problem.inputs.uru=[inf];
% problem.inputs.url=[-deg2rad(2) -deg2rad(10) -inf];
% problem.inputs.uru=[deg2rad(2) deg2rad(10) inf];

% 4.4 (NO)

% 4.5 Input constraint error bounds
problem.inputs.uConstraintTol=[deg2rad(1)];
problem.inputs.urConstraintTol=[deg2rad(1)];
% problem.inputs.uConstraintTol=[deg2rad(0.5) deg2rad(0.5) 0.1];
% problem.inputs.urConstraintTol=[deg2rad(0.5) deg2rad(0.5) 0.1];

% 4.6 (NO)

% 4.7 Guesses
% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=zeros(size(guess.time));
% guess.inputs(:,1)=[0 0];

% guess.inputs(:,1)=[alpha0  alpha0];
% guess.inputs(:,2)=[phi_0  phi_0];
% guess.inputs(:,3)=[1  1];

%% 5. PARAMETERS DATA
% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

%% 6. CONSTRAINTS
% ?? Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
% 6.1 EQUALITY CONSTRAINTS (no bounds, since it is just equality)
problem.constraints.ng_eq=0;
problem.constraints.gTol_eq=[];

% 6.2 INEQUALITY CONSTRAINTS
problem.constraints.gl=[0 0]; % g cannot take values lower than zero
problem.constraints.gu=[inf inf];
problem.constraints.gTol_neq=[auxdata.track_N/100 auxdata.track_N/100]; % tolerance for g

% problem.constraints.gl=[0 0 0 0 0 0 0 0 0 0];
% problem.constraints.gu=[inf inf inf inf inf inf inf inf inf inf];
% problem.constraints.gTol_neq=[(auxdata.obs_r_1+5)^2-(auxdata.obs_r_1)^2 (auxdata.obs_r_2+5)^2-(auxdata.obs_r_2)^2 (auxdata.obs_r_3+5)^2-(auxdata.obs_r_3)^2 (auxdata.obs_r_4+5)^2-(auxdata.obs_r_4)^2 (auxdata.obs_r_5+5)^2-(auxdata.obs_r_5)^2 (auxdata.obs_r_6+5)^2-(auxdata.obs_r_6)^2 (auxdata.obs_r_7+5)^2-(auxdata.obs_r_7)^2 (auxdata.obs_r_8+5)^2-(auxdata.obs_r_8)^2 (auxdata.obs_r_9+5)^2-(auxdata.obs_r_9)^2 (auxdata.obs_r_10+5)^2-(auxdata.obs_r_10)^2]/1e04;

% problem.constraints.gl=[0 0];
% problem.constraints.gu=[inf inf];
% problem.constraints.gTol_neq=[(auxdata.obs_r_1+5)^2-(auxdata.obs_r_1)^2 (auxdata.obs_r_4+5)^2-(auxdata.obs_r_4)^2]/1e04;


% % problem.constraints.g_neq_ActiveTime{1}=[guess.tf/2 guess.tf];
% problem.constraints.g_neq_ActiveTime{1}=[0 guess.tf];
% problem.constraints.g_neq_ActiveTime{2}=[];
% problem.constraints.g_neq_ActiveTime{3}=[];
% problem.constraints.g_neq_ActiveTime{4}=[0 guess.tf];
% % problem.constraints.g_neq_ActiveTime{4}=[0 guess.tf/2];
% problem.constraints.g_neq_ActiveTime{5}=[];


% 6.3 BOUNDARY CONSTRAINTS
% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=[0 0];
problem.constraints.bu=[0 0];
problem.constraints.bTol=[1e-2 1e-2];

%% 7. OUTPUT FOLDING
% store the necessary problem parameters used in the functions
problem.data.auxdata=auxdata;

% Get function handles and return to Main.m
problem.data.InternalDynamics=InternalDynamics;
problem.data.functionfg=@fg;
problem.data.plantmodel = func2str(InternalDynamics);
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.sim.functions=SimDynamics;
problem.sim.inputX=[];
problem.sim.inputU=1:length(problem.inputs.ul);
problem.functions_unscaled={@L_unscaled,@E_unscaled,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
problem.data.functions_unscaled=problem.functions_unscaled;
problem.data.ng_eq=problem.constraints.ng_eq;
problem.constraintErrorTol=[problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];
% errors organized as: [path constraints (eq, neq) upper bound, path constraints (eq, neq) lower bound, state upper bound, state lower bound, input upper bound, input lower bound]
%                                                               (or something similar to that)


function stageCost=L_unscaled(x,xr,u,ur,p,t,data)

% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorized stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------

auxdata = data.auxdata;
n = x(:,1);
alpha = u(:,1);

stageCost = (1 - n.*auxdata.trackInterp.C(t))./(auxdata.V.*cos(alpha)); % dt/ds
% stageCost = 0*t;

%------------- END OF CODE --------------


function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 

% E_unscaled - Returns the boundary value cost
%
% Syntax:  boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%
%------------- BEGIN CODE --------------

boundaryCost = 0;
% boundaryCost= tf;

%------------- END OF CODE --------------


function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)

% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------
varargin=varargin{1};

bc(1,:)=[x0(1)-xf(1)]; % state and control match at t0 and tf. bc must be a column vector
bc(2,:)=[u0(1)-uf(1)];
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
%------------- BEGIN CODE --------------
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if ((strcmp(options.discretization,'hpLGR')) || (strcmp(options.discretization,'globalLGR')))  && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc,diff(t_segment)];
        end
    end
end

%------------- END OF CODE --------------

