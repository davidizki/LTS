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
% Load data from input Excel file
% Vars input
Nstates = 4;
Ncontrols = 2;
Nvars = Nstates + Ncontrols;
Nfields = 17;
numericsInput = readmatrix('LTS_inputData.xlsx','sheet','numericsInput');
numericsInput = numericsInput(1:Nvars,6:(6+Nfields-1)); % 6 is due to the first 5 fields in Excel file -no relevant numeric data-
% varsInput_nan = isnan(varsInput); % may be useful if some of the ICs are non-empty
% varsInput = num2cell(varsInput);
% for ii = 1:size(varsInput,1)
%     for jj = 1:size(varsInput,2)
%         if varsInput_nan(ii,jj)
%             varsInput{ii,jj} = [];
%         end
%     end
% end

% Track input
trackInput = readmatrix('LTS_inputData.xlsx','sheet','trackInput'); %,'OutputType','string');
trackInput = trackInput(:,2);
auxdata.track_name = "Track03"; % Name of the folder (and the .txt file)
auxdata.track_N = trackInput(1,1); % [m] semi-width -note that minimum TOTAL track width in FSG is 3 m-
[auxdata.trackInterp, auxdata.trackData] = LTS_trackGenerator(auxdata.track_name,auxdata.track_N);

% Car input
carInput = readmatrix('LTS_inputData.xlsx','sheet','carInput','OutputType','string');
carInput_fieldnames = carInput(:,1);
carInput_fieldvalues = str2double(carInput(:,3));

for ii = 1:length(carInput_fieldnames)
    auxdata.(carInput_fieldnames(ii)) = carInput_fieldvalues(ii);
end

% Tyres input
tyresInput = readmatrix('LTS_inputData.xlsx','sheet','tyresInput','OutputType','string');
tyresInput_fieldnames = tyresInput(:,1);
tyresInput_fieldvalues = str2double(tyresInput(:,3));

for ii = 1:length(tyresInput_fieldnames)
    auxdata.(tyresInput_fieldnames(ii)) = tyresInput_fieldvalues(ii);
end

% Interpolants for maximum coefficients
auxdata.mux_max_interp = griddedInterpolant([auxdata.Fz1 auxdata.Fz2],[auxdata.mux_1 auxdata.mux_2],'linear','linear');
auxdata.muy_max_interp = griddedInterpolant([auxdata.Fz1 auxdata.Fz2],[auxdata.muy_1 auxdata.muy_2],'linear','linear');
auxdata.kappa_max_interp = griddedInterpolant([auxdata.Fz1 auxdata.Fz2],[auxdata.kappa_1 auxdata.kappa_2],'linear','linear');
auxdata.alpha_max_interp = griddedInterpolant([auxdata.Fz1 auxdata.Fz2],[auxdata.alpha_1 auxdata.alpha_2],'linear','linear');

% % Thrust model
% x=[0 0.5 1];
% y=[0 0.48 1];
% FFModel=pchip(x,y);

%% 2. TIME INFORMATION (track centreline coordinate information)
t0 = 0;
tf = auxdata.trackData.s(end);

% Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max=0;
guess.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed. Possibility to add a guess.
problem.time.tf_min=tf; % total distance is known
problem.time.tf_max=tf; 
guess.tf=tf;

%% 3. STATES DATA
% 3.1 Bounds
% 3.2 Rate bounds
% 3.3 State error tol
% 3.4 State bounds error tol
% 3.5 ICs (& ICs bounds)
% 3.6 TCs ((& TCs bounds)
% 3.7 Guesses

% 3.1 State bounds. xl=< x <=xu
problem.states.xl = numericsInput(1:Nstates,1).';
problem.states.xu = numericsInput(1:Nstates,2).';

% 3.2 State rate bounds. xrl=< x <=xru
problem.states.xrl = numericsInput(1:Nstates,3).';
problem.states.xru = numericsInput(1:Nstates,4).';

% 3.3 State error bounds - Integration errors (local and absolute?)
problem.states.xErrorTol_local = numericsInput(1:Nstates,5).';
problem.states.xErrorTol_integral = numericsInput(1:Nstates,6).';

% 3.4 State constraint error bounds - By how much can the variable bounds be violated?
problem.states.xConstraintTol = numericsInput(1:Nstates,7).';
problem.states.xrConstraintTol = numericsInput(1:Nstates,8).';

% 3.5 Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0 = []; % empty -> free
problem.states.x0l = numericsInput(1:Nstates,10).';
problem.states.x0u = numericsInput(1:Nstates,11).';

% 3.6 Terminal state bounds. xfl=< xf <=xfu
problem.states.xf = []; % empty -> free
problem.states.xfl = numericsInput(1:Nstates,13).';
problem.states.xfu = numericsInput(1:Nstates,14).';

% 3.7 Guess the state trajectories with [t0 tf] and [x0 xf] or providing a reference time vector [t0:1:tf]
...and then zeros(size(guess.time)) -for example-
    
guess.time(:,1) = linspace(t0,tf,100);
% guess.states = numericsInput(1:Nstates,15:16).';

% UPDATE MATLAB TO USE THIS % interpolant_guess_states= griddedInterpolant([t0 tf].',numericsInput(1:Nstates,15:16).');
% guess.states = interpolant_guess_states(guess.time);
for ii = 1:Nstates
    guess.states(:,ii) = interp1([t0 tf],numericsInput(ii,15:16),guess.time);
%     if ii == 3 % use track orientation as guess for xi
%         guess.inputs(:,ii) = auxdata.trackInterp.theta(guess.time);
%     end
%     if ii == 6
%         guess.inputs(:,ii) = 
%     end
end

%% 4. CONTROLS DATA
% 4.0 # control actions
% 4.1 Bounds
% 4.2 Rate bounds
% (NO) 4.3 Control error tol: the input is directly specified (it is not integrated) -> No integration error like in the states.
% 4.4 Control bounds error tol
% 4.5 ICs (& ICs bounds)
% (NO) 4.6 TCs ((& TCs bounds): not sure whether the code is able to handle them. Anyways, it is not usual to need them.
% 4.7 Guesses

% 4.0 Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       

% 4.1 Input bounds
problem.inputs.ul = numericsInput(Nstates+1:Nvars,1).';
problem.inputs.uu = numericsInput(Nstates+1:Nvars,2).';

% 4.2 Input rate bounds
problem.inputs.url = numericsInput(Nstates+1:Nvars,3).'; % check carefully
problem.inputs.uru = numericsInput(Nstates+1:Nvars,4).';

% 4.3 (NO)

% 4.4 Input constraint error bounds
problem.inputs.uConstraintTol = numericsInput(Nstates+1:Nvars,7).';
problem.inputs.urConstraintTol = numericsInput(Nstates+1:Nvars,8).';

% 4.5 Initial conditions
problem.inputs.u0=[]; % empty -> free
problem.inputs.u0l = numericsInput(Nstates+1:Nvars,10).';
problem.inputs.u0u = numericsInput(Nstates+1:Nvars,11).';

% 4.6 (NO)

% 4.7 Guesses
% Guess the input sequences with [u0 uf]
% guess.inputs = numericsInput(Nstates+1:Nvars,15:16).';

for ii = 1:Ncontrols
    guess.inputs(:,ii) = interp1([t0 tf],numericsInput(Nstates+ii,15:16),guess.time);
    if ii == 1 % use Ackermann as guess for delta
        guess.inputs(:,ii) = atan(auxdata.trackInterp.C(guess.time).*auxdata.w);
        % Alternative: guess.inputs(:,ii) = auxdata.trackInterp.C(guess.time);
    end
end

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
% problem.constraints.ng_eq = 2; % number of equality constraints
% problem.constraints.gTol_eq = [0 0];
problem.constraints.ng_eq = 0; % number of equality constraints
problem.constraints.gTol_eq = [];

% 6.2 INEQUALITY CONSTRAINTS
% problem.constraints.gl=[0 0 0]; % g cannot take values lower than zero
% problem.constraints.gu=[inf inf inf]; % one-sided constraints
% problem.constraints.gTol_neq=[1 1 1]; % tolerance for each g
problem.constraints.gl=[]; % g cannot take values lower than zero
problem.constraints.gu=[]; % one-sided constraints
problem.constraints.gTol_neq=[]; % tolerance for each g

% 6.3 BOUNDARY CONSTRAINTS
% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
% problem.constraints.bl = zeros(1,Nvars);
% problem.constraints.bu = zeros(1,Nvars);
% problem.constraints.bTol = numericsInput(:,17).';
problem.constraints.bl = [];
problem.constraints.bu = [];
problem.constraints.bTol = [];

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
% Order: [path constraints (eq+neq) upper bound, path constraints (eq+neq) lower bound, state upper bound, state lower bound, input upper bound, input lower bound]
problem.constraintErrorTol=[problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.constraints.gTol_eq,problem.constraints.gTol_neq,...
    problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];


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
n = x(:,2);
u = x(:,3);
v = x(:,4);

% stageCost = 0*t;
stageCost = (1 - n.*auxdata.trackInterp.C(t))./u; % ds/dt (dt/ds in Perantoni)


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

% boundaryCost= tf;
boundaryCost = 0;


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

% state and control match at t0 and tf. bc must be a column vector
% for ii = 1:length(x0)
%     bc(ii,:) = [x0(ii)-xf(ii)];
% end
% for ii = 1:length(u0)
%     bc(length(x0)+ii,:) = [u0(ii)-uf(ii)];
% end
bc = [];

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

