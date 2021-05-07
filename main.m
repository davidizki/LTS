%% main - LTS
% AUTHOR:
% David Izquierdo
% 
% FUNCTION: 
% Main script to solve the Optimal Control Problem
% 
% INPUTS:
% (no inputs)
%
% OUTPUTS:
% (no outputs)
%
% STRUCTURE:
% 0. MATLAB SETUP
% 1. OPTIMAL CONTROL PROBLEM
% 2. POST-PROCESSING

%% 0. MATLAB SETUP
clear; clc; close all; format compact; % compact: lines in output are closer together
addpath(genpath("trackGenerator_LTS")); % add path and all subfolders to MATLAB path
set(0, 'DefaultFigureVisible', 'off')
set(groot,'defaulttextinterpreter','latex'); % Set latex as interpreter (remember to use $ $)
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

%% 1. OPTIMAL CONTROL PROBLEM
[problem,guess] = LTS; % Fetch the problem definition
options=problem.settings(30); % Get options and solver settings. 60 -> h-method with N=60 nodes
[solution,MRHistory,data_initial,data_finalMR] = solveMyProblem(problem,guess,options); % Solve the OCP

[simulation.tv,simulation.xv,simulation.uv] = simulateSolution(problem,solution,'ode113'); % Check the model in open loop

%% 2. POST-PROCESSING
% 1. Summary Messages
disp(problem.constraintErrorTol); % show maximum allowed constraint violation
fprintf('SUMMARY |  %.2f s - TIME (Total)\n',sum(MRHistory.timeHistory));
fprintf('        |  %d - # ITERATIONS (Refinement)\n',length(MRHistory.iterHistory));
fprintf('        |  %d - # EVALUATIONS (of Lagrangian Hessian)\n',sum(MRHistory.iterHistory));

% 2.a Plot-Variables (plotvar)
plotvar.xx=linspace(solution.t0,solution.tf,1000).'; % "time" vector
plotvar.x1=speval(solution,'X',1,plotvar.xx); % State 1
plotvar.u1=speval(solution,'U',1,plotvar.xx); % Input 1

% Rename variables
plotvar.S = plotvar.xx; % At all points defined by linspace
plotvar.N = plotvar.x1;
plotvar.alpha = plotvar.u1;
plotvar.S_mesh = solution.T; % Only at collocation points
plotvar.N_mesh = solution.X;
plotvar.N_mesh = plotvar.N_mesh(1:length(plotvar.S_mesh)); % If using hp methods, X is 1 item longer (repeats last element) than T
plotvar.nodes_locations = cumsum(data_finalMR.options.tau);

% 2.b Plot-Parameters (plotparam)
plotparam.Nfigures = 0;
plotparam.F = gobjects(1);
plotparam.Fnames = strings;
plotparam.Nnodes = length(data_finalMR.options.tau);
plotparam.formattype = strcat('-dpng'); % .png
plotparam.resolution = strcat('-r350'); % -fillpage. Increase the number for higher quality
plotparam.showfigs = 0;
plotparam.savefigs = 0;

if plotparam.showfigs || plotparam.savefigs % skip figures generation if no show & no save
    
    % 3. Default Plots
    [plotparam] = genSolutionPlots(options,solution,plotparam); % Built-in figure generation (check settings for the type of plots to be generated)
    
    % 4. Track Post-processing
    if options.plot_track
        [plotparam] = LTS_trackPostprocessor(problem.data.auxdata.trackData,problem.data.auxdata.trackInterp,...
            plotvar,plotparam); % Trajectory in (x,y) coords
    end
    
    % 5. ... more Post-processing
    
    % 6. Show and Save Figures
    if plotparam.showfigs % show figures
        for ii = 1:plotparam.Nfigures
            set(plotparam.F(ii),'visible','on');
        end
    end
    
    if plotparam.savefigs % save figures
        mkdir(datestr(datetime,'yyyy-mm-dd HH.MM.SS'));
        cd(datestr(datetime,'yyyy-mm-dd HH.MM.SS'));
        for ii = 1:plotparam.Nfigures
            set(0,'currentfigure',plotparam.F(ii)); pause(0.05);
            print(sprintf('fig_%d_%s',ii,plotparam.Fnames(ii)),plotparam.formattype,plotparam.resolution);
        end
        cd ..
    end
    clear ii
    
end

set(0, 'DefaultFigureVisible', 'on') % leave it active as default for other files

% 
% xx=linspace(solution.T(1,1),solution.tf,1000);
% 
% figure
% center=[problem.data.auxdata.obs_epos_1 problem.data.auxdata.obs_npos_1];
% obspos = [center-problem.data.auxdata.obs_r_1 2*problem.data.auxdata.obs_r_1 2*problem.data.auxdata.obs_r_1];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% hold on
% center=[problem.data.auxdata.obs_epos_2 problem.data.auxdata.obs_npos_2];
% obspos = [center-problem.data.auxdata.obs_r_2 2*problem.data.auxdata.obs_r_2 2*problem.data.auxdata.obs_r_2];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% center=[problem.data.auxdata.obs_epos_3 problem.data.auxdata.obs_npos_3];
% obspos = [center-problem.data.auxdata.obs_r_3 2*problem.data.auxdata.obs_r_3 2*problem.data.auxdata.obs_r_3];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% center=[problem.data.auxdata.obs_epos_4 problem.data.auxdata.obs_npos_4];
% obspos = [center-problem.data.auxdata.obs_r_4 2*problem.data.auxdata.obs_r_4 2*problem.data.auxdata.obs_r_4];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% center=[problem.data.auxdata.obs_epos_5 problem.data.auxdata.obs_npos_5];
% obspos = [center-problem.data.auxdata.obs_r_5 2*problem.data.auxdata.obs_r_5 2*problem.data.auxdata.obs_r_5];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% center=[problem.data.auxdata.obs_epos_6 problem.data.auxdata.obs_npos_6];
% obspos = [center-problem.data.auxdata.obs_r_6 2*problem.data.auxdata.obs_r_6 2*problem.data.auxdata.obs_r_6];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% center=[problem.data.auxdata.obs_epos_7 problem.data.auxdata.obs_npos_7];
% obspos = [center-problem.data.auxdata.obs_r_7 2*problem.data.auxdata.obs_r_7 2*problem.data.auxdata.obs_r_7];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% center=[problem.data.auxdata.obs_epos_8 problem.data.auxdata.obs_npos_8];
% obspos = [center-problem.data.auxdata.obs_r_8 2*problem.data.auxdata.obs_r_8 2*problem.data.auxdata.obs_r_8];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% center=[problem.data.auxdata.obs_epos_9 problem.data.auxdata.obs_npos_9];
% obspos = [center-problem.data.auxdata.obs_r_9 2*problem.data.auxdata.obs_r_9 2*problem.data.auxdata.obs_r_9];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% center=[problem.data.auxdata.obs_epos_10 problem.data.auxdata.obs_npos_10];
% obspos = [center-problem.data.auxdata.obs_r_10 2*problem.data.auxdata.obs_r_10 2*problem.data.auxdata.obs_r_10];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% 
% plot(speval(solution,'X',3,xx),speval(solution,'X',2,xx),'b-','LineWidth',2)
% xlabel('East Position [m]')
% ylabel('North Position [m]')
% % grid on
% plot(problem.states.x0(3)-1000, problem.states.x0(2)-1000, '.k', 'MarkerSize',20)
% plot(problem.states.xfl(3)-1000, problem.states.xfl(2)-1000, '.k', 'MarkerSize',20)
% text(problem.states.x0(3)+20000,problem.states.x0(2),'ORG')
% text(problem.states.xfl(3)+10000,problem.states.xfl(2)+10000,'DES')
% xlim([-1 10]*10^5)
% ylim([-0.5 9]*10^5)
% 
% figure
% center=[problem.data.auxdata.obs_epos_1 problem.data.auxdata.obs_npos_1];
% obspos = [center-problem.data.auxdata.obs_r_1 2*problem.data.auxdata.obs_r_1 2*problem.data.auxdata.obs_r_1];
% rectangle('Position',obspos,'Curvature',[1 1], 'FaceColor', 'red', 'Edgecolor','none');
% hold on
% plot(speval(solution,'X',3,xx),speval(solution,'X',2,xx),'b-','LineWidth',2)
% xlabel('East Position [m]')
% ylabel('North Position [m]')
% grid on
% plot(problem.states.x0(3)-1000, problem.states.x0(2)-1000, '.k', 'MarkerSize',20)
% plot(problem.states.xfl(3)-1000, problem.states.xfl(2)-1000, '.k', 'MarkerSize',20)
% text(problem.states.x0(3)+10000,problem.states.x0(2),'ORG')
% text(problem.states.xfl(3)-10000,problem.states.xfl(2)-5000,'DES')
% text(8.52e05, 7.8e05,'NO FLIGHT ZONE','Color','white','FontSize',14)
% xlim([8.5 9.1]*10^5)
% ylim([7.5 8.1]*10^5)
% 
% %%
% figure
% subplot(2,1,1)
% hold on
% plot([solution.T(1,1); solution.tf],[problem.states.xu(1), problem.states.xu(1)],'r-' )
% plot(xx,speval(solution,'X',1,xx),'b-' )
% xlim([0 solution.tf])
% xlabel('Time [s]')
% ylabel('Altitude [m]')
% grid on
% 
% subplot(2,1,2)
% plot([solution.T(1,1); solution.tf],[problem.states.xl(4), problem.states.xl(4)],'r-' )
% plot([solution.T(1,1); solution.tf],[problem.states.xu(4), problem.states.xu(4)],'r-' )
% hold on
% plot(xx,speval(solution,'X',4,xx),'b-' )
% xlim([0 solution.tf])
% xlabel('Time [s]')
% ylabel('True Airspeed [m/s]')
% grid on
% 
% figure
% subplot(2,1,1)
% hold on
% plot(xx,speval(solution,'X',5,xx)*180/pi,'b-' )
% hold on
% xlim([0 solution.tf])
% xlabel('Time [s]')
% ylabel('Flight Path Angle [deg]')
% grid on
% 
% 
% subplot(2,1,2)
% hold on
% plot(xx,speval(solution,'X',6,xx)*180/pi,'b-' )
% hold on
% xlim([0 solution.tf])
% xlabel('Time [s]')
% ylabel('Tracking Angle [deg]')
% grid on
% 
% figure
% subplot(2,1,1)
% hold on
% plot(xx,speval(solution,'X',7,xx)/9.81,'b-' )
% xlim([0 solution.tf])
% xlabel('Time [s]')
% ylabel('Aircraft Mass [kg]')
% grid on
% 
% subplot(2,1,2)
% hold on
% plot([solution.T(1,1); solution.tf],[problem.inputs.ul(1), problem.inputs.ul(1)]*180/pi,'r-' )
% plot([solution.T(1,1); solution.tf],[problem.inputs.uu(1), problem.inputs.uu(1)]*180/pi,'r-' )
% plot(xx,speval(solution,'U',1,xx)*180/pi,'b-' )
% 
% xlim([0 solution.tf])
% xlabel('Time [s]')
% ylabel('Angle of attack (Control) [deg]')
% grid on
% 
% 
% figure
% subplot(2,1,1)
% plot([solution.T(1,1); solution.tf],[problem.inputs.ul(2), problem.inputs.ul(2)]*180/pi,'r-' )
% plot([solution.T(1,1); solution.tf],[problem.inputs.uu(2), problem.inputs.uu(2)]*180/pi,'r-' )
% hold on
% plot(xx,speval(solution,'U',2,xx)*180/pi,'b-' )
% 
% xlim([0 solution.tf])
% xlabel('Time [s]')
% ylabel('Roll angle (Control) [deg]')
% grid on
% 
% subplot(2,1,2)
% plot([solution.T(1,1); solution.tf],[problem.inputs.ul(3), problem.inputs.ul(3)],'r-' )
% plot([solution.T(1,1); solution.tf],[problem.inputs.uu(3), problem.inputs.uu(3)],'r-' )
% hold on
% plot(xx,speval(solution,'U',3,xx),'b-' )
% xlim([0 solution.tf])
% xlabel('Time [s]')
% ylabel('Throttle Setting (Control) [-]')
% grid on
