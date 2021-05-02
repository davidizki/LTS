function [plotparam] = LTS_trackPostprocessor(track,trackInterp,plotvar,plotparam)
%% LTS_trackPostprocessor
% AUTHOR:
% David Izquierdo
% 
% FUNCTION: 
% Create figures associated to the track layout
% 
% INPUTS:
% track, trackInterp: outputs coming from LTS_trackGenerator
% plotvar, plotparam: variables contaning data and information for the plots
%
% OUTPUTS:
% plotparam: updated with the new total number of figures 
%
% STRUCTURE:
% 1. PREPARATION OF DATA
% 2. TRACK FIGURES
% 3. OUTPUTS

%% 1. PREPARATION OF DATA
% Unfold inputs
S = plotvar.S;
N = plotvar.N;
alpha = plotvar.alpha;
S_mesh = plotvar.S_mesh;
N_mesh = plotvar.N_mesh;
nodes_locations = plotvar.nodes_locations;

Nfigures = plotparam.Nfigures;
F = plotparam.F;
Fnames = plotparam.Fnames;

% Continuous interpolation of the solution
Xc = trackInterp.x(S);
Yc = trackInterp.y(S);
THETA = trackInterp.theta(S);

X = Xc + N.*(-sin(THETA)); % N > 0 if the car is to the left of the centreline (see Perantoni)
Y = Yc + N.*(cos(THETA));

% Solution at mesh points
Xc_mesh = trackInterp.x(S_mesh);
Yc_mesh = trackInterp.y(S_mesh);
THETA_mesh = trackInterp.theta(S_mesh);

X_mesh = Xc_mesh + N_mesh.*(-sin(THETA_mesh)); % N > 0 if the car is to the left of the centreline (see Perantoni)
Y_mesh = Yc_mesh + N_mesh.*(cos(THETA_mesh));

%% 2. TRACK FIGURES
% 1. theta of each segment - TO CHECK THAT THETAS RESPECT CONTINUITY AND THAT THETA APPROXIMATION IS COHERENT
Nfigures = Nfigures + 1;
F(Nfigures) = figure(Nfigures);
Fnames(Nfigures) = "track_theta_C";
yyaxis left
plot(1.5:track.Npoints-1+0.5,track.theta_seg, 'r*');
hold on; plot(track.theta, 'g*'); % theta of each point
hold on; plot(track.theta(end)*ones(track.Npoints,1)-track.delta, 'b--'); % last point line. To check
xlabel('Data point [\#]')
ylabel('$\theta [rad]$')
yyaxis right
ylabel('$C\ [m^{-1}]$')
hold on; plot(track.C,'m')
legend('$\theta$ on the segment','$\theta$ at the beginning of the segment','Last $\theta$ (corrected to match $1^{st}$)','C')
title('Track Orientation and Curvature vs. Data Points')
grid on; grid minor
movegui('east')

% 2. track by segments - TO CHECK TRACK BASE DATAPOINTS
Nfigures = Nfigures + 1;
F(Nfigures) = figure(Nfigures);
Fnames(Nfigures) = "track_centreline";
plot(track.x, track.y, 'b-o', 'markersize', 2); hold on;
plot(track.x(1), track.y(1), 'ro', 'markersize', 5)
legend('Centreline','Start-Finish')
title('Track Centreline')
xlabel('$x\ [m]$')
ylabel('$y\ [m]$')
grid on
axis equal
movegui('west')

% 3. track by segments with limits - TO CHECK TRACK LIMITS
Nfigures = Nfigures + 1;
F(Nfigures) = figure(Nfigures);
Fnames(Nfigures) = "track_layout";
hold on; plot(track.x, track.y, 'b-o', 'markersize', 2);
hold on; plot(track.x(1), track.y(1), 'ro', 'markersize', 5);
hold on; plot(track.xL, track.yL, 'm-o', 'markersize', 2);
hold on; plot(track.xR, track.yR, 'g-o', 'markersize', 2);
legend('Centreline','Start-Finish','Left limits','Right limits');
title('Track Layout')
xlabel('$x\ [m]$')
ylabel('$y\ [m]$')
grid on
axis equal
movegui('west')


s_refined = 0:0.1:track.s(end); % sampling rate for s
% 4. theta interpolated vs discrete - TO CHECK TRACK ORIENTATION
Nfigures = Nfigures + 1;
F(Nfigures) = figure(Nfigures);
Fnames(Nfigures) = "track_theta";
hold on; plot(track.s,track.theta,'r*','markersize',3);
hold on; plot(s_refined,trackInterp.theta(s_refined),'k');
legend('$\theta$ - Discrete','$\theta$ - Interpolant')
title('Track Orientation - Discrete vs. Interpolated')
xlabel('$s\ [m]$')
ylabel('$\theta\ [rad]$')
grid on; grid minor
movegui('east')

% 5. curvature interpolated vs discrete + alpha - TO CHECK TRACK CURVATURE
Nfigures = Nfigures + 1;
F(Nfigures) = figure(Nfigures);
Fnames(Nfigures) = "track_C_alpha";
hold on; plot(track.s,track.C,'r*','markersize',3);
hold on; plot(s_refined,trackInterp.C(s_refined),'k');
legend('$C$ - Discrete','$C$ - Interpolant')
title('Track Curvature - Discrete vs. Interpolated')
xlabel('$s\ [m]$')
ylabel('$C\ [m^{-1}]$')
grid on; grid minor
movegui('east')
legend('-DynamicLegend');

yyaxis right
hold on; plot(S,alpha,'b','linewidth',1,'DisplayName','$\alpha$')
ylabel('$\alpha\ [rad]$')
grid on
grid minor

% 6. circuit lines interpolated vs discrete + trajectory - TO CHECK OPTIMAL TRAJECTORY
Nfigures = Nfigures + 1;
F(Nfigures) = figure(Nfigures);
Fnames(Nfigures) = "track_layout_optTraj";
hold on; plot(track.x,track.y,'k*','markersize',3);
hold on; plot(trackInterp.x(s_refined),trackInterp.y(s_refined),'k','linewidth',1.25);
hold on; plot(track.xL,track.yL,'m*','markersize',3);
hold on; plot(trackInterp.xL(s_refined),trackInterp.yL(s_refined),'m');
hold on; plot(track.xR,track.yR,'g*','markersize',3);
hold on; plot(trackInterp.xR(s_refined),trackInterp.yR(s_refined),'g');
legend('$(x,y)$ - Discrete','$(x,y)$ - Interpolant','$(x,y)_L$ - Discrete','$(x,y)_L$ - Interpolant',...
    '$(x,y)_R$ - Discrete','$(x,y)_R$ - Interpolant')
title('Track Layout and Optimal Trajectory')
xlabel('$x\ [m]$')
ylabel('$y\ [m]$')
grid on; grid minor
axis equal
movegui('west')
legend('-DynamicLegend');

hold on; plot(X_mesh,Y_mesh,'b*','DisplayName','Racing line - Mesh')
hold on; plot(X,Y,'b','linewidth',1.5,'DisplayName','Racing line')

% 7. nodes distribution - TO CHECK MESH REFINEMENT LOGIC
Nfigures = Nfigures + 1;
F(Nfigures) = figure(Nfigures);
Fnames(Nfigures) = "track_nodes";
histogram(nodes_locations,0:0.05:1);
title('Final Nodes Distribution');
xlabel('Fraction of track [-]');
ylabel('Nodes [\#]');
grid on
grid minor
xlim([0 1]);

%% 3. OUTPUTS
plotparam.Nfigures = Nfigures;
plotparam.F = F;
plotparam.Fnames = Fnames;

