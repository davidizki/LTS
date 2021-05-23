function [trackInterp, track] = LTS_trackGenerator(track_name,track_halfwidth)
%% LTS_trackGenerator
% AUTHOR:
% David Izquierdo
% 
% FUNCTION: 
% Create the interpolants associated to track variables in terms of elapsed centreline distance (s)
% 
% INPUTS:
% track_name: Name of the folder where the file containing a list of (x,y) points that defines the track centreline is placed
% track_halfwidth: Track half-width (regardless of vehicle track)
% showfigs (0|1): Enable/Disable figures
%
% OUTPUTS:
% trackInterp: struct containing the interpolants
%     trackInterp.(x|y): track centreline interpolator in terms of s
%     trackInterp.(x|y)R, track.(x|y)L: track limits interpolators in terms of s
%     trackInterp.theta: track centreline angle interpolator in terms of s
%     trackInterp.C: curvature interpolator in terms of s
%
% STRUCTURE:
% 0. USER INPUTS
% 1. IMPORT DATA
% 2. CALCULATE OUTPUT VARIABLES
% 3. BUILD THE INTERPOLANTS

%% 0. USER INPUTS

track.name = track_name;
% track.scalingFactor = 0.1; % if txt data dimension is not meters
track.N = track_halfwidth; % [m] -note that minimum TOTAL track width in FSG is 3 m-

%% 1. IMPORT DATA
% Note that there are Npoints points (with the 1st=last duplicated) and Npoints-1 segments joining them

track.M = LTS_trackMatrixCreator(track.name);
% track.M = track.M*track.scalingFactor;
track.x = track.M(:,1);
track.y = track.M(:,2);

track.Npoints = length(track.x);

%% 2. CALCULATE OUTPUT VARIABLES (s, theta, C, xyR, xyL)
% 2.1 s
track.s_seg = sqrt(diff(track.x).^2 + diff(track.y).^2); % [m] length of each segment
track.s = [0; cumsum(track.s_seg)]; % [m] elapsed distance

% 2.2 theta
track.theta_seg = atan2(diff(track.y),diff(track.x)); % [-] orientation of each segment. Theta sign convention == Perantoni
for ii = 2:track.Npoints-1 % correct the thetas to ensure continuity (no jumps) -> important to compute curvature
    if track.theta_seg(ii) - track.theta_seg(ii-1) > pi
        track.theta_seg(ii:end) = track.theta_seg(ii:end) - 2*pi;
    elseif track.theta_seg(ii) - track.theta_seg(ii-1) < -pi
        track.theta_seg(ii:end) = track.theta_seg(ii:end) + 2*pi;
    end
end

% Check if track is CW or CCW. The code does NOT work for self-intersecting track
if (track.theta_seg(end) - track.theta_seg(1)) < -pi
    track.delta = -2*pi; % delta last-first: if CW, the last segment will be oriented ~ -2*pi with respect to the first one
elseif (track.theta_seg(end) - track.theta_seg(1)) > pi
    track.delta = 2*pi; % if CCW, ~ +2*pi
else
    error('Unable to identify track rotation sense')
end

% Compute theta approximation at data points
for ii = 2:track.Npoints-1
    track.theta(ii,1) = dot(track.theta_seg(ii-1:ii),track.s_seg(ii-1:ii))/sum(track.s_seg(ii-1:ii));
end

track.theta(1) = mean([track.theta_seg(end)-track.delta track.theta_seg(1)]); % 1st element is not ok, must be computed by hand.
track.theta(end+1) = track.theta(1) + track.delta; % extra last element matching the first one ±2*pi

% Check beggining-end matching of theta (should give 0 -unlike theta_seg, which can be different-)
if abs(abs(track.theta(1)) - abs(track.theta(end)-track.delta)) > 1e-4
    error('Unmatched start-finish thetas')
end

% 2.3 C
% C is defined as dtheta/ds -> Computed via central finite differences.
% If C<0 -> dtheta/ds<0 -> (theta>0 if CCW from above) CW looking from above -> RIGHT TURN
for ii = 2:track.Npoints-1
    track.C(ii,1) = (track.s_seg(ii-1)^2*track.theta(ii+1) - (track.s_seg(ii-1)^2 - track.s_seg(ii)^2)*track.theta(ii)...
    - track.s_seg(ii)^2*track.theta(ii-1))/...
    (track.s_seg(ii-1)^2*track.s_seg(ii) + track.s_seg(ii-1)*track.s_seg(ii)^2);
end

% For the first point
track.C(1) = (track.s_seg(end)^2*track.theta(2) - (track.s_seg(end)^2 - track.s_seg(1)^2)*track.theta(1)...
    - track.s_seg(1)^2*(track.theta(end-1)-track.delta))/...
    (track.s_seg(end)^2*track.s_seg(1) + track.s_seg(end)*track.s_seg(1)^2);
track.C(end+1) = track.C(1); % repeated

% Check that curvature verifies closure condition
Cint = trapz(track.s,track.C); % should be equal to 2*pi if CCW, -2*pi if CCW -assuming no self-intersections-

% 2.4 xyR, xyL
% Using the track half-width and the theta associated to each point, track limits can be defined

track.xL = track.x + track.N*(-sin(track.theta));
track.xR = track.x + track.N*(sin(track.theta));
track.yL = track.y + track.N*(cos(track.theta));
track.yR = track.y + track.N*(-cos(track.theta));

%% 3. BUILD THE INTERPOLANTS (theta, C, xy, xyR, xyL)
interpMethod = 'makima';

trackInterp.theta = griddedInterpolant(track.s,track.theta,interpMethod);
trackInterp.C = griddedInterpolant(track.s,track.C,interpMethod);
trackInterp.x = griddedInterpolant(track.s,track.x,interpMethod);
trackInterp.y = griddedInterpolant(track.s,track.y,interpMethod);
trackInterp.xL = griddedInterpolant(track.s,track.xL,interpMethod);
trackInterp.yL = griddedInterpolant(track.s,track.yL,interpMethod);
trackInterp.xR = griddedInterpolant(track.s,track.xR,interpMethod);
trackInterp.yR = griddedInterpolant(track.s,track.yR,interpMethod);

% Check that curvature verifies closure condition with interpolant
s_refined = 0:0.1:track.s(end);
Cint2 = trapz(s_refined,trackInterp.C(s_refined)); % should be equal to 2*pi if CCW, -2*pi if CCW -assuming no self-intersections-


