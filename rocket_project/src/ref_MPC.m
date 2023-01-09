%
% Trace out an EPFL in ref_time seconds
%
function Ref4 = ref_EPFL(t, roll_max, tilt)

ref_time = 30;

if nargin < 2
    roll_max = deg2rad(15);
end
if nargin < 3
    tilt = true;
end

% Coordinates (x, y, heading)
 coords = [ ...
                0 0  45; 0 2  45; 1 1  45; 2 2  45; 2 0 -90;          % 'M'
                3 0 -90; 3 2 -90; 4 2 -90; 4 1 -90; 3 1 -90; 3 0 -90; % 'P'
                7 0 0; 5 0 90; 5 2 45; 7 2 0];                        % 'C'
            coords(:,1:2) = coords(:,1:2) / 2;
            coords(:,3) = coords(:,3) * rad2deg(roll_max)/90;
            
            nCoords = size(coords, 1);

% Break the path into legs, compute their end times such that
% the end of the path is reached at ref_time
legs = coords(2:end,1:2) - coords(1:end-1,1:2);
distances = vecnorm(legs, 2, 2);
leg_endtimes = [0; ref_time * cumsum(distances) / sum(distances)]';

% Find target index for each time point
target_id = sum(t(:) > leg_endtimes, 2) + 1;
% Limit target_id to final point
target_id = min(nCoords, target_id);

% % Return target coordinates for each time point
% XZ = [coords(target_id, 1:2)];
% XYZ = [XZ(1), 0, XZ(2)];
% Roll = deg2rad(coords(target_id,3));
% 
% if tilt
%     % Rotate
%     alpha = deg2rad(-15);
%     beta = deg2rad(19);
%     gamma = deg2rad(-24);
%     
%     T = Rocket.eul2mat([alpha, beta, gamma]);
%     XYZ = (T * XYZ')';
% end
% Ref4 = [XYZ, Roll]; % 4D X 0 Z roll
  % Return target coordinates for each time point
XZ = coords(target_id,1:2);
Roll = deg2rad(coords(target_id,3));
Ref4 = [XZ(:,1), 0, XZ(:,2), Roll]; % 4D X 0 Z roll

% Rotate about x axis
if tilt
    alpha = deg2rad(20);
    XZ = ([cos(alpha) -sin(alpha); sin(alpha) cos(alpha)] * XZ')';
    
    % Rotate about z axis
    gamma = deg2rad(-30);
    Ref4 = [cos(gamma) * XZ(:,1), sin(gamma) * XZ(:,1), XZ(:,2), Roll]; % 4D X 0 Z roll
end
end