function tf = rocket_use_toolbox(newval)
% ROCKET_USE_TOOLBOX  Global switch: may the helpers use the Aerospace Toolbox?
%
% The physics helpers (rocket_air_density, rocket_quat_to_dcm, rocket_quat_from_euler)
% use the Aerospace Toolbox when it is installed AND this switch allows it, otherwise the
% hand-rolled math. Use it to force the hand-rolled path even when the toolbox is present
% -- e.g. to A/B the two, or to keep a run reproducible on a machine without the toolbox.
%
%   rocket_use_toolbox(false)   % force hand-rolled math everywhere
%   rocket_use_toolbox(true)    % allow the toolbox again (the default)
%   tf = rocket_use_toolbox     % query the current setting
%
% Default is unchanged: with no call, this returns true, so the helpers use the toolbox
% whenever it is installed. Calling it with false is a pure override -- the only thing it
% adds is the ability to force the hand-rolled path. The setting is a persistent flag
% (per MATLAB session); it resets to the default on `clear functions`.

persistent allow
if nargin >= 1
    allow = logical(newval);
end
if isempty(allow)
    allow = true;    % default: use the toolbox if it is installed
end
tf = allow;
end
