function S = rocket_init(P)
% ROCKET_INIT  Initial state: sitting on the rail, motor unlit.

S = struct();
S.t       = 0;
S.t_pad   = 0;      % time since start, until ignition
S.t_burn  = 0;      % time since ignition, drives the thrust curve
S.ignited = false;

S.pos_ned = [0;0;0];
S.vel_ned = [0;0;0];
S.gyro    = [0;0;0];

% Attitude: nose up, tilted by the rail angle toward the rail azimuth.
% Body X must point out the nose, so "vertical" means body X along NED -Z.
%
% Built exactly as SIM_Rocket.cpp does it:
%     rail_dcm.from_euler(0, radians(90 - rail_tilt_deg), radians(rail_azimuth_deg))
% POSITIVE 90 deg pitch. Positive pitch raises the nose, so -90 points it at the
% GROUND -- which is what this used to do, silently flying the whole MATLAB model
% upside down relative to the C++ one. rocket_selftest catches it.
S.quat = rocket_quat_from_euler(0, ...
                                deg2rad(90 - P.rail_tilt_deg), ...
                                deg2rad(P.rail_azimuth_deg));
S.quat = S.quat / norm(S.quat);

S.mass       = P.dry_mass + P.prop_mass;
S.thrust     = 0;
S.impulse_used = 0;
S.q          = 0;
S.tilt_deg   = P.rail_tilt_deg;

% Stationary accelerometer reading: specific force, gravity excluded, so the nose
% axis reads +g. See the note in rocket_step.
S.accel_body = [P.g; 0; 0];
end
