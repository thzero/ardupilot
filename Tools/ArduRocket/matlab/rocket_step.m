function S = rocket_step(S, P, fin)
% ROCKET_STEP  Advance the rocket one timestep under the given fin commands.
%
%   fin  4x1 normalised deflections in -1..1, fins numbered clockwise from the nose
%
% Frames: NED for position/velocity, body for gyro/accel. Body X points out the
% NOSE, so a vertical rocket has body X pointing up.

dt = P.dt;

% ---- mass and thrust ---------------------------------------------------------
if S.ignited
    thrust = interp1(P.thrust_time, P.thrust_newtons, S.t_burn, 'linear', 0);
    thrust = max(thrust, 0);
else
    thrust = 0;
end
% Deplete mass against IMPULSE delivered, not elapsed time, so editing the thrust
% curve cannot silently change how much propellant is burnt.
S.impulse_used = min(S.impulse_used + thrust*dt, P.total_impulse);
burnt_frac = S.impulse_used / P.total_impulse;
S.mass   = P.dry_mass + P.prop_mass*(1 - burnt_frac);
S.thrust = thrust;

% ---- attitude ----------------------------------------------------------------
R = rocket_quat_to_dcm(S.quat);        % body -> NED
nose_ned = R(:,1);              % body X in NED

% ---- airspeed and dynamic pressure -------------------------------------------
vel_air_ned = S.vel_ned;                 % no wind model yet
vel_air_bf  = R.' * vel_air_ned;
speed = norm(vel_air_bf);
% Density falls with altitude, and fin authority is proportional to it, so a fixed
% sea-level value overstates control authority through the whole upper coast.
% Matches SIM_Rocket, which gets this from the SITL base class.
S.rho = rocket_air_density(P.origin_alt - S.pos_ned(3));
S.q = 0.5 * S.rho * speed^2;

% Angle of attack / sideslip, small-angle, relative to the nose axis.
if speed > 0.5
    alpha =  atan2(vel_air_bf(3), vel_air_bf(1));   % about body Y
    beta  =  atan2(vel_air_bf(2), vel_air_bf(1));   % about body Z
else
    alpha = 0; beta = 0;
end

% ---- fin forces and moments, ONE FIN AT A TIME -------------------------------
% Deliberately NO mixing here. This model only sees four servo positions and must
% not assume how the flight code produced them -- that convention belongs solely to
% AP_FinMixerRocket. Keeping a copy of it here is what let this file and
% SIM_Rocket.cpp silently disagree by a factor of 2.
%
% Each fin's lift acts TANGENTIALLY (perpendicular to the plane containing the body
% axis and the fin), so a fin at angle th with force f contributes
%     M_x -= f * fin_radius_m
%     M_y -= f * fin_arm_m * cos(th)
%     M_z -= f * fin_arm_m * sin(th)
% The pair differences fall out of this; they are never assumed.
M = zeros(3,1);
for i = 1:4
    f  = fin(i) * P.fin_force_gain * S.q;      % N, tangential
    th = deg2rad(P.fin_angle_deg(i));
    M(1) = M(1) - f * P.fin_radius_m;
    M(2) = M(2) - f * P.fin_arm_m * cos(th);
    M(3) = M(3) - f * P.fin_arm_m * sin(th);
end

% Airframe stability: negative stability_gain = CP aft of CG = self-correcting.
M(2) = M(2) + alpha * P.stability_gain * S.q;
M(3) = M(3) - beta  * P.stability_gain * S.q;

% Aerodynamic rate damping.
%
% A pitch rate w gives a fin at distance r from the CG a local angle of attack of
% w*r/V, so the moment is q*S*CLa*(w*r/V)*r = 0.5*rho*V*S*CLa*r^2 * w -- i.e.
% proportional to V*w, NOT q*w. Matches SIM_Rocket.cpp.
% rot_damping_coeff was derived at sea level, so scale it with density too.
kd = P.rot_damping_coeff * speed * (S.rho / 1.225);
% Spin damping comes from the same fin force acting at the fin RADIUS rather than
% its axial arm, so it scales as (radius/arm)^2 -- not a hardcoded 0.1, which
% over-damped spin by about 5x.
M(1) = M(1) - S.gyro(1) * kd * 0.0183;
M(2) = M(2) - S.gyro(2) * kd;
M(3) = M(3) - S.gyro(3) * kd;

rot_accel = [M(1)/P.J_spin; M(2)/P.J_tilt; M(3)/P.J_tilt];

% ---- specific force ----------------------------------------------------------
% What an accelerometer measures: thrust + aero, gravity EXCLUDED. A stationary
% vertical rocket reads about +9.81 on the nose axis, not zero.
accel_body = [thrust/S.mass; 0; 0];
if speed > 0.1
    drag_bf = -(vel_air_bf/speed) * (S.q * P.drag_area);
    accel_body = accel_body + drag_bf/S.mass;
end

% ---- rail constraint ---------------------------------------------------------
% While on the rail the airframe cannot rotate and can only slide along it. This
% matters because it covers exactly the phase where the fins have no authority.
dist_up = -S.pos_ned(3);
on_rail = S.ignited && (dist_up < P.rail_length*cosd(P.rail_tilt_deg));
if on_rail || ~S.ignited
    rot_accel = zeros(3,1);
    S.gyro    = zeros(3,1);
end

% ---- integrate ---------------------------------------------------------------
% Total acceleration in NED = rotated specific force + gravity.
accel_ned = R*accel_body + [0;0;P.g];

if ~S.ignited
    accel_ned = zeros(3,1);          % clamped to the pad
elseif on_rail
    % project onto the rail direction; the rail carries any side load
    accel_ned = nose_ned * dot(accel_ned, nose_ned);
end

S.vel_ned = S.vel_ned + accel_ned*dt;
S.pos_ned = S.pos_ned + S.vel_ned*dt;
if -S.pos_ned(3) < 0
    S.pos_ned(3) = 0; S.vel_ned = zeros(3,1);
end

S.gyro = S.gyro + rot_accel*dt;
S.quat = rocket_quat_integrate(S.quat, S.gyro, dt);
S.accel_body = accel_body;

% ---- bookkeeping -------------------------------------------------------------
S.t = S.t + dt;
if ~S.ignited
    S.t_pad = S.t_pad + dt;
    if S.t_pad >= P.ignition_delay
        S.ignited = true;
        fprintf('  IGNITION at t=%.2f s\n', S.t);
    end
else
    S.t_burn = S.t_burn + dt;
end

S.tilt_deg = acosd(max(-1,min(1, nose_ned(3)*-1)));
end
