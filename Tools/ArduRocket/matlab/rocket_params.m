function P = rocket_params()
% ROCKET_PARAMS  Airframe, motor and aerodynamic constants.
%
% Starting values are mirrored from libraries/SITL/SIM_Rocket.h so the MATLAB
% plant begins as a known-equivalent of the C++ one. Diverge deliberately, not by
% accident: if you change a value here, note whether SIM_Rocket should follow.
%
% Those values came from a real OpenRocket export (see ARDUROCKET_PLAN.md section 9).
% Summary: ~11.2 kg off the pad, M-class motor (~5400 N.s), 4.02 in diameter,
% Mach 1.2, ~3900 m apogee.

P = struct();

% ---- integration ----
P.dt = 0.0025;              % 400 Hz, matching SCHED_LOOP_RATE

% ---- mass and motor ----
P.dry_mass   = 8.47;        % kg, burnt out
P.prop_mass  = 2.72;        % kg consumed (11.19 -> 8.47)
P.total_impulse = 5414;     % N.s -- mass depletes against IMPULSE, not time,
                            % so a thrust curve change does not silently change
                            % the burnt mass
P.ignition_delay = 3.0;     % s after arming (a launch controller, not ArduPilot)

% Thrust curve breakpoints, linearly interpolated.
P.thrust_time    = [0.00 0.05 0.10 0.20 0.40 0.80 1.50 2.20 3.00 3.60 4.10 4.40 4.60 4.70];
P.thrust_newtons = [0    1400 1850 1922.8 1830 1700 1550 1400 1200 950  600  300  100  0];

% ---- inertia ----
% From the OpenRocket export's moment-of-inertia columns (lb ft^2 -> SI), not
% estimated. The same conversion reproduces the known 11.19 kg liftoff mass, which
% is what confirms the units. Previous values (1.80 / 0.015) assumed a 1.5 m
% airframe; the .ork says 2.4448 m, so tilt inertia was understated by 2.8x.
P.J_tilt = 4.962;           % kg m^2 about body Y and Z
P.J_spin = 0.0208;          % kg m^2 about body X

% ---- aerodynamics ----
% Air density is NOT a constant here -- see rocket_air_density.m, called each step
% with the current altitude. P.rho_sl is only the sea-level reference used to scale
% coefficients that were derived at sea level.
P.rho_sl = 1.225;           % kg/m^3 at sea level
% FIN GEOMETRY, not a mixing table. See rocket_step.m -- each fin is handled on its
% own so this file never encodes AP_FinMixerRocket's convention.
%
% Fin planform from the .ork: 4 trapezoidal fins.
P.fin.root_chord  = 305;    % mm
P.fin.tip_chord   = 102;    % mm
P.fin.semispan    = 102;    % mm, exposed (root to tip)
P.fin.sweep       = 178;    % mm, leading-edge sweep (root LE to tip LE, aft)
P.fin.body_radius = 49.5;   % mm

% CONTROL TAB on the fin TRAILING EDGE, in millimetres. This is the airframe's steering
% surface; fin_force_gain is now DERIVED from it (rocket_fin_gain.m), not assumed.
%   width  = flap depth forward from the trailing edge  (drives effectiveness)
%   height = tab length along the trailing edge (spanwise)
%   root   = spanwise distance from the fin root to the tab's inboard end
%   axis   = hinge inset aft of the tab's forward edge (0 = hinge at that edge)
%   max_deg= deflection at full command
% The values below are the OLD 25%/75%/20deg assumption expressed in mm -- REPLACE them
% with the measured tab and re-run; the authority updates. (These defaults reproduce a
% force_gain of ~0.00554, within ~1.5% of the previous 0.00546.)
P.tab.width   = 50;    % mm
P.tab.height  = 77;    % mm
P.tab.root    = 13;    % mm
P.tab.axis    = 0;     % mm
P.tab.max_deg = 20;    % deg

P.fin_angle_deg = [270 180 90 0];   % fin positions around the body
[P.fin_force_gain, P.fin_radius_m] = rocket_fin_gain(P.fin, P.tab);

% Tilt moment arm: axial distance from the CG to the tab's centre of pressure. Kept as a
% DIRECT measurement from the OpenRocket CG and fin-CP positions (0.6781 m) rather than
% derived -- a swept fin makes the tab's axial station sensitive to exactly where the tab
% sits, and the .ork gives CG and CP directly, so read it off rather than approximate it.
% If you move the tab far inboard/outboard, update this from the .ork.
P.fin_arm_m = 0.6781;   % m, CG -> tab CP (axial)
% 2 calibers of static margin, from the .ork design (fg.4.K-L.reversed).
%   |Ka| = CN_alpha * A_ref * (x_cp - x_cg) = 12.6 * 0.007707 * 0.198 = 0.0192
% NEGATIVE = CP aft of CG = passively stable.
% The earlier -0.050 came from test.csv, an OLDER revision showing 3.87 calibers.
% Do not re-derive from that CSV; it predates the .ork.
P.stability_gain  = -0.0192;
% Aerodynamic rate damping as M = rot_damping_coeff * V * omega [N m].
% Proportional to V, NOT to q -- see rocket_step.m. Computed from the fin geometry
% above: 0.5*rho*(4*S_fin)*CLa*arm^2. Gives a damping ratio of 0.059, constant
% across the envelope, which is the physically expected behaviour and sits in the
% real sounding-rocket band of 0.05-0.2. The previous 0.35 with the q*omega law
% gave 7 at rail exit rising to 183 at Mach 1.2.
P.rot_damping_coeff = 0.0458;  % N m / ((m/s) (rad/s))
% Axial drag as Cd * reference area. Both numbers were previously wrong in opposite
% directions (Cd 0.55, A_ref 0.0082) so the total looked plausible. The export gives
% Cd = 0.59 subsonic, 0.68 transonic, 0.69 supersonic; the .ork diameter gives
% A_ref = 0.007707 m^2. 0.65 is representative across boost and early coast.
% Cd and A_ref must be a CONSISTENT pair: OpenRocket normalises its Cd against
% its own reference diameter (4.02 in, CSV col53), NOT the 3.90 in body tube. Using
% the tube area here (the old 0.00501) mixed the two and understated drag ~7%.
P.drag_area       = 0.005376; % Cd(0.656) * A_ref(0.008189, from OR ref diameter)

% ---- launch rail ----
P.rail_length   = 1.8288;   % 72 in
P.rail_tilt_deg = 0.0;      % NAR/Tripoli cap is 20
P.rail_azimuth_deg = 0.0;

% ---- environment ----
P.g = 9.80665;
P.origin_lat = -35.363262;  % SITL default home
P.origin_lon = 149.165237;
P.origin_alt = 584.0;

% ===========================================================================
% REMAINING ASSUMPTION
% ===========================================================================
% Inertia, fin geometry, mass, thrust and stability now come from the OpenRocket
% export and the .ork design file. One assumption is left:
%
%   the CONTROL TAB dimensions. The .ork's tabheight/tablength are the STRUCTURAL
%   through-the-wall mounting tab, not a control surface, so the tab is now a set of
%   millimetre inputs (P.tab above) that you MEASURE on the real airframe. Until then the
%   defaults are the old 25%/75%/20deg assumption in mm; fin_force_gain is derived from
%   them (rocket_fin_gain.m) and updates when you set the real numbers.
%
% Measured effect of the corrected plant (8 m/s crosswind, SITL):
%   >100 m/s   mean tilt  2.1 deg   fins  0 us
%   50-100     mean tilt  6.0 deg   fins  9 us
%   25-50      mean tilt 16.4 deg   fins 69 us
%   10-25      mean tilt 26.4 deg   fins 246 us
% i.e. the controller holds while fast and is outrun as dynamic pressure drains.
% The ATC_* gains have never been tuned; that is the next job.

end
