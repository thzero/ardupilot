function [F_body, M_body] = rocket_aero_fcn(fin, vel_air_bf, gyro, rho, P) %#codegen
% ROCKET_AERO_FCN  Body-frame aerodynamic force and moment for the Simulink plant.
%
% Drop this into a MATLAB Function block in the Aerospace Blockset model (see
% README.md in this folder). It computes the SAME aerodynamics as the script model's
% rocket_step.m -- KEEP THE TWO IN SYNC; a divergence here is exactly the C++/MATLAB
% split that bit this project before.
%
% Thrust is NOT included here. It is added to the 6DOF block's force input by the
% thrust/mass subsystem, because thrust acts on the mass the 6DOF block also integrates.
%
% Inputs
%   fin         4x1 normalised fin deflections, -1..1 (from the servo-decode block)
%   vel_air_bf  3x1 air-relative velocity in the BODY frame, m/s
%   gyro        3x1 body rates [spin; tilt_y; tilt_z], rad/s
%   rho         air density, kg/m^3 (from the ISA Atmosphere block)
%   P           struct of constants from rocket_params(): fin_force_gain, fin_arm_m,
%               fin_radius_m, fin_angle_deg(1x4), stability_gain, rot_damping_coeff,
%               drag_area  (set as a Simulink parameter -- see README.md)
% Outputs
%   F_body      3x1 aerodynamic force (drag only), body frame, N
%   M_body      3x1 aerodynamic moment (fins + stability + damping), N.m

speed = norm(vel_air_bf);
q = 0.5 * rho * speed^2;

% angle of attack / sideslip about the nose axis (small-angle), guarded at low speed
if speed > 0.5
    alpha = atan2(vel_air_bf(3), vel_air_bf(1));
    beta  = atan2(vel_air_bf(2), vel_air_bf(1));
else
    alpha = 0;
    beta  = 0;
end

% Fins, ONE AT A TIME -- no mixing (that convention lives only in AP_FinMixerRocket).
% Each fin's lift acts tangentially: M_x from the radius, M_y/M_z from the axial arm.
M = zeros(3,1);
for i = 1:4
    f  = fin(i) * P.fin_force_gain * q;   % N, tangential
    th = deg2rad(P.fin_angle_deg(i));
    M(1) = M(1) - f * P.fin_radius_m;
    M(2) = M(2) - f * P.fin_arm_m * cos(th);
    M(3) = M(3) - f * P.fin_arm_m * sin(th);
end

% Weathercock / static stability. Negative stability_gain = CP aft of CG = self-correcting.
M(2) = M(2) + alpha * P.stability_gain * q;
M(3) = M(3) - beta  * P.stability_gain * q;

% Aerodynamic rate damping: moment ~ V*omega (NOT q*omega), scaled with density.
% Spin damping uses (fin_radius/fin_arm)^2 = 0.0183.
kd = P.rot_damping_coeff * speed * (rho / 1.225);
M(1) = M(1) - gyro(1) * kd * 0.0183;
M(2) = M(2) - gyro(2) * kd;
M(3) = M(3) - gyro(3) * kd;

% Axial drag along the air-relative velocity.
F_body = zeros(3,1);
if speed > 0.1
    F_body = -(vel_air_bf / speed) * (q * P.drag_area);
end

M_body = M;
end
