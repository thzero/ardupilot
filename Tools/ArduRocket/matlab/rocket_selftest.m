function rocket_selftest()
% ROCKET_SELFTEST  Verify the hand-written maths against known answers.
%
%   >> rocket_selftest
%
% These functions exist so the model needs only base MATLAB, with no Aerospace
% Toolbox. That is a reasonable trade ONLY if they are actually correct, so this
% checks them against cases whose answers are known independently rather than
% against the implementation itself.
%
% Every assertion below was first verified in Python, so a failure here means a
% MATLAB porting error rather than wrong maths.

fprintf('\n=== ArduRocket MATLAB self-test ===\n\n');
pass = 0; fail = 0;

    function check(name, got, want, tol)
        if all(abs(got(:) - want(:)) < tol)
            fprintf('  PASS  %s\n', name); pass = pass + 1;
        else
            fprintf('  FAIL  %s\n        got  %s\n        want %s\n', name, ...
                    mat2str(got(:).', 5), mat2str(want(:).', 5));
            fail = fail + 1;
        end
    end

%% 1. Identity -------------------------------------------------------------
check('identity quaternion -> identity matrix', ...
      rocket_quat_to_dcm([1;0;0;0]), eye(3), 1e-12);

%% 2. Known 90 degree rotations --------------------------------------------
% A quaternion for +90 deg about an axis is [cos45; sin45*axis].
s = sqrt(0.5);
% +90 about X: Y->Z, Z->-Y
check('90 deg about X', rocket_quat_to_dcm([s;s;0;0]), ...
      [1 0 0; 0 0 -1; 0 1 0], 1e-12);
% +90 about Y: Z->X, X->-Z
check('90 deg about Y', rocket_quat_to_dcm([s;0;s;0]), ...
      [0 0 1; 0 1 0; -1 0 0], 1e-12);
% +90 about Z: X->Y, Y->-X
check('90 deg about Z', rocket_quat_to_dcm([s;0;0;s]), ...
      [0 -1 0; 1 0 0; 0 0 1], 1e-12);

%% 3. Euler round trip -----------------------------------------------------
% -90 pitch must put body X (the nose) along NED -Z, i.e. straight up. This is
% the exact case rocket_init relies on, so getting it wrong would tilt the
% whole model over.
R = rocket_quat_to_dcm(rocket_quat_from_euler(0, -pi/2, 0));
check('nose-up: body X maps to NED -Z', R(:,1), [0;0;-1], 1e-12);

%% 4. Rotation matrices must be orthonormal --------------------------------
q = rocket_quat_from_euler(0.3, -0.7, 1.1);
R = rocket_quat_to_dcm(q);
check('R''*R = I (orthonormal)', R.'*R, eye(3), 1e-12);
check('det(R) = +1 (no reflection)', det(R), 1, 1e-12);

%% 5. Quaternion multiply --------------------------------------------------
check('q * identity = q', rocket_quat_mul(q, [1;0;0;0]), q, 1e-12);
% Two 90 deg turns about Z compose to 180 deg about Z
check('90+90 about Z = 180 about Z', ...
      rocket_quat_to_dcm(rocket_quat_mul([s;0;0;s], [s;0;0;s])), ...
      [-1 0 0; 0 -1 0; 0 0 1], 1e-12);

%% 6. Integration accuracy -------------------------------------------------
% Spin at a constant rate for exactly a quarter turn and see where we land.
% This is the one that matters: it measures the first-order integrator's error
% at the step size the model actually runs at.
dt = 0.0025;                 % 400 Hz, as the model runs
rate = 1.0;                  % rad/s about Z
nsteps = round((pi/2) / (rate*dt));
qi = [1;0;0;0];
for k = 1:nsteps
    qi = rocket_quat_integrate(qi, [0;0;rate], dt);
end
ang = 2*atan2(norm(qi(2:4)), qi(1));
check('quarter turn integrates to pi/2', ang, pi/2, 1e-3);
check('quaternion stays unit norm', norm(qi), 1, 1e-12);

% Drift over a whole flight's worth of steps at a realistic body rate.
qi = [1;0;0;0];
for k = 1:24000               % 60 s at 400 Hz
    qi = rocket_quat_integrate(qi, [0.05; -0.02; 0.03], dt);
end
check('unit norm after 24000 steps', norm(qi), 1, 1e-10);

%% 7. Atmosphere -----------------------------------------------------------
% ISA reference values. atmosisa would give these; so must we.
check('density at sea level',  rocket_air_density(0),    1.2250, 1e-3);
check('density at 1000 m',     rocket_air_density(1000), 1.1116, 1e-3);
check('density at 3000 m',     rocket_air_density(3000), 0.9091, 1e-3);
check('density at 11000 m',    rocket_air_density(11000), 0.3639, 1e-3);

%% 8. Physics sanity -------------------------------------------------------
% Sitting on the pad the accelerometer must read +1g along the nose, NOT zero.
% Getting this wrong breaks launch detection and the EKF in ways that look like
% control bugs, so it is worth asserting.
P = rocket_params();
S = rocket_init(P);
check('stationary accel_body = +g on the nose', S.accel_body, [P.g;0;0], 1e-9);
check('starts vertical', S.tilt_deg, 0, 1e-9);

Rn = rocket_quat_to_dcm(S.quat);
check('initial attitude points the nose up', Rn(:,1), [0;0;-1], 1e-9);

%% -------------------------------------------------------------------------
fprintf('\n  %d passed, %d failed\n\n', pass, fail);
if fail > 0
    error('rocket_selftest: %d check(s) failed', fail);
end
end
