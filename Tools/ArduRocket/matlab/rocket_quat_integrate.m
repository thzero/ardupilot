function q = rocket_quat_integrate(q, w, dt)
% ROCKET_QUAT_INTEGRATE  Advance a quaternion by a body-frame rate for dt.
%
%   q  [w x y z] scalar first
%   w  body angular rate [p q r], rad/s
%
% First-order integration with renormalisation. That is adequate at the 400 Hz
% step this model runs at (w*dt stays well below 0.1 rad) but it IS first order:
% the renormalise hides drift rather than preventing it. rocket_selftest measures
% the actual error over a full flight's worth of steps.
qd = 0.5*[ -q(2)*w(1) - q(3)*w(2) - q(4)*w(3);
            q(1)*w(1) + q(3)*w(3) - q(4)*w(2);
            q(1)*w(2) - q(2)*w(3) + q(4)*w(1);
            q(1)*w(3) + q(2)*w(2) - q(3)*w(1)];
q = q + qd*dt;
q = q / norm(q);
end
