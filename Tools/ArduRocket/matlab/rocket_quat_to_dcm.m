function R = rocket_quat_to_dcm(q)
% ROCKET_QUAT_TO_DCM  Rotation matrix (body -> NED) from a quaternion.
%
%   q = [w x y z], scalar first, unit norm.
%
% Equivalent to quat2dcm(q)' from the Aerospace Toolbox -- note the TRANSPOSE:
% quat2dcm returns NED->body, this returns body->NED. Written out rather than
% called so the model needs only base MATLAB. Verified by rocket_selftest.
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y);
       2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x);
       2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)];
end
