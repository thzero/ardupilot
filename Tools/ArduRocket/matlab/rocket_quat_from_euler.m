function q = rocket_quat_from_euler(roll, pitch, yaw)
% ROCKET_QUAT_FROM_EULER  Quaternion from a 3-2-1 (yaw-pitch-roll) sequence.
%   Angles in radians. Returns [w x y z], scalar first.
%   Equivalent to angle2quat(yaw,pitch,roll) from the Aerospace Toolbox.
cr=cos(roll/2); sr=sin(roll/2);
cp=cos(pitch/2); sp=sin(pitch/2);
cy=cos(yaw/2);  sy=sin(yaw/2);
q = [cr*cp*cy + sr*sp*sy;
     sr*cp*cy - cr*sp*sy;
     cr*sp*cy + sr*cp*sy;
     cr*cp*sy - sr*sp*cy];
end
