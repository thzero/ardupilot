function [fin, ok] = rocket_servo_decode(bytes) %#codegen
% ROCKET_SERVO_DECODE  Servo packet from SITL (--model JSON) -> fin deflections.
%
% MATLAB Function block that unpacks the binary servo packet ArduPilot's JSON backend
% sends. Same format the script bridge (rocket_sim.m) decodes:
%
%   40 bytes: magic uint16 = 18458 | frame_rate uint16 | frame_count uint32 |
%             pwm[16] uint16
%
% PWM 1000..2000 maps to a normalised fin deflection -1..1 on outputs 1-4.
%
%   fin  4x1 in -1..1
%   ok   true if a valid servo packet was decoded (feed this to an Enable/trigger so
%        the model only steps on a real packet -- see README.md, lockstep note)

fin = zeros(4,1);
ok  = false;
if numel(bytes) < 40
    return
end
b = uint8(bytes(:)).';
magic = typecast(b(1:2), 'uint16');
if magic ~= 18458          % 29569 would be the 32-channel variant; we send 16
    return
end
pwm = typecast(b(9:16), 'uint16');     % first 4 channels = 8 bytes
fin = (double(pwm(:)) - 1500) / 500;
fin = max(min(fin, 1), -1);
ok  = true;
end
