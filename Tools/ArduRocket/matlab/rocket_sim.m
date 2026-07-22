function rocket_sim(varargin)
% ROCKET_SIM  MATLAB physics engine driving ArduRocket SITL over the JSON backend.
%
% MATLAB owns the rocket physics. ArduPilot runs OUR REAL FLIGHT CODE, unmodified,
% and decides the fin deflections. Every step:
%
%     MATLAB  --- JSON sensor state (UDP) --->  ArduPilot SITL
%             <-- binary packet, 16x PWM ----
%
% so the loop is closed through the actual controller, and you watch the reaction
% here in MATLAB.
%
% USAGE
%   1. Start SITL:   build/sitl/bin/rocket --model JSON --speedup 1 -w
%      (it will sit at "Waiting for connection" until a GCS/script attaches to
%       TCP 5760 -- that is separate from this physics link)
%   2. In MATLAB:    rocket_sim
%   3. Arm from your GCS. Ignition follows 3 s later.
%
% OPTIONS (name/value)
%   'Duration'  seconds of flight to simulate      (default 40)
%   'Port'      UDP port MATLAB listens on         (default 9002)
%   'Plot'      plot results at the end            (default true)
%
% PROTOCOL NOTES, all verified against libraries/SITL/SIM_JSON.{h,cpp}
%   - SITL SENDS servo packets to 127.0.0.1:9002 from an EPHEMERAL source port,
%     and listens for the reply on that same socket. So we must reply to the
%     sender's address/port, not to a fixed port. Do not "fix" this to 9003.
%   - The servo packet is 40 bytes: magic uint16 = 18458, frame_rate uint16,
%     frame_count uint32, pwm[16] uint16.
%   - The JSON reply must be wrapped in NEWLINES, i.e. "\n{...}\n". SIM_JSON
%     converts '\n' to nul and parses between the last two nuls, so a message
%     without a leading newline is silently never parsed.
%   - Required fields: timestamp, imu.gyro, imu.accel_body, velocity, and one of
%     attitude/quaternion. Omitting any means the frame is dropped.
%   - Lockstep: SITL waits for our reply, so MATLAB sets the pace. Run as slow as
%     the model needs; there is no real-time race.

p = inputParser;
addParameter(p,'Duration',40);
addParameter(p,'Port',9002);
addParameter(p,'Plot',true);
parse(p,varargin{:});
opt = p.Results;

P = rocket_params();
S = rocket_init(P);

u = udpport("datagram","IPV4","LocalPort",opt.Port,"Timeout",10);
cleanupObj = onCleanup(@() clear_port(u));
fprintf('Listening on UDP %d. Start SITL with --model JSON.\n', opt.Port);

% history for plotting
N = ceil(opt.Duration/P.dt) + 10;
H = struct('t',nan(N,1),'alt',nan(N,1),'vel',nan(N,1),'tilt',nan(N,1), ...
           'fin',nan(N,4),'q',nan(N,1),'mass',nan(N,1),'thrust',nan(N,1));
k = 0;
lastReport = 0;

while S.t < opt.Duration
    dg = read(u,1,"uint8");
    if isempty(dg) || numel(dg.Data) < 40
        fprintf('No servo packet -- is SITL running with --model JSON?\n');
        break;
    end

    [pwm, ok] = decode_servo_packet(dg.Data);
    if ~ok
        continue;   % not a servo packet; ignore
    end

    % PWM 1000-2000 -> normalised fin deflection -1..1, fins on outputs 1-4
    fin = (double(pwm(1:4)) - 1500) / 500;
    fin = max(min(fin,1),-1);

    S = rocket_step(S, P, fin);

    write(u, [uint8(10) uint8(build_json(S)) uint8(10)], "uint8", ...
          dg.SenderAddress, dg.SenderPort);

    k = k + 1;
    if k <= N
        H.t(k)=S.t; H.alt(k)=-S.pos_ned(3); H.vel(k)=norm(S.vel_ned);
        H.tilt(k)=S.tilt_deg; H.fin(k,:)=fin(:).'; H.q(k)=S.q;
        H.mass(k)=S.mass; H.thrust(k)=S.thrust;
    end

    if S.t - lastReport >= 1.0
        lastReport = S.t;
        fprintf('t=%5.1fs  alt=%7.1fm  v=%6.1fm/s  tilt=%5.2fdeg  fin=[%+.2f %+.2f %+.2f %+.2f]\n', ...
                S.t, -S.pos_ned(3), norm(S.vel_ned), S.tilt_deg, fin);
    end
end

if opt.Plot && k > 1
    plot_results(H, k);
end
end

% ---------------------------------------------------------------------------
function [pwm, ok] = decode_servo_packet(bytes)
% 40 bytes: magic u16 | frame_rate u16 | frame_count u32 | pwm[16] u16
pwm = zeros(1,16,'uint16'); ok = false;
if numel(bytes) < 40, return; end
b = uint8(bytes(:).');
magic = typecast(b(1:2),'uint16');
if magic ~= 18458          % servo_packet_16 magic; 29569 would be the 32ch variant
    return;
end
pwm = typecast(b(9:40),'uint16');
ok = true;
end

% ---------------------------------------------------------------------------
function s = build_json(S)
% Only the fields SIM_JSON actually reads. Frames are NED for position/velocity,
% body for gyro/accel.
%
% accel_body is SPECIFIC FORCE -- what an accelerometer measures, i.e. thrust and
% aero only, with gravity EXCLUDED. A stationary vertical rocket therefore reads
% about +9.81 on the nose axis, not zero. Getting this wrong makes launch
% detection and the EKF both misbehave in ways that look like control bugs.
s = sprintf(['{"timestamp":%.6f,' ...
             '"imu":{"gyro":[%.6f,%.6f,%.6f],"accel_body":[%.6f,%.6f,%.6f]},' ...
             '"position":[%.6f,%.6f,%.6f],' ...
             '"quaternion":[%.8f,%.8f,%.8f,%.8f],' ...
             '"velocity":[%.6f,%.6f,%.6f]}'], ...
            S.t, ...
            S.gyro(1), S.gyro(2), S.gyro(3), ...
            S.accel_body(1), S.accel_body(2), S.accel_body(3), ...
            S.pos_ned(1), S.pos_ned(2), S.pos_ned(3), ...
            S.quat(1), S.quat(2), S.quat(3), S.quat(4), ...
            S.vel_ned(1), S.vel_ned(2), S.vel_ned(3));
end

% ---------------------------------------------------------------------------
function plot_results(H, k)
idx = 1:k;
figure('Name','ArduRocket - MATLAB plant, ArduPilot controller');
subplot(3,2,1); plot(H.t(idx),H.alt(idx));   grid on; ylabel('altitude (m)');
subplot(3,2,2); plot(H.t(idx),H.vel(idx));   grid on; ylabel('speed (m/s)');
subplot(3,2,3); plot(H.t(idx),H.tilt(idx));  grid on; ylabel('tilt from vertical (deg)');
subplot(3,2,4); plot(H.t(idx),H.fin(idx,:)); grid on; ylabel('fin cmd (-1..1)');
    legend('fin1','fin2','fin3','fin4','Location','best');
subplot(3,2,5); plot(H.t(idx),H.q(idx));     grid on; ylabel('dynamic pressure (Pa)'); xlabel('t (s)');
subplot(3,2,6); plot(H.t(idx),H.thrust(idx));grid on; ylabel('thrust (N)'); xlabel('t (s)');
end

% ---------------------------------------------------------------------------
function clear_port(u)
try, clear u; catch, end
end
