function build_rocket_model(modelName)
% BUILD_ROCKET_MODEL  Assemble the Aerospace Blockset plant model programmatically.
%
%   build_rocket_model            % creates 'ardurocket_plant'
%   build_rocket_model('myname')
%
% SCAFFOLD, NOT A FINISHED MODEL. This has never been run (no MATLAB/Simulink in the
% environment it was written in), and the Aerospace Blockset / UDP block library PATHS
% vary by release, so library blocks are added with a try/catch: if a path is wrong the
% script WARNS and continues, and you drag that block in by hand. It reliably creates the
% model, adds the three MATLAB Function blocks (calling the external functions in this
% folder), positions everything, and wires the connections it safely can. You finish the
% thrust/mass subsystem, the rail constraint, and the 6DOF port wiring -- see README.md.
%
% The MATLAB Function blocks are thin wrappers that CALL rocket_aero_fcn / _servo_decode /
% _json_encode, so there is one source of truth (this folder must be on the MATLAB path).

if nargin < 1
    modelName = 'ardurocket_plant';
end

% This folder must be on the path so the model can call the function-block sources.
here = fileparts(mfilename('fullpath'));
addpath(here, fileparts(here));   % simulink/ and the parent matlab/ (for rocket_params)

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% Load the airframe constants into the model workspace so the blocks can use P.
hws = get_param(modelName, 'ModelWorkspace');
assignin(hws, 'P', rocket_params());

    function tryAdd(src, name, pos)
        % Add a library block; warn (do not error) if the library path is wrong so the
        % script can finish. Leaves a labelled placeholder you replace by hand.
        dst = [modelName '/' name];
        try
            add_block(src, dst, 'Position', pos);
        catch ME
            warning('build_rocket_model:block', ...
                ['Could not add "%s" from "%s" (%s).\n' ...
                 '  -> drag this block in manually and name it "%s".'], ...
                name, src, ME.message, name);
            add_block('built-in/Note', dst, 'Position', pos, ...
                      'Text', sprintf('PLACE BY HAND:\n%s', src)); %#ok<*NASGU>
        end
    end

    function addFcn(name, code, pos)
        % A MATLAB Function block whose body is `code` (a call to the external function).
        dst = [modelName '/' name];
        add_block('simulink/User-Defined Functions/MATLAB Function', dst, 'Position', pos);
        try
            cfg = get_param(dst, 'MATLABFunctionConfiguration');
            cfg.FunctionScript = code;
        catch ME
            warning('build_rocket_model:fcn', ...
                ['Could not set code for "%s" via API (%s).\n' ...
                 '  -> open the block and paste:\n%s'], name, ME.message, code);
        end
    end

% --- MATLAB Function blocks (reliable): thin wrappers over the external functions ---
addFcn('servo_decode', ...
    ['function [fin, ok] = servo_decode(bytes)' newline ...
     '  [fin, ok] = rocket_servo_decode(bytes);' newline 'end'], ...
    [60 60 160 110]);
addFcn('aero', ...
    ['function [F_body, M_body] = aero(fin, vel_air_bf, gyro, rho, P)' newline ...
     '  [F_body, M_body] = rocket_aero_fcn(fin, vel_air_bf, gyro, rho, P);' newline 'end'], ...
    [300 60 420 140]);
addFcn('json_encode', ...
    ['function bytes = json_encode(t, gyro, accel_body, pos_ned, quat, vel_ned)' newline ...
     '  bytes = rocket_json_encode(t, gyro, accel_body, pos_ned, quat, vel_ned);' newline 'end'], ...
    [560 200 700 280]);

% --- Library blocks (best-effort paths; warn if the release differs) ---
tryAdd('aeroblks/Equations of Motion/6DOF/Custom Variable Mass 6DOF (Quaternion)', ...
       'sixdof', [480 60 560 160]);
tryAdd('aeroblks/Environment/Atmosphere/ISA Atmosphere Model', ...
       'atmos', [300 200 400 250]);
tryAdd('instrument/UDP Receive', 'udp_rx', [40 200 120 250]);
tryAdd('instrument/UDP Send',    'udp_tx', [740 200 820 250]);

% --- Wire the connections we can do safely (the rest is manual, see README.md) ---
    function tryLine(a, b)
        try
            add_line(modelName, a, b, 'autorouting', 'on');
        catch
            % ports differ by release / the block was placed by hand -- skip quietly
        end
    end
tryLine('udp_rx/1',       'servo_decode/1');   % bytes -> decode
tryLine('servo_decode/1', 'aero/1');           % fin   -> aero
tryLine('atmos/1',        'aero/4');            % rho   -> aero (check ISA output port #)
tryLine('json_encode/1',  'udp_tx/1');         % bytes -> send

Simulink.BlockDiagram.arrangeSystem(modelName);   % tidy layout
save_system(modelName);

fprintf(['\nBuilt "%s" (scaffold). Finish by hand (see README.md):\n' ...
         '  1. Thrust & mass subsystem -> 6DOF Mass / dMass/dt / Inertia inputs.\n' ...
         '  2. Wire aero F_body/M_body (+ thrust) into the 6DOF Forces/Moments inputs.\n' ...
         '  3. 6DOF outputs (pos, vel, quat, pqr, accel) -> json_encode inputs.\n' ...
         '  4. Rail constraint (enabled subsystem overriding the 6DOF on the rail).\n' ...
         '  5. Lockstep: step the model per received packet (servo_decode ok flag).\n' ...
         '  6. Set the 6DOF initial attitude to the rail attitude.\n'], modelName);
end
