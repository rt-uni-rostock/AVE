% generate parameter structure
parameter = struct();
parameter.lever_arm.sensor1 = [1;0;3];
parameter.lever_arm.sensor2 = [-1;0;3];
parameter.max_iterations    = int32(100);
parameter.controller.Kp     = eye(3, 'single');
parameter.controller.Ki     = zeros(3, 'single');

% specify destination
dstAddress = [127,0,0,1];
dstPort    = 12701;
signalName = 'Param';
outputDir  = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1); % this directory

% send UDP message containing parameter structure and generate unpacker subsystem for simulink including data dictionary
ave.SendStructAndBuildUnpacker(parameter, dstAddress, dstPort, signalName, outputDir);
