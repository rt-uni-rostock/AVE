function udpPort = Port(vehicleID, systemID, messageID)
    %ave.network.Port Generate a unique port. The 16-bit port value is set as (binary): 01vv vvvv ssss mmmm,
    % where the bits 'v', 's', 'm' indicate vehicleID, systemID and messageID, respectively.
    % 
    % PARAMETERS
    % vehicleID ... Vehicle identifier in range [0,63].
    % systemID  ... System identifier in range [0,15].
    % messageID ... Message identifier in range [0,15].
    % 
    % RETURN
    % udpPort ... A unique UDP port in range [16384,32767].
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20250319    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vehicleID = mod(uint16(vehicleID),64);
    systemID = mod(uint16(systemID),16);
    messageID = mod(uint16(messageID),16);
    udpPort = uint16(16384) + uint16(vehicleID)*uint16(256) + uint16(systemID)*uint16(16) + uint16(messageID);
end

