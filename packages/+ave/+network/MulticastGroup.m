function group = MulticastGroup(vehicleID, systemID, messageID)
    %ave.network.MulticastGroup Generate a unique multicast group address. The group address is set to 239.vehicleID.systemID.messageID.
    % 
    % PARAMETERS
    % vehicleID ... Vehicle identifier in range [0,255].
    % systemID  ... System identifier in range [0,255].
    % messageID ... Message identifier in range [0,255].
    % 
    % RETURN
    % group ... A unique multicast group address.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20250319    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vehicleID = uint8(vehicleID);
    systemID = uint8(systemID);
    messageID = uint8(messageID);
    group = [repmat(uint8(239),[numel(vehicleID),1]), reshape(vehicleID,[],1), reshape(systemID,[],1), reshape(messageID,[],1)];
end

