function CheckMessageSerializers(listOfSubsystemReferences)
    %ave.CheckMessageSerializers Check a list of subsystem references that represent message serializers. All signals are
    % checked for uniqueness. A warning message is generated if the check fails.
    % 
    % PARAMETERS
    % listOfSubsystemReferences ... A cell array of strings representing the names of serializer subsystems to be checked, e.g. {'M1_Pack','M2_Pack','M3_Pack'}.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221101    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    numSystems = length(listOfSubsystemReferences);
    for i = 1:numSystems
        iSubSysRef = listOfSubsystemReferences{i};
        open_system(iSubSysRef);
        iSignals = strsplit(get_param([iSubSysRef '/ElementSelector'],'DataStoreElements'), '#');
        close_system(iSubSysRef);
        for j = (i + 1):numSystems
            jSubSysRef = listOfSubsystemReferences{j};
            open_system(jSubSysRef);
            jSignals = strsplit(get_param([jSubSysRef '/ElementSelector'],'DataStoreElements'), '#');
            close_system(jSubSysRef);
            iIndices = find(ismember(iSignals,jSignals));
            if(~isempty(iIndices))
                warning(['WARNING: The two reference subsystems "' iSubSysRef '" and "' jSubSysRef '" are designed to encode the same signal! The dubplicated signals are: "' strjoin(iSignals(iIndices),'", "') '"']);
            end
        end
    end
end

