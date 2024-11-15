function varNames = CreateSimulinkDataObjectsFromStruct(inputStructure, structureName, busName, dataStoreName, reservedNames)
    %ave.CreateSimulinkDataObjectsFromStruct Create simulink bus objects and a simulink signal object to be used for data store
    % read/write blocks based on a given input structure. The input structure must only contain built-in datatypes or nested
    % structures. For all nested structures, bus objects with auto-generated names are created. The structure data object, all
    % bus objects and the simulink signal object are assigned to the base workspace under the specified names (there is no way
    % to programatically create busses other than assigning them to the base workspace).
    % 
    % PARAMETERS
    % inputStructure      ... [struct] The input structure to be converted. It must only contain nested busses or built-in
    %                         datatypes such as double, int32, uint8, etc. (logical datatypes are replaced by boolean ones).
    % structureName       ... [char] The name under which the input structure is to be written to the base workspace.
    % busName             ... [char] The main bus name of the root object. All nested busses are named automatically based on
    %                         this name.
    % dataStoreName       ... [char] The name of the simulink signal object to be used as data store name in data store
    %                         read/write blocks.
    % reservedNames       ... [cell array] An optional cell array that contains reserved names that must not be used for bus
    %                         name generation.
    % 
    % RETURN
    % varNames            ... [cell array] List of all variable names that have been assigned to the base workspace including
    %                         structure, bus objects and the simulink signal object.
    % 
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230109    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    % Check for valid inputs
    if(nargin < 5)
        reservedNames = cell.empty();
    end
    assert(isstruct(inputStructure), 'Input "inputStructure" must be a struct!');
    assert(ischar(structureName) && ~isempty(structureName), 'Input "structureName" must be a string with at least one character!');
    assert(ValidString(structureName), 'Input "structureName" is not a valid string. Allowed characters are: "0...9", "a...z", "A...Z" and "_"! The first character must be a letter!');
    assert(ischar(busName) && ~isempty(busName), 'Input "busName" must be a string with at least one character!');
    assert(ValidString(busName), 'Input "busName" is not a valid string. Allowed characters are: "0...9", "a...z", "A...Z" and "_"! The first character must be a letter!');
    assert(ischar(dataStoreName) && ~isempty(dataStoreName), 'Input "dataStoreName" must be a string with at least one character!');
    assert(ValidString(dataStoreName), 'Input "dataStoreName" is not a valid string. Allowed characters are: "0...9", "a...z", "A...Z" and "_"! The first character must be a letter!');
    assert(iscell(reservedNames), 'Input "reservedNames" must be a cell array!');
    assert(~any(strcmp(reservedNames,structureName)), ['Input "reservedNames" contains the structure name "', structureName, '"!']);
    assert(~any(strcmp(reservedNames,busName)), ['Input "reservedNames" contains the bus name "', busName, '"!']);
    assert(~any(strcmp(reservedNames,dataStoreName)), ['Input "reservedNames" contains the data store name "', dataStoreName, '"!']);

    % Assign input structure to the base workspace
    assignin('base',structureName,inputStructure);

    % Create bus objects (are assigned to the base workspace)
    listOfBusNames = ave.Struct2Bus(inputStructure, busName, [reservedNames, {structureName}, {busName}, {dataStoreName}]);

    % Generate a simulink signal (to base workspace)
    evalin('base',[dataStoreName, '              = Simulink.Signal;']);
    evalin('base',[dataStoreName, '.Description  = ''Data store name consistent with bus "', busName, '" and struct "', structureName ,'"'';']);
    evalin('base',[dataStoreName, '.DataType     = ''Bus: ', busName, ''';']);
    evalin('base',[dataStoreName, '.InitialValue = ''', structureName, ''';']);
    evalin('base',[dataStoreName, '.Complexity   = ''real'';']);

    % List of all variables that have to be assigned to the data dictionary
    varNames = [{structureName}, {busName}, {dataStoreName}, reshape(listOfBusNames, [1, numel(listOfBusNames)])];
end

function valid = ValidString(str)
    valid = false;
    if(isempty(str))
        return;
    end
    if(~(((str(1) >= 'A') && (str(1) <= 'Z')) || ((str(1) >= 'a') && (str(1) <= 'z'))))
        return;
    end
    for i = 2:strlength(str)
        if((str(i) >= '0') && (str(i) <= '9'))
            continue;
        elseif((str(i) >= 'A') && (str(i) <= 'Z'))
            continue;
        elseif((str(i) >= 'a') && (str(i) <= 'z'))
            continue;
        elseif('_' == str(i))
            continue;
        end
        return;
    end
    valid = true;
end

