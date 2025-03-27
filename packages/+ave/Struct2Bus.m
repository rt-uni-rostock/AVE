function listOfBusNames = Struct2Bus(s, busName, reservedNames)
    %ave.Struct2Bus Create one or several simulink bus objects from a structure and assign them to the base workspace.
    % Simply assign a value using built-in datatypes. When using complex signals, assign values with a non-zero imaginary part.
    % For all bus elements, the following properties are set:
    % Min = [];
    % Max = [];
    % DimensionsMode = 'Fixed';
    % Unit = '';
    % Description = '';
    % 
    % PARAMETERS
    % s             ... [struct] Input structure to be converted to bus objects.
    % busName       ... [char] Name of the bus object.
    % reservedNames ... [cell array] An optional cell array that contains reserved names that must not be used for bus name generation.
    % 
    % RETURN
    % listOfBusNames ... [cell array] List of all bus names that have been assigned to the base workspace.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20221028    Robert Damerius        Initial release.
    % 20230329    Robert Damerius        Limit length of bus names to maximum MATLAB identifier length (namelengthmax).
    % 20230607    Robert Damerius        Renaming 'logical' datatypes to 'boolean' to support boolean data types for use in Simulink.
    % 20230623    Robert Damerius        Remove duplicated assignment of bus element data type.
    % 20241122    Robert Damerius        Add support for enumeration data types.
    % 20250327    Robert Damerius        Add support for multi-dimensional busses.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(nargin < 3)
        reservedNames = cell.empty();
    end

    % Check for valid inputs
    assert(isstruct(s), 'Input "s" must be a struct!');
    assert(ischar(busName), 'Input "busName" must be a character array!');
    assert(iscell(reservedNames), 'Input "reservedNames" must be a cell array!');

    % All signal names of the input struct are reserved and must not be used for automatic signal name generation
    reservedNames = [reshape(reservedNames, [1, numel(reservedNames)]), GetReservedNames(s)];

    % Run a recursive function to process all nested elements
    if(length(busName) > namelengthmax())
        busName((namelengthmax()+1):end) = [];
    end
    [~,listOfBusNames] = Struct2BusRecursive(s, busName, reservedNames, busName);
end


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Private Helper Functions
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [reservedNames,assignedBusNames] = Struct2BusRecursive(s, busName, reservedNames, rootName)
    % Create a simulink bus object that is going to contain all elements of the current struct level
    bus = Simulink.Bus;

    % Update outputs: the bus name must not be used by further signal name generation
    reservedNames = [reservedNames, {busName}];
    assignedBusNames = cell.empty();

    % Go through all fieldnames of the current struct
    strFields = fieldnames(s);
    for i = 1:length(strFields)
        % Get basic signal information
        name = strFields{i};
        dataType = class(s.(name));
        dimensions = size(s.(name));

        % If current field is a struct, then it's a nested bus that has to be converted recursively
        if(strcmp('struct',char(dataType)))
            % Generate a unique name for the nested bus and take into account the maximum MATLAB identifier length (namelengthmax)
            nestedBusName = GenerateUniqueNestedBusName(rootName, name, reservedNames);

            % Add bus element of type bus
            bus.Elements(i).Name = name;
            bus.Elements(i).DataType = ['Bus: ' nestedBusName];
            bus.Elements(i).Complexity = 'real';
            bus.Elements(i).Dimensions = dimensions;
            bus.Elements(i).Min = [];
            bus.Elements(i).Max = [];
            bus.Elements(i).DimensionsMode = 'Fixed';
            bus.Elements(i).Unit = '';
            bus.Elements(i).Description = '';

            % Run recursive function to convert that nested bus
            [reservedNames,newBusNames] = Struct2BusRecursive(s.(name)(1), nestedBusName, reservedNames, rootName);
            assignedBusNames = [assignedBusNames; newBusNames];
            continue;

        % If enumeration type, add prefix 'Enum: '
        elseif(isenum(s.(name)))
            dataType = ['Enum: ' dataType];

        % Rename 'logical' to 'boolean'
        elseif(strcmp('logical',char(dataType)))
            dataType = 'boolean';
        end

        % Otherwise it's a default signal
        bus.Elements(i).Name = name;
        bus.Elements(i).DataType = dataType;
        bus.Elements(i).Dimensions = dimensions;
        bus.Elements(i).Min = [];
        bus.Elements(i).Max = [];
        bus.Elements(i).DimensionsMode = 'Fixed';
        bus.Elements(i).Unit = '';
        bus.Elements(i).Description = '';
        bus.Elements(i).Complexity = 'real';
        if(~isreal(s.(name)))
            bus.Elements(i).Complexity = 'complex';
        end
    end

    % Assign new bus object to the base workspace
    assignin('base', busName, bus);
    assignedBusNames = [assignedBusNames; {busName}];
end

function reservedNames = GetReservedNames(s)
    reservedNames = unique(GetReservedNamesRecursive(s));
end

function reservedNames = GetReservedNamesRecursive(s)
    strFields = fieldnames(s);
    reservedNames = cell.empty();
    for i = 1:length(strFields)
        name = strFields{i};
        dataType = class(s.(name));
        if(strcmp('struct',char(dataType)))
            nestedReservedNames = GetReservedNamesRecursive(s.(name)(1));
            reservedNames = [reservedNames, {name}, nestedReservedNames];
        else
            reservedNames = [reservedNames, {name}];
        end
    end
end

function nestedBusName = GenerateUniqueNestedBusName(rootName,name,reservedNames)
    m = namelengthmax();

    % Initial base name, limited to the maximum number of characters
    baseName = [rootName '_' name];
    if(length(baseName) > m)
        baseName((m+1):end) = [];
    end

    % Use the basename as default nested bus name
    nestedBusName = baseName;

    % As long as generated name already exists, generate a new one (keep maximum length)
    variableNum = uint32(0);
    while(~isempty(find(strcmp(reservedNames,nestedBusName),1)))
        strNumber = num2str(variableNum);
        nestedBusName = [baseName(1:end-length(strNumber)) strNumber];
        variableNum = variableNum + uint32(1);
    end
end

