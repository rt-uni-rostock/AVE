function [signalNames, signalDataTypes, signalDimensions, signalComplexity] = GetSignalInfoFromStruct(s, rootLevelName)
    %ave.GetSignalInfoFromStruct Get signal information from a structure. All outputs have the same number of elements.
    % 
    % PARAMETERS
    % s                ... [struct] The input structure.
    % rootLevelName    ... [char] Optional root level name to be set for signal names, e.g. the name of the struct.
    % 
    % RETURN
    % signalNames      ... [cell array] Signal names (char) of all leaf elements including the whole branch name, e.g.
    %                      "rootName.nestedStruct.value".
    % signalDataTypes  ... [cell array] Datatypes (char) of all leaf elements.
    % signalDimensions ... [cell array] Dimension (numeric) of all leaf elements.
    % signalComplexity ... [cell array] Signal complexity (char) of all leaf elements, either 'real' or 'complex'.
    % 
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230110    Robert Damerius        Initial release.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(nargin < 2)
        rootLevelName = '';
    end
    assert(isstruct(s), 'Input "s" must be a struct!');
    assert(ischar(rootLevelName), 'Input "rootLevelName" must be a struct!');
    [signalNames, signalDataTypes, signalDimensions, signalComplexity] = FindSignalInfoRecursive(s, rootLevelName);
end


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Private Helper Functions
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [signalNames, signalDataTypes, signalDimensions, signalComplexity] = FindSignalInfoRecursive(s, structureName)
    signalNames = cell.empty();
    signalDataTypes = cell.empty();
    signalDimensions = cell.empty();
    signalComplexity = cell.empty();

    % Go through all fieldnames of the current struct
    strFields = fieldnames(s);
    for i = 1:length(strFields)
        % Get basic signal information
        name = strFields{i};
        dataType = class(s.(name));
        dimensions = size(s.(name));
        complexity = 'real';
        if(~isreal(s.(name)))
            complexity = 'complex';
        end

        % Set name (root to this element)
        elementName = name;
        if(~isempty(structureName))
            elementName = strcat(structureName,'.',name);
        end

        % If current field is a struct, find those signal info recursively
        if(strcmp('struct',char(dataType)))
            [nestedNames, nestedDataTypes, nestedDimensions, nestedomplexity] = FindSignalInfoRecursive(s.(name), elementName);
            signalNames = [signalNames; nestedNames];
            signalDataTypes = [signalDataTypes; nestedDataTypes];
            signalDimensions = [signalDimensions; nestedDimensions];
            signalComplexity = [signalComplexity; nestedomplexity];
        else
            % Otherwise it's a default signal
            signalNames = [signalNames; {elementName}];
            signalDataTypes = [signalDataTypes; {dataType}];
            signalDimensions = [signalDimensions; {dimensions}];
            signalComplexity = [signalComplexity; {complexity}];
        end
    end
end

