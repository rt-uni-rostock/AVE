function CreateDataDictionaryFromBaseWorkspaceVariables(filenameDataDictionary, varNames)
    %ave.CreateDataDictionaryFromBaseWorkspaceVariables Create a simulink data dictionary and import a set of variables from the
    % base workspace. The data dictionary is deleted and created. The variable names are made unique to prevent duplicated
    % imports.
    % 
    % PARAMETERS
    % filenameDataDictionary ... [char] The filename of the simulink data dictionary to be written.
    % varNames               ... [cell array] Cell array of variable names indicating all variables in the base workspace that
    %                            are to be imported into the simulink data dictionary.
    % 
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230110    Robert Damerius        Initial release.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(ischar(filenameDataDictionary), 'Input "filenameDataDictionary" must be a string!');
    assert(endsWith(filenameDataDictionary,'.sldd'), 'Input "filenameDataDictionary" must have extension ".sldd"');
    assert(iscell(varNames), 'Input "varNames" must be a cell array!');

    % Ensure unique names in the list of variables
    varNames = unique(varNames);

    % Delete data dictionary
    Simulink.data.dictionary.closeAll();
    if(isfile(filenameDataDictionary))
        delete(filenameDataDictionary);
    end

    % Create a new fresh data dictionary and import variables
    ave.ImportBaseWorkspaceVariablesToDataDictionary(filenameDataDictionary, varNames);
end

