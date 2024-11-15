function ImportBaseWorkspaceVariablesToDataDictionary(filenameDataDictionary, varNames, varargin)
    %ave.ImportBaseWorkspaceVariablesToDataDictionary Import a set of variables from the base workspace to a given simulink
    % data dictionary. The variable names are made unique to prevent duplicated imports. If this data dictionary does not exist
    % then it's created.
    % 
    % PARAMETERS
    % filenameDataDictionary ... [char] The filename of the simulink data dictionary to be written.
    % varNames               ... [cell array] Cell array of variable names indicating all variables in the base workspace that
    %                            are to be imported into the simulink data dictionary.
    % append                 ... (optional) true if data dictionary should be appended with variables, false if it should be
    %                            recreated. The default value is true.
    % 
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230110    Robert Damerius        Initial release.
    % 20230427    Robert Damerius        Add optional argument for appending data to the existing data dictionary.
    % 20230623    Robert Damerius        Closing all data dictionaries before importing the data.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(ischar(filenameDataDictionary), 'Input "filenameDataDictionary" must be a string!');
    assert(endsWith(filenameDataDictionary,'.sldd'), 'Input "filenameDataDictionary" must have extension ".sldd"');
    assert(iscell(varNames), 'Input "varNames" must be a cell array!');

    % Handle optional arguments
    append = true;
    if(nargin > 2)
        append = logical(varargin{1});
        assert(isscalar(append),'Optional input "append" must be scalar!');
    end

    % Ensure unique names in the list of variables
    varNames = unique(varNames);

    % Delete data dictionary if appending is disabled
    if(~append && isfile(filenameDataDictionary))
        delete(filenameDataDictionary);
    end

    % Import into data dictionary
    Simulink.data.dictionary.closeAll();
    if(isfile(filenameDataDictionary))
        dictionaryObject = Simulink.data.dictionary.open(filenameDataDictionary);
        importFromBaseWorkspace(dictionaryObject,'varList',varNames);
        saveChanges(dictionaryObject);
        close(dictionaryObject);
    else
        dictionaryObject = Simulink.data.dictionary.create(filenameDataDictionary);
        importFromBaseWorkspace(dictionaryObject,'varList',varNames);
        saveChanges(dictionaryObject);
        close(dictionaryObject);
    end
end

