function value = DataDictGetValue(filenameDataDictionary, varName)
    %ave.DataDictGetValue Get a value from a data dictionary (design data).
    % 
    % PARAMETERS
    % filenameDataDictionary ... [char] The file name of the data dictionary that contains the value to get.
    % varName                ... [char] Name of the variable to get.
    % 
    % RETURN
    % value ... Value of the variable.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20201104    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(ischar(filenameDataDictionary), 'Input "filenameDataDictionary" must be a string!');
    assert(ischar(varName), 'Input "varName" must be a string!');

    % Read value from data dictionary
    dataDict = Simulink.data.dictionary.open(filenameDataDictionary);
    dataSection = getSection(dataDict,'Design Data');
    entry = getEntry(dataSection,varName);
    value = entry.getValue();
    dataDict.close();
end

