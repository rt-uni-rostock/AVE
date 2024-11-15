function UpdateMessageComponents(directoryCallbacks, directorySubsystemReferences, filenameDataDictionary, prefixCallbacks)
    %ave.UpdateMessageComponents Generate or update message components to exchange user-defined data between simulink models.
    % Specify your custom signal by a data generation callback function. Using this callback, a set of subsystem
    % reference models is generated for serializing (packing) and deserializing (unpacking) the designed data signal.
    % All input callbacks must be located in an input directory (directoryCallbacks). All subsystem reference models are
    % generated in an output directory (directorySubsystemReferences). A simulink data dictionary containing all the required
    % information is written to an output file (filenameDataDictionary).
    % For the use in a simulink model, add the generated simulink data dictionary to the model data and include the required
    % serializer/deserializer subsystem reference models.
    % During the data generation the base workspace is used as for some components like bus signals as there is no way to
    % programatically create busses other than assigning them to the base workspace. The variables created to the base
    % workspace are saved into the data dictionary and are no longer needed.
    % 
    % 
    % PARAMETERS
    % directoryCallbacks            ... [char] Name of the directory containing the data generation callback functions that are
    %                                   to be called. The directory may contain subdirectories. All sub-directories containing
    %                                   callback functions are added to the path.
    % directorySubsystemReferences  ... [char] Name of the output directory where to store the subsystem reference models for
    %                                   data encoding and decoding. This directory is created if it does not exist.
    % filenameDataDictionary        ... [char] Output filename where to store the simulink data dictionary that contains all
    %                                   generated data including structures, bus objects and simulink signals for data stores.
    % prefixCallbacks               ... [char] Optional prefix string for data generation callbacks in the input directory
    %                                   (directoryCallbacks). Use a prefix to make the callback search process only find those
    %                                   files with that prefix.
    % 
    % 
    % DETAILS
    % The data generation callback function must have the following prototype:
    %     structData = PREFIXCallbackFunction()
    %
    % where PREFIX must be equal to prefixCallbacks.
    % The callback must have no input parameters. The return values are as follows:
    %     structData               ... [struct] Structure containing nested structues and built-in datatypes.
    % 
    % The generated variables (struct, bus, datastore) are named according to the name of the callback excluding the PREFIX.
    % 
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230110    Robert Damerius        Initial release.
    % 20230427    Robert Damerius        Variable names for struct, bus, datastore are set automatically based on the callback
    %                                    function name.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(nargin < 4)
        prefixCallbacks = '';
    end
    assert(ischar(directoryCallbacks) && ~isempty(directoryCallbacks), 'Input "directoryCallbacks" must be a non-empty string!');
    assert(ischar(directorySubsystemReferences) && ~isempty(directorySubsystemReferences), 'Input "directorySubsystemReferences" must be a non-empty string!');
    assert(ischar(filenameDataDictionary) && ~isempty(filenameDataDictionary), 'Input "filenameDataDictionary" must be a non-empty string!');
    assert(ischar(prefixCallbacks), 'Input "prefixCallbacks" must be a string!');
    fprintf('[AVE Message Generator] callbacks="%s" subsystems="%s" datadict="%s"\n',directoryCallbacks,directorySubsystemReferences,filenameDataDictionary);

    % Find all callbacks (all sub-directories containing possible callback functions are added to the path)
    callbackFunctions = FindAllCallbacksAndAddPath(directoryCallbacks, prefixCallbacks);

    % Run data specification callbacks
    [varNames, specificationInfo] = EvalulateAllCallbacks(callbackFunctions, prefixCallbacks);
    CheckForUniqueSpecifications(specificationInfo);

    % Create a fresh data dictionary file containing all variables indicated by "varNames"
    fprintf('[AVE Message Generator] Deleting and recreating data dictionary "%s"\n',filenameDataDictionary);
    ave.CreateDataDictionaryFromBaseWorkspaceVariables(filenameDataDictionary, varNames);

    % Generate CODEC subsystem references and select all signals
    suffixEncoder = '_Pack.slx';
    suffixDecoder = '_Unpack.slx';
    GenerateCodecSubsystemReferences(directorySubsystemReferences, specificationInfo, filenameDataDictionary, suffixEncoder, suffixDecoder);
    fprintf('[AVE Message Generator] Completed\n');

    %#TODO: only update subsystem references if structure has been changed (names, structural change)
    %#TODO: also remove subsystem references that are no longer needed
end


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Private helper functions
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function callbackFunctions = FindAllCallbacksAndAddPath(directoryCallbacks, prefixCallbacks)
    % Get list of all MATLAB files that are in that directory, including all subdirectories
    listings = dir(fullfile(directoryCallbacks, ['**',filesep,strcat(prefixCallbacks,'*.m')]));

    % Ignore directories
    listings = listings(~[listings.isdir]);

    % Get all function names and add folder containing the file to the path
    callbackFunctions = cell.empty();
    fprintf('[AVE Message Generator] Found %d callback(s) in "%s"\n', numel(listings), directoryCallbacks);
    for i = 1:numel(listings)
        callbackFunctions{i} = extractBefore(listings(i).name, ".");
        addpath(listings(i).folder);
    end
end

function [structName, busName, dataStoreName, prefixSubsystemReference] = GenerateNames(functionName, prefixCallbacks)
    assert(startsWith(functionName,prefixCallbacks),['The callback function name "', functionName, '" does not start with the prefix "', prefixCallbacks, '"!']);
    suffix = functionName((1+length(prefixCallbacks)):end);
    assert(~isempty(suffix),['The callback function name "', functionName, '" just consists the prefix "', prefixCallbacks, '" but has no actual name to be used to automatically generate names for bus, struct, datastore, etc.!']);
    structName = strcat('struct',suffix);
    busName = strcat('bus',suffix);
    dataStoreName = strcat('datastore',suffix);
    prefixSubsystemReference = strcat('MESSAGE_',suffix);
end

function [varNames, specificationInfo] = EvalulateAllCallbacks(callbackFunctions, prefixCallbacks)
    varNames = cell.empty();
    specificationInfo = cell.empty();
    for i = 1:numel(callbackFunctions)
        % Run the callback
        fprintf('[AVE Message Generator] (%d / %d) Running callback "%s"\n',i,numel(callbackFunctions),callbackFunctions{i});
        structData = eval(callbackFunctions{i});
        [structName, busName, dataStoreName, prefixSubsystemReference] = GenerateNames(callbackFunctions{i}, prefixCallbacks);

        % Check return values
        assert(isstruct(structData), ['Return value (structData) of callback function "',callbackFunctions{i},'" must be a struct!']);

        % Create simulink bus objects and a simulink signal object in the base workspace and save variable names
        newVarNames = ave.CreateSimulinkDataObjectsFromStruct(structData, structName, busName, dataStoreName, varNames); % all current varNames are reserved names and must not be used
        varNames = [varNames, newVarNames];

        % Save specification information
        specificationInfo{i}.callbackFunction = callbackFunctions{i};
        specificationInfo{i}.structData = structData;
        specificationInfo{i}.structName = structName;
        specificationInfo{i}.busName = busName;
        specificationInfo{i}.dataStoreName = dataStoreName;
        specificationInfo{i}.prefixSubsystemReference = prefixSubsystemReference;
    end
end

function CheckForUniqueSpecifications(specificationInfo)
    % Variable names must be unique for struct, bus, datastore
    for i = 1:numel(specificationInfo)
        for j = (i+1):numel(specificationInfo)
            % Check structure name vs all names
            assert(~strcmp(specificationInfo{i}.structName, specificationInfo{j}.structName),    ['Structure name "' , specificationInfo{j}.structName,    '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as structure name by callback "',  specificationInfo{i}.callbackFunction, '"!']);
            assert(~strcmp(specificationInfo{i}.structName, specificationInfo{j}.busName),       ['Bus name "'       , specificationInfo{j}.busName,       '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as structure name by callback "',  specificationInfo{i}.callbackFunction, '"!']);
            assert(~strcmp(specificationInfo{i}.structName, specificationInfo{j}.dataStoreName), ['Data store name "', specificationInfo{j}.dataStoreName, '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as structure name by callback "',  specificationInfo{i}.callbackFunction, '"!']);

            % Check bus name vs all names
            assert(~strcmp(specificationInfo{i}.busName,       specificationInfo{j}.structName),    ['Structure name "' , specificationInfo{j}.structName,    '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as bus name by callback "',        specificationInfo{i}.callbackFunction, '"!']);
            assert(~strcmp(specificationInfo{i}.busName,       specificationInfo{j}.busName),       ['Bus name "'       , specificationInfo{j}.busName,       '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as bus name by callback "',        specificationInfo{i}.callbackFunction, '"!']);
            assert(~strcmp(specificationInfo{i}.busName,       specificationInfo{j}.dataStoreName), ['Data store name "', specificationInfo{j}.dataStoreName, '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as bus name by callback "',        specificationInfo{i}.callbackFunction, '"!']);

            % Check data store name vs all names
            assert(~strcmp(specificationInfo{i}.dataStoreName, specificationInfo{j}.structName),    ['Structure name "' , specificationInfo{j}.structName,    '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as data store name by callback "', specificationInfo{i}.callbackFunction, '"!']);
            assert(~strcmp(specificationInfo{i}.dataStoreName, specificationInfo{j}.busName),       ['Bus name "'       , specificationInfo{j}.busName,       '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as data store name by callback "', specificationInfo{i}.callbackFunction, '"!']);
            assert(~strcmp(specificationInfo{i}.dataStoreName, specificationInfo{j}.dataStoreName), ['Data store name "', specificationInfo{j}.dataStoreName, '" defined by callback "', specificationInfo{j}.callbackFunction, '" has already been defined as data store name by callback "', specificationInfo{i}.callbackFunction, '"!']);
        end
    end
end

function GenerateCodecSubsystemReferences(directorySubsystemReferences, specificationInfo, filenameDataDictionary, suffixEncoder, suffixDecoder)
    % Create output directory
    [~,~] = mkdir(directorySubsystemReferences); % retreive and ignore return values to disable warning prints (directory already exists)

    % For each specification, generate codec (subsystem reference)
    for i = 1:numel(specificationInfo)
        % Create subsystem references (encoder and decoder)
        fprintf('[AVE Message Generator] Creating Message "%s"\n',specificationInfo{i}.prefixSubsystemReference);
        filenameSerializer = fullfile(directorySubsystemReferences, strcat(specificationInfo{i}.prefixSubsystemReference, suffixEncoder));
        filenameDeserializer = fullfile(directorySubsystemReferences, strcat(specificationInfo{i}.prefixSubsystemReference, suffixDecoder));
        ave.CreateMessageSerializer(filenameSerializer, filenameDeserializer, specificationInfo{i}.dataStoreName);

        % Select encoder signals and update decoder based on encoder
        SelectAllSignalsForEncoding(filenameSerializer, specificationInfo{i}.structData, specificationInfo{i}.dataStoreName);
        ave.UpdateMessageSerializer(filenameSerializer, filenameDeserializer, filenameDataDictionary, specificationInfo{i}.structName);
    end
end

function SelectAllSignalsForEncoding(filenameEncoder, structData, dataStoreName)
    % Name of the subsystem reference model
    [~,modelName,~] = fileparts(filenameEncoder);

    % Internal block names (absolute path names)
    nameDataStoreRead  = [modelName '/ElementSelector'];

    % Ensure that no models of the same name are opened
    close_system(modelName, 0); % 0 to ignore warnings if system is not open

    % Generate string for data store elements and update encoder subsystem
    elementNames = GenerateStringOfAllSignals(structData, dataStoreName);
    open_system(filenameEncoder);
    set_param(nameDataStoreRead, 'DataStoreName', dataStoreName);
    set_param(nameDataStoreRead, 'DataStoreElements', elementNames);
    save_system(filenameEncoder);
    close_system(filenameEncoder);
end

function str = GenerateStringOfAllSignals(structData, dataStoreName)
    % Get names and dimension from structure data (data store name indicates root name)
    [names,~,dimensions,~] = ave.GetSignalInfoFromStruct(structData, dataStoreName);

    % Append matrix selector "(:,:,...)" for multidimensional elements to signal name
    for i = 1:numel(names)
        if(prod(dimensions{i}) > 1)
            N = numel(dimensions{i});
            suffix = '(';
            for n = 1:N
                if(1 == n)
                    suffix = strcat(suffix, ':');
                else
                    suffix = strcat(suffix, ',:');
                end
            end
            suffix = strcat(suffix, ')');
            names{i} = strcat(names{i}, suffix);
        end
    end

    % Concatenate all strings using the '#' delimiter
    str = strjoin(names, '#');
end

