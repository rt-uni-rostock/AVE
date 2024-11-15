function UpdateMessageSerializer(filenameSubsystemReferencePack, filenameSubsystemReferenceUnpack, filenameDataDictionary, nameOfStructure)
    %ave.UpdateMessageSerializer Update two subsystem references (unpack and pack). The two subsystem references must base on
    % templates that can be obtained by calling ave.CreateMessageSerializer(). In the pack subsystem reference the signals to
    % be serialized can be selected via the ElementSelector block. This function updates the internal pack and unpack
    % subsystem reference systems based on the selected signals in the serializer. To be able to update the blocks, this function
    % must know the signal specification (structure). Therefore the data dictionary and the corresponding variable name of the
    % designed structure are necessary.
    % 
    % PARAMETERS
    % filenameSubsystemReferencePack    ... [char] Filename of the subsystem reference representing the serializer, e.g. packing.
    % filenameSubsystemReferenceUnpack  ... [char] Filename of the subsystem reference representing the deserializer, e.g. unpacking.
    % filenameDataDictionary            ... [char] Filename of the data dictionary that contains the bus specification.
    % nameOfStructure                   ... [char] The name of the variable that indicates the struct inside the data dictionary.
    % 
    % DETAILS
    % When selecting multi-dimensional signals, all elements of the signal are selected automatically. The selection of, e.g.
    % specific elements of a multi-dimensional signal is not supported!
    % 
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230110    Robert Damerius        Initial release.
    % 20230608    Robert Damerius        Replace logical datatype by boolean datatype for simulink support.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    assert(ischar(filenameSubsystemReferencePack) && ~isempty(filenameSubsystemReferencePack), 'Input "filenameSubsystemReferencePack" must be a non-empty string!');
    assert(ischar(filenameSubsystemReferenceUnpack) && ~isempty(filenameSubsystemReferenceUnpack), 'Input "filenameSubsystemReferenceUnpack" must be a non-empty string!');
    assert(ischar(filenameDataDictionary), 'Input "filenameDataDictionary" must be a string!');
    assert(endsWith(filenameDataDictionary,'.sldd'), 'Input "filenameDataDictionary" must have extension ".sldd"');
    assert(ischar(nameOfStructure) && ~isempty(nameOfStructure), 'Input "nameOfModuleStruct" must be a string with at least one character!');

    % Get model names
    [~,nameSubsystemReferenceEncoder,~] = fileparts(filenameSubsystemReferencePack);
    [~,nameSubsystemReferenceDecoder,~] = fileparts(filenameSubsystemReferenceUnpack);
    assert(~isempty(nameSubsystemReferenceEncoder), 'The actual filename (modelname) of input "filenameSubsystemReferencePack" must not be empty!');
    assert(~isempty(nameSubsystemReferenceDecoder), 'The actual filename (modelname) of input "filenameSubsystemReferenceUnpack" must not be empty!');

    % Get the system structure from the given data dictionary
    structData = ave.DataDictGetValue(filenameDataDictionary, nameOfStructure);

    % Internal block names (absolute path names)
    nameDataStoreRead    = [nameSubsystemReferenceEncoder '/ElementSelector'];
    nameBytePack         = [nameSubsystemReferenceEncoder '/AutogenBytePack'];
    nameNumBytesTX       = [nameSubsystemReferenceEncoder '/AutogenNumBytes'];
    nameNumBytesRX       = [nameSubsystemReferenceDecoder '/ForLoop/AutogenNumBytes'];
    nameByteSelector     = [nameSubsystemReferenceDecoder '/ForLoop/AutogenByteSelector'];
    nameEnabledSubsystem = [nameSubsystemReferenceDecoder '/ForLoop/EnabledSubsystem'];
    nameByteUnpack       = [nameSubsystemReferenceDecoder '/ForLoop/EnabledSubsystem/AutogenByteUnpack'];
    nameDataStoreWrite   = [nameSubsystemReferenceDecoder '/ForLoop/EnabledSubsystem/AutogenDataStoreWrite'];

    % Ensure that no models of the same name are opened
    close_system(nameSubsystemReferenceEncoder, 0); % 0 to ignore warnings if system is not open
    close_system(nameSubsystemReferenceDecoder, 0); % 0 to ignore warnings if system is not open

    % Open subsystems and delete all lines between data store and byte (un-)packing blocks
    open_system(filenameSubsystemReferencePack);
    open_system(filenameSubsystemReferenceUnpack);
    lineHandles = get_param(nameDataStoreRead,'LineHandles');   delete_line(lineHandles.Outport(lineHandles.Outport >= 0));
    lineHandles = get_param(nameBytePack,'LineHandles');        delete_line(lineHandles.Inport(lineHandles.Inport >= 0));
    lineHandles = get_param(nameByteUnpack,'LineHandles');      delete_line(lineHandles.Outport(lineHandles.Outport >= 0));
    lineHandles = get_param(nameDataStoreWrite,'LineHandles');  delete_line(lineHandles.Inport(lineHandles.Inport >= 0));

    % Get the data store name from the element selector
    dataStoreName = get_param(nameDataStoreRead, 'DataStoreName');

    % Get all unique names from element selector
    cellArrayOfSignals = unique(strsplit(get_param(nameDataStoreRead,'DataStoreElements'), '#'));

    % Write back unique names to element selector to ensure no duplicated signal names
    % Write same names to data store write block of decoder subsystem
    % Also make sure that the decoder uses the same data store name
    set_param(nameDataStoreRead,'DataStoreElements',strjoin(cellArrayOfSignals,'#'));
    set_param(nameDataStoreWrite,'DataStoreElements',strjoin(cellArrayOfSignals,'#'));
    set_param(nameDataStoreWrite,'DataStoreName',dataStoreName);

    % Remove array selection fields "(:,:)" for further analysis: only full matrix selection is supported
    cellArrayOfSignals = strrep(cellArrayOfSignals,'(:,:)','');

    % Generate data type list and calculate number of bytes
    numBytes = uint32(0);
    strListOfDataTypes = '';
    strListOfDimensions = '';
    for i = 1:length(cellArrayOfSignals)
        % Iterate through layer names and find final signal
        layerNames = strsplit(cellArrayOfSignals{i},'.');
        currentClass = structData;
        for k = 2:length(layerNames)
            name = layerNames{k};
            currentClass = currentClass.(name);
        end

        % Get data type (lower case string) and dimension of signal
        dataType = lower(class(currentClass));
        dimension = size(currentClass);

        % Replace logical datatype by boolean datatype
        if(strcmp(dataType,'logical'))
            dataType = 'boolean';
        end

        % Get size of data type
        %L = uint32(1);
        switch(dataType)
            case {'double','int64','uint64'}
                L = uint32(8);
            case {'single','int32','uint32'}
                L = uint32(4);
            case {'int16','uint16'}
                L = uint32(2);
            case {'int8','uint8','logical','bool','boolean'}
                L = uint32(1);
            otherwise
                % If current signal is still a struct, then we throw an error
                error(['Data type "' dataType '" is not supported for element selection!']);
        end

        % Update total number of bytes
        numBytes = numBytes + uint32(prod(dimension)) * L;

        % Update list of data types
        if(isempty(strListOfDataTypes))
            strListOfDataTypes = ['''' dataType ''''];
        else
            strListOfDataTypes = strcat(strListOfDataTypes,',''',dataType,'''');
        end

        % Update list of dimensions
        if(isempty(strListOfDimensions))
            strListOfDimensions = ['[' num2str(dimension) ']'];
        else
            strListOfDimensions = strcat(strListOfDimensions,',[',num2str(dimension),']');
        end
    end

    % Write data type list to byte pack/unpack blocks
    set_param(nameBytePack,'datatypes',['{' strListOfDataTypes '}']);
    set_param(nameByteUnpack,'datatypes',['{' strListOfDataTypes '}'],'dimensions',['{' strListOfDimensions '}']);

    % Write number of bytes constant block and byte selector block
    set_param(nameNumBytesTX,'Value',num2str(numBytes));
    set_param(nameNumBytesRX,'Value',num2str(numBytes));
    set_param(nameByteSelector,'Indices',['1:' num2str(numBytes)]);

    % Autoconnect element selector with byte pack block
    portsDataStoreRead = get_param(nameDataStoreRead,'PortHandles');
    portsBytePack = get_param(nameBytePack,'PortHandles');
    add_line(nameSubsystemReferenceEncoder,portsDataStoreRead.Outport,portsBytePack.Inport);

    % Autoconnect byte unpack with data store write block
    portsByteUnpack = get_param(nameByteUnpack,'PortHandles');
    portsDataStoreWrite = get_param(nameDataStoreWrite,'PortHandles');
    add_line(nameEnabledSubsystem,portsByteUnpack.Outport,portsDataStoreWrite.Inport);

    % Save and close subsystems
    save_system(filenameSubsystemReferencePack);
    save_system(filenameSubsystemReferenceUnpack);
    close_system(filenameSubsystemReferencePack);
    close_system(filenameSubsystemReferenceUnpack);

    % Report error if number of bytes exceeds maximum UDP data length (65507)
    if(numBytes > uint32(65507))
        warning(['The UDP message data must not be greater than 65507 bytes! However, it is ' num2str(numBytes) ' bytes long! Reduce the size by selecting a lower number of signals!']);
    end
end

