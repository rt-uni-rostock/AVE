function SendStructAndBuildUnpacker(s, dstAddress, dstPort, signalName, outputDirectory)
    % This function is designed to send structured data in a uni-directional way from MATLAB to simulink, for example to update
    % online parameters for a running model or target application. It serializes a given structure, sends the binary to the
    % specified destination via UDP and generates a subsystem reference model as well as a simulink data dictionary for
    % unpacking the binary message into a simulink bus. The data dictionary and subsystem reference model are only created if
    % they do not exist or structural changes require an update.
    % 
    % INPUT
    % s               ... The structured data to be send.
    % dstAddress      ... A 4-dimensional vector indicating the destination IPv4 address.
    % dstPort         ... The UDP destination port.
    % signalName      ... The name to be used for file and data generation (see below).
    % outputDirectory ... The output directory where to store generated files. If this parameter is not given, then the current
    %                     working directory is used.
    % 
    % NOTES
    % The files and variable names being generated are based on the signalName input. This input is converted to a valid MATLAB
    % variable name. The filenames are as follows ("[NAME]" is replaced by the valid signal name):
    %   * Data Dictionary: "Data_[NAME].sldd"
    %   * Subs. Ref. Model: "Unpack_[NAME].slx"
    % 
    % The data dictionary contains the following data:
    %   * Structure: "struct[NAME]"
    %   * Bus Object: "bus[NAME]"
    %   * Data Store: "datastore[NAME]"
    arguments
        s
        dstAddress (1,4) uint8
        dstPort (1,1) uint16
        signalName
        outputDirectory = pwd()
    end
    assert(isstruct(s), 'Input "s" must be a struct!');
    signalName = matlab.lang.makeValidName(char(signalName));
    structName             = strcat('struct', signalName);
    busName                = strcat('bus', signalName);
    dataStoreName          = strcat('datastore', signalName);
    dataDictName           = strcat('Data_', signalName, '.sldd');
    modelNameUnpack        = strcat('Unpack_', signalName);
    fileNameDataDictionary = fullfile(outputDirectory, dataDictName);
    fileNameModel          = fullfile(outputDirectory, strcat(modelNameUnpack, '.slx'));

    % send UDP message
    fprintf('[AVE] Send message to %u.%u.%u.%u:%u\n', dstAddress(1), dstAddress(2), dstAddress(3), dstAddress(4), dstPort);
    structInfo = ave.GetStructInfo(s);
    SendStructInfo(structInfo, dstAddress, dstPort);

    % check if data dictionary and/or model have to be re-created
    [generateDataDict, generateModel] = CheckGenerationRequirements(fileNameDataDictionary, fileNameModel, structName, structInfo);

    % create data dictionary
    if(generateDataDict)
        fprintf('[AVE] Create data dictionary "%s"\n', fileNameDataDictionary);
        varNames = ave.CreateSimulinkDataObjectsFromStruct(s, structName, busName, dataStoreName, {});
        ave.CreateDataDictionaryFromBaseWorkspaceVariables(fileNameDataDictionary, varNames);
    end

    % create subsystem reference model for binary message unpacking
    if(generateModel)
        fprintf('[AVE] Create subsystem reference model "%s"\n', fileNameModel);
        mUnpackBus = NewModel(modelNameUnpack, fileNameModel);
        GenerateUnpackModel(modelNameUnpack, dataStoreName, dataDictName, structInfo);
        SaveModel(mUnpackBus, fileNameModel);
    end
    fprintf('[AVE] Done\n\n');
end

function SendStructInfo(structInfo, dstAddress, dstPort)
    % Send a structInfo to the specified destination. The source port is selected randomly.
    % The destination address may also be a broadcast address.
    arguments
        structInfo
        dstAddress (1,4) uint8
        dstPort (1,1) uint16
    end
    try
        % serialize structInfo to a vector of bytes.
        cbytes = cell.empty();
        for i = 1:numel(structInfo)
            v = reshape(structInfo{i}.Value, 1, []);
            cbytes{i} = typecast(v, 'uint8');
        end
        bytes = [cbytes{:}];

        % warn if it does not fit into a UDP message
        if(numel(bytes) > 65507)
            warning('The serialized structure does not fit into a single UDP message!');
        end

        % open UDP socket and send bytes to destination
        udpSocket = udpport("IPV4", "OutputDatagramSize", 65507);
        udpSocket.EnableBroadcast = true;
        strDestination = strcat(num2str(dstAddress(1)), '.', num2str(dstAddress(2)), '.', num2str(dstAddress(3)), '.', num2str(dstAddress(4)));
        write(udpSocket, bytes, "uint8", strDestination, dstPort);
        flush(udpSocket, "output");
    catch ME
        warning(ME.identifier, '%s', ME.message);
    end
end

function [generateDataDict, generateModel] = CheckGenerationRequirements(fileNameDataDictionary, fileNameModel, structName, structInfo)
    [structureChanged, valueChanged] = DetectStructChanges(structInfo, fileNameDataDictionary, structName);
    generateDataDict = ~exist(fileNameDataDictionary, 'file') || ...
                       structureChanged || ...
                       valueChanged;
    generateModel = ~exist(fileNameModel, 'file') || ...
                    structureChanged;
end

function [structureChanged, valueChanged] = DetectStructChanges(structInfo, fileNameDataDictionary, structName)
    % structureChanged ... the structure has been changed excluding the value (e.g. Name, DataType, Dimensions, etc. changed)
    % valueChanged ... structure has been changed including the value
    try
        latestStruct = ave.DataDictGetValue(fileNameDataDictionary, structName);
    catch ME
        structureChanged = true;
        valueChanged = true;
        return;
    end
    latestStructInfo = ave.GetStructInfo(latestStruct);
    structureChanged = (numel(structInfo) ~= numel(latestStructInfo));
    valueChanged = structureChanged;
    if(~structureChanged)
        for i = 1:numel(structInfo)
            structureChanged = structureChanged || ...
                               ~isequal(structInfo{i}.Name, latestStructInfo{i}.Name) || ...
                               ~isequal(structInfo{i}.DataType, latestStructInfo{i}.DataType) || ...
                               ~isequal(structInfo{i}.Dimensions, latestStructInfo{i}.Dimensions) || ...
                               ~isequal(structInfo{i}.Complexity, latestStructInfo{i}.Complexity);
            valueChanged = structureChanged || valueChanged || ~isequal(structInfo{i}.Value, latestStructInfo{i}.Value);
        end
    end
end

function model = NewModel(modelName, fileName)
    close_system(modelName,0);
    if(exist(fileName,'file'))
        delete(fileName);
    end
    model = new_system(modelName,"Subsystem");
    model = load_system(model);
end

function SaveModel(model, fileName)
    save_system(model,fileName);
    close_system(model);
end

function numBytes = GetNumberOfBytes(structInfo)
    numBytes = uint32(0);
    for i = 1:numel(structInfo)
        switch(structInfo{i}.DataType)
            case {'double','int64','uint64'}
                L = uint32(8);
            case {'single','int32','uint32'}
                L = uint32(4);
            case {'int16','uint16'}
                L = uint32(2);
            case {'int8','uint8','logical','bool','boolean'}
                L = uint32(1);
            otherwise
                error(['Data type "' structInfo{i}.DataType '" is not supported for element selection!']);
        end
        numBytes = numBytes + uint32(prod(structInfo{i}.Dimensions))*L;
    end
end

function strElements = MakeDataStoreElements(structInfo, dataStoreName)
    cellElements = cell.empty();
    for i = 1:numel(structInfo)
        strPostfix = '';
        if(numel(structInfo{i}.Dimensions) > 1)
            strPostfix = '(:,:)';
        end
        cellElements{i} = strcat(dataStoreName, '.', structInfo{i}.Name, strPostfix);
    end
    strElements = strjoin(cellElements, '#');
end

function GenerateUnpackModel(modelName, dataStoreName, dataDictName, structInfo)
    numBytes = GetNumberOfBytes(structInfo);
    strDimensions = ['{' strjoin(cellfun(@(x)(mat2str(x.Dimensions)),structInfo,'UniformOutput',false),',') '}'];
    strDataTypes = ['{''' strjoin(cellfun(@(x)(x.DataType),structInfo,'UniformOutput',false),''',''') '''}'];
    strElements = MakeDataStoreElements(structInfo, dataStoreName);

    % link data dictionary
    set_param(modelName, 'DataDictionary', dataDictName);

    % blocks at root level
    subSysName = [modelName '/unpack'];
    h_inbytes = add_block('simulink/Sources/In1', [modelName '/bytes'], 'OutDataTypeStr', 'uint8', 'Position', [0 113 30 127]);
    h_inlength = add_block('simulink/Sources/In1', [modelName '/length'], 'OutDataTypeStr', 'uint32', 'PortDimensions', '1', 'Position', [0 13 30 27]);
    h_compare = add_block('simulink/Logic and Bit Operations/Compare To Constant', [modelName '/CompareToConstant'], 'ShowName', 'off', 'relop', '==', 'const', num2str(numBytes), 'Position', [85 10 175 30]);
    h_width = add_block('simulink/Signal Attributes/Width', [modelName '/Width'], 'ShowName', 'off', 'DataType', 'uint32', 'Position', [85 55 115 85]);
    h_relop = add_block('simulink/Logic and Bit Operations/Relational Operator', [modelName '/RelationalOperator'], 'ShowName', 'off', 'relop', '<=', 'Position', [150 32 175 83]);
    h_and = add_block('simulink/Logic and Bit Operations/Logical Operator', [modelName '/AND'], 'ShowName', 'off', 'Inputs', '2', 'Position', [245 1 270 79]);
    h_outsuccess = add_block('simulink/Sinks/Out1', [modelName '/success'], 'OutDataTypeStr', 'boolean', 'PortDimensions', '1', 'Position', [475 33 505 47]);
    h_enabled = add_block('simulink/Ports & Subsystems/Enabled Subsystem', subSysName, 'Position', [300 87 415 153]);
    
    add_line(modelName, get_param(h_inlength,'PortHandles').Outport(1), get_param(h_relop,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_inlength,'PortHandles').Outport(1), get_param(h_compare,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_inbytes,'PortHandles').Outport(1), get_param(h_width,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_inbytes,'PortHandles').Outport(1), get_param(h_enabled,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_width,'PortHandles').Outport(1), get_param(h_relop,'PortHandles').Inport(2), 'autorouting', 'smart');
    add_line(modelName, get_param(h_compare,'PortHandles').Outport(1), get_param(h_and,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_relop,'PortHandles').Outport(1), get_param(h_and,'PortHandles').Inport(2), 'autorouting', 'smart');
    add_line(modelName, get_param(h_and,'PortHandles').Outport(1), get_param(h_enabled,'PortHandles').Enable(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_and,'PortHandles').Outport(1), get_param(h_outsuccess,'PortHandles').Inport(1), 'autorouting', 'smart');
    
    % blocks in enabled subsystem
    delete_line(get_param([subSysName '/In1'],'LineHandles').Outport(1));
    delete_block([subSysName '/Out1']);
    h_bytepack = add_block('embeddedtargetslib/Host Communication/Byte Unpack', [subSysName '/ByteUnpack'], 'MakeNameUnique', 'on', 'ShowName', 'off', 'Dimensions', strDimensions, 'DataTypes', strDataTypes, 'Position', [290 100 350 100+30*numel(structInfo)]);
    portPositionByteUnpack = get_param(get_param(h_bytepack,'PortHandles').Inport(1),'Position');
    h_selector = add_block('simulink/Signal Routing/Selector', [subSysName '/Selector'], 'ShowName', 'off', 'InputPortWidth', '-1', 'Indices', ['1:' num2str(numBytes)], 'Position', [portPositionByteUnpack(1)-120 portPositionByteUnpack(2)-20 portPositionByteUnpack(1)-80 portPositionByteUnpack(2)+20]);
    set_param([subSysName '/In1'], 'Position', [portPositionByteUnpack(1)-235 portPositionByteUnpack(2)-7 portPositionByteUnpack(1)-205 portPositionByteUnpack(2)+7]);
    set_param([subSysName '/Enable'], 'Position', [portPositionByteUnpack(1)-230 portPositionByteUnpack(2)-70 portPositionByteUnpack(1)-210 portPositionByteUnpack(2)-50]);
    posByteUnpack = get_param(h_bytepack, 'Position');
    h_dswrite = add_block('simulink/Signal Routing/Data Store Write', [subSysName '/DataStoreWrite'], 'Position', posByteUnpack + [200 0 600 0], 'dataStoreName', dataStoreName, 'DataStoreElements', strElements);

    add_line(subSysName, get_param([subSysName '/In1'],'PortHandles').Outport(1), get_param(h_selector,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(subSysName, get_param(h_selector,'PortHandles').Outport(1), get_param(h_bytepack,'PortHandles').Inport(1), 'autorouting', 'smart');
    for i = 1:numel(structInfo)
        add_line(subSysName, get_param(h_bytepack,'PortHandles').Outport(i), get_param(h_dswrite,'PortHandles').Inport(i), 'autorouting', 'smart');
    end
end

