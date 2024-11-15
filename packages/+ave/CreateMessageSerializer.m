function CreateMessageSerializer(destinationFilePack, destinationFileUnpack, dataStoreName)
    %ave.CreateMessageSerializer Create two simulink subsystem references to serialize and deserialize signals of a data store.
    % Both subsystem references are based on templates. Modify the ElementSelector in the serializer subsystem reference by specifying the
    % data store name and selecting all elements to be packed. Then call ave.UpdateMessageSerializer to automatically update the deserializer based
    % on the serializer.
    % 
    % PARAMETERS
    % destinationFilePack    ... [char] Destination file where to write the serializer subsystem reference model.
    % destinationFileUnpack  ... [char] Destination file where to write the deserializer subsystem reference model.
    % dataStoreName          ... [char] Name of the data store to be assigned in the data store read/write blocks.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230109    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(nargin < 1)
        destinationFilePack = fullfile(pwd,'Pack.slx');
    end
    if(nargin < 2)
        destinationFileUnpack = fullfile(pwd,'Unpack.slx');
    end
    if(nargin < 3)
        dataStoreName = '';
    end
    assert(ischar(destinationFilePack) && ~isempty(destinationFilePack), 'Input "destinationFilePack" must be a string with at least one character!');
    assert(ischar(destinationFileUnpack) && ~isempty(destinationFileUnpack), 'Input "destinationFileUnpack" must be a string with at least one character!');
    assert(ischar(dataStoreName), 'Input "dataStoreName" must be a string!');

    % Copy the template encoders and decoders to the specified destination
    % The templates are, relative to THIS file, in a 'template' subdirectory of the AVE package
    fullpath = mfilename('fullpath');
    filename = mfilename();
    dirTemplate = [fullpath(1:(end-length(filename))) 'template' filesep];
    sourceFileEncoder = [dirTemplate 'AVE_TEMPLATE_MESSAGE_PACK.slx'];
    sourceFileDecoder = [dirTemplate 'AVE_TEMPLATE_MESSAGE_UNPACK.slx'];
    [~,~] = copyfile(sourceFileEncoder, destinationFilePack, 'f');
    [~,~] = copyfile(sourceFileDecoder, destinationFileUnpack, 'f');

    % If data source is specified, open both template and set initial data store name
    if(~isempty(dataStoreName))
        % Update encoder model
        [~,modelNameEncoder,~] = fileparts(destinationFilePack);
        close_system(modelNameEncoder, 0); % ensure that no other model with that name is loaded (0: ignore warnings of unopened systems)
        open_system(destinationFilePack);
        set_param([modelNameEncoder '/ElementSelector'], 'DataStoreName', dataStoreName);
        save_system(destinationFilePack);
        close_system(destinationFilePack);

        % Update decoder model
        [~,modelNameDecoder,~] = fileparts(destinationFileUnpack);
        close_system(modelNameDecoder, 0); % ensure that no other model with that name is loaded (0: ignore warnings of unopened systems)
        open_system(destinationFileUnpack);
        set_param([modelNameDecoder '/ForLoop/EnabledSubsystem/AutogenDataStoreWrite'], 'DataStoreName', dataStoreName);
        save_system(destinationFileUnpack);
        close_system(destinationFileUnpack);
    end
end

