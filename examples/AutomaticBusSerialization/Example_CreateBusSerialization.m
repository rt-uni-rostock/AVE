% directories and filenames
thisDirectory          = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1);
directoryCallbacks     = 'Messages';
directorySubsystems    = 'Serialization';
filenameDataDictionary = 'MessageData.sldd';
callbackPrefix         = 'MESSAGE_';

% generate all model data and subsystem references for all busses
ave.UpdateMessageComponents(fullfile(thisDirectory,directoryCallbacks), fullfile(thisDirectory,directorySubsystems), fullfile(thisDirectory,filenameDataDictionary), callbackPrefix);

