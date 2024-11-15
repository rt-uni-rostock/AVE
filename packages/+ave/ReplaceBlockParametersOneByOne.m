function ReplaceBlockParametersOneByOne(modelName, inputPattern, outputPattern, varargin)
    %ave.ReplaceBlockParametersOneByOne Replace all block parameters of all blocks inside a model one by one. After a changed
    % parameter, the model and all its references are saved and the replacement is repeated until no more parameters have been
    % replaced.
    % 
    % PARAMETERS
    % modelName      ... [char] Name of the simulink model.
    % inputPattern   ... [char] Input pattern that should be replaced.
    % outputPattern  ... [char] Output pattern to be used for replacing.
    % option         ... [char] Optional option, possible options are:
    %                           'silent' : Make no prints to the console.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20230131    Robert Damerius        Initial release.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    silent = false;
    if(~isempty(varargin))
        assert(1 == numel(varargin), 'Input "option" must be given as one argument to the function!');
        assert(ischar(varargin{1}), 'Input "option" must be a character string!');
        silent = strcmpi(varargin{1}, 'silent');
    end
    assert(ischar(modelName), 'ERROR: Input "modelName" must be a character string!');
    assert(ischar(inputPattern), 'ERROR: Input "inputPattern" must be a character string!');
    assert(ischar(outputPattern), 'ERROR: Input "outputPattern" must be a character string!');
    if(~silent)
        fprintf('Model:          [%s]\n',modelName);
        fprintf('Input pattern:  [%s]\n',inputPattern);
        fprintf('Output pattern: [%s]\n\n',outputPattern);
    end
    while(ReplaceOneParameter(modelName, inputPattern, outputPattern, silent))
        save_system(modelName, 'SaveDirtyReferencedModels','on','OverwriteIfChangedOnDisk',true);
    end
    if(~silent)
        fprintf('\nReplacing finished\n');
    end
end

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Internal helper function for the actual replacement of one parameter
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function replacedSomething = ReplaceOneParameter(modelName, inputPattern, outputPattern, silent)
    replacedSomething = false;
    objects = find_system(modelName);
    for i = 2:length(objects)
        dialogParams = get_param(objects{i},'DialogParameters');
        if ~isempty(dialogParams)
            dialogNames = fieldnames(dialogParams);
            for j = 1:length(dialogNames)
                param = get_param(objects{i},dialogNames{j});
                if(ischar(param) && contains(param,inputPattern))
                    newParam = strrep(param,inputPattern,outputPattern);
                    if(~silent)
                        fprintf('block=[%s] dialog=[%s]: "%s" replaced by "%s"\n',regexprep(objects{i},'\s+',' '),dialogNames{j},param,newParam);
                    end
                    set_param(objects{i},dialogNames{j},newParam);
                    replacedSomething = true;
                    return;
                end
            end
        end
    end
end

