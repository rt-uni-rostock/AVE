function ReplaceBlockParameters(modelName, inputPattern, outputPattern, varargin)
    %ave.ReplaceBlockParameters Replace all block parameters of all blocks inside a model at once. The operation may fail if
    % a parameter to be changed affects a reference link to another model, e.g. subsystem references. In this case use
    % ave.ReplaceBlockParametersOneByOne.
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
    % 20221028    Robert Damerius        Initial release.
    % 20230130    Robert Damerius        Added 'silent' option.
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
                end
            end
        end
    end
    if(~silent)
        fprintf('\nReplacing finished\n');
    end
end

