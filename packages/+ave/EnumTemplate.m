function EnumTemplate()
    %ave.EnumTemplate Create an enumeration template file in the current working directory.
    % 
    % REVISION HISTORY
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Version     Author                 Changes
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % 20250314    Robert Damerius        Initial release.
    % 
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    filename = fullfile(pwd,'EnumTemplate.m');
    if(~exist(filename,'file'))
        f = fopen(filename,'w');
        if(f ~= -1)
            fprintf(f,'classdef EnumTemplate < uint8\n');
            fprintf(f,'    enumeration\n');
            fprintf(f,'        RED(0)\n');
            fprintf(f,'        GREEN(1)\n');
            fprintf(f,'        BLUE(2)\n');
            fprintf(f,'    end\n');
            fprintf(f,'    methods(Static)\n');
            fprintf(f,'        function defaultValue = getDefaultValue(), defaultValue = EnumTemplate(0); assert(numel(enumeration(EnumTemplate(0))) == (1 + max(enumeration(EnumTemplate(0)))),''Enumerations must start with zero and be incremented by one!''); end\n');
            fprintf(f,'        function lowerLimit = getLowerLimit(), lowerLimit = min(enumeration(EnumTemplate.getDefaultValue())); end\n');
            fprintf(f,'        function upperLimit = getUpperLimit(), upperLimit = max(enumeration(EnumTemplate.getDefaultValue())); end\n');
            fprintf(f,'        function indices = getIndices(), indices = flip(num2cell(enumeration(EnumTemplate.getDefaultValue()))); end\n');
            fprintf(f,'    end\n');
            fprintf(f,'end\n\n');
            fclose(f);
        end
    end
end

