function jsonData = read_json_arrays(filename)
    % Check if file exists
    if ~isfile(filename)
        error('File "%s" does not exist.', filename);
    end

    % Read the entire JSON file as text
    fid = fopen(filename, 'r');
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);

    % Parse JSON string into MATLAB structure
    jsonData = jsondecode(str);

    % Extract arrays (this depends on the structure of the JSON)
    % If it's a struct with known fields:
    %if isstruct(jsonData)
    %    data = struct();
    %    fields = fieldnames(jsonData);
    %    for i = 1:length(fields)
    %        field = fields{i};
    %        value = jsonData.(field);
    %        if isnumeric(value) || iscell(value)
    %            data.(field) = value;
    %        end
    %    end
    %else
    %    % If the JSON is a raw array or cell array
    %    data = jsonData;
    %end
end