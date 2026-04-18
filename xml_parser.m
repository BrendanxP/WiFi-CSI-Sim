function [xml_string, model_extension] = xml_parser(path)
%% Load Robot SDF Model as Text
% https://answers.ros.org/question/382549/spawn-gazebo-model-editor-model-with-ros-service/


% Check SDF/URDF file extension

% Check if file exists
if ~isfile(path)
    error('SDF file does not exist.');
end

% Find the position of the last dot
dotLocations = strfind(path,'.');
if isempty(dotLocations)
    model_extension = ""; 
    error("No dot found in: " + path)
end

% Extract substring after the last dot
model_extension = lower(extractAfter(path, dotLocations(end)));
% Check if the extension is 3 or 4 characters
if strlength(model_extension) < 3 || strlength(model_extension) > 4
    error("sdf/urdf ground robot file extension not correct size: %s",model_extension)
% Check if extension is correct
elseif ~strcmp(model_extension,"sdf") && ~strcmp(model_extension,"urdf")
    error("sdf/urdf ground robot file extension not correct format: %s",model_extension)
end


%% OLD SDF PARSER
% % read sdf as textfile
% sdf_table_ground = table2array(readtable(sdf_path_ground, 'FileType', "text",'ReadVariableNames',false,'ReadRowNames',false,'NumHeaderLines',0));
% sdf_string_ground = "";
% 
% % iterate through every line of the sdf
% for n_sdf = 1:length(sdf_table_ground)
%     % attach line to string
%     sdf_string_ground = strcat(sdf_string_ground, sdf_table_ground{n_sdf}, "\n");
% end
% disp(sdf_string_ground)


%% NEW SDF PARSER
xml_string = regexprep(fileread(path), '[\r\n]+', '');
