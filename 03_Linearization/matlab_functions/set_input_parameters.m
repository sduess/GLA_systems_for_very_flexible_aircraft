function parameter = set_input_parameters(json_parameter_file_path, dt)
%set_input_parameters Save input settings to struct.
json_parameter_file = fileread(json_parameter_file_path);
parameter =struct();
parameter =jsondecode(json_parameter_file);
parameter.dt = dt;
end

