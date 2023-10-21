function postprocess_linear_system(route_directory, case_name, SHARPy_output_folder)

    addpath(strcat(route_directory,'/../05_Utils/matlab_functions/'));
    %% Reade from SHARPy generated state space model
    filepath_to_SHARPy_linear_system_file = strcat(SHARPy_output_folder,case_name, '/savedata/',case_name, '.linss.h5');
    [state_space_system, eta_ref] = read_SHARPy_state_space_system(filepath_to_SHARPy_linear_system_file);
    %% Get initial settings
    parameter_file = strcat(route_directory,'/case_settings/',  'parameter_state_space_', case_name,  '.json');
    input_settings = set_input_parameters(parameter_file, state_space_system.Ts);
    %% Get indices
    input_settings.index =  get_indices_for_case(input_settings);
    %% Remove unused input and outputs from the generated ROM in SHARPy and link deflection and its rate    
    state_space_system = adjust_state_space_system(state_space_system, input_settings);
    %% Save State space system, reference displacements, settings, and case name
    save(strcat(route_directory,'/linear_statespace_files_matlab/',case_name), ...
        'state_space_system', ...
        'eta_ref', ...
        'input_settings', ...
        'case_name', ...
        '-v7.3');

end
