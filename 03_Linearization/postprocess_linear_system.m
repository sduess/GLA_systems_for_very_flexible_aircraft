function postprocess_linear_system(route_directory, case_name, SHARPy_output_folder)
%postprocess_linear_system Load and adjust SHARPy linear system for MATLAB
%   Reads the linear state space system exported by SHARPy, adjusts it for
%   closed-loop simulation by reducing the state space system to relevant
%   inputs and outs, as well as saves the result as a .mat file in
%   linear_statespace_files_matlab/.
%
%   Inputs:
%       route_directory     - absolute path to the 03_Linearization folder
%       case_name           - SHARPy case name string; used to locate the
%                             .linss.h5 file and the JSON parameter file
%       SHARPy_output_folder - path to the SHARPy output root folder
%                              (case subfolder and savedata/ are appended)

    addpath(fullfile(route_directory, '..', '05_Utils', 'matlab_functions'));

    %% Read from SHARPy generated state space model
    filepath_to_SHARPy_linear_system_file = fullfile(SHARPy_output_folder, case_name, 'savedata', strcat(case_name, '.linss.h5'));
    [state_space_system, eta_ref] = read_SHARPy_state_space_system(filepath_to_SHARPy_linear_system_file);
    
    %% Get initial settings
    parameter_file = fullfile(route_directory, 'case_settings', strcat('parameter_state_space_', case_name, '.json'));
    input_settings = set_input_parameters(parameter_file, state_space_system.Ts);
    
    %% Get indices
    input_settings.index = get_indices_for_case(input_settings);
    
    %% Remove unused input and outputs from the generated ROM in SHARPy and link deflection and its rate
    state_space_system = adjust_state_space_system(state_space_system, input_settings);
    
    %% Save State space system, reference displacements, settings, and case name
    out_dir = fullfile(route_directory, 'linear_statespace_files_matlab');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    save(fullfile(out_dir, case_name), ...
        'state_space_system', ...
        'eta_ref', ...
        'input_settings', ...
        'case_name', ...
        '-v7.3');

end
