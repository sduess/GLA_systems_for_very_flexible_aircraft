function simulation_output = run_linear_open_loop_response(linear_sys, ...
    input_settings, gust_time_series,simulink_file_directory)

    
    %% Configure Simulink Inputs
    addpath(simulink_file_directory);
    model_name  = "Open_Loop_Gust_Response";
    simIn = Simulink.SimulationInput(model_name);
    simIn = setVariable(simIn,'linear_sys',linear_sys);
    simIn = setVariable(simIn,'input_settings',input_settings);
    simIn = setVariable(simIn,'gust', gust_time_series);

    % Control surface input mapping
    num_cs_inputs = size(linear_sys, 2)-1;
    if num_cs_inputs > 2
        control_surface_mapping = [ones(1, num_cs_inputs/2) -ones(1, num_cs_inputs/2)];
    else
        control_surface_mapping = 1;
    end
    simIn = setVariable(simIn,'control_surface_mapping', control_surface_mapping);

    %% Run Simulink
    simulation_output = sim(simIn);    
    
end