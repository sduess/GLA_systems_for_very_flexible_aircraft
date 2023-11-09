function simulation_output = run_linear_closed_loop_LQG_response(linear_sys, ...
    sys_kf, input_settings, gust_time_series, LQR_gain,simulink_file_directory, ...
    Qn, Rn)

    
    %% Configure Simulink Inputs
    addpath(simulink_file_directory);
    model_name  = "LQG";
    simIn = Simulink.SimulationInput(model_name);
    simIn = setVariable(simIn,'linear_sys',linear_sys);
    simIn = setVariable(simIn,'sys_kf',sys_kf);
    simIn = setVariable(simIn,'input_settings',input_settings);
    simIn = setVariable(simIn,'gust', gust_time_series);
    simIn = setVariable(simIn,'LQR_gain', LQR_gain);
    simIn = setVariable(simIn,'saturation_deflection_rate',input_settings.saturation_control_deflection_rate);

    % Control surface input mapping
    num_cs_inputs = size(linear_sys, 2)-1;
    if num_cs_inputs > 2
        control_surface_mapping = [ones(1, num_cs_inputs/2) -ones(1, num_cs_inputs/2)];
    else
        control_surface_mapping = 1;
    end
    simIn = setVariable(simIn,'control_surface_mapping', control_surface_mapping);

    % Kalman Filter Tuning
    simIn = setVariable(simIn,'Qn', Qn);
    simIn = setVariable(simIn,'Rn', Rn);
    %% Run Simulink
    simulation_output = sim(simIn);    
    
end