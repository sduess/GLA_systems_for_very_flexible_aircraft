function simulation_output = run_linear_closed_loop_LQR_response(linear_sys, ...
    input_settings, gust_time_series, LQR_gain,simulink_file_directory)
    %% Configure Simulink Inputs
    addpath(simulink_file_directory);
    model_name  = "LQR";
    simIn = Simulink.SimulationInput(model_name);
    simIn = setVariable(simIn,'linear_sys',linear_sys);
    simIn = setVariable(simIn,'input_settings',input_settings);
    simIn = setVariable(simIn,'gust', gust_time_series);
    simIn = setVariable(simIn,'LQR_gain', LQR_gain);
    simIn = setVariable(simIn,'saturation_deflection_rate',input_settings.saturation_control_deflection_rate);
    %% Run Simulink
    simulation_output = sim(simIn);    
    
end