function run_closed_loop_discrete_gust_simulations(gust_lengths, ...
    gust_intensities, input_settings, controller_inputs, case_name, ...
    route_directory, simulink_file_directory, ...
    sys_final, sys_kf, LQR_gain)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    addpath(strcat(route_directory,'/../05_Utils/matlab_functions/'));
    for ilength=1:length(gust_lengths)
        for jintensity=1:length(gust_intensities)
            gust_lengths(ilength)
            gust_intensities(jintensity)
            gust_time_series = get_1minuscosine_gust_input(gust_lengths(ilength), ...
                                                    gust_intensities(jintensity), ...
                                                    input_settings.dt, ...
                                                    input_settings.u_inf, ...
                                                    input_settings.flight_time);
            if strcmp(controller_inputs.controller_name, 'LQG')
                simulation_output = run_linear_closed_loop_LQG_response(sys_final, ...
                    sys_kf, input_settings, gust_time_series, LQR_gain,simulink_file_directory, ...
                    controller_inputs.Qn, controller_inputs.Rn);
            elseif strcmp(controller_inputs.controller_name, 'LQR')          
                simulation_output = run_linear_closed_loop_LQR_response(sys_final, ...
                    input_settings, gust_time_series, LQR_gain,simulink_file_directory);
            
            elseif strcmp(controller_inputs.controller_name,'Open-Loop')
                % Run LQR with zero LQR gain
                % TODO: Implement Better Solution
                simulation_output = run_linear_open_loop_response(sys_final, ...
                    input_settings, gust_time_series,simulink_file_directory);

            else
                error ("Controller name unknown. Please chose LQG, LQR, or open-loop or feel free to add your  controller!")
            end
    
            % Save Results
            simulation_case_name = strcat(controller_inputs.controller_name,'_output_discrete_gust_H',...
                                            strrep(num2str(gust_lengths(ilength)),'.', 'd'), '_I', ...
                                            strrep(num2str(gust_intensities(jintensity)),'.', 'd'),'_',case_name);
            output_directory = strcat(route_directory,'/results_linear_closed_loop_simulations/');
            write_simulation_output_to_file(output_directory, simulation_case_name, simulation_output, controller_inputs.controller_name)
        end
    end
end