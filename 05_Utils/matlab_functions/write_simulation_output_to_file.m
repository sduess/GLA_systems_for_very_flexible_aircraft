function write_simulation_output_to_file(output_directory, case_name, simulation_output, controller_type)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    writematrix([simulation_output.actual_output.Time, simulation_output.actual_output.Data], ...
        strcat(output_directory,'/sensor_measurements_',case_name, '.csv'));
    writematrix([simulation_output.control_input.Time, simulation_output.control_input.Data], ...
         strcat(output_directory,'./control_input_',case_name, '.csv'));
    if strcmp(controller_type, 'LQG')
        writematrix([simulation_output.xhat.Time, simulation_output.xhat.Data], ...
             strcat(output_directory,'./xhat_',case_name, '.csv'));
        writematrix([simulation_output.yhat.Time, simulation_output.yhat.Data], ...
             strcat(output_directory,'./yhat_',case_name, '.csv'));
    end
end