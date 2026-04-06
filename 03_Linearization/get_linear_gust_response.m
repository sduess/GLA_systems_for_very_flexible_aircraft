function get_linear_gust_response(state_space_system, input_settings, gust_lengths, gust_intensities, output_folder)
%get_linear_gust_response Simulate open-loop gust response of linear system
%   Runs time-domain simulations for all combinations of gust length and
%   intensity using a 1-minus-cosine gust profile. Results are written to
%   CSV files in the specified output folder.
%
%   Inputs:
%       state_space_system  - struct with adjusted system matrices as
%                             returned by adjust_state_space_system,
%                             requires fields: A, B_gust, C, D_gust
%       input_settings      - struct with case parameters, requires fields:
%                             dt, u_inf, flight_time, case_name, rbm,
%                             index.eta_tip, index.eta_dot_tip, index.rbm
%       gust_lengths        - vector of gust wavelengths [m]
%       gust_intensities    - vector of gust velocities [m/s]
%       output_folder       - path to folder where CSV results are written

    if ~exist(output_folder, 'dir'), mkdir(output_folder); end

    list_indices = [input_settings.index.eta_tip input_settings.index.eta_dot_tip];
    if input_settings.rbm
        list_indices = [list_indices input_settings.index.rbm];
    end

    % Construct system considering only the gust input for open loop simulations
    % and extracting only eta and eta dot at the wing tip as well as the rbm velocities
    % to reduce the size of the system while retrieving relevant output parameters.
    ss_gust = ss(state_space_system.A,...
        state_space_system.B_gust,...
        state_space_system.C(list_indices, :), ...
        state_space_system.D_gust(list_indices, :), ...
        input_settings.dt);

    for i_length=1:length(gust_lengths)
        for i_intensity=1:length(gust_intensities)
            % Get gust for specific length and intensity
            gust = get_1minuscosine_gust_input(gust_lengths(i_length), ...
                gust_intensities(i_intensity), ...
                input_settings.dt, ...
                input_settings.u_inf, ...
                input_settings.flight_time);
                
            % Time domain simulation
            output_gust_response = lsim(ss_gust, gust(:,2), gust(:,1));

            % Write results
            file_name = strcat('gust_response_L_',...
                num2str(gust_lengths(i_length)), ...
                '_I_', ...
                num2str(gust_intensities(i_intensity)), '_', ...
                input_settings.case_name);

            writematrix([gust(:,1), output_gust_response], ...
                fullfile(output_folder, strcat(file_name, '.csv')));

        end
    end
end