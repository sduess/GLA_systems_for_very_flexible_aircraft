function [list_simulation_outputs, reference_values] = test_kalman_filter(sys_kf_new, sys_LQR, input_settings, Qn, Rn, nonlinear_data_file, deflection_input_file)

    list_simulation_outputs = [];
    n_ts_start = 50;
    nonlinear_data = readmatrix(nonlinear_data_file);
   
    nonlinear_data = nonlinear_data(n_ts_start:end,:);
    nonlinear_data(1,:) = [];
    nonlinear_data(:,1) = nonlinear_data(:,1) - n_ts_start * input_settings.dt;
    nonlinear_data(1,1) = 0.;
    gust = nonlinear_data(:,6);

    sensor_measurements = [nonlinear_data(:,9) nonlinear_data(:,9) ... %eta right wing
        nonlinear_data(:,14) nonlinear_data(:,14) ... % eta left wing
        nonlinear_data(:,12) nonlinear_data(:,12) ... % eta dot right wing
        nonlinear_data(:,17) nonlinear_data(:,17)...
        ]; % eta dot left wing

    % sensor_measurements = [nonlinear_data(:,7:9) nonlinear_data(:,13:15) ... %eta right wing
    %     nonlinear_data(:,7:9) nonlinear_data(:,13:15) ... % eta left wing
    %     nonlinear_data(:,10:12) nonlinear_data(:,16:18) ... % eta dot right wing
    %     nonlinear_data(:,10:12) nonlinear_data(:,16:18)]; % eta dot left wing
    % 
    % % flip signs for symmetry
    % sensor_measurements(:,8) = -sensor_measurements(:,8);
    % sensor_measurements(:,10) = -sensor_measurements(:,10);
    % sensor_measurements(:,12) = -sensor_measurements(:,12);    
    % 
    % sensor_measurements(:,8+12) = -sensor_measurements(:,8+12);
    % sensor_measurements(:,10+12) = -sensor_measurements(:,10+12);
    % sensor_measurements(:,12+12) = -sensor_measurements(:,12+12);

    n_ts = size(sensor_measurements,1);
    input_settings.flight_time = n_ts * input_settings.dt;

    % control_input_delta = zeros(n_ts,1);
    % control_input_delta_dot = zeros(n_ts,1);
    control_input_delta = readmatrix(deflection_input_file); %deflection_input_linear_L10_I',intensity,'.csv')); %'/delta_continuous.csv')); %'/deflection_input_linear_L10_I',intensity,'.csv'));
    control_input_delta = control_input_delta(n_ts_start:end,:);
    % control_input_delta = [zeros(n_ts-length(control_input_delta),1); control_input_delta]
    control_input_delta_dot = diff(control_input_delta(1:n_ts,:))/input_settings.dt;
    control_input_delta_dot = [0; control_input_delta_dot];
    control_input_delta = control_input_delta(1:n_ts,:);

    reference_values_nonlinear =sensor_measurements(2,:); %  mean(sensor_measurements(end-20:end,:),1); % sensor_measurements(1,:); % 
    reference_values = reference_values_nonlinear;
    reference_values_nonlinear(5:end) = 0.;
%% Add timesteps to time histories
    control_input_delta_time_series = timeseries(control_input_delta,nonlinear_data(:,1));
    gust_time_series = timeseries(gust,nonlinear_data(:,1));
    sensor_measurements_time_series = timeseries(sensor_measurements,nonlinear_data(:,1));
    control_input_delta_dot_time_series = timeseries(control_input_delta_dot,nonlinear_data(:,1));
    %% Configure Simulink Inputs
    sys_LQR = ss(sys_LQR.A, sys_LQR.B, sys_LQR.C(input_settings.sensors,:), sys_LQR.D(input_settings.sensors,:), input_settings.dt);
    model_name = "kalman_filter_check";
    simIn = Simulink.SimulationInput(model_name);
    simIn = setVariable(simIn,'sys_kf_new',sys_kf_new);
    simIn = setVariable(simIn,'sys_LQR',sys_LQR);
    simIn = setVariable(simIn,'reference_values_nonlinear',reference_values_nonlinear);
    simIn = setVariable(simIn,'control_input_delta',control_input_delta_time_series);
    simIn = setVariable(simIn,'control_input_delta_dot',control_input_delta_dot_time_series);
    simIn = setVariable(simIn,'sensor_measurements',sensor_measurements_time_series);
    simIn = setVariable(simIn,'input_settings',input_settings);
    simIn = setVariable(simIn,'initial_state_kf',zeros(1,size(sys_kf_new.A,1)));
    simIn = setVariable(simIn,'gust', gust_time_series);
    simIn = setVariable(simIn,'Qn', Qn);
    simIn = setVariable(simIn,'Rn', Rn);
    %% Run Simulink
    list_simulation_outputs = [list_simulation_outputs sim(simIn)];
end