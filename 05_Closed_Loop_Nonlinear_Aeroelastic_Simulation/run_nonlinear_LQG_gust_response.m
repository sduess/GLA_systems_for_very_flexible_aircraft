function out = run_nonlinear_LQG_gust_response(case_name,sys_kf, ...
    input_settings, reference_values_nonlinear, Qn, Rn,num_sensors, ...
    initial_control_input_SHARPy, initial_control_input,controller_final)
%% run_nonlinear_LQG_gust_response
% 
% Simulates the nonlinear gust response of an aircraft model using a Linear 
% Quadratic Gaussian (LQG) controller by running a Simulink model 
% ('UDP_Stateflow_LQG_FLEXOP') with specified system parameters, control 
% inputs, and simulation settings.
%
% Syntax:
%   out = run_nonlinear_LQG_gust_response(case_name, sys_kf, input_settings, ...
%       reference_values_nonlinear, Qn, Rn, num_sensors, ...
%       initial_control_input_SHARPy, initial_control_input, controller_final)
%
% Inputs:
%   case_name - (string) Identifier used for naming the saved output file.
%   sys_kf - (struct) State-space system containing Kalman filter/system dynamics.
%   input_settings - (struct) Flight simulation settings.
%   reference_values_nonlinear - (array) Reference signal or trajectory.
%   Qn - (matrix) State weighting matrix for LQG design.
%   Rn - (matrix) Measurement noise covariance matrix for Kalman filter.
%   num_sensors - (int) Number of sensors used in feedback.
%   initial_control_input_SHARPy - (matrix) Initial control inputs from SHARPy.
%   initial_control_input - (array) Initial control input vector.
%   controller_final - (struct) Final controller configuration for simulation.
%
% Output:
%   out - (Simulink.SimulationOutput) Contains logged results from the simulation.
%
% Description:
%   This function sets up a Simulink simulation environment using the provided
%   LQG controller and system parameters to simulate the aircraft's response 
%   to gust inputs. Simulation inputs are passed using a Simulink.SimulationInput
%   object. The simulation output is saved to a .mat file for further analysis.
%
% Saved File:
%   './simulation_output_nonlinear_LQG_<case_name>.mat' - Stores the simulation result.

%% Define the simulink file and inputs

    model_name = "UDP_Stateflow_LQG_FLEXOP_rbm_with_thrust";
    simIn = Simulink.SimulationInput(model_name);
    simIn = setVariable(simIn,'sys_kf_new',sys_kf);
    simIn = setVariable(simIn,'reference_values_nonlinear',reference_values_nonlinear);
    simIn = setVariable(simIn,'initial_control_input_SHARPy', initial_control_input_SHARPy);
    simIn = setVariable(simIn,'initial_control_input', initial_control_input);
    simIn = setVariable(simIn,'input_settings',input_settings);
    simIn = setVariable(simIn,'controller_final',controller_final);
    simIn = setVariable(simIn,'initial_state_kf',0);
    simIn = setVariable(simIn,'num_sensors', num_sensors);
    simIn = setVariable(simIn,'num_cs_sharpy',size(initial_control_input_SHARPy,1));
    simIn = setVariable(simIn,'Qn', Qn);
    simIn = setVariable(simIn,'Rn', Rn);
    %% Run Simulink
    out = sim(simIn);
    save(strcat("./simulation_output_nonlinear_LQG_", case_name, ".mat"), "out");

end
% 
% run_nonlinear_LQG_gust_response(sys_kf_new, input_settings, reference_values, Qn, Rn, initial_control_input_SHARPy)

