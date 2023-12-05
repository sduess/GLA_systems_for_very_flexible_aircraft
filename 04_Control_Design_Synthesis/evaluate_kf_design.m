clear all;

folder = '/home/sduess/Documents/SHARPY_simulations/GLA_systems_for_very_flexible_aircraft/02_Open_Loop_Nonlinear_Gust_Response/'
sensor_file = strcat(folder, '/results_data/superflexop_free_gust_comp2_L_10_I_10_p_0_f_0_cfl_1_uinf45_wing_only_dynamic_cs_input.txt')
cs_file = strcat(folder, '/predefined_cs_inputs/linear_LQG_L10_I10.txt');
% load('controller_files_matlab/superflexop_cruise_linear_ROM_clamped_wing_only_acc_num_modes16_acc0_onlyz1_onlypos0.mat');
load('controller_files_matlab/superflexop_cruise_linear_ROM_clamped_wing_only_acc_num_modes16_acc0_onlyz1_onlypos0_rotdot1_joinedcs1_woIP_wo5thBM.mat'), %superflexop_cruise_linear_ROM_clamped_wing_only_acc_num_modes16_acc0_onlyz1_onlypos0_rotdot1.mat');
Qn=eye(89).*1e-9; 
Qn(1,1) = 2.3;
Qn(72,72) = 1e-10;
Rn=1e-6;
Rn = eye(length(input_settings.sensors)) .* 1e-6;
Rn(3,3) =1e-6;
Rn(4,4)= 1e-6;

Rn(7,7) = 1e-4;
Rn(8,8)= 1e-4;
simulation_out = test_kalman_filter(sys_kf, sys_LQR, input_settings, Qn, Rn, sensor_file, cs_file)

plot_results_kalman_filter(simulation_out, Rn,input_settings)

% %% Plot Filter
% filtered_data = squeeze(simulation_out.filtered.Data)';
% figure()
% plot(simulation_out.unfiltered.Time, simulation_out.unfiltered.Data(:,end), '-')
% hold on
% plot(simulation_out.filtered.Time, filtered_data(:,end), '--')
% hold off
% figure()
% plot(simulation_out.known_input.Time, rad2deg(simulation_out.known_input.Data(:,1)));
%% Get Poles
rad2degnum_cs = 1
system_open_loop = ss(sys_LQR.A(1:end-num_cs:1:end-num_cs,1:end-num_cs), sys_LQR.B(1:end-num_cs,:), sys_LQR.C(:,1:end-num_cs), sys_LQR.D, input_settings.dt);
system_closed_loop = ss(sys_LQR.A-sys_LQR.B(:,1:num_cs) * controller_final.K, ...
        sys_LQR.B(:,end),...
        sys_LQR.C-sys_LQR.D(:,1:num_cs)*controller_final.K,...
        sys_LQR.D(:,end), ...
        input_settings.dt);
figure()
pzmap(system_open_loop, system_closed_loop)
zgrid()