function [Q, R] = define_Q_and_R_matrices_for_LQR(num_inputs, ...
                         num_aero_states, num_modes, ...
                         num_rbm, LQR_tuning)
    R = LQR_tuning.R_values;
    num_structural_modes = num_modes - num_rbm;
    Q = diag([ones(double(num_aero_states) + 2*double(num_structural_modes) + ...
                   double(num_rbm) + double(num_inputs),...
                  1,'double')]) .* LQR_tuning.initial_diagonal_values; 
   
    % Penalize aero states
   idx_start = 0;
   [Q, idx_start] = penalize_states(Q, LQR_tuning.idx_penalized_aero_state, idx_start, LQR_tuning.weights_for_penalized_aero_state, num_aero_states);
   
   % Penalize structural modal displacements
   [Q, idx_start] = penalize_states(Q, LQR_tuning.idx_penalized_modal_displacement, idx_start, LQR_tuning.weights_for_penalized_modal_displacement, num_structural_modes);
   
   % Penalize rigid body modes
   if num_rbm > 0
        [Q, idx_start] = penalize_states(Q, LQR_tuning.idx_penalized_rbm_modes, idx_start, LQR_tuning.weights_for_penalized_rbm_modes, num_rbm);
   end
   
   % Penalize structural modal velocities
   [Q, idx_start] = penalize_states(Q, LQR_tuning.idx_penalized_modal_velocities, idx_start, LQR_tuning.weights_for_penalized_modal_velocities, num_structural_modes);

   % Penalize control deflection states   
   [Q, idx_start] = penalize_states(Q, 1:num_inputs, idx_start, ones(num_inputs,1).*LQR_tuning.weights_for_penalized_control_surfaces, num_inputs);
end

function [Q, idx_start] = penalize_states(Q, idx_weights, idx_start, weights, num_states_of_group)
   for istate=1:length(idx_weights)
       idx = double(idx_start) + double(idx_weights(istate));
        Q(idx, idx)= weights(istate);
   end
   idx_start = double(idx_start) + double(num_states_of_group);
end