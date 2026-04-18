function [Q, R] = define_Q_and_R_matrices_for_LQR(num_inputs, ...
                         num_aero_states, num_modes, ...
                         num_rbm, LQR_tuning)
%define_Q_and_R_matrices_for_LQR Build LQR cost matrices from tuning weights
%   Constructs a diagonal Q matrix by walking through the state vector in
%   the order it is assembled by adjust_state_space_system and
%   controller_synthesis. The indices in LQR_tuning are LOCAL to each group
%   (i.e. index 1 means the first state within that group, not the first
%   state overall). The state group order is:
%
%       1. Aero states              (num_aero_states)
%       2. Structural modal displ.  (num_structural_modes = num_modes - num_rbm)
%       3. Rigid body modes         (num_rbm)            [skipped if num_rbm == 0]
%       4. Structural modal veloc.  (num_structural_modes)
%       5. Control surface states   (num_inputs)
%
%   Inputs:
%       num_inputs        - number of control surface inputs (= num_cs)
%       num_aero_states   - number of aerodynamic states
%       num_modes         - total number of structural modes (incl. RBM)
%       num_rbm           - number of rigid body modes (0 for clamped)
%       LQR_tuning        - struct with fields:
%           R_values, initial_diagonal_values,
%           idx_penalized_aero_state, weights_for_penalized_aero_state,
%           idx_penalized_modal_displacement, weights_for_penalized_modal_displacement,
%           idx_penalized_rbm_modes, weights_for_penalized_rbm_modes,
%           idx_penalized_modal_velocities, weights_for_penalized_modal_velocities,
%           weights_for_penalized_control_surfaces

    R = LQR_tuning.R_values;
    num_structural_modes = num_modes - num_rbm;
    n_states = num_aero_states + 2*num_structural_modes + num_rbm + num_inputs;
    Q = diag(ones(n_states, 1)) .* LQR_tuning.initial_diagonal_values;

    % Walk through state groups in order, tracking the running offset
    idx_start = 0;
    [Q, idx_start] = penalize_states(Q, LQR_tuning.idx_penalized_aero_state, idx_start, ...
        LQR_tuning.weights_for_penalized_aero_state, num_aero_states);

    [Q, idx_start] = penalize_states(Q, LQR_tuning.idx_penalized_modal_displacement, idx_start, ...
        LQR_tuning.weights_for_penalized_modal_displacement, num_structural_modes);

    if num_rbm > 0
        [Q, idx_start] = penalize_states(Q, LQR_tuning.idx_penalized_rbm_modes, idx_start, ...
            LQR_tuning.weights_for_penalized_rbm_modes, num_rbm);
    end

    [Q, idx_start] = penalize_states(Q, LQR_tuning.idx_penalized_modal_velocities, idx_start, ...
        LQR_tuning.weights_for_penalized_modal_velocities, num_structural_modes);

    [Q, ~] = penalize_states(Q, 1:num_inputs, idx_start, ...
        ones(num_inputs, 1) .* LQR_tuning.weights_for_penalized_control_surfaces, num_inputs);
end

function [Q, idx_start] = penalize_states(Q, idx_weights, idx_start, weights, num_states_of_group)
    for istate = 1:length(idx_weights)
        idx = idx_start + idx_weights(istate);
        Q(idx, idx) = weights(istate);
    end
    idx_start = idx_start + num_states_of_group;
end