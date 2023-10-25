function [Q, R] = define_Q_and_R_matrices_for_LQR(num_inputs, ...
                         C_matrix, num_aero_states, num_modes, ...
                         num_rbm, list_penalized_modes, list_modes_penalized_rbm,...
                         R_factor, delta_factor)
%define_Q_and_R_matrices_for_LQR Sets the Q and R matrices depending on C
%matrix and number of known control inputs.
% arguments num_inputs = 1; end  % default value
% arguments C_matrix = 1; end  % default value
% arguments num_aero_states = 80; end  % default value
% arguments num_modes = 20; end  % default valuevalue
% arguments num_rbm = 0; end  % default valuevalue
% arguments list_modes = [1 2]; end  % default value
% arguments num_rbm = 0; end  % default valuevalue
% arguments list_modes = [1 2]; end  % default value


% TODO: change for elev vs ailerons?
 R =eye(num_inputs); %.*R_factor*delta_factor;

num_structural_modes = num_modes - num_rbm;
    Q = diag([0.001*ones(num_aero_states,1,'double')
         0.001*ones(num_structural_modes, 1,'double')
         0.001*ones(num_rbm, 1,'double')
         0.001*ones(num_structural_modes, 1,'double')
         1*ones(num_inputs, 1,'double') % [0.5 1000 0.5 1000]' [0.05 10000 0.05 10000]' %
         ]); 
    for imode=1:length(list_penalized_modes)
        idx_penalized = double(num_aero_states) + double(list_penalized_modes(imode));
        Q(idx_penalized,idx_penalized) = 1000000;
        idx_penalized = double(idx_penalized) + double(num_modes);
        Q(idx_penalized, idx_penalized) = 1000.;
    end
    for irbmmode=1:length(list_modes_penalized_rbm)
        idx_penalized = double(num_aero_states) + double(num_structural_modes)+ double(list_modes_penalized_rbm(irbmmode));
          Q(idx_penalized,...
            idx_penalized) = 100000000;
    end
    Q = Q./5000.;
%      Q = 1. * C_matrix'*C_matrix;
% end

end