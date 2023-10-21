function [Q, R] = define_Q_and_R_matrices_for_LQR(num_inputs, ...
                         C_matrix, num_aero_states, num_modes, ...
                         num_rbm, list_modes, R_factor, delta_factor)
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
 R =eye(num_inputs); %.*R_factor*delta_factor;


% if i_modes == num_modes
%     i_modes
%     Q = diag([  0.0001*ones(num_aero_states,1,'double')
%              [10000 10 10]' %10.*ones(num_modes, 1,'double')         
%              [100 10 1 10 10]' %0.0001*ones(num_rbm, 1,'double') %  [0.0001 0.0001 10 0.0001 0.0001]' 
%              [10000 10 10]' %10.*ones(num_modes, 1,'double')
%              [10 10000]' %0.5*ones(num_inputs, 1,'double') % [0.5 1000 0.5 1000]' [0.05 10000 0.05 10000]' %
%              ]);  
% else
num_structural_modes = num_modes - num_rbm;
    Q = diag([0.001*ones(num_aero_states,1,'double')
         0.001*ones(num_structural_modes, 1,'double')
         0.001*ones(num_rbm, 1,'double')
         0.001*ones(num_structural_modes, 1,'double')
         1*ones(num_inputs, 1,'double') % [0.5 1000 0.5 1000]' [0.05 10000 0.05 10000]' %
         ]); 
    for imode=1:length(list_modes)
        Q(num_aero_states + list_modes(imode), ...
          num_aero_states + list_modes(imode)) = 1000000;
        Q(num_aero_states + list_modes(imode) + num_structural_modes + num_rbm, ...
          num_aero_states + list_modes(imode)  + num_structural_modes + num_rbm) = 1000.;
    end
    Q = Q./5000.;
%      Q = 1. * C_matrix'*C_matrix;
% end

end