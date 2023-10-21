function sys_reduced = remove_inputs_from_state_space_model(sys, list_removed_inputs, gust_inputs_included)
%remove_inputs_from_state_space_model Removes inputs from state-space model
%   When removing control inputs (delta_dot) from the linear state-space 
%   model, we have to not only remove the corresponding columns from B and
%   D matrices but also the columns linked to the aileron deflection delta
%   in the A and C as well as the corresponding rows in the A and B
%   matrices. If the inputs contain the gust input, the number of inputs
%   have to be corrected since the gust is not linked to delta in the state
%   vector x.

num_inputs = size(sys.B,2) - gust_inputs_included * 1;
num_states = size(sys.A,1);
ss_matrices = {sys.A; sys.B; sys.C; sys.D};
list_removed_delta_state =  num_states - num_inputs + list_removed_inputs;
for i=1:size(ss_matrices,1)
    if  ~mod(i,2) % even (B or D matrix)
        removed_columns = list_removed_inputs;
    else % uneven (A or C matrix)
        removed_columns = list_removed_delta_state;
    end
    ss_matrices{i}(:, removed_columns) = [];
        if i <= 2
            % remove rows from A and B linked to state delta n+1
            ss_matrices{i}(list_removed_delta_state, :) = [];
        end
end
sys_reduced = ss(ss_matrices{1}, ss_matrices{2}, ss_matrices{3}, ss_matrices{4}, sys.Ts);

end