function sys_reduced = remove_state_from_state_space_model(sys, list_removed_states)
% remove_state_from_state_space_model Remove state from linear SS model.
%   Removes the rows linked to the to be removed states from the A and B
%   matrices as well as the corresponding columns of the A and C matrices.
ss_matrices = {sys.A; sys.B; sys.C; sys.D};
for i=1:size(ss_matrices,1)-1
    if  mod(i,2) % uneven (A or C matrix)
        ss_matrices{i}(:,  list_removed_states) = [];
    end
    if i <= 2
        % remove rows from A and B linked to state to be removed
        ss_matrices{i}( list_removed_states, :) = [];
    end
end
sys_reduced = ss(ss_matrices{1}, ss_matrices{2}, ss_matrices{3}, ss_matrices{4}, sys.Ts);

end