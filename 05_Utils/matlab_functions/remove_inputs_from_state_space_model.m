function sys_reduced = remove_inputs_from_state_space_model(sys, list_removed_inputs, num_non_cs_inputs)
%remove_inputs_from_state_space_model Remove CS inputs from augmented system
%   Removes control surface (CS) inputs from the state-space system. Since
%   the system is augmented with delta positions as states, removing a CS
%   input (delta_dot column) also requires removing:
%       - the corresponding delta state column from A and C
%       - the corresponding delta state row from A and B
%   Trailing non-CS inputs (gust, thrust) have no associated delta state
%   and must be excluded from the state index calculation.
%
%   Inputs:
%       sys                - augmented state-space system (ss object)
%       list_removed_inputs - 1-based indices of CS inputs to remove
%       num_non_cs_inputs  - number of trailing non-CS inputs (e.g. 1 for
%                            gust only, 2 for gust + thrust)

    num_cs_inputs = size(sys.B, 2) - num_non_cs_inputs;
    num_states    = size(sys.A, 1);

    % Delta states occupy the last num_cs_inputs rows/columns of A.
    % Map removed input indices to their corresponding delta state indices.
    list_removed_delta_states = num_states - num_cs_inputs + list_removed_inputs;

    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;

    % Remove delta state columns from A and C
    A(:, list_removed_delta_states) = [];
    C(:, list_removed_delta_states) = [];

    % Remove delta state rows from A and B
    A(list_removed_delta_states, :) = [];
    B(list_removed_delta_states, :) = [];

    % Remove delta_dot input columns from B and D
    B(:, list_removed_inputs) = [];
    D(:, list_removed_inputs) = [];

    sys_reduced = ss(A, B, C, D, sys.Ts);

end