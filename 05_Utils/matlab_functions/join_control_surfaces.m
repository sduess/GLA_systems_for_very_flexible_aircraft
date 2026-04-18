function sys_joined = join_control_surfaces(sys, num_cs_total, indices_of_cs_to_join)
%join_control_surfaces Merge several control surfaces (CS) into one effective input
%   Assumes the system has been augmented by adjust_state_space_system so
%   that the last num_cs_total states are delta positions (one per CS) and
%   the inputs are the corresponding delta_dot rates.
%
%   When the CS are joined, it assumes that all CS of the same type (e.g. 
%   ailerons) deflect identically. Then the control surface deflection
%   rate (delta_dot) inputs can be reduced to one input as well as the 
%   control surface deflection states. Hence, the number of columns in
%   the B and D matrices are reduced to the updated number of inputs and
%   the reduced number of states enables the reduction of rows and/or
%   columns in the A, B, and C matrices.
%
%   Inputs:
%       sys                  - augmented state-space system (ss object)
%       num_cs_total         - total number of CS currently in the system
%       indices_of_cs_to_join - indices of CS to merge (1-based, local to
%                              the CS block); first entry is the target

num_cs_to_join = length(indices_of_cs_to_join);

% Extract matrices — ss object fields cannot be edited in place
A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

% The augmented delta states occupy the last num_cs_total rows/columns of A.
% delta_vector_start is the state index of the first delta state.
delta_vector_start = size(A, 1) - (num_cs_total - 1);
idx_delta_target  = delta_vector_start + (indices_of_cs_to_join(1) - 1); % target delta state
idx_ddot_target   = indices_of_cs_to_join(1);                            % target delta_dot input

% Accumulate contributions of joined CS into the target
for i_cs = 2:num_cs_to_join
    A(:, idx_delta_target) = A(:, idx_delta_target) + ...
        A(:, delta_vector_start + (indices_of_cs_to_join(i_cs) - 1));
    B(:, idx_ddot_target) = B(:, idx_ddot_target) + B(:, indices_of_cs_to_join(i_cs));
    D(:, idx_ddot_target) = D(:, idx_ddot_target) + D(:, indices_of_cs_to_join(i_cs));
end

idx_removed_inputs = indices_of_cs_to_join(2:end);
idx_removed_states = delta_vector_start + indices_of_cs_to_join(2:end) - 1;

% Remove redundant delta_dot input columns from B and D
B(:, idx_removed_inputs) = [];
D(:, idx_removed_inputs) = [];

% Remove redundant delta state rows from B (augmented integration rows)
% and redundant delta state rows/columns from A, and columns from C
B(idx_removed_states, :) = [];
A(:, idx_removed_states) = [];
A(idx_removed_states, :) = [];
C(:, idx_removed_states) = [];

sys_joined = ss(A, B, C, D, sys.Ts);
end

