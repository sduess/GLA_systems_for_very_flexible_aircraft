function sys_symmetric = make_control_inputs_symmetric(sys, num_symmetric_cs, num_ignored_last_columns)
%make_control_inputs_symmetric Reduce left/right CS pairs to one symmetric input
%   Assumes the control surfaces are ordered as:
%       [CS_left_1, ..., CS_left_n, CS_right_1, ..., CS_right_n, ignored...]
%   where n = num_symmetric_cs and "ignored" are non-CS inputs (gust,
%   thrust) that must be preserved.
%
%   The left and right surfaces deflect physically symmetrically (same
%   direction and magnitude). However, in the common SHARPy aircraft 
%   setup, the y-axis of the local coordinate frames runs from root to tip 
%   on each wing, so a rotation around y has opposite signs on the left and 
%   right side. Consequently, the same physical deflection is represented with 
%   opposite signs in the state-space model, and the effective contribution of 
%   one symmetric input u is:
%       B_eff(:,i) = B_left(:,i) - B_right(:,i)
%   The right-side inputs and their augmented delta states are then removed.
%
%   Inputs:
%       sys                    - augmented state-space system (ss object)
%       num_symmetric_cs       - number of left/right CS pairs
%       num_ignored_last_columns - number of trailing input columns to keep
%                                  (gust + thrust inputs)

    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;

    % Accumulate symmetric contribution into left-side columns/states,
    % subtracting the right side to account for the opposite sign convention.
    % The left delta states occupy columns end-2*n+1 : end-n of A,
    % the right delta states occupy columns end-n+1 : end of A.
    for i_cs = 1:num_symmetric_cs
        A(:, end-2*num_symmetric_cs+i_cs) = A(:, end-2*num_symmetric_cs+i_cs) - ...
                                             A(:, end-num_symmetric_cs+i_cs);
        B(:, i_cs) = B(:, i_cs) - B(:, i_cs + num_symmetric_cs);
        D(:, i_cs) = D(:, i_cs) - D(:, i_cs + num_symmetric_cs);
    end

    % Remove redundant right-side delta_dot input columns from B and D,
    % keeping the trailing non-CS inputs (gust, thrust).
    B(:, num_symmetric_cs+1:end-num_ignored_last_columns) = [];
    D(:, num_symmetric_cs+1:end-num_ignored_last_columns) = [];

    % Remove redundant right-side delta state rows from B (augmented
    % integration rows) and rows/columns from A, columns from C.
    B(end-num_symmetric_cs+1:end, :) = [];
    A(:, end-num_symmetric_cs+1:end) = [];
    A(end-num_symmetric_cs+1:end, :) = [];
    C(:, end-num_symmetric_cs+1:end) = [];

    sys_symmetric = ss(A, B, C, D, sys.Ts);
end

