function sys_symmetric_inputs = join_control_surfaces(sys, num_cs_total, indices_of_cs_to_join)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
num_cs_to_join = length(indices_of_cs_to_join);
% Get matrices as they cannot be adjusted while being assigned to a state
% space system.
A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

% Get indices
delta_vector_start = size(A,1) -(num_cs_total-1);
idx_delta_start = delta_vector_start + (indices_of_cs_to_join(1) - 1);
idx_delta_dot_start = indices_of_cs_to_join(1);
% Adjust A, B and D matrices
for i_cs=2:num_cs_to_join
    
    A(:,idx_delta_start) = A(:,idx_delta_start)+ ...
        A(:,delta_vector_start+(indices_of_cs_to_join(i_cs) - 1));
    B(:,idx_delta_dot_start) = B(:,idx_delta_dot_start) + B(:,indices_of_cs_to_join(i_cs));
    D(:,idx_delta_dot_start) = D(:,idx_delta_dot_start) + D(:,indices_of_cs_to_join(i_cs));
end

% delete unused inputs in B, and D matrices 
B(:,indices_of_cs_to_join(2:end)) = [];
B(delta_vector_start+indices_of_cs_to_join(2:end)-1, :) = [];
D(:,indices_of_cs_to_join(2:end)) = [];
% 
% % delete unused delta states from A  and C matrix
A(:,delta_vector_start+indices_of_cs_to_join(2:end)-1) = [];
A(delta_vector_start+indices_of_cs_to_join(2:end)-1, :) = [];
C(:,delta_vector_start++indices_of_cs_to_join(2:end)-1) = [];

sys_symmetric_inputs = ss(A, B, C, D, sys.Ts);
end

