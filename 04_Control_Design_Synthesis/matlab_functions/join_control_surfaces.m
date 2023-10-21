function sys_symmetric_inputs = join_control_surfaces(sys, num_symmetric_cs, num_ignored_last_columns)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;
% Adjust A, B and D matrices
delta_start = num_symmetric_cs-1;
delta_dot_start = 1;
for i_cs=1:num_symmetric_cs-1

    A(:,end-delta_start) = A(:,end-delta_start)+ ...
        A(:,end-delta_start+i_cs);
    B(:,delta_dot_start) = B(:,delta_dot_start) + B(:, delta_dot_start +i_cs );
    D(:,delta_dot_start) = D(:,delta_dot_start) + D(:,delta_dot_start + i_cs);
end

% delete unused inputs in B, and D matrices 
B(:,2:end-num_ignored_last_columns) = [];
B(end-delta_start+1:end, :) = [];
D(:,2:end-num_ignored_last_columns) = [];
% 
% % delete unused delta states from A  and C matrix
A(:,end-delta_start+1:end) = [];
A(end-delta_start+1:end, :) = [];
C(:,end-delta_start+1:end) = [];

sys_symmetric_inputs = ss(A, B, C, D, sys.Ts);
end

