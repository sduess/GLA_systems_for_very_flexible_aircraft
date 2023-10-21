function sys_symmetric_inputs = make_control_inputs_symmetric(sys, num_symmetric_cs, num_ignored_last_columns)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    
    % Adjust A, B and D matrices
    for i_cs=1:num_symmetric_cs
        A(:,end-2*num_symmetric_cs+i_cs) = A(:,end-2*num_symmetric_cs+i_cs)- ...
            A(:,end-num_symmetric_cs+i_cs);
        B(:,i_cs) = B(:,i_cs) - B(:,i_cs + num_symmetric_cs);
        D(:,i_cs) = D(:,i_cs) - D(:,i_cs + num_symmetric_cs);
    end
    % delete unused inputs in B, and D matrices 
    B(:,num_symmetric_cs+1:end-(num_ignored_last_columns)) = [];
    B(end-num_symmetric_cs+1:end, :) = [];
    D(:,num_symmetric_cs+1:end-(num_ignored_last_columns)) = [];

    % delete unused delta states from A  and C matrix
    A(:,end-num_symmetric_cs+1:end) = [];
    A(end-num_symmetric_cs+1:end, :) = [];
    C(:,end-num_symmetric_cs+1:end) = [];

    sys_symmetric_inputs = ss(A, B, C, D, sys.Ts);
end

