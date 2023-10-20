function [state_space_system, eta_ref] = read_SHARPy_state_space_system(absolute_file_path)
%read_SHARPy_state_space_system Gets sate space model from SHARPy output

% Reference Point and Timestep
eta_ref = h5read(absolute_file_path, '/linearisation_vectors/eta');
dt = h5read(absolute_file_path, '/ss/dt');
% Read state space matrices 
A = get_real_matrix_if_exisiting(h5read(absolute_file_path, '/ss/A')); 
B = get_real_matrix_if_exisiting(h5read(absolute_file_path, '/ss/B')); 
C = get_real_matrix_if_exisiting(h5read(absolute_file_path, '/ss/C'));
D = get_real_matrix_if_exisiting(h5read(absolute_file_path, '/ss/D'));


% Sometimes matrices are exported transposed
if size(A, 1) ~= size(B, 1)
   A = transpose(A); 
   B = transpose(B); 
end
if size(C, 1) ~= size(D, 1)
   C = transpose(C); 
   D = transpose(D); 
end

% Assemble final discrete state space system
state_space_system = ss(A, B, C, D, dt);

end 

function matrix = get_real_matrix_if_exisiting(matrix)
    if isa(matrix, 'struct')
        matrix = matrix.r;
    end
end