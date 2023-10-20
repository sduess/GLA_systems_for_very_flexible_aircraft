function flag_finish = convert_SHARPy_h5_file_export_to_matlab_format(file_name_sharpy, ...
    folder, complex, all_complex)
% Reads the state space system data from an h5-file created from SHARPy
% and stores it in a matlab format file.
absolute_file_path = strcat(folder, file_name_sharpy);
% Reference Point and Timestep
eta_ref = h5read(absolute_file_path, '/linearisation_vectors/eta');
eta_dot_ref = h5read(absolute_file_path, '/linearisation_vectors/eta_dot');
dt = h5read(absolute_file_path, '/ss/dt');
% state space matrices 
if complex
    A_full = h5read(absolute_file_path, '/ss/A').r; 
    B_full = h5read(absolute_file_path, '/ss/B').r; 
    C_full = h5read(absolute_file_path, '/ss/C').r;
    D_full = h5read(absolute_file_path, '/ss/D');
    if all_complex
        D_full = h5read(absolute_file_path, '/ss/D').r;
    end
else
    A_full = h5read(absolute_file_path, '/ss/A'); 
    B_full = h5read(absolute_file_path, '/ss/B'); 
    C_full = h5read(absolute_file_path, '/ss/C');
    D_full = h5read(absolute_file_path, '/ss/D');
end
    

save(strcat('./linear_statespace_files_matlab/', ...
            file_name_sharpy(1:length(file_name_sharpy)-9)), ...
     'A_full', ...
     'B_full', ...
     'C_full',  ...
     'D_full', ...
     'eta_ref', ...
     'eta_dot_ref', ...
     'dt', ...
     '-v7.3');

flag_finish = true;
end 
