function indices = get_indices_for_case(input_settings)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
indices = struct();

% Vertical tip displacements

indices.tip_displacement_in_y = input_settings.n_nodes * 6 + 9 * input_settings.rbm...
    + input_settings.n_nodes_wing * 6 -3;
indices.tip_displacement_in_y = [indices.tip_displacement_in_y ...
    indices.tip_displacement_in_y + input_settings.n_nodes_wing * 6];
% Eta tip
indices.eta_tip = [indices.tip_displacement_in_y(1)-2:indices.tip_displacement_in_y(1)+3 ...
    indices.tip_displacement_in_y(2)-2:indices.tip_displacement_in_y(2)+3];
% Eta dot tip
indices.eta_dot_tip = indices.eta_tip + input_settings.n_nodes * 6 + 9 * input_settings.rbm;

if input_settings.rbm
    rbm_start = input_settings.n_nodes * 6 * 3 + 9 * 2 + 1;
    indices.rbm = rbm_start:rbm_start+8;
end

end