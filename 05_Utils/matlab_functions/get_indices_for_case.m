function indices = get_indices_for_case(input_settings)
%get_indices_for_case Compute output vector indices for key quantities
%   Computes indices into the SHARPy output vector (y = C*x) for the
%   quantities used in control design and simulation. The output vector
%   is structured as indicated in the SHARPy log file when creating the
%   linear system. Currently it has the form:
%       [forces (n_nodes*6+num_rbm_modes), eta (n_nodes*6), beta_bar (num_rbm_modes), eta_dot (n_nodes*6), beta (num_rbm_modes),
%        eta_dot_dot (n_nodes*6), beta_dot (num_rbm_modes)]
%   where eta contains 6 DOFs per node: 3 translations (x,y,z) and
%   3 rotations. There are 9 additional RBM modes (beta) if the system represents
%   a free-flying configuration (rbm = true).
%
%   Input:
%       input_settings  - struct with required fields:
%           n_nodes         : total number of structural nodes
%           n_nodes_wing    : number of nodes per wing
%           rbm             : logical flag, true if free-flight (RBM active)
%
%   Output:
%       indices         - struct with fields:
%           tip_displacement_in_y  : [1x2] indices of the y-displacement
%                                    output for left and right wing tips
%           eta_tip                : [1x12] indices of the full 6-DOF eta
%                                    block at both wing tips
%           eta_dot_tip            : [1x12] indices of eta_dot at both tips
%                                    (same layout, offset by one eta block)
%           rbm                    : [1x9] indices of rigid body motion
%                                    outputs (only present if rbm is true)
indices = struct();

num_rbm_modes = 9 * input_settings.rbm;
num_eta_components = 6;

% Vertical tip displacements
offset_y_component = 3;
indices.tip_displacement_in_y = input_settings.n_nodes * num_eta_components + num_rbm_modes...
    + input_settings.n_nodes_wing * num_eta_components - offset_y_component;

indices.tip_displacement_in_y = [indices.tip_displacement_in_y ...
    indices.tip_displacement_in_y + input_settings.n_nodes_wing * num_eta_components];

% All displacements and rotation at tip
indices.eta_tip = [indices.tip_displacement_in_y(1)-2:indices.tip_displacement_in_y(1)+3 ...
    indices.tip_displacement_in_y(2)-2:indices.tip_displacement_in_y(2)+3];

% Time derivative of displacements and rotations at tip
indices.eta_dot_tip = indices.eta_tip + input_settings.n_nodes * num_eta_components + num_rbm_modes;

if input_settings.rbm
    % 3 to skip forces (-rbm_modes), eta, and eta_dot, and 2 times the rbm_modes from beta_bar and rbm forces
    rbm_start = input_settings.n_nodes * num_eta_components * 3 + num_rbm_modes * 2 + 1;
    indices.rbm = rbm_start:rbm_start+num_rbm_modes-1;
end

end