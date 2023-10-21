function controller_reduced = setup_controller_struct(sys, num_inputs, ...
    num_aero_states, num_modes, num_rbm, list_modes, R_factor, delta_factor)
    controller_reduced.A = sys.A;
    controller_reduced.B_LQR = sys.B(:,1:num_inputs);
    controller_reduced.C = sys.C;
    controller_reduced.D = sys.D;
    controller_reduced.B = sys.B;
    controller_reduced = design_LQR_controller(controller_reduced, num_inputs, ...
                                               num_aero_states, num_modes, num_rbm, list_modes, R_factor, delta_factor);
end