function plot_results_kalman_filter(list_simulations, Rn,input_settings)
    % plot_input_comparison(list_simulations)
    plot_output_comparison(list_simulations, Rn)
    plot_state_comparison(list_simulations, Rn, input_settings)
end

function plot_input_comparison(list_simulations)

    for i_case=1:length(list_simulations)
        figure_name =  strcat('delta dot icase: ', int2str(i_case));
        figure('Name',figure_name);
        hold on;
        plot(list_simulations(i_case).known_input.Time, list_simulations(i_case).known_input.Data(:,1))
        plot(list_simulations(i_case).linear_u.Time, list_simulations(i_case).linear_u.Data(:,1), '--')

        legend('kf input','linear input')
        % savefig(figure_name)
        hold off;

    end
    for i_case=1:length(list_simulations)

        figure_name =  strcat('delta icase: ', int2str(i_case));
        figure('Name',figure_name);
        hold on;
        plot(list_simulations(i_case).known_input.Time, list_simulations(i_case).known_input.Data(:,2))
        plot(list_simulations(i_case).linear_x.Time, list_simulations(i_case).linear_x.Data(:,end), '--')
    
        legend('kf input','linear input')
        % savefig(figure_name)
        hold off;

    end   
        figure_name = 'gust_input';
        figure('Name',figure_name);
        hold on; 
    for i_case=1:length(list_simulations)

        plot(list_simulations(i_case).linear_u.Time, list_simulations(i_case).linear_u.Data(:,2), '-')
    

    end
        legend("001", "005", "01", "10")
        % savefig(figure_name)
        hold off;
end

function plot_output_comparison(list_simulations, Rn)

    for i_case=1:length(list_simulations)
        figure()
        i_case = 1;
        i_output = 3;
        hold on;
        plot(list_simulations(i_case).reference_value_correction.Time, list_simulations(i_case).reference_value_correction.Data(:,i_output))
        plot(list_simulations(i_case).linear_y.Time, list_simulations(i_case).linear_y.Data(:,i_output))
        plot(list_simulations(i_case).yhat.Time, list_simulations(i_case).yhat.Data(:,i_output))
        writematrix([list_simulations(i_case).reference_value_correction.Data(:,i_output) list_simulations(i_case).linear_y.Data(:,i_output) list_simulations(i_case).yhat.Data(:,i_output)], ...
            './wingtip_estimattion_pre_defined.csv');
        figure_name =  strcat('Rn = ', Rn, ';  output icase: ', int2str(i_case));
        figure('Name',figure_name);
        % linear_y = list_simulations(i_case).linear_y.Data(:,input_settings.sensors) 
        for i_output=1:size(list_simulations(i_case).reference_value_correction.Data,2)
        
            subplot(4,6, i_output);
            hold on;
            plot(list_simulations(i_case).reference_value_correction.Time, list_simulations(i_case).reference_value_correction.Data(:,i_output))
            plot(list_simulations(i_case).linear_y.Time, list_simulations(i_case).linear_y.Data(:,i_output))
            plot(list_simulations(i_case).yhat.Time, list_simulations(i_case).yhat.Data(:,i_output))
        end
        legend('nonlinear sensor measurements','linear output','estimated output')
        % savefig(figure_name)
        hold off;

    end
end



function plot_state_comparison(list_simulations, Rn,input_settings)
        figure()
        i_case = 1;
        i_state = input_settings.num_aero_states+1;
        hold on;
        plot(list_simulations(i_case).linear_x.Time, list_simulations(i_case).linear_x.Data(:,i_state))
        plot(list_simulations(i_case).xhat.Time, list_simulations(i_case).xhat.Data(:,i_state))
         ylabel('q1')
        writematrix([list_simulations(i_case).linear_x.Data(:,i_state) list_simulations(i_case).xhat.Data(:,i_state)], ...
            './q1_pre_defined.csv');
        figure()
        i_case = 1;
        i_state = input_settings.num_aero_states+1+input_settings.num_modes;
        hold on;
        plot(list_simulations(i_case).linear_x.Time, list_simulations(i_case).linear_x.Data(:,i_state))
        plot(list_simulations(i_case).xhat.Time, list_simulations(i_case).xhat.Data(:,i_state))
        ylabel('q1 dot')
        writematrix([list_simulations(i_case).linear_x.Data(:,i_state) list_simulations(i_case).xhat.Data(:,i_state)], ...
            './q1_dot_pre_defined.csv');
       num_modes_per_graph = 10
    for i_case=1:length(list_simulations)

        figure_name =  strcat('Rn = ', Rn, ';  states icase: ', int2str(i_case));
        figure('Name',figure_name);
        state_start=0'
        for i_state=1:size(list_simulations(i_case).xhat.Data,2) 
            i_state
            if mod(i_state-1,num_modes_per_graph)==0
                figure('Name',strcat('states ', int2str(i_state), '-', int2str(i_state+num_modes_per_graph)));
                state_start = i_state-1;
            end
         
                subplot(5,2, i_state-state_start);
        title(strcat('state ', int2str(i_state)));
            hold on;
            plot(list_simulations(i_case).linear_x.Time, list_simulations(i_case).linear_x.Data(:,i_state))
            plot(list_simulations(i_case).xhat.Time, list_simulations(i_case).xhat.Data(:,i_state))
        end

        legend('linear state', 'estimated state')
        % % savefig(figure_name)
        hold off;
    end
end