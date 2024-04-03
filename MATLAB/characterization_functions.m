classdef characterization_functions
    %characterization_functions - class to house functions for system
    %characterizations including characterizing the sensing subsystem and
    %attacking subsystem

    methods (Static)
        
    %% Attacking Subsystem Evaluation Functions
    
    %{
        Purpose: Generate a series of desired range and velocity test
            points that can be used to test the attacking subsystem's spoofing
            accuracy
        Inputs:
            num_cases: the number of test cases to generate
            valid_ranges: valid spoofing target test ranges [min,max]
            valid_velocities: valid spoofing target test velocities
                [min,max]
    %}
    function [ranges,velocities] = initialize_attack_subsystem_test_cases( ...
                num_cases, ...
                valid_ranges, ...
                valid_velocities)


            %initialize the output arrays
            ranges = zeros(num_cases,1);
            velocities = zeros(num_cases,1);

            %using a for loop initialize all of the test cases
            for i = 1:num_cases
                
                % select parameters for the test case
                %select a random range
                ranges(i) = rand * (valid_ranges(2) - valid_ranges(1)) + valid_ranges(1);

                %select a random velocity
                velocities(i) = rand * (valid_velocities(2) - valid_velocities(1)) + valid_velocities(1);
            end
        end

        %{
            Purpose: compute the objects sensed by a victim given a
            spoofing attack
            Inputs:
                config_path - path to the .json configuration file
                spoof_range - actual range of the object to be added
                spoof_velocity - actual velocity of the object ot be
                    spoofed
                frames_to_compute - number of frames to simulate before
                    recording a result
                attack_start_frame - the frame that the attack starts at
            Outputs:
                estimated_ranges - the estimated ranges for each frame that
                    the victim was under attack
                estimated_velocities - the estimated velocities for each
                    frame that the victim was under attack
                desired_ranges - the desired spoof location
                desired_velocities - the desired spoof velocity
        %}
        function [estimated_ranges,estimated_velocities,...
                desired_ranges,desired_velocities] = ...
            compute_sensed_targets(config_path,spoof_range,spoof_velocity,frames_to_compute,attack_start_frame)
            
            simulator = Simulator_revB();
            
            simulator.load_params_from_JSON(config_path);
            
            %apply timing offsets as desired
            simulator.Victim.timing_offset_us = 0;
            simulator.Attacker.Subsystem_tracking.timing_offset_us = 0;
            
            %configure the FMCW parameters
            simulator.configure_FMCW_Radar_parameters();
            
            %load default attacker, and victim positions and velocities
            simulator.load_realistic_attacker_and_victim_position_and_velocity_random();
            
            %load the target
            simulator.load_target_realistic(50,15);
        
            %initialize victim and simulation parameters
        
            
            %specify whether or not to record a move of the range-doppler plot
            record_movie = false;
            range_lims = [simulator.Victim.Radar_Signal_Processor.distance_detection_range(1) - 5,150];
            vel_lims = [-50,50];
            vel_exclusion_region = [-0.1,0.1];
            frame_period_ms = simulator.Victim.FramePeriodicity_ms;
            enable_simplified_plots = false;
            simulator.Victim.Radar_Signal_Processor.configure_movie_capture(frames_to_compute, ...
                record_movie,range_lims,vel_lims,vel_exclusion_region,enable_simplified_plots);
            
            %pre-compute the victim's chirps
            simulator.Victim.precompute_radar_chirps();
        
            %initialize the attacker parameters
            simulator.Attacker.initialize_attacker(...
                                    simulator.Victim.FMCW_sampling_rate_Hz * 1e-6,...
                                    simulator.Victim.StartFrequency_GHz,...
                                    simulator.Victim.Chirp_Tx_Bandwidth_MHz);
            
            %initialize the sensing subsystem's debugger
            simulator.Attacker.Subsystem_spectrum_sensing.initialize_debugger(0,simulator.Victim,frames_to_compute);
            %set to change 1 to zero to disable
            
            %specify the type of emulation ("target", 
                    % "velocity spoof - noisy", 
                    % "velocity spoof - similar velocity",
                    % "range spoof - similar slope")
            
            %attack_type = "range spoof - similar slope";
            attack_type = "target";
            %attack_type = "target";
            %initialize the attacker
            simulator.Attacker.Subsystem_attacking.set_attack_mode(attack_type);
            
            %if it is desired to specify a specific attack location
            simulator.Attacker.Subsystem_attacking.set_desired_attack_location(spoof_range,spoof_velocity);
        
            simulator.run_simulation_attack_no_target(frames_to_compute,false);
            
            %get the return values
            final_attack_frame = simulator.Attacker.Subsystem_attacking.frame;
            num_attack_frames = frames_to_compute - attack_start_frame + 1;

            %get the estimated ranges and velocities
            range_detections = simulator.Victim.Radar_Signal_Processor.range_estimates;
            velocity_detections = simulator.Victim.Radar_Signal_Processor.velocity_estimates;
            
            %identify the point in the detections that is closest to desired target
            %location
            
            
            point = [spoof_range, spoof_velocity];
            
            closest_points = characterization_functions.identify_nearest_detections( ...
                range_detections,velocity_detections, ...
                point);
            %get the detections corresponding to the attack only
            attack_detections = closest_points( ...
                attack_start_frame:attack_start_frame + num_attack_frames - 1, :);
            
            estimated_ranges = attack_detections(:,1);
            estimated_velocities = attack_detections(:,2);
       
            if anynan(estimated_ranges)
                stop = true;
            end

            %compute the desired spoofing ranges and velocities based on
            %the final attacker frame
            end_idx = num_attack_frames - 1;
            start_idx = 0;
            desired_ranges = (spoof_range - spoof_velocity * (start_idx:end_idx) ...
                * simulator.Victim.FramePeriodicity_ms * 1e-3).';
            
            desired_velocities = spoof_velocity * ones(num_attack_frames,1);
        end

        %{
            Purpose: Run through all test cases for evaluating the spoofing
                location accuracy and save the results to a file
            Inputs:
                config_path - path to the .json configuration file
                spoof_ranges - actual range of the object to be added
                spoof_velocities - actual velocity of the object ot be
                    spoofed
                frames_to_compute - number of frames to simulate before
                    recording a result
                attack_start_frame - the frame that the attack starts at
                num_cases - the number of test cases
                save_file_name - the name of the file (without the .csv) to
                    save the results to
            Outputs:
                test_data_spoofing_performance - the data from each of the
                    test cases
        %}
        function test_data_spoofing_performance = ...
                run_spoofing_performance_test_cases( ...
                    num_cases, ...
                    frames_to_compute, ...
                    attack_start_frame, ...
                    spoof_ranges, ...
                    spoof_velocities, ...
                    config_path, ...
                    save_file_name)

            save_file_headers = [...
                "Spoof Ranges (m)",...
                "Spoof Velocities (m/s)",...
                "Estimated Ranges (m)",...
                "Estimated velocities (m/s)",...
                "Absolute Range Spoof Error (m)",...
                "Absolute Velocity Spoof Error (m)"];

            frames_per_case = frames_to_compute - attack_start_frame + 1;

            test_data_spoofing_performance = zeros(num_cases * frames_per_case,size(save_file_headers,2));
            
            status = sprintf("Running Test: %d of %d",1, num_cases);
            progress_bar = waitbar(0,status,"Name","Testing Sensing Subsystem");
            for i = 1:num_cases
                %update the waitbar
                status = sprintf("Running Test: %d of %d",i, num_cases);
                waitbar(i/num_cases,progress_bar,status);
            
                %run the test case
            
                    [estimated_ranges,estimated_velocities,...
                    desired_ranges,desired_velocities] = ...
                            characterization_functions.compute_sensed_targets(config_path,...
                            spoof_ranges(i),...
                            spoof_velocities(i),...
                            frames_to_compute,...
                            attack_start_frame);
            
                %save values - resulting averages from simulation runs
                start_idx = frames_per_case * (i - 1) + 1;
                end_idx = frames_per_case * i;
               
                test_data_spoofing_performance(start_idx:end_idx,1) = desired_ranges;
                test_data_spoofing_performance(start_idx:end_idx,2) = desired_velocities;
                test_data_spoofing_performance(start_idx:end_idx,3) = estimated_ranges;
                test_data_spoofing_performance(start_idx:end_idx,4) = estimated_velocities;
                test_data_spoofing_performance(start_idx:end_idx,5) = abs(desired_ranges - estimated_ranges);
                test_data_spoofing_performance(start_idx:end_idx,6) = abs(desired_velocities - estimated_velocities);
                
                %save the results to a file - continuously saving the data
                %allows for obtaining some data even in the event of a error or
                %crash

                %save the testing data for results
                test_data_results = array2table(test_data_spoofing_performance,"VariableNames",save_file_headers);
                writetable(test_data_results,save_file_name + ".csv",'WriteRowNames',true); 
            end

            
        end
        
        %% Plotting functions for spoofing evaluation
        %{
            Purpose: generate a plot for the test configuration ranges and
                velocities
            Inputs: 
                read_file_path - csv file containing the simulation
                    results, desired spoof ranges and desired spoof
                    velocities
        %}
        function plot_spoofing_test_configurations(read_file_path)
            table_data_results = readtable(read_file_path);
            ranges_m = table_data_results{:,1};
            velocities_m_s = table_data_results{:,2};
            figure;
            font_size = 17;
            set(gcf,'Position',[100 100 400 400])
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            scatter(velocities_m_s, ranges_m);
            ylabel("Spoofing Range (m)","FontSize",font_size)
            xlabel("Spoofing Velocity (m/s)","FontSize",font_size)
            title("Spoofing Cases to Test","FontSize",font_size)
            saveas(gcf, "generated_plots/spoofing_test_configurations.png")
        end

        %{
            Purpose: generate a plot for CDF of the test configuration
                spoofing ranges and velocities
            Inputs: 
                read_file_path - csv file containing the simulation
                    results, desired spoof ranges and desired spoof
                    velocities
        %}
        function plot_spoofing_test_configuration_cdfs(read_file_path)
            table_data_results = readtable(read_file_path);
            ranges_m = table_data_results{:,1};
            velocities_m_s = table_data_results{:,2};
            figure;
            font_size = 17;
            set(gcf,'Position',[100 100 800 400])
            ax = gca;
            ax.FontSize = font_size;
            %plot cdf for slopes
            subplot(1,2,1);
            [h,stats] = cdfplot(ranges_m);
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            h.LineWidth = 4.0;
            xlabel("Spoofing Range (m)","FontSize",font_size)
            title({"CDF of Spoofing "; " Ranges"},"FontSize",font_size)

            %plot cdf for chirp cycle times
            subplot(1,2,2);
            [h,stats] = cdfplot(velocities_m_s);
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            h.LineWidth = 4.0;
            xlabel("Spoofing Velocity (m/s)","FontSize",font_size)
            title({"CDF of Test Spoofing";" Velocities"},"FontSize",font_size)
            saveas(gcf, "generated_plots/spoofing_test_configuration_cdfs.png")
        end

        %{
            Purpose: generate a table of the key statistics for the
                absolute error of the spoofing range accuracy and plot the cdf of the
                absolute error
        %}
        function summary_table = generate_range_spoofing_accuracy_summary(read_file_path)
            
            %plot the cdf of the chirp slope errors
            table_data_results = readtable(read_file_path);
            actual_values = table_data_results{:,1};
            estimated_values = table_data_results{:,3};
            abs_errors = table_data_results{:,5};

            %get the number of failed trials
            num_failed_trials = size(abs_errors(isnan(abs_errors)),1);
            percent_failed_trials = num_failed_trials / size(abs_errors,1);

            %remove all NaN values from the dataset
            abs_errors = abs_errors(~isnan(abs_errors));

            %plot the results
            
            %statistics based on absolute errors
            mean_error = mean(abs_errors);
            variance_error = var(abs_errors);
            MSE_error = mse(actual_values,estimated_values);
            tail_95th_percentile = prctile(abs_errors,95);

            figure;
            [h,stats] = cdfplot(abs_errors);
            xlabel("Range Spoofing Error (m)")
            title("CDF of Chirp Slope Errors")
            if max(abs_errors) > 5 * tail_95th_percentile
                xlim([0,2 * tail_95th_percentile])
            end
            
            variable_names = ["Mean (m)", "Variance (m)^2", "MSE (m)^2", "95th Percentile (m)", "Percent Failed Trials"];
            summary_table = array2table( ...
                [mean_error,...
                variance_error,...
                MSE_error,...
                tail_95th_percentile,...
                percent_failed_trials], ...
                "VariableNames",variable_names);
            
            saveas(gcf, "generated_plots/range_spoofing_accuracy_error_cdf.png")

        end

        %{
            Purpose: generate a table of the key statistics for the
                absolute error of given metric and plot the cdf of the
                absolute error
            Inputs:
                read_file_path - path to the read file containing the
                    necessary information to generate the testing summary
                actual_values_idx - column index in the read file
                    corresponding to the actual values
                estimated_values_idx - column index in the read file
                    corresponding to the estimated values
                abs_errors_idx - column index in the read file
                    corresponding to the absolute errors for the metric
                metric_title - a string of the name of the metric that the summary is
                    being generated for
                metric_units - a string of the units of the metric that the
                    summary is being generated for
                percentile - the percentile to compute for the tail
                scale_factor - scale the error by a specified factor
                (used to change unit scales)
                override_current_figure - on true, over-rides current
                    figure, on false, adds plot to the current figure
        %}
        function summary_table = generate_testing_summary_absolute_error(...
                read_file_path,...
                actual_values_idx, ...
                estimated_values_idx, ...
                abs_errors_idx, ...
                metric_title, ...
                metric_units, ...
                percentile,...
                scale_factor, ...
                override_current_figure)
            
            %plot the cdf of the chirp slope errors
            table_data_results = readtable(read_file_path);
            actual_values = table_data_results{:,actual_values_idx} * scale_factor;
            estimated_values = table_data_results{:,estimated_values_idx} * scale_factor;
            abs_errors = table_data_results{:,abs_errors_idx} * scale_factor;

            %get the number of failed trials
            num_failed_trials = size(abs_errors(isnan(abs_errors)),1);
            percent_failed_trials = num_failed_trials / size(abs_errors,1);

            %remove all NaN values from the dataset
            abs_errors = abs_errors(~isnan(abs_errors));

            %plot the results
            
            %statistics based on absolute errors
            mean_error = mean(abs_errors);
            variance_error = var(abs_errors);
            MSE_error = mse(actual_values,estimated_values);
            tail = prctile(abs_errors,percentile);
            
            %override current figure if desired
            font_size = 17;
            if override_current_figure
                clf;
                figure;
                set(gcf,'Position',[100 100 400 400])
                [h,stats] = cdfplot(abs_errors);
            else
                gcf;
                hold on;
                [h,stats] = cdfplot(abs_errors);
                h.LineWidth = 4.0;
                h.LineStyle = ":";
                hold off;
                %add the legend - assuming this is the USRP results
                legend(["Sim","USRP"],"Location","southeast")
            end
            
            %finish plotting
            h.LineWidth = 4.0;
            xlabel(metric_title + " (" + metric_units +")" ,"FontSize",font_size - 2);
            ylabel("CDF","FontSize",font_size - 2);
            title({"CDF of Absolute "; metric_title + " Error"},"FontSize",font_size);
            if max(abs_errors) > 5 * tail
                xlim([0,4 * tail])
            end
            ax = gca;
            ax.LineWidth = 2.0;
            ax.FontSize = font_size;
            
            
            variable_names = [...
                "Mean (" + metric_units + ")",...
                "Variance ("+ metric_units + ")^2",...
                "MSE (" + metric_units + ")^2",...
                int2str(percentile) + "th Percentile (" + metric_units + ")",...
                "Percent Failed Trials"];
            summary_table = array2table( ...
                [mean_error,...
                variance_error,...
                MSE_error,...
                tail,...
                percent_failed_trials], ...
                "VariableNames",variable_names);

            %set the save file path
            save_file_name= strrep(metric_title," ","_");

            %save the cdf plot
            saveas(gcf, "generated_plots/" + save_file_name + "_absolute_error_cdf.png");

            %save a csv of the summary table
            writetable(summary_table,save_file_name + "_absolute_error_summary.csv",'WriteRowNames',true); 
        end

        %{
            Purpose: generate a table of the key statistics for the
                relative error of given metric and plot the cdf of the
                relative error
            Inputs:
                read_file_path - path to the read file containing the
                    necessary information to generate the testing summary
                actual_values_idx - column index in the read file
                    corresponding to the actual values
                estimated_values_idx - column index in the read file
                    corresponding to the estimated values
                abs_errors_idx - column index in the read file
                    corresponding to the absolute errors for the metric
                metric_title - a string of the name of the metric that the summary is
                    being generated for
                metric_units - a string of the units of the metric that the
                    summary is being generated for
                percentile - the percentile to compute for the tail
                scale_factor - scale the error by a specified factor
                (used to change unit scales)
                override_current_figure - on true, over-rides current
                    figure, on false, adds plot to the current figure
        %}
        function summary_table = generate_testing_summary_relative_error(...
                read_file_path,...
                actual_values_idx, ...
                estimated_values_idx, ...
                abs_errors_idx, ...
                metric_title, ...
                metric_units, ...
                percentile,...
                scale_factor, ...
                override_current_figure)
            
            %plot the cdf of the chirp slope errors
            table_data_results = readtable(read_file_path);
            actual_values = table_data_results{:,actual_values_idx} * scale_factor;
            estimated_values = table_data_results{:,estimated_values_idx} * scale_factor;
            absolute_errors = table_data_results{:,abs_errors_idx} * scale_factor;
            relative_errors = 100 * absolute_errors ./ actual_values;

            %get the number of failed trials
            num_failed_trials = size(relative_errors(isnan(relative_errors)),1);
            percent_failed_trials = num_failed_trials / size(relative_errors,1);

            %remove all NaN values from the dataset
            relative_errors = relative_errors(~isnan(relative_errors));

            %plot the results
            
            %statistics based on absolute errors
            mean_error = mean(relative_errors);
            variance_error = var(relative_errors);
            MSE_error = mse(actual_values,estimated_values);
            tail = prctile(relative_errors,percentile);

            %override current figure if desired
            font_size = 17;
            if override_current_figure
                clf;
                figure;
                set(gcf,'Position',[100 100 400 450])
                [h,stats] = cdfplot(relative_errors);
            else
                gcf;
                hold on;
                [h,stats] = cdfplot(relative_errors);
                h.LineWidth = 4.0;
                h.LineStyle = "--";
                hold off;
                %add the legend - assuming these are the USRP results
                legend(["Sim","USRP"],"Location","southeast")
            end
            
            %finish plotting
            h.LineWidth = 4.0;
            xlabel({"Relative Error";"(Percent " + metric_units + ")"} ,"FontSize",font_size - 2);
            ylabel("CDF","FontSize",font_size - 2);
            title({"CDF of Relative "; metric_title + " Error"},"FontSize",font_size);
            if max(relative_errors) > 5 * tail
                xlim([0,2 * tail])
            end
            ax = gca;
            ax.LineWidth = 2.0;
            ax.FontSize = font_size;
            
            
            variable_names = [...
                "Mean (" + metric_units + ")",...
                "Variance ("+ metric_units + ")^2",...
                "MSE (" + metric_units + ")^2",...
                int2str(percentile) + "th Percentile (" + metric_units + ")",...
                "Percent Failed Trials"];
            summary_table = array2table( ...
                [mean_error,...
                variance_error,...
                MSE_error,...
                tail,...
                percent_failed_trials], ...
                "VariableNames",variable_names);

            %set the save file path
            save_file_name= strrep(metric_title," ","_");

            %save the cdf plot
            saveas(gcf, "generated_plots/" + save_file_name + "_relative_error_cdf.png");

            %save a csv of the summary table
            writetable(summary_table,save_file_name + "_relative_error_summary.csv",'WriteRowNames',true); 
        end

%% Sensing Subsystem Evaluation Functions

        %{
            Purpose: Initialize a series of random victim configurations to
                be used to test the sensing subsystem performance
            Inputs:
                num_cases - the number of test cases to generate
                valid_slopes - the range of valid slopes expressed as
                    [min,max]
                valid_chirp_periods - the range of valid chirp periods
                    expressed as [min,max]
                valid_BWs_MHz - the range of valid bandwidths for a victim
                    expressed as [min,max]
                num_adc_samples - the number of ADC samples
            Outputs:
                slopes - the slope for each victim test
                chirp_periods - the chirp period for each victim test
                adc_sampling_rates - the adc sampling rate for each victim
                    test
        %}
        function [slopes,chirp_periods,adc_sampling_rates] = initialize_sensing_subsystem_test_cases_general( ...
                num_cases, ...
                valid_slopes, ...
                valid_chirp_periods, ...
                valid_BWs_MHz, ...
                num_adc_samples)


            %initialize the output arrays
            slopes = zeros(num_cases,1);
            chirp_periods = zeros(num_cases,1);
            adc_sampling_rates = zeros(num_cases,1);

            %using a for loop initialize all of the test cases
            for i = 1:num_cases
                
                %% select parameters for the test case
                %select a random slope
                slope_MHz_us = (rand * (valid_slopes(2) - valid_slopes(1)))...
                    + valid_slopes(1);
                
                %determine the range of chirp cycle periods that work with the selected
                %slope
                chirp_cycle_time_range_us = [0,0];
                
                %compute the min chirp cycle time
                chirp_cycle_time_range_us(1) = ...
                    max(valid_chirp_periods(1),valid_BWs_MHz(1)/slope_MHz_us);
                
                %compute the max chirp cycle time
                chirp_cycle_time_range_us(2) = ...
                    min(valid_chirp_periods(2),valid_BWs_MHz(2)/slope_MHz_us);
                
                %randomly select a chirp cycle time
                chirp_cycle_time_us = (rand * (chirp_cycle_time_range_us(2) - chirp_cycle_time_range_us(1)))...
                    + chirp_cycle_time_range_us(1);
                
                %compute the required ADC sample rate to achieve the maximum BW
                sampling_period = chirp_cycle_time_us - 7.3;
                ADC_SampleRate_MSps = num_adc_samples / sampling_period;
                
                
                slopes(i) = slope_MHz_us;
                chirp_periods(i) = chirp_cycle_time_us;
                adc_sampling_rates(i) = ADC_SampleRate_MSps;
            end
        end


        %{
            Purpose: Initialize a series of random victim configurations to
                be used to test the sensing subsystem performance
            Inputs:
                slopes - the slope for each victim test
                chirp_periods - the chirp period for each victim test
                adc_sampling_rates - the adc sampling rate for each victim
                    test
                num_adc_samples - the number of ADC samples
                config_path - path to a config file for initializing other
                    simulation variables
            Outputs:
                configurations_passed: a bool specifying that all
                    configurations are usable
        %}
        function configurations_passed = check_test_configurations( ...
                slopes, ...
                chirp_periods, ...
                adc_sampling_rates, ...
                num_adc_samples,...
                config_path)

            num_cases = size(slopes,1);
            configurations_passed = true;
            for i = 1:num_cases

                %initialize the simulator
                simulator = Simulator_revB();
                simulator.load_params_from_JSON(config_path);

                %initialize the victim parameters
                simulator.Victim.FrequencySlope_MHz_us = slopes(i);
                simulator.Victim.ChirpCycleTime_us = chirp_periods(i);
                simulator.Victim.ADC_Samples = num_adc_samples;
                simulator.Victim.ADC_SampleRate_MSps = adc_sampling_rates(i);
                simulator.Victim.compute_calculated_vals();
                
                %set attacker parameters
                simulator.Attacker.Subsystem_tracking.FrequencySlope_MHz_us = slopes(i);
                simulator.Attacker.Subsystem_tracking.ChirpCycleTime_us = chirp_periods(i);
                simulator.Attacker.Subsystem_tracking.ADC_Samples = num_adc_samples;
                simulator.Attacker.Subsystem_tracking.ADC_SampleRate_MSps = adc_sampling_rates(i);
                simulator.Attacker.Subsystem_tracking.compute_calculated_vals();
                
                %apply timing offsets as desired
                simulator.Victim.timing_offset_us = 0;
                simulator.Attacker.Subsystem_tracking.timing_offset_us = 0;
                
                %configure the FMCW parameters
                simulator.configure_FMCW_Radar_parameters();

                if simulator.Victim.IdleTime_us <= 0
                    configurations_passed = false;
                end
            end
            
            if configurations_passed
                fprintf("All Configurations passed \n");
            else
                fprintf("A configuration has failed \n");
            end
        end
        
        %{
            Purpose: generate a plot for the test configuration slopes and
                chirp cycle times
            Inputs: 
                read_file_path - csv file containing the simulation
                    results, actual chirp slopes, and actual chirp cycle times
        %}
        function plot_test_configurations(read_file_path)
            table_data_results = readtable(read_file_path);
            slopes_MHz_us = table_data_results{:,1};
            chirp_cycle_times_MHz_us = table_data_results{:,2};
            figure;
            font_size = 17;
            set(gcf,'Position',[100 100 400 400])
            scatter(chirp_cycle_times_MHz_us, slopes_MHz_us);
            ylabel("Chirp Slope (MHz/us)","FontSize",font_size)
            xlabel("Chirp Cycle Time (us)","FontSize",font_size)
            title("Victim Configurations to Test","FontSize",font_size)
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            saveas(gcf, "generated_plots/victim_test_configurations.png")
        end

        %{
            Purpose: generate a plot for CDF of the test configuration slopes and
                chirp cycle times
            Inputs: 
                read_file_path - csv file containing the simulation
                    results, actual chirp slopes, and actual chirp cycle times
        %}
        function plot_test_configuration_cdfs(read_file_path)
            table_data_results = readtable(read_file_path);
            slopes_MHz_us = table_data_results{:,1};
            chirp_cycle_times_MHz_us = table_data_results{:,2};
            figure;
            font_size = 17;
            set(gcf,'Position',[100 100 800 400])
            %plot cdf for slopes
            subplot(1,2,1);
            [h,stats] = cdfplot(slopes_MHz_us);
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            h.LineWidth = 4.0;
            xlabel("Chirp Slope (MHz/us)","FontSize",font_size)
            ylabel("CDF",FontSize=font_size)
            title("CDF of Test Slopes", "FontSize",font_size)

            %plot cdf for chirp cycle times
            subplot(1,2,2);
            [h,stats] = cdfplot(chirp_cycle_times_MHz_us);
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            h.LineWidth = 4.0;
            xlabel("Chirp Cycle Time (us)","FontSize",font_size)
            ylabel("CDF",FontSize=font_size)
            title("CDF of Test Chirp Cycle Times","FontSize",font_size)
            saveas(gcf, "generated_plots/victim_test_configuration_cdfs.png")
        end

        
        %{
            Purpose: Plot the absolute error of the predicted frame start
            time with respect to the number of frames sensed
            Inputs:
                read_file_path: the path to the file 
                use_log_scale: on true, uses a log scale
        
        %}
        function plot_predicted_start_time_errors(read_file_path,use_log_scale)
            %access the saved data for the predicted start time errors
            table_data_results = readtable(read_file_path);
            prediction_errors = abs(table_data_results{:,:});

            %create arrays for mean absolute error, variance, and 95th
            %percentile
            mean_absolute_errors = zeros(size(prediction_errors,2),1);
            variances = zeros(size(prediction_errors,2),1);
            tail_95th_percentiles = zeros(size(prediction_errors,2),1);


            %for each frame
            for i = 1:size(prediction_errors,2)
                mean_absolute_errors(i) = mean(prediction_errors(:,i));
                variances(i) = var(prediction_errors(:,i));
                tail_95th_percentiles(i) = prctile(prediction_errors(:,i),95);
            end            
            
            %compute range errors
            errors_in_range = mean_absolute_errors * 1e-6 * physconst("LightSpeed");
            
            %plot a figure with error bars
            error_bars = tail_95th_percentiles;
            figure;
            font_size = 17;
            clf;
            set(gcf,'Position',[100 100 350 350])
            errorbar(mean_absolute_errors,error_bars,"LineWidth",2.0);
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            title("Frame Prediction Error","FontSize",font_size)
            xlabel({"Number of Frames"; "Sensed"},"FontSize",font_size)
            ylabel("Error (us)","FontSize",font_size)
            y_lim = 5 *max(mean_absolute_errors);
            ylim([0, y_lim])
            xlim([0,size(prediction_errors,2)+ 1])
            
            if  use_log_scale
                set(gca, 'YScale','log');
            end
            
            %add in the axis for range error
            yyaxis right
            plot(errors_in_range,"LineWidth",2.0)
            ylabel("Range Error (m)","FontSize",font_size)
            set(gca,"YColor", 'black')
            y_lim = 5 * max(errors_in_range);
            ylim([0, y_lim])
            if use_log_scale
                set(gca, 'YScale','log');
            end
            grid on;

            saveas(gcf, "generated_plots/predicted_start_time_errors.png")
            
        end
%% Functions to Run Sensing Subsystem Simulations

        %{
            Purpose: compute the estimated chirp slope, chirp duration, and
            frame duration for a given sensing subsystem
            Inputs:
                config_path - path to the .json configuration file
                slope_MHz_us - the chirp slope to use for testing
                chirp_cycle_period_us - the chirp cycle period
                adc_sampling_rates - the adc sampling rate for each victim
                    test
                num_adc_samples - the number of ADC samples
                num_chirps - the number of chirps per frame
                frames_to_compute - number of frames to simulate before
                    recording a result
                attack_start_frame - the frame that the attack starts at
            Outputs:
                actual_slope
                estimated_slope
                actual_chirp_duration
                estimated_chirp_duration
                actual_frame_duration
                estimated_chirp_duration
                ADC_SampleRate_MSps
                predicted_start_time_errors
        %}
        function [actual_slope,estimated_slope,...
            actual_chirp_duration,estimated_chirp_duration,...
            actual_frame_duration, estimated_frame_duration,...
            ADC_SampleRate_MSps, predicted_start_time_errors] = ...
            compute_sensed_values( ...
                config_path, ...
                slope_MHz_us, ...
                chirp_cycle_period_us, ...
                ADC_SampleRate_MSps, ...
                num_adc_samples, ...
                num_chirps, ...
                frames_to_compute)
            
            %initialize the simulator
            simulator = Simulator_revB();
            simulator.load_params_from_JSON(config_path);

            %initialize the victim parameters
            simulator.Victim.FrequencySlope_MHz_us = slope_MHz_us;
            simulator.Victim.ChirpCycleTime_us = chirp_cycle_period_us;
            simulator.Victim.ADC_Samples = num_adc_samples;
            simulator.Victim.ADC_SampleRate_MSps = ADC_SampleRate_MSps;
            simulator.Victim.NumChirps = num_chirps;
            simulator.Victim.compute_calculated_vals();
            
            %set attacker parameters
            simulator.Attacker.Subsystem_tracking.FrequencySlope_MHz_us = slope_MHz_us;
            simulator.Attacker.Subsystem_tracking.ChirpCycleTime_us = chirp_cycle_period_us;
            simulator.Attacker.Subsystem_tracking.ADC_Samples = num_adc_samples;
            simulator.Attacker.Subsystem_tracking.ADC_SampleRate_MSps = ADC_SampleRate_MSps;
            simulator.Attacker.Subsystem_tracking.NumChirps = num_chirps;
            simulator.Attacker.Subsystem_tracking.compute_calculated_vals();
            
            %apply timing offsets as desired
            simulator.Victim.timing_offset_us = 0;
            simulator.Attacker.Subsystem_tracking.timing_offset_us = 0;
            
            %configure the FMCW parameters
            simulator.configure_FMCW_Radar_parameters();
            
            %load default attacker, and victim positions and velocities
            simulator.load_realistic_attacker_and_victim_position_and_velocity_random();
            
            %load the target
            simulator.load_target_realistic(50,15);
        
            %initialize victim and simulation parameters
        
            
            %specify whether or not to record a move of the range-doppler plot
            record_movie = false;
            range_lims = [simulator.Victim.Radar_Signal_Processor.distance_detection_range(1) - 5,150];
            vel_lims = [-50,50];
            vel_exclusion_region = [-1,1];
            frame_period_ms = simulator.Victim.FramePeriodicity_ms;
            enable_simplified_plots = false;
            simulator.Victim.Radar_Signal_Processor.configure_movie_capture(frames_to_compute, ...
                record_movie,range_lims,vel_lims,vel_exclusion_region, enable_simplified_plots);
            
            %pre-compute the victim's chirps
            simulator.Victim.precompute_radar_chirps();
        
            %initialize the attacker parameters
            simulator.Attacker.initialize_attacker(...
                                    simulator.Victim.FMCW_sampling_rate_Hz * 1e-6,...
                                    simulator.Victim.StartFrequency_GHz,...
                                    simulator.Victim.Chirp_Tx_Bandwidth_MHz);
            
            %initialize the sensing subsystem's debugger
            simulator.Attacker.Subsystem_spectrum_sensing.initialize_debugger(0,simulator.Victim,frames_to_compute);
            %set to change 1 to zero to disable
            
            %specify the type of emulation ("target", 
                    % "velocity spoof - noisy", 
                    % "velocity spoof - similar velocity",
                    % "range spoof - similar slope")
            
            %attack_type = "range spoof - similar slope";
            attack_type = "range spoof - similar slope,velocity spoof - noisy";
            %attack_type = "target";
            %initialize the attacker
            simulator.Attacker.Subsystem_attacking.set_attack_mode(attack_type);
            
            %if it is desired to specify a specific attack location
            %simulator.Attacker.Subsystem_attacking.set_desired_attack_location(100,7);
        
            simulator.run_simulation_attack_no_target(frames_to_compute,false);
            
            %get the return values
        
            %frame duration
            estimated_frame_duration = simulator.Attacker.Subsystem_spectrum_sensing.frame_tracking.average_frame_duration * 1e-3;
            actual_frame_duration = simulator.Victim.FramePeriodicity_ms;
            %chirp duration
            estimated_chirp_duration = simulator.Attacker.Subsystem_spectrum_sensing.frame_tracking.average_chirp_duration;
            actual_chirp_duration = simulator.Victim.ChirpCycleTime_us;
            if abs(estimated_chirp_duration - actual_chirp_duration) > .5
                stop = true;
            end
            %slope
            estimated_slope = simulator.Attacker.Subsystem_spectrum_sensing.frame_tracking.average_slope;
            actual_slope = simulator.Victim.FrequencySlope_MHz_us;

            %adc sample rate
            ADC_SampleRate_MSps = simulator.Victim.ADC_SampleRate_MSps;

            %frame start time prediction errors
            %compare the predicted frame start times with the actual frame start times
            actual_frame_start_times = (3:frames_to_compute).' * simulator.Victim.FramePeriodicity_ms * 1e3 ... 
                                            + simulator.Victim.IdleTime_us;
            %initialize array of NaNs to hold the predicted start times
            predicted_frame_start_times = NaN(size(actual_frame_start_times,1),1);
            num_captured_frames = ...
                simulator.Attacker.Subsystem_spectrum_sensing.frame_tracking.num_captured_frames;
            predicted_frame_start_times(1:num_captured_frames - 1) = ...
                simulator.Attacker.Subsystem_spectrum_sensing.frame_tracking.captured_frames(2:...
                num_captured_frames,7);
            predicted_start_time_errors = predicted_frame_start_times - actual_frame_start_times;
        end

        %{
            Purpose: run the given sensing subsystem performance test cases
            and saves the results to a csv file
            Inputs:
                num_cases - number of test cases to run
                frames_to_compute - number of frames to simulate for each
                    run
                slopes_MHz_us - array of slopes to test
                chirp_cycle_periods_us - array of chirp cycle periods
                adc_sampling_rates - array of adc sampling rates
                num_adc_samples - the number of ADC samples
                num_chirps - the number of chirps per frame
                config_path - path to the json config
                save_file_name - path to save the results file to (without
                    the .csv extension
            Outputs:
                testing_data_parameter_estimations - key parameter
                    estimations for each test case
                testing_data_frame_start_prediction_errors - the error in
                    the frame start time prediction for each frame in each test
                    case
        %}
        function [testing_data_parameter_estimations, ...
                testing_data_frame_start_prediction_errors] = ...
                run_sensing_performance_test_cases( ...
                    num_cases, ...
                    frames_to_compute, ...
                    slopes_MHz_us, ...
                    chirp_cycle_periods_us, ...
                    adc_sampling_rates, ...
                    num_adc_samples,...
                    num_chirps, ...
                    config_path, ...
                    save_file_name)

            save_file_headers = ["Actual Chirp Slope (Mhz/us)",...
                "Actual Chirp Cycle Period (us)",...
                "Actual Frame Duration (ms)",...
                "ADC Sample Rate (MSPS)",...
                "Estimated Chirp Slope (Mhz/us)",...
                "Estimated Chirp Cycle Period (us)",...
                "Estimated Frame Duration (ms)",...
                "Absolute Slope Error (MHz/us)",...
                "Absolute Chirp Cycle Period Error (us)",...
                "Absolute Frame Duration Error (ms)"];

            testing_data_parameter_estimations = zeros(num_cases,size(save_file_headers,2));
            testing_data_frame_start_prediction_errors = zeros(num_cases,frames_to_compute-2);
            
            status = sprintf("Running Test: %d of %d",1, num_cases);
            progress_bar = waitbar(0,status,"Name","Testing Sensing Subsystem");
            for i = 1:num_cases
                %update the waitbar
                status = sprintf("Running Test: %d of %d",i, num_cases);
                waitbar(i/num_cases,progress_bar,status);
            
                %run the test case
            
                            [actual_slope,estimated_slope,...
                        actual_chirp_duration,estimated_chirp_duration,...
                        actual_frame_duration, estimated_frame_duration,...
                        ADC_SampleRate_MSps, predicted_start_time_errors] = ...
                        characterization_functions.compute_sensed_values(config_path, ...
                            slopes_MHz_us(i), ...
                            chirp_cycle_periods_us(i), ...
                            adc_sampling_rates(i), ...
                            num_adc_samples, ...
                            num_chirps, ...
                            frames_to_compute);
            
                %save values - resulting averages from simulation runs
                testing_data_parameter_estimations(i,1) = actual_slope;
                testing_data_parameter_estimations(i,2) = actual_chirp_duration;
                testing_data_parameter_estimations(i,3) = actual_frame_duration;
                testing_data_parameter_estimations(i,4) = ADC_SampleRate_MSps;
                testing_data_parameter_estimations(i,5) = estimated_slope;
                testing_data_parameter_estimations(i,6) = estimated_chirp_duration;
                testing_data_parameter_estimations(i,7) = estimated_frame_duration;
                testing_data_parameter_estimations(i,8) = abs(actual_slope - estimated_slope);
                testing_data_parameter_estimations(i,9) = abs(actual_chirp_duration - estimated_chirp_duration);
                testing_data_parameter_estimations(i,10) = abs(actual_frame_duration - estimated_frame_duration);

                %save values for frame_start_time prediction errors
                testing_data_frame_start_prediction_errors(i,:) = predicted_start_time_errors;
                test_data_results = array2table(testing_data_frame_start_prediction_errors);
                writetable(test_data_results,save_file_name + "_frame_start_time_prediction_errors.csv",...
                    'WriteRowNames',true); 
                
                %save the results to a file - continuously saving the data
                %allows for obtaining some data even in the event of a error or
                %crash

                %save the testing data for results
                test_data_results = array2table(testing_data_parameter_estimations,"VariableNames",save_file_headers);
                writetable(test_data_results,save_file_name + "_parameter_estimations.csv",'WriteRowNames',true); 
            end

        end
    
        
        %% USRP Implementation Functions

        %% USRP SENSING SUBSYSTEM Testing Functions

        %{
            Purpose: Initialize a series of random victim configurations to
                be used to test the sensing subsystem performance
            Inputs:
                num_cases - the number of test cases to generate
                valid_chirp_periods - the range of valid chirp periods
                    expressed as [min,max]
                FMCW_BW_MHz - the desired FMCW Bandwidth for the USRP
                    to operate at
                num_adc_samples - the number of ADC samples
            Outputs:
                slopes - the slope for each victim test
                chirp_periods - the chirp period for each victim test
                adc_sampling_rates - the adc sampling rate for each victim
                    test
        %}
        function [slopes,chirp_periods,adc_sampling_rates] = USRP_initialize_sensing_subsystem_test_cases( ...
                num_cases, ...
                valid_chirp_periods, ...
                FMCW_BW_MHz, ...
                num_adc_samples)


            %initialize the output arrays
            slopes = zeros(num_cases,1);
            chirp_periods = zeros(num_cases,1);
            adc_sampling_rates = zeros(num_cases,1);

            %compute the valid slopes for the given bandwidth and chirp
            %period range
            valid_slopes = [0,0];
            valid_slopes(1) = FMCW_BW_MHz/valid_chirp_periods(2);
            valid_slopes(2) = FMCW_BW_MHz/(valid_chirp_periods(1) - 7); %to ensure a positive idle time

            %using a for loop initialize all of the test cases
            for i = 1:num_cases
                
                % select parameters for the test case
                %select a random slope
                slope_MHz_us = (rand * (valid_slopes(2) - valid_slopes(1)))...
                    + valid_slopes(1);
                
                %determine the range of chirp cycle periods that work with the selected
                %slope
                chirp_cycle_time_range_us = [0,0];
                
                %compute the min chirp cycle time
                chirp_cycle_time_range_us(1) = (FMCW_BW_MHz/slope_MHz_us) + 7; %to ensure a positive idle time
                
                %compute the max chirp cycle time
                chirp_cycle_time_range_us(2) = valid_chirp_periods(2);
                
                %randomly select a chirp cycle time
                chirp_cycle_time_us = (rand * (chirp_cycle_time_range_us(2) - chirp_cycle_time_range_us(1)))...
                    + chirp_cycle_time_range_us(1);
                
                %compute a rough guess for the ADC sample rate required to
                %have the exact bandwidth
                ramp_end_time = FMCW_BW_MHz / slope_MHz_us;
                sampling_period = ramp_end_time - 7.3;
                ADC_SampleRate_MSps = num_adc_samples / sampling_period;

                %slightly adjust the parameters so that configuration will
                %have exactly the provided bandwidth
                downsample_factor = ceil((FMCW_BW_MHz * 1e6) / (ADC_SampleRate_MSps * 1e6));
                ADC_SampleRate_MSps = FMCW_BW_MHz / downsample_factor;
                

                %adjust the chirp cycle period so that there is an integer
                %number of samples in the chirp cycle period
                chirp_cycle_time_us = ceil(chirp_cycle_time_us * ADC_SampleRate_MSps)...
                    / ADC_SampleRate_MSps;
    
                %compute other required chirp parameters
                adc_sampling_period = num_adc_samples / ADC_SampleRate_MSps;
                
                %for the ADC_valid time, find a time that results in an integer
                %number of samples, but is longer than 6.3us (chosen to allow
                %sufficient time for the signal to propogate)
                adc_valid_start_time = ceil(6.3 * ADC_SampleRate_MSps) / ADC_SampleRate_MSps;


                ramp_end_time = ceil((adc_sampling_period + adc_valid_start_time) *...
                                        ADC_SampleRate_MSps) / ADC_SampleRate_MSps;
                slope_MHz_us = FMCW_BW_MHz / ramp_end_time;

                slopes(i) = slope_MHz_us;
                chirp_periods(i) = chirp_cycle_time_us;
                adc_sampling_rates(i) = ADC_SampleRate_MSps;
            end
        end
        
        %{
            Purpose: Initialize a series of random victim configurations to
                be used to test the sensing subsystem performance
            Inputs:
                slopes - the slope for each victim test
                chirp_periods - the chirp period for each victim test
                adc_sampling_rates - the adc sampling rate for each victim
                    test
                num_adc_samples - the number of ADC samples
                VMCW_BW_MHz - the desired bandwidth for USRP operation
                config_path - path to a config file for initializing other
                    simulation variables
            Outputs:
                returns all of the valid slopes, periods, and ADC sampling
                rates
        %}
        function [good_slopes, good_chirp_periods, good_adc_sampling_rates] = ...
            USRP_check_test_configurations( ...
                slopes, ...
                chirp_periods, ...
                adc_sampling_rates, ...
                num_adc_samples,...
                FMCW_BW_MHz,...
                config_path)

            num_cases = size(slopes,1);
            configurations_passed = true;
            failed_cases = 0;

            good_slopes = [];
            good_chirp_periods = [];
            good_adc_sampling_rates = [];

            for i = 1:num_cases

                %initialize the simulator
                simulator = Simulator_revB();
                simulator.load_params_from_JSON(config_path);

                %initialize the victim parameters
                simulator.Victim.FrequencySlope_MHz_us = slopes(i);
                simulator.Victim.ChirpCycleTime_us = chirp_periods(i);
                simulator.Victim.ADC_Samples = num_adc_samples;
                simulator.Victim.ADC_SampleRate_MSps = adc_sampling_rates(i);
                simulator.Victim.compute_calculated_vals();
                
                %set attacker parameters
                simulator.Attacker.Subsystem_tracking.FrequencySlope_MHz_us = slopes(i);
                simulator.Attacker.Subsystem_tracking.ChirpCycleTime_us = chirp_periods(i);
                simulator.Attacker.Subsystem_tracking.ADC_Samples = num_adc_samples;
                simulator.Attacker.Subsystem_tracking.ADC_SampleRate_MSps = adc_sampling_rates(i);
                simulator.Attacker.Subsystem_tracking.compute_calculated_vals();
                
                %apply timing offsets as desired
                simulator.Victim.timing_offset_us = 0;
                simulator.Attacker.Subsystem_tracking.timing_offset_us = 0;
                
                %configure the FMCW parameters
                simulator.configure_FMCW_Radar_parameters();

                if simulator.Victim.IdleTime_us <= 0
                    configurations_passed = false;
                    failed_cases = failed_cases + 1;
                elseif abs(simulator.Victim.FMCW_sampling_rate_Hz * 1e-6 - FMCW_BW_MHz) >= 1e-3
                    configurations_passed = false;
                    failed_cases = failed_cases + 1;
                else
                    good_slopes = [good_slopes; slopes(i)];
                    good_chirp_periods = [good_chirp_periods; chirp_periods(i)];
                    good_adc_sampling_rates = [good_adc_sampling_rates; adc_sampling_rates(i)];
                end
            end
            
            if configurations_passed
                fprintf("All Configurations passed \n");
            else
                fprintf("%d configuration(s) failed \n",failed_cases);
            end
        end

        %{
            Purpose: save the chirp for a  given test case to a file for
            testing on the USRP radios
            Inputs:
                config_path - path to the .json configuration file
                save_file_path - path to save the chirp to
                test_case_number - the number of the current test case
                slope_MHz_us - the chirp slope to use for testing
                chirp_cycle_period_us - the chirp cycle period
                adc_sampling_rates - the adc sampling rate for each victim
                    test
                num_adc_samples - the number of ADC samples
                num_chirps - the number of chirps per frame
                frames_to_compute - number of frames to simulate before
                    recording a result
                attack_start_frame - the frame that the attack starts at
        %}
        function [actual_slope,...
            actual_chirp_duration, ...
            actual_frame_duration,...
            ADC_SampleRate_MSps] = ...
            USRP_save_test_case_chirp( ...
                config_path, ...
                save_file_path, ...
                test_case_number, ...
                slope_MHz_us, ...
                chirp_cycle_period_us, ...
                ADC_SampleRate_MSps, ...
                num_adc_samples, ...
                num_chirps, ...
                frames_to_compute)
            
            %initialize the simulator
            simulator = Simulator_revB();
            simulator.load_params_from_JSON(config_path);

            %initialize the victim parameters
            simulator.Victim.FrequencySlope_MHz_us = slope_MHz_us;
            simulator.Victim.ChirpCycleTime_us = chirp_cycle_period_us;
            simulator.Victim.ADC_Samples = num_adc_samples;
            simulator.Victim.ADC_SampleRate_MSps = ADC_SampleRate_MSps;
            simulator.Victim.NumChirps = num_chirps;
            simulator.Victim.compute_calculated_vals();
            
            %set attacker parameters
            simulator.Attacker.Subsystem_tracking.FrequencySlope_MHz_us = slope_MHz_us;
            simulator.Attacker.Subsystem_tracking.ChirpCycleTime_us = chirp_cycle_period_us;
            simulator.Attacker.Subsystem_tracking.ADC_Samples = num_adc_samples;
            simulator.Attacker.Subsystem_tracking.ADC_SampleRate_MSps = ADC_SampleRate_MSps;
            simulator.Attacker.Subsystem_tracking.NumChirps = num_chirps;
            simulator.Attacker.Subsystem_tracking.compute_calculated_vals();
            
            %apply timing offsets as desired
            simulator.Victim.timing_offset_us = 0;
            simulator.Attacker.Subsystem_tracking.timing_offset_us = 0;
            
            %configure the FMCW parameters
            simulator.configure_FMCW_Radar_parameters();
            
            %load default attacker, and victim positions and velocities
            simulator.load_usrp_attacker_and_victim_position_and_velocity();
            
            %load the target
            simulator.load_target_realistic(50,15);
                    
            %specify whether or not to record a move of the range-doppler plot
            record_movie = false;
            range_lims = [simulator.Victim.Radar_Signal_Processor.distance_detection_range(1) - 5,150];
            vel_lims = [-20,20];
            vel_exclusion_region = [-1,1];
            frame_period_ms = 500;
            enable_simplified_plots = false;
            simulator.Victim.Radar_Signal_Processor.configure_movie_capture(frames_to_compute, ...
                record_movie,range_lims,vel_lims,vel_exclusion_region, enable_simplified_plots);
            
            %pre-compute the victim's chirps
            simulator.Victim.precompute_radar_chirps();
        
            %initialize the attacker
            %initialize the attacker parameters
            simulator.Attacker.initialize_attacker(...
                    simulator.Victim.FMCW_sampling_rate_Hz * 1e-6,...
                    simulator.Victim.StartFrequency_GHz,...
                    simulator.Victim.Chirp_Tx_Bandwidth_MHz);
            
            %initialize the sensing subsystem's debugger
            simulator.Attacker.Subsystem_spectrum_sensing.initialize_debugger(0,simulator.Victim,1);
            %set to change 1 to zero to disable
            
            %specify the type of emulation ("target", 
                    % "velocity spoof - noisy", 
                    % "velocity spoof - similar velocity",
                    % "range spoof - similar slope")
            
            attack_type = "velocity spoof - similar velocity";
            desired_pos = 75;
            desired_vel = 7;
            %attack_type = "velocity spoof - similar velocity,range spoof - similar slope ";
            %initialize the attacker
            simulator.Attacker.Subsystem_attacking.set_attack_mode(attack_type);
            
            %if it is desired to specify a specific attack location
            if attack_type == "target"
                simulator.Attacker.Subsystem_attacking.set_desired_attack_location(desired_pos,desired_vel);
            end

            file_name = sprintf("MATLAB_chirp_full_%d.bin",test_case_number);
            simulator.save_to_file(simulator.Victim.chirp,save_file_path + file_name,"float");
        
            %frame duration
            actual_frame_duration = simulator.Victim.FramePeriodicity_ms;
            %chirp duration
            actual_chirp_duration = simulator.Victim.ChirpCycleTime_us;
            %slope
            actual_slope = simulator.Victim.FrequencySlope_MHz_us;

            %adc sample rate
            ADC_SampleRate_MSps = simulator.Victim.ADC_SampleRate_MSps;
        end

                %{
            Purpose: run the given sensing subsystem performance test cases
            and saves the results to a csv file
            Inputs:
                num_cases - number of test cases to run
                frames_to_compute - number of frames to simulate for each
                    run
                slopes_MHz_us - array of slopes to test
                chirp_cycle_periods_us - array of chirp cycle periods
                adc_sampling_rates - array of adc sampling rates
                num_adc_samples - the number of ADC samples
                num_chirps - the number of chirps per frame
                config_path - path to the json config
                save_file_name - path to save the results file to (without
                    the .csv extension
                chirp_save_file_path - path to the save location for each
                test case's chirp
            Outputs:
                testing_data_parameter_estimations - key parameter
                    estimations for each test case
                testing_data_frame_start_prediction_errors - the error in
                    the frame start time prediction for each frame in each test
                    case
        %}
        function [testing_data_parameter_estimations, ...
                testing_data_frame_start_prediction_errors] = ...
                USRP_save_test_case_chirps( ...
                    num_cases, ...
                    frames_to_compute, ...
                    slopes_MHz_us, ...
                    chirp_cycle_periods_us, ...
                    adc_sampling_rates, ...
                    num_adc_samples,...
                    num_chirps, ...
                    config_path, ...
                    save_file_name,...
                    chirp_save_file_path)

            save_file_headers = ["Actual Chirp Slope (Mhz/us)",...
                "Actual Chirp Cycle Period (us)",...
                "Actual Frame Duration (ms)",...
                "ADC Sample Rate (MSPS)",...
                "Estimated Chirp Slope (Mhz/us)",...
                "Estimated Chirp Cycle Period (us)",...
                "Estimated Frame Duration (ms)",...
                "Absolute Slope Error (MHz/us)",...
                "Absolute Chirp Cycle Period Error (us)",...
                "Absolute Frame Duration Error (ms)"];

            testing_data_parameter_estimations = zeros(num_cases,size(save_file_headers,2));
            testing_data_frame_start_prediction_errors = zeros(num_cases,frames_to_compute-2);
            
            status = sprintf("Saving Test Chirp: %d of %d",1, num_cases);
            progress_bar = waitbar(0,status,"Name","Initializing USRP Test Cases");
            for i = 1:num_cases
                %update the waitbar
                status = sprintf("Saving Test Chirp: %d of %d",i, num_cases);
                waitbar(i/num_cases,progress_bar,status);
            
                %run the test case
                [actual_slope,...
                actual_chirp_duration, ...
                actual_frame_duration,...
                ADC_SampleRate_MSps] = ...
                    characterization_functions.USRP_save_test_case_chirp(config_path, ...
                    chirp_save_file_path, ...
                    i, ...
                    slopes_MHz_us(i), ...
                    chirp_cycle_periods_us(i), ...
                    adc_sampling_rates(i), ...
                    num_adc_samples, ...
                    num_chirps, ...
                    frames_to_compute);
            
                %save values - resulting averages from simulation runs
                testing_data_parameter_estimations(i,1) = actual_slope;
                testing_data_parameter_estimations(i,2) = actual_chirp_duration;
                testing_data_parameter_estimations(i,3) = actual_frame_duration;
                testing_data_parameter_estimations(i,4) = ADC_SampleRate_MSps;
                testing_data_parameter_estimations(i,5) = 0;
                testing_data_parameter_estimations(i,6) = 0;
                testing_data_parameter_estimations(i,7) = 0;
                testing_data_parameter_estimations(i,8) = 0;
                testing_data_parameter_estimations(i,9) = 0;
                testing_data_parameter_estimations(i,10) = 0;

%                 %save values for frame_start_time prediction errors
%                 testing_data_frame_start_prediction_errors(i,:) = predicted_start_time_errors;
%                 test_data_results = array2table(testing_data_frame_start_prediction_errors);
%                 writetable(test_data_results,save_file_name + "_frame_start_time_prediction_errors.csv",...
%                     'WriteRowNames',true); 
                
                %save the results to a file - continuously saving the data
                %allows for obtaining some data even in the event of a error or
                %crash

                %save the testing data for results
                test_data_results = array2table(testing_data_parameter_estimations,"VariableNames",save_file_headers);
                writetable(test_data_results,save_file_name + "_parameter_estimations.csv",'WriteRowNames',true); 
            end
        end

        %{
            Purpose: compute the estimated chirp slope, chirp duration, and
            frame duration for a given sensing subsystem
            Inputs:
                slope_MHz_us - the chirp slope to use for testing
                chirp_cycle_period_us - the chirp cycle period
                adc_sampling_rates - the adc sampling rate for each victim
                    test
                frame_period_ms - the frame period for the given trial
                usrp_results_path - the path to the folder containing the
                    parameter estimations for a given test case
                test_case_number - the number of the current test case
                
            Outputs:
                actual_slope
                estimated_slope
                actual_chirp_duration
                estimated_chirp_duration
                actual_frame_duration
                estimated_chirp_duration
                ADC_SampleRate_MSps
        %}
        function [actual_slope,estimated_slope,...
            actual_chirp_duration,estimated_chirp_duration,...
            actual_frame_duration, estimated_frame_duration,...
            ADC_SampleRate_MSps] = ...
            USRP_obtain_sensed_values( ...
                slope_MHz_us, ...
                chirp_cycle_period_us, ...
                ADC_SampleRate_MSps,...
                frame_period_ms, ...
                usrp_results_path,...
                test_case_number)
            
            %get the return values
            file_name = sprintf("cpp_estimated_parameters_%d.bin",test_case_number);
            path = usrp_results_path + file_name;
            simulator = Simulator_revB();
            read_data = simulator.read_from_file(path,false,"double");
        
            %frame duration
            estimated_frame_duration = read_data(1);
            actual_frame_duration = frame_period_ms;
            %chirp duration
            estimated_chirp_duration = read_data(2);
            actual_chirp_duration = chirp_cycle_period_us;
            %slope
            estimated_slope = read_data(3);
            actual_slope = slope_MHz_us;

        end
        
        %{
            Purpose: load the given test case results from the USRP 
            trials and save the results to a csv file
            Inputs:
                num_cases - number of test cases to run
                slopes_MHz_us - array of slopes to test
                chirp_cycle_periods_us - array of chirp cycle periods
                adc_sampling_rates - array of adc sampling rates
                frame_period_ms - the frame period in ms
                save_file_name - path to save the results file to (without
                    the .csv extension
                usrp_results_path - path to the folder where the usrp
                    result files are stored (one file per set of parameter
                    estimations)
            Outputs:
                testing_data_parameter_estimations - key parameter
                    estimations for each test case
                testing_data_frame_start_prediction_errors - the error in
                    the frame start time prediction for each frame in each test
                    case (NOT ENABLED AT THIS TIME FOR USRP)
        %}
        function testing_data_parameter_estimations = ...
                USRP_get_test_case_results( ...
                    num_cases, ...
                    slopes_MHz_us, ...
                    chirp_cycle_periods_us, ...
                    adc_sampling_rates, ...
                    frame_period_ms, ...
                    save_file_name,...
                    usrp_results_path)

            save_file_headers = ["Actual Chirp Slope (Mhz/us)",...
                "Actual Chirp Cycle Period (us)",...
                "Actual Frame Duration (ms)",...
                "ADC Sample Rate (MSPS)",...
                "Estimated Chirp Slope (Mhz/us)",...
                "Estimated Chirp Cycle Period (us)",...
                "Estimated Frame Duration (ms)",...
                "Absolute Slope Error (MHz/us)",...
                "Absolute Chirp Cycle Period Error (us)",...
                "Absolute Frame Duration Error (ms)"];

            testing_data_parameter_estimations = zeros(num_cases,size(save_file_headers,2));
            
            status = sprintf("Reading Test: %d of %d",1, num_cases);
            progress_bar = waitbar(0,status,"Name","Testing Sensing Subsystem");
            for i = 1:num_cases
                %update the waitbar
                status = sprintf("Reading Test: %d of %d",i, num_cases);
                waitbar(i/num_cases,progress_bar,status);
            
                %obtain the results
            
                [actual_slope,estimated_slope,...
                actual_chirp_duration,estimated_chirp_duration,...
                actual_frame_duration, estimated_frame_duration,...
                ADC_SampleRate_MSps] = ...
                    characterization_functions.USRP_obtain_sensed_values(...
                    slopes_MHz_us(i), ...
                    chirp_cycle_periods_us(i), ...
                    adc_sampling_rates(i),...
                    frame_period_ms, ...
                    usrp_results_path,...
                    i);
                
            
                %save values - resulting averages from simulation runs
                testing_data_parameter_estimations(i,1) = actual_slope;
                testing_data_parameter_estimations(i,2) = actual_chirp_duration;
                testing_data_parameter_estimations(i,3) = actual_frame_duration;
                testing_data_parameter_estimations(i,4) = ADC_SampleRate_MSps;
                testing_data_parameter_estimations(i,5) = estimated_slope;
                testing_data_parameter_estimations(i,6) = estimated_chirp_duration;
                testing_data_parameter_estimations(i,7) = estimated_frame_duration;
                testing_data_parameter_estimations(i,8) = abs(actual_slope - estimated_slope);
                testing_data_parameter_estimations(i,9) = abs(actual_chirp_duration - estimated_chirp_duration);
                testing_data_parameter_estimations(i,10) = abs(actual_frame_duration - estimated_frame_duration);


                %save the testing data for results
                test_data_results = array2table(testing_data_parameter_estimations,"VariableNames",save_file_headers);
                writetable(test_data_results,save_file_name + "_parameter_estimations.csv",'WriteRowNames',true); 
            end
        end


        %% USRP SPOOFING PERFORMANCE TESTING - Characterization Functions

            %{
        Purpose: Generate a series of desired range and velocity test
            points that can be used to test the attacking subsystem's spoofing
            accuracy
        Inputs:
            num_cases: the number of test cases to generate
            valid_ranges: valid spoofing target test ranges [min,max]
            valid_velocities: valid spoofing target test velocities
                [min,max]
        %}
        function [ranges,velocities] = initialize_USRP_attack_subsystem_test_cases( ...
                    num_cases, ...
                    valid_ranges, ...
                    valid_velocities)


            %initialize the output arrays
            ranges = zeros(num_cases,1);
            velocities = zeros(num_cases,1);
    
            %using a for loop initialize all of the test cases
            for i = 1:num_cases
                
                % select parameters for the test case
                %select a random range
                ranges(i) = rand * (valid_ranges(2) - valid_ranges(1)) + valid_ranges(1);
    
                %select a random velocity
                velocities(i) = sign(randn) * rand * (valid_velocities(2) - valid_velocities(1)) + valid_velocities(1);
            end
        end

        %{
            Purpose: save USRP Attack subsystem evaluation configuration to
            a file
            Inputs:
                config_path: the path to the configuration that is to be
                used for attack subsystem evaluation
        %}
        function save_USRP_attack_subsystem_chirp_to_file( ...
                config_path)

            simulator = Simulator_revB();
            
            simulator.load_params_from_JSON(config_path);
            
            %apply timing offsets as desired
            simulator.Victim.timing_offset_us = 0;
            simulator.Attacker.Subsystem_tracking.timing_offset_us = 0;
            
            %configure the FMCW parameters
            simulator.configure_FMCW_Radar_parameters();
            
            %load default attacker, and victim positions and velocities
            simulator.load_usrp_attacker_and_victim_position_and_velocity();

            %pre-compute the victim's chirps
            simulator.Victim.precompute_radar_chirps();

            %save the full chirp as a binary file
            path = "/home/david/Documents/MATLAB_generated/MATLAB_full_chirps/MATLAB_chirp_full.bin";
            simulator.save_to_file(simulator.Victim.chirp,path,'float32');
        end

        function save_desired_spoofing_ranges_and_velocities_to_file( ...
                velocities,...
                velocity_file_path, ...
                ranges,...
                range_file_path)

            num_trials = size(ranges,1);

            for i = 1:num_trials
                %save range trial
                file_name = sprintf("MATLAB_desired_range_spoofs_%d.bin",i);
                fileID = fopen(range_file_path + file_name,'w');
                fwrite(fileID,[ranges(i)], "double");
                fclose(fileID);
                
                %save velocity file
                file_name = sprintf("MATLAB_desired_velocity_spoofs_%d.bin",i);
                fileID = fopen(velocity_file_path + file_name,'w');
                fwrite(fileID,[velocities(i)], "double");
                fclose(fileID);
            end

            

        end
        
        %{
            Purpose: Run through all test cases for evaluating the spoofing
                location accuracy and save the results to a file
            Inputs:
                config_path - path to the .json configuration file
                spoof_ranges - actual range of the object to be added
                spoof_velocities - actual velocity of the object ot be
                    spoofed
                frames_to_compute - number of frames to simulate before
                    recording a result
                attack_start_frame - the frame that the attack starts at
                num_cases - the number of test cases
                save_file_name - the name of the file (without the .csv) to
                    save the results to
            Outputs:
                test_data_spoofing_performance - the data from each of the
                    test cases
        %}
        function test_data_spoofing_performance = ...
                USRP_get_spoofing_test_results( ...
                    num_cases, ...
                    frames_to_compute, ...
                    attack_start_frame, ...
                    num_attack_frames, ...
                    spoof_ranges, ...
                    spoof_velocities, ...
                    config_path, ...
                    save_file_name)

            save_file_headers = [...
                "Spoof Ranges (m)",...
                "Spoof Velocities (m/s)",...
                "Estimated Ranges (m)",...
                "Estimated velocities (m/s)",...
                "Absolute Range Spoof Error (m)",...
                "Absolute Velocity Spoof Error (m)"];

            test_data_spoofing_performance = zeros(num_cases * num_attack_frames,size(save_file_headers,2));
            
            status = sprintf("Running Test: %d of %d",1, num_cases);
            progress_bar = waitbar(0,status,"Name","Testing Sensing Subsystem");
            for i = 1:num_cases
                %update the waitbar
                status = sprintf("Running Test: %d of %d",i, num_cases);
                waitbar(i/num_cases,progress_bar,status);
            
                %run the test case
            
                    [estimated_ranges,estimated_velocities,...
                    desired_ranges,desired_velocities] = ...
                            characterization_functions.USRP_compute_sensed_targets( ...
                            config_path,...
                            spoof_ranges(i),...
                            spoof_velocities(i),...
                            frames_to_compute,...
                            attack_start_frame, ...
                            num_attack_frames, ...
                            i);
            
                %save values - resulting averages from simulation runs
                start_idx = num_attack_frames * (i - 1) + 1;
                end_idx = num_attack_frames * i;
               
                test_data_spoofing_performance(start_idx:end_idx,1) = desired_ranges;
                test_data_spoofing_performance(start_idx:end_idx,2) = desired_velocities;
                test_data_spoofing_performance(start_idx:end_idx,3) = estimated_ranges;
                test_data_spoofing_performance(start_idx:end_idx,4) = estimated_velocities;
                test_data_spoofing_performance(start_idx:end_idx,5) = abs(desired_ranges - estimated_ranges);
                test_data_spoofing_performance(start_idx:end_idx,6) = abs(desired_velocities - estimated_velocities);
                
                %save the results to a file - continuously saving the data
                %allows for obtaining some data even in the event of a error or
                %crash

                %save the testing data for results
                test_data_results = array2table(test_data_spoofing_performance,"VariableNames",save_file_headers);
                writetable(test_data_results,save_file_name + ".csv",'WriteRowNames',true); 
            end
            
        end

        %{
            Purpose: compute the objects sensed by a victim given a
            spoofing attack
            Inputs:
                config_path - path to the .json configuration file
                spoof_range - actual range of the object to be added
                spoof_velocity - actual velocity of the object ot be
                    spoofed
                frames_to_compute - number of frames to simulate before
                    recording a result
                attack_start_frame - the frame that the attack starts at
            Outputs:
                estimated_ranges - the estimated ranges for each frame that
                    the victim was under attack
                estimated_velocities - the estimated velocities for each
                    frame that the victim was under attack
                desired_ranges - the desired spoof location
                desired_velocities - the desired spoof velocity
        %}
        function [estimated_ranges,estimated_velocities,...
                desired_ranges,desired_velocities] = ...
            USRP_compute_sensed_targets( ...
                config_path, ...
                spoof_range, ...
                spoof_velocity, ...
                num_frames, ...
                attack_start_frame, ...
                num_attack_frames, ...
                trial_number)
            
            simulator = Simulator_revB();
            
            simulator.load_params_from_JSON(config_path);
            
            %apply timing offsets as desired
            simulator.Victim.timing_offset_us = 0;
            simulator.Attacker.Subsystem_tracking.timing_offset_us = 0;
            
            %configure the FMCW parameters
            simulator.configure_FMCW_Radar_parameters();
            
            %load default attacker, and victim positions and velocities
            simulator.load_usrp_attacker_and_victim_position_and_velocity();
            
            %pre-compute the victim's chirps
            simulator.Victim.precompute_radar_chirps();
            
            % read the received data for the given trial
            path = sprintf("/home/david/Documents/MATLAB_generated/cpp_rx_data_files/cpp_rx_data_%d.bin",trial_number);
            read_data = simulator.read_from_file(path,true,"float");
            
            num_chirps = simulator.Victim.NumChirps;
            samples_per_chirp = simulator.Victim.ChirpCycleTime_us * 1e-6 * simulator.Victim.FMCW_sampling_rate_Hz;
            read_data = reshape(read_data,int32(samples_per_chirp),num_chirps,[]);
            
            %specify whether or not to record a move of the range-doppler plot
            record_movie = false;
            range_lims = [simulator.Victim.Radar_Signal_Processor.distance_detection_range(1) - 5,150];
            vel_lims = [-20,20];
            vel_exclusion_region = [-1,1];
            frame_period_ms = 500;
            enable_simplified_plots = false;
            simulator.Victim.Radar_Signal_Processor.configure_movie_capture(num_frames, ...
                record_movie,range_lims,vel_lims,vel_exclusion_region,enable_simplified_plots);
            
            for frame = int32(1:num_frames)
    
                %initialize a clean radar cube
                simulator.Victim.Radar_Signal_Processor.reset_radar_cube();
                simulator.Victim.current_frame = frame;
                
                for chirp = int32(1:num_chirps)
                
                    %get the Tx chirp and the chirp received from the emulator
                    Rx_sig = read_data(:,chirp,frame);
                    Tx_sig = simulator.Victim.chirps(:,chirp);      
                            
                    %assemble the radar cube
                    simulator.Victim.Radar_Signal_Processor.update_radar_cube(Tx_sig,Rx_sig,chirp);
                end
            
                simulator.Victim.Radar_Signal_Processor.process_radar_cube();
            end
            
            %identify the nearest detections
            
            %get the estimated ranges and velocities
            range_detections = simulator.Victim.Radar_Signal_Processor.range_estimates;
            velocity_detections = simulator.Victim.Radar_Signal_Processor.velocity_estimates;
            
            %identify the point in the detections that is closest to desired target
            %location
            
            
            point = [spoof_range, spoof_velocity];
            
            closest_points = characterization_functions.identify_nearest_detections( ...
                range_detections,velocity_detections, ...
                point);
            %get the detections corresponding to the attack only
            attack_detections = closest_points( ...
                attack_start_frame:attack_start_frame + num_attack_frames - 1, :);
            
            estimated_ranges = attack_detections(:,1);
            estimated_velocities = attack_detections(:,2);

            desired_ranges = spoof_range * ones(num_attack_frames,1);
            
            desired_velocities = spoof_velocity * ones(num_attack_frames,1);
        end
        
        %{
            Purpose: For a set of range detections and velocity detections,
            identify which detection is closest to the given point
            
            Inputs:
                Range_detections: an N x M matrix of range detections where
                    N is the number of frames and M is the number of
                    detections for each frame. NaN values are automitacally
                    filtered out.
                Velocity_detections: an N x M matrix of velocity detections
                point: the [Range, Velocity] of the actual target location
        %}
        function nearest_points = identify_nearest_detections(...
                                        range_detections, ...
                                        velocity_detections, ...
                                        point)
            num_frames = size(range_detections,1);
            closest_ranges = zeros(num_frames,1);
            closest_velocities = zeros(num_frames,1);
            
            for i = 1:num_frames
                ranges = range_detections(i,:)';
                velocities = velocity_detections(i,:)';
                
                %remove NaN values
                ranges = ranges(~isnan(ranges));
                velocities = velocities(~isnan(ranges));
                
                if ~isempty(ranges)
                    [idx, ~] = dsearchn([ranges , velocities], point);
                    closest_ranges(i) = range_detections(i,idx);
                    closest_velocities(i) = velocity_detections(i,idx);
                else
                    closest_ranges(i) = 0;
                    closest_velocities(i) = 0;
                end
            end
            
            nearest_points = [closest_ranges, closest_velocities];
        end

        %{
            Purpose: For a set of range detections and velocity detections,
            identify which detection is closest to the given point
            
            Inputs:
                Range_detections: an N x M matrix of range detections where
                    N is the number of frames and M is the number of
                    detections for each frame. NaN values are automitacally
                    filtered out.
                Velocity_detections: an N x M matrix of velocity detections
                max_range: the maximum range for the detection region in m
                max_vel: the maximum absolute velocity for the detection
                    region in m/s
                max_num_points: the maximum number of points to detect
        %}
        function [valid_ranges,valid_velocities] = identify_detections_in_region(...
                                        range_detections, ...
                                        velocity_detections, ...
                                        range_region, ...
                                        max_vel,...
                                        vel_exclusion_region, ...
                                        max_num_points)
            num_frames = size(range_detections,1);
            valid_ranges = zeros(num_frames,max_num_points);
            valid_velocities = zeros(num_frames,max_num_points);
            
            for i = 1:num_frames
                ranges = range_detections(i,:)';
                velocities = velocity_detections(i,:)';
                
                %remove NaN values
                ranges = ranges(~isnan(ranges));
                velocities = velocities(~isnan(ranges));
                
                if ~isempty(ranges)
                    %identify the valid points
                    valid_idxs = (ranges >= range_region(1) & ranges <= range_region(2))...
                            & (velocities <= vel_exclusion_region(1) | velocities >= vel_exclusion_region(2))...
                            & abs(velocities) <= max_vel;
                    
                    ranges = ranges(valid_idxs);
                    velocities = velocities(valid_idxs);

                    num_valid_points = min(max_num_points,sum(valid_idxs));

                    valid_ranges(i,1:num_valid_points) = ranges(1:num_valid_points);
                    valid_velocities(i,1:num_valid_points) = velocities(1:num_valid_points);
                end
            end
        end

        function print_attack_frame_detections( ...
                                            ranges, ...
                                            velocities, ...
                                            attack_start_frame, ...
                                            num_attack_frames)
            %get the detections corresponding to the attack frame
            attack_ranges = ranges( ...
                attack_start_frame:attack_start_frame + num_attack_frames - 1, :);
            attack_velocities = velocities( ...
                attack_start_frame:attack_start_frame + num_attack_frames - 1, :);

            %print the attack detections out
            %print attack ranges
            fprintf("Detected Ranges \n")
            fmt = ['%f\t' repmat('%f\t',1,size(attack_ranges,2)-1) '\n'];
            fprintf(fmt,attack_ranges.')
            
            %print attack velocities
            fprintf("Detected Velocities \n");
            fmt = ['%f\t' repmat('%f\t',1,size(attack_velocities,2)-1) '\n'];
            fprintf(fmt,attack_velocities.')
        end

        function plot_detetections(ranges, range_limits, max_frame, save_to_file)

            clf;
            set(gcf,'Position',[0 0 350 350])
            font_size = 17;
            hold on;
            for i = 1:size(ranges,2)
                scatter(1:size(ranges,1),ranges(:,i),"o",'filled',"b");
            end
            hold off;
            ylim(range_limits)
            xlim([1,max_frame]);
            ax = gca;
            title({"Detected Object" ; "Location"},"FontSize",font_size)
            xlabel("Frame","FontSize",font_size)
            ylabel("Detected Range","FontSize",font_size)
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            if save_to_file
                print('-r300',"generated_plots/detections",'-dsvg')
                print('-r300',"generated_plots/detections",'-dpng')
            end
        end

        function F_detections = generate_detection_movie(ranges, range_limits)
            
            num_frames = size(ranges,1);
            
            %declare an empty array to store the resulting frames
            F_detections(num_frames) = struct('cdata',[],'colormap',[]);

            %for each frame, plot the detections up to that point
            for i = 1:num_frames
                characterization_functions.plot_detetections(ranges(1:i,:),range_limits,num_frames, false);
                drawnow;
                F_detections(i) = getframe(gcf);
            end
            
        end

        function play_movie(F_movie,frame_rate)
            fig = figure;
            movie(fig,F_movie,1,frame_rate);
        end

        function save_movie_to_gif(F_movie,frame_rate,file_name)
            for i = 1:length(F_movie)
                [A,map] = rgb2ind(frame2im(F_movie(i)),256);
                if i == 1
                    imwrite(A,map,file_name,'gif','LoopCount',Inf,"DelayTime",1/frame_rate);
                else
                    imwrite(A,map,file_name,'gif','WriteMode','append','DelayTime',1/frame_rate);
                end
            end
        end
        
        function [track_positions,track_IDs] = perform_object_tracking( ...
                                    ranges, ...
                                    velocities, ...
                                    frame_period_ms, ...
                                    max_num_tracks, ...
                                    range_resolution, ...
                                    velocity_resolution)

            %obtain specific tracking parameters
            max_num_detections = size(ranges,2);

            confirmation_threshold = [1,3]; %track confirmed if 2 detections in last 4 updates
            deletion_threshold = [2,10]; %track is deleted if it isn't assigned to any detections in 3 of 5 last updates

            tracker = radarTracker( ...
                "MaxNumTracks",max_num_tracks, ...
                "MaxNumDetections",max_num_detections, ...
                "ConfirmationThreshold",confirmation_threshold, ...
                "DeletionThreshold",deletion_threshold);

            %specify detection measurement parameters
            MP2 = struct('RangeResolution',range_resolution, ...
                'VelocityResolution',velocity_resolution, ...
                "MeasurementNoise",diag([1e3,5,1]));

            %initialize an array to track the position for each unique
            %track
            track_positions = cell(1,0);
            track_IDs = [];

            %specify tracker initialized as false (initialized once first
            %detection occurs

            for i = 1:size(ranges,1)

                %get the detected ranges/velocities that are non  zero
                valid_idxs = ranges(i,:) > 0;

                %get the detected ranges from the frame
                detected_ranges = ranges(i,valid_idxs);
                detected_velocities = -1 * velocities(i,valid_idxs);

                %format the detections into an objectDetection object
                detection_time = frame_period_ms * 1e-3 * (i - 1);
                
                detections = cell(1,size(detected_ranges,2));

                for j = 1:size(detected_ranges,2)
                    %create a detection object for each detection
                    detection = objectDetection( ...
                        detection_time, ...
                        [detected_ranges(j);detected_velocities(j);0], ...
                        'MeasurementParameters',MP2);

                    detections{j} = detection;
                end

                if ~isempty(detections) || isLocked(tracker)
                    
                    [confirmedTracks,~,allTracks] = tracker(detections,detection_time);

                    if ~isempty(allTracks)
                        positions = getTrackPositions(allTracks,"constvel");
                        for k = 1:size(allTracks,2)
                            track_ID = allTracks(k).TrackID;
                            
                            %for new track IDs
                            if isempty(find(track_IDs==track_ID,1))
                                
                                %save the new track ID
                                track_IDs = [track_IDs,track_ID];
                                
                                %prepare for a new set of track positions
                                %to be saved in the cell array
                                track_positions{size(track_positions,2) + 1} = [];
                            end

                            %save the [frame,position] for the track
                            idx = find(track_IDs == track_ID,1);
                            track_positions{idx} = ...
                                [track_positions{idx};
                                i,positions(k)];
                        end
                        
                    end
                end
            end
            
        end

        function plot_tracks(track_positions,track_IDs, range_limits, num_frames)
            clf;
            set(gcf,'Position',[100 100 400 400])
            font_size = 15;
            hold on;
            for i = 1:size(track_positions,2)
                series_name = sprintf("Track %d",track_IDs(i));
                frames = track_positions{i}(:,1);
                positions = track_positions{i}(:,2);
                scatter(frames,positions,"DisplayName",series_name,"Marker","o","MarkerFaceColor","flat");
            end
            hold off;
            ylim(range_limits)
            xlim([1,num_frames]);
            ax = gca;
            ax.FontSize = font_size;
            title("Tracked Object Locations","FontSize",font_size)
            xlabel("Frame","FontSize",font_size)
            ylabel("Detected Range","FontSize",font_size)
%             legend('show')
            print('-r300',"generated_plots/tracks",'-dsvg')
            print('-r300',"generated_plots/tracks",'-dpng')
        end



        
    end
end