classdef Simulation_reconfigurable < handle
    properties
        frames_to_compute;
        simulator;
        sim_config;
    end
    methods (Access = public)
        function obj = Simulation_reconfigurable()
            
        end

        function initialize_simulation(obj, sim_config, target_start, target_velocity)
            obj.sim_config = sim_config;
            obj.simulator = Simulator_revB();
            user = sim_config.TestSettings.Configurations.user;
            json_config_file = sim_config.TestSettings.Configurations.json_config;
            obj.frames_to_compute = sim_config.TestSettings.Configurations.frames_per_sim;

            david_file_path = "/home/david/Documents/BlackBoxRadarAttacks/MATLAB/config_files/";
            kristen_file_path = "/home/kristenangell/Documents/RadarSecurityResearch/MATLAB/Simulink Model/config_files/";
            windows_file_path = "C:\Users\Operator\Documents\RadarSecurityResearch\MATLAB\Simulink Model\config_files\";

            % file_path = "B210_params.json";
            % file_path = "B210_params_highBW.json";
            % file_path = "B210_params_highvres.json";
            % file_path = "B210_params_lowBW.json";
            % file_path = "B210_params_sensing_system.json";
            % file_path = "X310_params_100MHzBW.json";
            % file_path = "realistic_params.json";


            if (user == "kristen")
                user_path = kristen_file_path;
            elseif(user == "windows")
                user_path = windows_file_path;
            else
                user_path = david_file_path;
            end

            obj.simulator.load_params_from_JSON(user_path + json_config_file);

            %apply timing offsets as desired
            obj.simulator.Victim.timing_offset_us = 0;
            obj.simulator.Attacker.Subsystem_tracking.timing_offset_us = 0;

            %configure the FMCW parameters
            obj.simulator.configure_FMCW_Radar_parameters();

            %load default attacker, and victim positions and velocities
            obj.simulator.load_realistic_attacker_and_victim_position_and_velocity();

            %print out key parameters
            %obj.simulator.Victim.print_chirp_parameters;
% 
%             obj.simulator.Victim.print_frame_parameters;
%             %log detection first, then log all the data about it
%             obj.simulator.Victim.print_performance_specs;
%             obj.simulator.Victim.print_FMCW_specs;

            obj.simulator.load_target_realistic(target_start, target_velocity);

            %specify whether or not to record a move of the range-doppler plot
            record_movie = false;
            range_lims = [0,150];
            vel_lims = [-50,50];
            vel_exclusion_region = [-0.1,0.1];
            frame_period_ms = obj.simulator.Victim.FramePeriodicity_ms;
            enable_simplified_plots = false;
            obj.simulator.Victim.Radar_Signal_Processor.configure_movie_capture(obj.frames_to_compute, ...
                record_movie,range_lims,vel_lims,vel_exclusion_region,enable_simplified_plots);

            %pre-compute the victim's chirps
            obj.simulator.Victim.precompute_radar_chirps();
        end

        function run_simulation(obj)

            %run the simulation (without an attacker for now)
            if(obj.sim_config.TestSettings.Configurations.attack_enable)
                %initialize the attacker parameters
                obj.simulator.Attacker.initialize_attacker(...
                    obj.simulator.Victim.FMCW_sampling_rate_Hz * 1e-6,...
                    obj.simulator.Victim.StartFrequency_GHz,...
                    obj.simulator.Victim.Chirp_Tx_Bandwidth_MHz);

                %initialize the sensing subsystem's debugger
                obj.simulator.Attacker.Subsystem_spectrum_sensing.initialize_debugger(1,obj.simulator.Victim,obj.frames_to_compute);

                %specify the type of emulation ("target",
                % "velocity spoof - noisy",
                % "velocity spoof - similar velocity",
                % "range spoof - similar slope")

                attack_type = obj.sim_config.TestSettings.Configurations.attack_type;
                
                %initialize the attacker
                obj.simulator.Attacker.Subsystem_attacking.set_attack_mode(attack_type);

                %if it is desired to specify a specific attack location
                if obj.sim_config.TestSettings.Configurations.set_attack_location
%                     attack_position = obj.sim_config.TestSettings.Configurations.attack_pos;
%                     attack_velocity = obj.sim_config.TestSettings.Configurations.attack_velocity;
% 
%                     obj.simulator.Attacker.Subsystem_attacking.set_desired_attack_location(attack_position,attack_velocity);

                      valid_ranges = [50,100];
                    valid_velocities = [-25,25];
                    attack_position = rand * (valid_ranges(2) - valid_ranges(1)) + valid_ranges(1);

                    %select a random velocity
                    attack_velocity = rand * (valid_velocities(2) - valid_velocities(1)) + valid_velocities(1);
                    obj.simulator.Attacker.Subsystem_attacking.set_desired_attack_location(attack_position,attack_velocity);
                
                end
                


                %run the simulation (without an attacker for now)
                obj.simulator.run_simulation_with_attack(obj.frames_to_compute, false);
            else
                obj.simulator.run_simulation_no_attack(obj.frames_to_compute, false);
            end

        end

        function [detected, actual_ranges, estimated_ranges, estimated_velocities, actual_velocities, percent_error_ranges, percent_error_velocities, false_positives] = simulation_results(obj)
            % report the range estimates
            estimated_ranges = obj.simulator.Victim.Radar_Signal_Processor.range_estimates
            % compute the actual range per frame
            actual_ranges = performance_functions.actual_ranges(obj.frames_to_compute, obj.simulator.Victim.FramePeriodicity_ms*.001, obj.simulator.SimulatedTarget.velocity_meters_per_s, obj.simulator.Victim.velocity_m_per_s, obj.simulator.SimulatedTarget.position_m, obj.simulator.Victim.position_m)
            % report the velocity estimates
            estimated_velocities = obj.simulator.Victim.Radar_Signal_Processor.velocity_estimates
            % compute the actual velocity per frame
            actual_velocities = performance_functions.actual_velocities(obj.frames_to_compute, obj.simulator.SimulatedTarget.velocity_meters_per_s, obj.simulator.Victim.velocity_m_per_s)
            
           
            % determine if an object has been detected in each frame, and determine
            % false positives
            [detected, col_detection, false_positives] = performance_functions.detection(obj.frames_to_compute, estimated_ranges, actual_ranges, estimated_velocities, actual_velocities, obj.sim_config.TestSettings.Configurations.k_range,obj.sim_config.TestSettings.Configurations.k_vel)

            % compute the velocity and range error per frame

            percent_error_ranges = performance_functions.range_error(actual_ranges, estimated_ranges, col_detection)
            percent_error_velocities = performance_functions.velocity_error(estimated_velocities, actual_velocities, col_detection)

           
        end
    end
end
