%% For use in Simulink Model

%% Notes
%{
    - If the start frequency is higher than 77 GHz, there is a little bit
    of extra work that has to be done. When creating this simulation, I
    removed this functionality to simplify things. I did implement the
    functionality in previous versions though which can be found in the
    archived functions folder. See the configure_FMCW_parameters function
    for this information

    - There are a lot of other default radar configuration scenarios that I
    have computed, but in this simulation I'm only using the "realistic"
    settings for now. The other configurations can be found in the archived
    Simulator code

    - Chirp construction is slightly different. See the notes in the Radar
    class for more details

    - Removed all of the plotting functionality in this revision as it
    won't be used in these simulations
%}

classdef Simulator_revB < handle
    %SIMULATOR class used to support simulation in the simulink model

    properties (Access = public)
        Attacker                % an Attacker() class object
        Victim                  % a Radar() class object
        SimulatedTarget         % a Target() class object 

        %variables used in computing the FMCW simulation - MOVE TO PRIVATE
        channel_target          %phased.FreeSpace object for a target

        %phased.FreeSpace objects for the attacker's spectrum sensing and
        %attacking subsystems
        channel_attacker

        %phased.FreeSpace object for the attacker's tracking subsystem
        channel_attacker_tracking

        %the following properties are used as support parameters when
        %simulating the interaction between various parts of the FMCW
        %simulation
        sensing_subsystem_support

        %variable for configuration files
        radar_config
    end

    %the following properties are used as support parameters when
    %simulating the interaction between various parts of the FMCW
    %simulation
    properties (Access = private)
        
    end
    
    methods (Access = public)
        function obj = Simulator_revB()
            %Simulator Construct an instance of this class
            %   Detailed explanation goes here
            obj.Attacker = Attacker_revB();
            obj.Victim = Radar_revB();
        end

%% [1] Functions to configure the simulation
        
        function configure_FMCW_Radar_parameters(obj)
            %{
                Purpose: initializes all of the needed waveform parameters
                    to simulate the actual waveforms for the attacker and
                    victim
                Note: The attacker and Victim radar components will have
                the same setting. In practice this might not be the case,
                but for the sake of simplifying the simulation, they will
                have the same settings
            %}

            %added in to support setting up USRP's at a specific frequency
            %and in the case where the downsample factor should be one
            %lower (ex: TX_Bandwidth/ADC_Sample_Rate = 2.000001), but ceil
            %operation unnecessarially rounds up.
            if (abs((obj.Victim.Chirp_Tx_Bandwidth_MHz * 1e6) / (obj.Victim.ADC_SampleRate_MSps * 1e6)...
                    - round((obj.Victim.Chirp_Tx_Bandwidth_MHz * 1e6) / (obj.Victim.ADC_SampleRate_MSps * 1e6)))...
                    <= 1e-3)
                
                obj.Victim.downsample_factor = round((obj.Victim.Chirp_Tx_Bandwidth_MHz * 1e6) / (obj.Victim.ADC_SampleRate_MSps * 1e6));
            else
                obj.Victim.downsample_factor = ceil((obj.Victim.Chirp_Tx_Bandwidth_MHz * 1e6) / (obj.Victim.ADC_SampleRate_MSps * 1e6));
            end

            obj.Victim.Radar_Signal_Processor.decimation_factor = obj.Victim.downsample_factor;
            obj.Attacker.Subsystem_tracking.downsample_factor = obj.Victim.downsample_factor;
            obj.Attacker.Subsystem_tracking.Radar_Signal_Processor.decimation_factor = obj.Victim.downsample_factor;
            
            %removed the doubling of the chirp Tx Bandwidth since we are
            %assuming complex sampling

            obj.Victim.FMCW_sampling_rate_Hz = obj.Victim.ADC_SampleRate_MSps * 1e6 * obj.Victim.downsample_factor;
            obj.Attacker.Subsystem_tracking.FMCW_sampling_rate_Hz = obj.Victim.FMCW_sampling_rate_Hz;

            %set the FMCW sampling period as it is good to have for
            %reference
            obj.Victim.FMCW_sampling_period_s = 1/obj.Victim.FMCW_sampling_rate_Hz;
            obj.Attacker.Subsystem_tracking.FMCW_sampling_period_s = obj.Victim.FMCW_sampling_period_s;
            

            %set sweep time
            %select a sweep time that ensures that there is an integer
            %number of samples in the FMCW processing
            sweep_time = obj.Victim.RampEndTime_us * 1e-6;
            sweep_time = round(sweep_time * obj.Victim.FMCW_sampling_rate_Hz)/...
                obj.Victim.FMCW_sampling_rate_Hz;
            obj.Victim.sweep_time = sweep_time;
            obj.Attacker.Subsystem_tracking.sweep_time = sweep_time;
            
            
            %configure the FMCW waveforms for the victim and attacker
            %configure waveforms
            obj.Victim.configure_waveform_and_chirp();
            obj.Attacker.Subsystem_tracking.configure_waveform_and_chirp();
            
            
            %configure the free-space channels
            obj.channel_target = phased.FreeSpace( ...
                "PropagationSpeed",physconst('LightSpeed'), ...
                "OperatingFrequency", obj.Victim.StartFrequency_GHz * 1e9, ...
                "SampleRate", obj.Victim.FMCW_sampling_rate_Hz, ...
                "TwoWayPropagation", true);

            obj.channel_attacker = phased.FreeSpace( ...
                "PropagationSpeed",physconst('LightSpeed'), ...
                "OperatingFrequency", obj.Attacker.Subsystem_tracking.StartFrequency_GHz * 1e9, ...
                "SampleRate", obj.Attacker.Subsystem_tracking.FMCW_sampling_rate_Hz, ...
                "TwoWayPropagation", false);

            obj.channel_attacker_tracking = phased.FreeSpace( ...
                "PropagationSpeed",physconst('LightSpeed'), ...
                "OperatingFrequency", obj.Attacker.Subsystem_tracking.StartFrequency_GHz * 1e9, ...
                "SampleRate", obj.Attacker.Subsystem_tracking.FMCW_sampling_rate_Hz, ...
                "TwoWayPropagation", true);
            
            
            %configure transmitters, receivers, lowpass filters, and CFAR
            %detectors

            obj.Attacker.Subsystem_tracking.configure_transmitter_and_receiver();
            obj.Attacker.configure_transmitter_and_receiver(); %tx and rx for spectrum sensing and attacking modules
            obj.Attacker.Subsystem_tracking.configure_radar_signal_processor();

            obj.Victim.configure_transmitter_and_receiver();
            obj.Victim.configure_radar_signal_processor();
            %
        end

        function load_params_from_JSON(obj,file_path)

            %get information from the JSON configuration file
            json_text = fileread(file_path);
            obj.radar_config = jsondecode(json_text);

            %initialize victim
            %setup the victim's chirp parameters
            obj.Victim.StartFrequency_GHz         = obj.radar_config.RadarSettings.Victim.StartFrequency_GHz;
            obj.Victim.FrequencySlope_MHz_us      = obj.radar_config.RadarSettings.Victim.FrequencySlope_MHz_us;
            obj.Victim.TxStartTime_us             = obj.radar_config.RadarSettings.Victim.TxStartTime_us;
            obj.Victim.ADC_Samples                = obj.radar_config.RadarSettings.Victim.ADC_Samples;
            obj.Victim.ADC_SampleRate_MSps        = obj.radar_config.RadarSettings.Victim.ADC_SampleRate_MSps;
            obj.Victim.ChirpCycleTime_us          = obj.radar_config.RadarSettings.Victim.ChirpCycleTime_us;             

            %setup the victim's frame parameters
            obj.Victim.NumChirps                  = obj.radar_config.RadarSettings.Victim.NumChirps;
            obj.Victim.FramePeriodicity_ms        = obj.radar_config.RadarSettings.Victim.FramePeriodicity_ms;
            
            %define plot color default values
            obj.Victim.plotResolution_us = obj.radar_config.RadarSettings.Victim.plotResolution_us;
            obj.Victim.tx_period_plot_color = obj.radar_config.RadarSettings.Victim.tx_period_plot_color;
            obj.Victim.tx_sampling_period_plot_color = obj.radar_config.RadarSettings.Victim.tx_sampling_period_plot_color;
            obj.Victim.radar_name = obj.radar_config.RadarSettings.Victim.radar_name;

            %set timing offset to zero as this is the victim
            obj.Victim.timing_offset_us = obj.radar_config.RadarSettings.Victim.timing_offset_us;

            %compute all remaining "calculated" values
            obj.Victim.compute_calculated_vals();


            %load the attacker values into the JSON
            %setup the attacker's chirp parameters
            obj.Attacker.Subsystem_tracking.StartFrequency_GHz         = obj.radar_config.RadarSettings.Attacker.StartFrequency_GHz;
            obj.Attacker.Subsystem_tracking.FrequencySlope_MHz_us      = obj.radar_config.RadarSettings.Attacker.FrequencySlope_MHz_us;
            obj.Attacker.Subsystem_tracking.TxStartTime_us             = obj.radar_config.RadarSettings.Attacker.TxStartTime_us;
            obj.Attacker.Subsystem_tracking.ADC_Samples                = obj.radar_config.RadarSettings.Attacker.ADC_Samples;
            obj.Attacker.Subsystem_tracking.ADC_SampleRate_MSps        = obj.radar_config.RadarSettings.Attacker.ADC_SampleRate_MSps;
            obj.Attacker.Subsystem_tracking.ChirpCycleTime_us          = obj.radar_config.RadarSettings.Attacker.ChirpCycleTime_us;
            
            %setup the victim's frame parameters
            obj.Attacker.Subsystem_tracking.NumChirps                  = obj.radar_config.RadarSettings.Attacker.NumChirps;
            obj.Attacker.Subsystem_tracking.FramePeriodicity_ms        = obj.radar_config.RadarSettings.Attacker.FramePeriodicity_ms;
            
            %define plot color default values
            obj.Attacker.Subsystem_tracking.plotResolution_us = obj.radar_config.RadarSettings.Attacker.plotResolution_us;
            obj.Attacker.Subsystem_tracking.tx_period_plot_color = obj.radar_config.RadarSettings.Attacker.tx_period_plot_color;
            obj.Attacker.Subsystem_tracking.tx_sampling_period_plot_color = obj.radar_config.RadarSettings.Attacker.tx_sampling_period_plot_color;
            obj.Attacker.Subsystem_tracking.radar_name = obj.radar_config.RadarSettings.Attacker.radar_name;

            %define set the default offset to be 0us
            obj.Attacker.Subsystem_tracking.timing_offset_us = obj.radar_config.RadarSettings.Attacker.timing_offset_us;

            %compute all remaining "calculated" values
            obj.Attacker.Subsystem_tracking.compute_calculated_vals();
        end


        function load_target_realistic(obj, starting_m, starting_v)
            %{
                Purpose: load a realistic target
            %}

            %configure the simulated target
          
            position_m = [starting_m;0;0];
            velocity_meters_per_s = [starting_v;0;0];
            %try to modify to see if we can decrease power of object
            rcs_sq_meters = max((5 * randn) + 15,0.5);
            operating_frequency_Hz = obj.Victim.StartFrequency_GHz * 1e9;

            obj.SimulatedTarget = Target_revA(position_m,velocity_meters_per_s,rcs_sq_meters,operating_frequency_Hz);
        end
        
        function load_realistic_attacker_and_victim_position_and_velocity(obj)
            %{
                Purpose: configures a default scenario for the attacker and
                victim positions and velocities
            %}
            obj.Victim.position_m = [0;0;0];
            obj.Victim.velocity_m_per_s = [2;0;0];
            obj.Victim.platform = phased.Platform( ...
                'InitialPosition',obj.Victim.position_m, ...
                'Velocity',obj.Victim.velocity_m_per_s);

            obj.Attacker.position_m = [75;0;0];
            obj.Attacker.velocity_m_per_s = [0;0;0];
            obj.Attacker.configure_platform();
        end

        function load_realistic_attacker_and_victim_position_and_velocity_random(obj)
            %{
                Purpose: configures a default scenario for the attacker and
                vicitm positions and velocities, but position and
                velocities are chosen randomly
            %}
            obj.Victim.position_m = [0;0;0];
            obj.Victim.velocity_m_per_s = [0;0;0];
            obj.Victim.platform = phased.Platform( ...
                'InitialPosition',obj.Victim.position_m, ...
                'Velocity',obj.Victim.velocity_m_per_s);

            attack_pos = randi([20,100]);
            attack_vel = randi([-10,10]);
            obj.Attacker.position_m = [attack_pos;0;0];
            obj.Attacker.velocity_m_per_s = [attack_vel;0;0];
            obj.Attacker.configure_platform();
        end
        
        function load_usrp_attacker_and_victim_position_and_velocity(obj)
            %{
                Purpose: configures a default scenario for the attacker and
                victim positions and velocities
            %}
            obj.Victim.position_m = [0;0;0];
            obj.Victim.velocity_m_per_s = [0;0;0];
            obj.Victim.platform = phased.Platform( ...
                'InitialPosition',obj.Victim.position_m, ...
                'Velocity',obj.Victim.velocity_m_per_s);

            obj.Attacker.position_m = [0.25;0;0];
            obj.Attacker.velocity_m_per_s = [0;0;0];
            obj.Attacker.configure_platform();
        end
    

%% [2] Functions for running the FMCW Simulation on Matlab

        function [victim_pos, victim_vel,attacker_pos, attacker_vel, tgt_pos,tgt_vel] = FMCW_determine_positions_and_velocities(obj,victim_frame,victim_samples_sent)
            %{
                Purpose: determine the position and velocity for the
                    attacker,defender, and target at the end of a given chirp
                    in a given frame
                Inputs:
                    victim_samples_sent: the number of samples that the victim
                        has sent in the current frame
                    victim_frame: the desired frame
                Outputs:
                    [victim_pos, victim_vel]: the position and velocity of
                        the victim at the start of the desired chirp
                    [attacker_pos, attacker_vel]: the position and velocity of
                        the attacker at the start of the desired chirp
                    [target_pos, target_vel]: the position and velocity of
                        the attacker at the start of the desired chirp
            %}
            %calculate the time that the desired chirp will end at
            frame_start_time_s = double((obj.Victim.FramePeriodicity_ms * 1e-3) * (victim_frame - 1));
            current_time = frame_start_time_s + double(victim_samples_sent) / (obj.Victim.FMCW_sampling_rate_Hz);
            
            %reset the positions of each platform to be safe
            reset(obj.Victim.platform);
            reset(obj.Attacker.platform);
            reset(obj.SimulatedTarget.platform);

            if current_time <= 0
                time_increment = obj.Victim.ChirpCycleTime_us * 1e-6;
                [victim_pos, victim_vel] = obj.Victim.platform(time_increment);
                [tgt_pos,tgt_vel] = obj.SimulatedTarget.platform(time_increment);
                [attacker_pos, attacker_vel] = obj.Attacker.platform(time_increment);
            else
                %take a step for each platform so that they update to the
                %values at the start of the frame
                obj.Victim.platform(current_time);
                obj.SimulatedTarget.platform(current_time);
                obj.Attacker.platform(current_time);
    
                %repeat the process to obtain the positions and velocities at
                %the start of the given chirp
                [victim_pos, victim_vel] = obj.Victim.platform(current_time);
                [tgt_pos,tgt_vel] = obj.SimulatedTarget.platform(current_time);
                [attacker_pos, attacker_vel] = obj.Attacker.platform(current_time);
            end
        end     
        
        function received_signal = generate_sensing_subsystem_received_signal(obj)
            %{
                Purpose: the following code is used to determine the signal
                that the sensing subsystem would receive from the victim.
                Outputs:
                    received_signal: the signal received from the victim
            %}
            received_signal = zeros(obj.Attacker.Subsystem_spectrum_sensing.spectogram_params.num_ADC_samples_per_spectogram,1);
            received_signal_assembled = 0;
        
            while ~received_signal_assembled
                %if all of the chirps for a frame have been sent, just send
                %zeros until the end of the current frame
                if obj.sensing_subsystem_support.chirp_count >= obj.Victim.NumChirps
                    
                    if obj.sensing_subsystem_support.received_samples_left ...
                            <= obj.sensing_subsystem_support.frame_samples_left   %done sending chirps, but not end of the frame
                        points_to_insert = obj.sensing_subsystem_support.received_samples_left;
                        
        
                        %the following code optimizes the while loop script so that
                        %no computations are made when the "victim" is idle and not
                        %transmitting. To run the full thing, just comment out the
                        %if statement and only leave what is below the if
                        %statement.
                        if obj.sensing_subsystem_support.received_samples_left ...
                                ~= obj.sensing_subsystem_support.max_received_sample_index
                            received_signal(obj.sensing_subsystem_support.received_sample_index:...
                                obj.sensing_subsystem_support.received_sample_index + points_to_insert - 1)...
                                = zeros(points_to_insert,1);
                            received_signal_assembled = 1;
                        else
                            obj.Attacker.Subsystem_spectrum_sensing.sampling_window_count =...
                                obj.Attacker.Subsystem_spectrum_sensing.sampling_window_count + 1;
                            obj.Attacker.Subsystem_spectrum_sensing.previous_spectogram_points = [];
                        end
                        
                        obj.sensing_subsystem_support.received_sample_index = 1;
                        obj.sensing_subsystem_support.received_samples_left = obj.sensing_subsystem_support.max_received_sample_index;
        
                        obj.sensing_subsystem_support.frame_samples_left = ...
                            obj.sensing_subsystem_support.frame_samples_left -...
                            points_to_insert;
                        
                        if obj.sensing_subsystem_support.frame_samples_left == 0
                            obj.sensing_subsystem_support.frame_samples_left = ...
                                obj.sensing_subsystem_support.max_frame_sample_index;
                            obj.sensing_subsystem_support.frame_count = ...
                                obj.sensing_subsystem_support.frame_count + 1;
                            obj.sensing_subsystem_support.chirp_count = 0;
                        end
        
                    else                                                %represents the end of the specified frame
                        points_to_insert = ...
                            obj.sensing_subsystem_support.frame_samples_left;
                        
                        received_signal(obj.sensing_subsystem_support.received_sample_index:...
                            obj.sensing_subsystem_support.received_sample_index + points_to_insert - 1) = zeros(points_to_insert,1);
                        
                        
                        %increment the frame counter and start sending chirps again
                        %as new frame has started
                        obj.sensing_subsystem_support.frame_samples_left = ...
                            obj.sensing_subsystem_support.max_frame_sample_index;
                        obj.sensing_subsystem_support.frame_count = ...
                            obj.sensing_subsystem_support.frame_count + 1;
        
                        obj.sensing_subsystem_support.chirp_count = 0;
        
                        obj.sensing_subsystem_support.received_sample_index = ...
                            obj.sensing_subsystem_support.received_sample_index + points_to_insert;
                        obj.sensing_subsystem_support.received_samples_left ...
                            = obj.sensing_subsystem_support.max_received_sample_index -...
                            obj.sensing_subsystem_support.received_sample_index + 1;
                    end
                    
                %the next two statements deal with assembling chirps to be sent out

                %if there are more received samples to be added than chirp
                %samples to be added
                elseif obj.sensing_subsystem_support.received_samples_left >= ...
                        obj.sensing_subsystem_support.chirp_samples_left
                    points_to_insert = obj.sensing_subsystem_support.chirp_samples_left;
                    received_signal(obj.sensing_subsystem_support.received_sample_index:...
                        obj.sensing_subsystem_support.received_sample_index + points_to_insert - 1) =...
                        obj.Victim.chirp(obj.sensing_subsystem_support.chirp_sample_index:...
                        obj.sensing_subsystem_support.chirp_sample_index + points_to_insert - 1);
                    
                    obj.sensing_subsystem_support.chirp_sample_index = 1;
                    obj.sensing_subsystem_support.chirp_samples_left = ...
                        obj.sensing_subsystem_support.max_chirp_sample_index;
                    obj.sensing_subsystem_support.chirp_count = ...
                        obj.sensing_subsystem_support.chirp_count + 1;
                
                    obj.sensing_subsystem_support.received_sample_index = ...
                        obj.sensing_subsystem_support.received_sample_index + points_to_insert;
                    obj.sensing_subsystem_support.received_samples_left = ...
                        obj.sensing_subsystem_support.max_received_sample_index - ...
                        obj.sensing_subsystem_support.received_sample_index + 1;
                    
                    obj.sensing_subsystem_support.frame_samples_left = ...
                        obj.sensing_subsystem_support.frame_samples_left - points_to_insert;
        
        
                    if obj.sensing_subsystem_support.received_samples_left <= 0
                        obj.sensing_subsystem_support.received_sample_index = 1;
                        obj.sensing_subsystem_support.received_samples_left = ...
                            obj.sensing_subsystem_support.max_received_sample_index;
                        received_signal_assembled = 1;
                    end
                    
                %otherwise, there must be more chirp samples to insert for
                %the current chirp than there are remaining available
                %received samples
                else
                    points_to_insert = obj.sensing_subsystem_support.received_samples_left;
                    received_signal(obj.sensing_subsystem_support.received_sample_index:...
                        obj.sensing_subsystem_support.received_sample_index + points_to_insert - 1) =...
                        obj.Victim.chirp(obj.sensing_subsystem_support.chirp_sample_index:...
                        obj.sensing_subsystem_support.chirp_sample_index + points_to_insert - 1);
                
                    obj.sensing_subsystem_support.chirp_sample_index = ...
                        obj.sensing_subsystem_support.chirp_sample_index + points_to_insert;
                    obj.sensing_subsystem_support.chirp_samples_left = ...
                        obj.sensing_subsystem_support.max_chirp_sample_index - ...
                        obj.sensing_subsystem_support.chirp_sample_index + 1;
                
                    obj.sensing_subsystem_support.received_sample_index = 1;
                    obj.sensing_subsystem_support.received_samples_left = ...
                        obj.sensing_subsystem_support.max_received_sample_index;
                    received_signal_assembled = 1;
        
                    obj.sensing_subsystem_support.frame_samples_left =...
                        obj.sensing_subsystem_support.frame_samples_left - points_to_insert;
                end
            end
        end

        function run_simulation_no_attack(obj,frames_to_compute,wait_bar_enable)
            %{
                Purpose: runs the simulation (with no attacker) for the
                    given number of frames
                Inputs:
                    frames_to_compute: the number of frames to simulate
            %}
                
            noise_power_dBm = -174 + 10 * log10(obj.Victim.Chirp_Tx_Bandwidth_MHz * 1e6);
            
            if wait_bar_enable
                status = sprintf("Current frame: %d of %d",obj.Victim.current_frame, frames_to_compute);
                progress_bar = waitbar(0,status,"Name","Running Simulation");
            end
            
            sim_complete = false;
            while ~sim_complete
                
                %update the progress_bar
                if wait_bar_enable
                    status = sprintf("Current frame: %d of %d",obj.Victim.current_frame, frames_to_compute);
                    waitbar(obj.Victim.current_frame/frames_to_compute,progress_bar,status);
                end

                %get the transmitted signal from the radar
                sig = obj.Victim.get_radar_tx_signal();
                
                %compute the noise signal
                noise_sig = wgn(size(sig,1),1,noise_power_dBm - 30,"complex");
                
                %update positions
                [victim_pos, victim_vel,attacker_pos, attacker_vel, tgt_pos,tgt_vel] = ...
                    obj.FMCW_determine_positions_and_velocities(...
                    obj.Victim.current_frame,obj.Victim.num_samples_sent);
                
                %propagate the signal and reflect it off of the target
                sig_target = obj.channel_target(sig,victim_pos,tgt_pos,victim_vel,tgt_vel);
                sig_target = obj.SimulatedTarget.radar_target(sig_target);
                
                
                %have the radar receive the signal

                obj.Victim.receive_signal(sig_target + noise_sig);

                if obj.Victim.current_frame == frames_to_compute
                    if obj.Victim.current_chirp > obj.Victim.NumChirps
                        sim_complete = true;
                    end
                end
            end
        end
    
        function run_simulation_with_attack(obj,frames_to_compute,wait_bar_enable)
            %{
                Purpose: runs the simulation (with an attacker) for the
                    given number of frames
                Inputs:
                    frames_to_compute: the number of frames to simulate
            %}
            
            noise_power_dBm = -174 + 10 * log10(obj.Victim.Chirp_Tx_Bandwidth_MHz * 1e6);
            
            if wait_bar_enable
                status = sprintf("Current frame: %d or %d",obj.Victim.current_frame, frames_to_compute);
                progress_bar = waitbar(0,status,"Name","Running Simulation");
            end
            
            sim_complete = false;
            while ~sim_complete
                
                %update the progress_bar
                if wait_bar_enable
                    status = sprintf("Current frame: %d or %d",obj.Victim.current_frame, frames_to_compute);
                    waitbar(obj.Victim.current_frame/frames_to_compute,progress_bar,status);
                end

                %get the transmitted signal from the radar
                sig = obj.Victim.get_radar_tx_signal();
                
                %compute the noise signal
                noise_sig = wgn(size(sig,1),1,noise_power_dBm - 30,"complex");
                
                %update positions
                [victim_pos, victim_vel,attacker_pos, attacker_vel, tgt_pos,tgt_vel] = ...
                    obj.FMCW_determine_positions_and_velocities(...
                    obj.Victim.current_frame,obj.Victim.num_samples_sent);
                
                %propagate the signal and reflect it off of the target
                sig_target = obj.channel_target(sig,victim_pos,tgt_pos,victim_vel,tgt_vel);
                sig_target = obj.SimulatedTarget.radar_target(sig_target);
                
                %simulate attacker behavior
            
                    %attacker determines relative position and velocity of the victim
                    obj.Attacker.update_victim_pos_and_velocity(attacker_pos,victim_pos,attacker_vel, victim_vel);
                    obj.Attacker.update_target_pos_and_velocity(victim_pos, tgt_pos, victim_vel,tgt_vel)
                    
                    %assemble the noisy signal and the attacker signal
                    sig_attacker_sensing = noise_sig + sig;
                
                    %propogate the signal
                    sig_attacker_sensing = obj.channel_attacker(sig_attacker_sensing,victim_pos,attacker_pos,victim_vel,attacker_vel);
            
                    %receive the signal in the attacker
                    obj.Attacker.receive_signal(sig_attacker_sensing.');
            
                %get the transmitted signal from the attacker
                sig_attacker_attacking = obj.Attacker.transmit_attack(size(sig,1));
            
                sig_attacker_attacking = obj.channel_attacker(sig_attacker_attacking,victim_pos,attacker_pos,victim_vel,attacker_vel);
            
                %have the radar receive the signal

                obj.Victim.receive_signal(sig_target + sig_attacker_attacking + noise_sig);

                if obj.Victim.current_frame == frames_to_compute
                    if obj.Victim.current_chirp > obj.Victim.NumChirps
                        sim_complete = true;
                    end
                end
            end
        end

        function run_simulation_attack_no_target(obj,frames_to_compute,wait_bar_enable)
            %{
                Purpose: runs the simulation (with an attacker) for the
                    given number of frames
                Inputs:
                    frames_to_compute: the number of frames to simulate
            %}
            
            noise_power_dBm = -174 + 10 * log10(obj.Victim.Chirp_Tx_Bandwidth_MHz * 1e6);
            
            if wait_bar_enable
                status = sprintf("Current frame: %d or %d",obj.Victim.current_frame, frames_to_compute);
                progress_bar = waitbar(0,status,"Name","Running Simulation");
            end
            
            sim_complete = false;
            while ~sim_complete
                
                %update the progress_bar
                if wait_bar_enable
                    status = sprintf("Current frame: %d or %d",obj.Victim.current_frame, frames_to_compute);
                    waitbar(obj.Victim.current_frame/frames_to_compute,progress_bar,status);
                end

                %get the transmitted signal from the radar
                sig = obj.Victim.get_radar_tx_signal();
                
                %compute the noise signal
                noise_sig = wgn(size(sig,1),1,noise_power_dBm - 30,"complex");
                
                %update positions
                [victim_pos, victim_vel,attacker_pos, attacker_vel, tgt_pos,tgt_vel] = ...
                    obj.FMCW_determine_positions_and_velocities(...
                    obj.Victim.current_frame,obj.Victim.num_samples_sent);
                
                %propagate the signal and reflect it off of the target
%                 sig_target = obj.channel_target(sig,victim_pos,tgt_pos,victim_vel,tgt_vel);
%                 sig_target = obj.SimulatedTarget.radar_target(sig_target);
                
                %simulate attacker behavior
            
                    %attacker determines relative position and velocity of the victim
                    obj.Attacker.update_victim_pos_and_velocity(attacker_pos,victim_pos,attacker_vel, victim_vel);
                    obj.Attacker.update_target_pos_and_velocity(victim_pos, tgt_pos, victim_vel,tgt_vel)
                    
                    %assemble the noisy signal and the attacker signal
                    sig_attacker_sensing = noise_sig + sig;
                
                    %propogate the signal
                    sig_attacker_sensing = obj.channel_attacker(sig_attacker_sensing,victim_pos,attacker_pos,victim_vel,attacker_vel);
            
                    %receive the signal in the attacker
                    obj.Attacker.receive_signal(sig_attacker_sensing.');
            
                %get the transmitted signal from the attacker
                sig_attacker_attacking = obj.Attacker.transmit_attack(size(sig,1));
            
                sig_attacker_attacking = obj.channel_attacker(sig_attacker_attacking,victim_pos,attacker_pos,victim_vel,attacker_vel);
            
                %have the radar receive the signal

                obj.Victim.receive_signal(sig_attacker_attacking + noise_sig);

                if obj.Victim.current_frame == frames_to_compute
                    if obj.Victim.current_chirp > obj.Victim.NumChirps
                        sim_complete = true;
                    end
                end
            end
        end

        function run_sensing_subsystem_with_USRP_data(obj,USRP_data,wait_bar_enable)
            %{
                Purpose: runs the simulation (with an attacker) for the
                    given number of frames
                Inputs:
                    USRP Data: the recorded data from the USRP
                    wait_bar_enable: whether or not to use a wait bar for
                    the simulation
            %}
            
            buffers_received = size(USRP_data,1);
            current_buffer_idx = 1;

            %hardcode victim to remain at the first frame (assuming
            %stationary victim and attacker at this point
            victim_frame = 1;
            victim_num_samples_sent = 1;

            if wait_bar_enable
                status = sprintf("Current buffer: %d of %d",current_buffer_idx, buffers_received);
                progress_bar = waitbar(0,status,"Name","Running Simulation");
            end
            
            for current_buffer_idx = 1:buffers_received
                
                %update the progress_bar
                if wait_bar_enable
                    status = sprintf("Current buffer: %d of %d",current_buffer_idx, buffers_received);
                    waitbar(current_buffer_idx/buffers_received,progress_bar,status);
                end

                %get the transmitted signal from the radar
                sig = USRP_data(current_buffer_idx,:).';
                
                %update positions
                [victim_pos, victim_vel,attacker_pos, attacker_vel, tgt_pos,tgt_vel] = ...
                    obj.FMCW_determine_positions_and_velocities(...
                    victim_frame,victim_num_samples_sent);
                
%                 %propagate the signal and reflect it off of the target
%                 sig_target = obj.channel_target(sig,victim_pos,tgt_pos,victim_vel,tgt_vel);
%                 sig_target = obj.SimulatedTarget.radar_target(sig_target);
                
                %simulate attacker behavior
            
                    %attacker determines relative position and velocity of the victim
                    obj.Attacker.update_victim_pos_and_velocity(attacker_pos,victim_pos,attacker_vel, victim_vel);
                    obj.Attacker.update_target_pos_and_velocity(victim_pos, tgt_pos, victim_vel,tgt_vel)
                    
                    %assemble the noisy signal and the attacker signal
%                     sig_attacker_sensing = noise_sig + sig;
                    sig_attacker_sensing = sig;
                
                    %propogate the signal
%                     sig_attacker_sensing = obj.channel_attacker(sig_attacker_sensing,victim_pos,attacker_pos,victim_vel,attacker_vel);
            
                    %receive the signal in the attacker
                    obj.Attacker.receive_signal(sig_attacker_sensing.');
            
                %get the transmitted signal from the attacker - done only
                %to maintain proper operation
                sig_attacker_attacking = obj.Attacker.transmit_attack(size(sig,1));
            
%                 sig_attacker_attacking = obj.channel_attacker(sig_attacker_attacking,victim_pos,attacker_pos,victim_vel,attacker_vel);
            
%                 %have the radar receive the signal
% 
%                 obj.Victim.receive_signal(sig_target + sig_attacker_attacking + noise_sig);
% 
%                 if obj.Victim.current_frame == frames_to_compute
%                     if obj.Victim.current_chirp > obj.Victim.NumChirps
%                         sim_complete = true;
%                     end
%                 end
            end
        end
    
%% [3] Functions to support reading and writing data to a file for c++ script checking
        function save_to_file(obj,data_to_save,path,dtype)
            fileID = fopen(path,'w');
            data = double(data_to_save);
            fwrite(fileID,reshape([real(data),imag(data)].',[],1), dtype);
%             fwrite(fileID,reshape([real(data),imag(data)].',[],1), 'float32');
            fclose(fileID);
        end

        function save_data_to_file(obj,data_to_save,path,dtype)
            fileID = fopen(path,'w');
            fwrite(fileID,data_to_save, dtype);
            fclose(fileID);
        end

        function read_data = read_from_file(obj,path, complex_data, dtype)
            fileID = fopen(path,'r');
            read_data = fread(fileID,dtype);
            %read_data = fread(fileID,'float');
            
            %convert to complex values
            if complex_data
                read_data = reshape(read_data,2,[]).';
                read_data = read_data(:,1) + 1j * read_data(:,2);
            end
        end

    end

end