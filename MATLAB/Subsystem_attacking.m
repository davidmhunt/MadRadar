classdef Subsystem_attacking  < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties

        %the target emulator will associate itself with a Victim and will
        %emulate radar targets based on this
        Attacker

        %variables to track estimated parameters from the sensing subsystem
        chirps_to_compute                   %for determining how many chirps to compute
        estimated_chirp_cycle_time_us
        estimated_frequency_slope_MHz_us
        estimated_frame_periodicity_ms

        %calculated victim parameters based on the estimated parameters
        sweep_time_s
        idle_time_s
        num_samples_per_chirp
        num_samples_sweep_time
        num_samples_idle_time

        %variables to aid in the computation of the emulated chirp
        phase_shift_per_chirp
        t                           %varialbe to hold the times at which to compute chirps

        FrequencySlope_MHz_us       %initializing a varialbe to hold the slope of the emulated chirps as some attacks adjust this
        
        %variables to be used for simulating random phase variation used in
        %the "similar velocity" mode
        sigma                               %the variance of the additional phase variation
        velocity_spoof_adjustment           %the amount to adjust the velocity by (affects phase shift per chirp)
        additional_phase_variation          %for executing velocity attacks
        additional_phase_variation_noise    %for adding noise to the attack
        additional_time_delay_us            %for executing range attacks

        %variable to hold all of the emulated chirps for a specific frame
        emulated_chirps
        
        %variable to hold the waveform used by the sensing subsystem for
        %precise timing alignment
        chirp_waveform
        
        
        %parameters for the desired position and velocity of the added
        %points
        desired_range_m
        desired_velocity_m_s

        %parameter to keep track of the current frame that the emulator is
        %on
        frame

        %specify the type of emulation ("target", 
        % "velocity spoof - noisy", 
        % "velocity spoof - similar velocity",
        % "range spoof - similar slope")
        attack_mode
        
        attack_at_target_location

        %variables to support attacker streaming operations
        %variable to track the current state
        state % potential states: "Waiting for Configuration", "Waiting for Attack Start", "Attacking"
        configuration_loaded

        %structure to keep track of any future frame start times
        attack_streaming_params
        
        %variables to keep track of timing
        num_samples_streamed %specifies the number of samples that have previously been streamed

    end

    methods
        function obj = Subsystem_attacking(Attacker)
            %{
                Purpose: the purpose of the class is to initialize an FMCW
                Radar target emulator that will replicate what a radar
                would see from a target
                Inputs:
                    Attacker: The Attacker class object that will hold the
                    chirp values
                    desired_target_range_m: the desired range of the
                        emulated target
                    desired_target_velocity_m_s: the desired velocity of
                        the emulated target
            %}
            obj.Attacker = Attacker;
            
            %set the first frame to zero
            obj.frame = 0;

            %initialize streaming configurations
            obj.configuration_loaded = false;
            obj.state = "Waiting for Configuration";
            obj.num_samples_streamed = 0;

            obj.init_attack_streaming_params();
        end

        function set_attack_mode(obj, attack_mode)
            %{
                Purpose: initialize the attacker parameters for desired
                    attack mode with a default to attack at a target
                    location (unless initialized to a specific location)
                Inputs:
                    attack_mode: the mode of the attacker
            %}
            
            obj.attack_mode = attack_mode;
            obj.attack_at_target_location = true;
        end

        function set_desired_attack_location(obj,desired_range_m, desired_velocity_m_s)
            %{
                Purpose: initialize the attacker parameters for desired
                    range, desired velocity to place the attack at a
                    specific location
                Inputs:
                    desired_target_range_m: the desired range of the
                        emulated target
                    desired_target_velocity_m_s: the desired velocity of
                        the emulated target
            %}
            obj.attack_at_target_location = false;
            obj.desired_range_m = desired_range_m;
            obj.desired_velocity_m_s = desired_velocity_m_s;
        end

        function init_attack_streaming_params(obj)
            %{
                Purpose: initializes the attack streaming parameters
            %}
            
            %for loading frame start times
            obj.attack_streaming_params.frame_start_times_ms = zeros(128,1); %initialize to holdup to 128 future frames
            obj.attack_streaming_params.frame_start_times_samples = zeros(128,1);
            obj.attack_streaming_params.current_frame = 1;
            obj.attack_streaming_params.next_frame_to_load = 1;
            obj.attack_streaming_params.frames_loaded = false;
            
            %for streaming the actual attack
            obj.attack_streaming_params.current_chirp = 1;
            obj.attack_streaming_params.next_sample_index = 1;

        end

        function compute_calculated_values(obj, chirp_cycle_time_us, frequency_slope_MHz_us,estimated_frame_periodicity_ms, num_chirps)
            %{
                Purpose: computes all of the calculated parameters for the
                    attacking subsystem
                Inputs:
                    chirp_cycle_time_us: the estimated chirp cycle time in
                        us
                    frequency_slope_MHz_us: the estimated chirp slope in
                        MHz/us
            %}
            
            %specify that a configuration is now loaded:
            obj.configuration_loaded = true;
            
            %determine needed parameters
            fmcw_sampling_period_s = 1/(obj.Attacker.FMCW_sample_rate_Msps * 1e6);
            attacker_lambda_m = physconst("LightSpeed") / (obj.Attacker.StartFrequency_GHz * 1e9);

            %save estimated parameter values
            obj.chirps_to_compute = num_chirps;
            obj.estimated_chirp_cycle_time_us = chirp_cycle_time_us;
            obj.estimated_frequency_slope_MHz_us = frequency_slope_MHz_us;
            obj.estimated_frame_periodicity_ms = estimated_frame_periodicity_ms;
            
            %compute the sweep time and idle time for each chirp based on
            %the estimated parameters
            obj.calculate_victim_parameters()
            
            obj.t = 0:fmcw_sampling_period_s:obj.sweep_time_s ...
                - fmcw_sampling_period_s;

            %compute range specific parameters
            if contains(obj.attack_mode,"range spoof")
                obj.initialize_range_spoof();
            else
                obj.FrequencySlope_MHz_us = obj.estimated_frequency_slope_MHz_us;
                obj.additional_time_delay_us = 0;
            end
            
            %compute velocity specific parameters
            
            if obj.attack_at_target_location
                obj.desired_velocity_m_s = obj.Attacker.current_target_vel;
            end

            obj.phase_shift_per_chirp = 4 * pi * ...
                (obj.desired_velocity_m_s  - obj.Attacker.current_victim_vel/2)...
                * obj.estimated_chirp_cycle_time_us * 1e-6 / attacker_lambda_m;

            if contains(obj.attack_mode,"velocity spoof")
                obj.initialize_velocity_spoof()
            else
                obj.additional_phase_variation = zeros(obj.chirps_to_compute,1);
                obj.additional_phase_variation_noise = zeros(obj.chirps_to_compute,1);
            end

            %compute the waveform so that the sensing subsystem can use it
            %for precise timing
            obj.compute_waveform_for_precise_sensing();
        end

        function compute_waveform_for_precise_sensing(obj)
            %compute the waveform of a single chirp so that the sensing
            %subsystem can use it
            obj.chirp_waveform = cos(pi * obj.estimated_frequency_slope_MHz_us * 1e12 ...
                * obj.t.^(2)) + ...
                1i * sin(pi * obj.estimated_frequency_slope_MHz_us * 1e12...
                * obj.t.^(2));
        end

        function load_frame_start_time(obj,frame_start_time_ms)
            %{
                Purpose: load the time of a future frame start time into
                the attack_streaming_params.frame_start_times array
                Inputs: 
                    frame_start_time_ms: the start time that a future frame
                        will occur at
            %}
            next_frame_to_load = obj.attack_streaming_params.next_frame_to_load;

            %if this is the first frame loaded, compute the emulated chirps
            if next_frame_to_load == 1
                obj.compute_next_emulated_chirps();
            end

            %save the time that the next frame will start at
            obj.attack_streaming_params.frame_start_times_ms(next_frame_to_load) = frame_start_time_ms;

            %compute the sample index that the next frame will start at
            sample_index = round(frame_start_time_ms * 1e-3 * (obj.Attacker.FMCW_sample_rate_Msps * 1e6));
            obj.attack_streaming_params.frame_start_times_samples(next_frame_to_load) = sample_index;
            
            %%set the frames_loaded variable to be true
            obj.attack_streaming_params.frames_loaded = true;
            obj.attack_streaming_params.next_frame_to_load = next_frame_to_load + 1;
        end

        function calculate_victim_parameters(obj)
        %{
            Purpose: computes additional victim parameters like the sweep
            time, idle time, number of samples in the idle time, and
            samples per chirp
        %}
            %determine the number of samples in a chirp
            obj.num_samples_per_chirp = round(obj.estimated_chirp_cycle_time_us *...
                obj.Attacker.FMCW_sample_rate_Msps);

            %recompute the chirp cycle time so that it is a multiple of the
            %FMCW sampling rate
            obj.estimated_chirp_cycle_time_us = obj.num_samples_per_chirp /...
                (obj.Attacker.FMCW_sample_rate_Msps * 1e6) * 1e6;
            
            %compute the sweep and idle times
            obj.sweep_time_s = (obj.Attacker.Bandwidth_MHz / obj.estimated_frequency_slope_MHz_us) * 1e-6;
%             obj.num_samples_sweep_time = round(obj.sweep_time_s *...
%                 obj.Attacker.FMCW_sample_rate_Msps * 1e6);
%             obj.sweep_time_s = obj.num_samples_sweep_time /...
%                 (obj.Attacker.FMCW_sample_rate_Msps * 1e6);
            
            %compute the idle time, and correct the sweep time if the sweep
            %time is longer than the chirp cycle time
            if obj.sweep_time_s * 1e6 > obj.estimated_chirp_cycle_time_us
                obj.sweep_time_s = obj.estimated_chirp_cycle_time_us * 1e-6;
            end
            
            obj.num_samples_sweep_time = floor(obj.sweep_time_s *...
            obj.Attacker.FMCW_sample_rate_Msps * 1e6);
            obj.sweep_time_s = obj.num_samples_sweep_time /...
            (obj.Attacker.FMCW_sample_rate_Msps * 1e6);

            obj.idle_time_s = obj.estimated_chirp_cycle_time_us * 1e-6 - obj.sweep_time_s;
            obj.num_samples_idle_time = obj.num_samples_per_chirp - obj.num_samples_sweep_time;
        end
    
        function initialize_range_spoof(obj)
            %{
                Purpose: initializes unique range spoofing attacks
            %}
            if contains(obj.attack_mode,"similar slope") 
                if obj.Attacker.FMCW_sample_rate_Msps >= 500 %high BW attacks
                    obj.FrequencySlope_MHz_us = obj.estimated_frequency_slope_MHz_us + ...
                        obj.estimated_frequency_slope_MHz_us * 0.002; %- previously used 0.007
                    obj.additional_time_delay_us = ...
                        0.5 * obj.Attacker.Bandwidth_MHz *...
                        (1/obj.estimated_frequency_slope_MHz_us - 1/obj.FrequencySlope_MHz_us); %previously used 0.8
                elseif obj.Attacker.FMCW_sample_rate_Msps >= 50 % mid BW attacks
                        obj.FrequencySlope_MHz_us = obj.estimated_frequency_slope_MHz_us +...
                            obj.estimated_frequency_slope_MHz_us * 0.015;
                        obj.additional_time_delay_us = ...
                            0.4 * obj.Attacker.Bandwidth_MHz *...
                            (1/obj.estimated_frequency_slope_MHz_us - 1/obj.FrequencySlope_MHz_us);
                else % low BW attacks
                    obj.FrequencySlope_MHz_us = obj.estimated_frequency_slope_MHz_us + ...
                        obj.estimated_frequency_slope_MHz_us * 0.030;
                    obj.additional_time_delay_us = ...
                        0.4 * obj.Attacker.Bandwidth_MHz *...
                        (1/obj.estimated_frequency_slope_MHz_us - 1/obj.FrequencySlope_MHz_us);
                end
            end
        end

        function initialize_velocity_spoof(obj)

            attacker_lambda_m = physconst("LightSpeed") / (obj.Attacker.StartFrequency_GHz * 1e9);
            frac_chirps_to_attack = 0;
            if contains(obj.attack_mode,"noisy")
                %compute any additional phase variation (mu = 0)
                num_bins_to_target = max(obj.chirps_to_compute * 0.05,3); %previously used 0.2
                obj.sigma = num_bins_to_target * 2 * pi / obj.chirps_to_compute;
    
                %adjust the velocity by the desired amount

                %if there is also a range spoof, don't adjust the velocity
                if contains(obj.attack_mode,"range spoof")
                    obj.velocity_spoof_adjustment = 0;
                else
                    obj.velocity_spoof_adjustment = -3.0;
                end
                obj.phase_shift_per_chirp = 4 * pi * ...
                    (obj.desired_velocity_m_s - obj.Attacker.current_victim_vel/2 ...
                    + obj.velocity_spoof_adjustment) ...
                    * obj.estimated_chirp_cycle_time_us * 1e-6 / attacker_lambda_m;
                if obj.Attacker.FMCW_sample_rate_Msps >= 500 %high BW attacks
                    frac_chirps_to_attack = 0.025;
                elseif obj.Attacker.FMCW_sample_rate_Msps >= 50 %mid BW attacks
                    frac_chirps_to_attack = 0.025;
                else %low BW attacks
                    frac_chirps_to_attack = 0.2;
                end
            elseif contains(obj.attack_mode,"similar velocity")
                frac_chirps_to_attack = 0.10; %0.20 for usrp
                obj.additional_phase_variation_noise = zeros(obj.chirps_to_compute,1);
            end
            %adjust the velocity by the desired amount
            obj.velocity_spoof_adjustment = 0;
            obj.phase_shift_per_chirp = 4 * pi * ...
                (obj.desired_velocity_m_s - obj.Attacker.current_victim_vel/2 ...
                + obj.velocity_spoof_adjustment) ...
                * obj.estimated_chirp_cycle_time_us * 1e-6 / attacker_lambda_m;

            %compute what the phase shift values would actually be with
            %no attack
            phase_shift_at_chirps = obj.phase_shift_per_chirp * (0:obj.chirps_to_compute - 1);
            

            %compute what the phase shift values should be with the
            %attack
            num_bins_to_target = max(obj.chirps_to_compute * frac_chirps_to_attack,3);
            max_phase_shift = num_bins_to_target * 2 * pi / obj.chirps_to_compute;

            phase_shift_change_per_chirp = 2 * max_phase_shift / obj.chirps_to_compute;
            desired_phase_shift_deltas = (-1 * max_phase_shift : phase_shift_change_per_chirp : ...
                max_phase_shift - phase_shift_change_per_chirp) ...
                + obj.phase_shift_per_chirp;
            desired_phase_shift_at_chirps = zeros(1,obj.chirps_to_compute);

            for chirp = 1:obj.chirps_to_compute - 1
                desired_phase_shift_at_chirps(chirp + 1) = ...
                    desired_phase_shift_at_chirps(chirp) + ...
                    desired_phase_shift_deltas(chirp);
            end
            obj.additional_phase_variation = desired_phase_shift_at_chirps - phase_shift_at_chirps;
        end

        function scaling_val = compute_power_scaling(obj)
            if contains(obj.attack_mode,"target")
                if obj.desired_range_m < obj.Attacker.current_victim_pos
                    scaling_val = 1.0 * 1e-2;
                    return;
                else
                    loss_attacker = 1 / (4 * pi * obj.Attacker.current_victim_pos^(2));
                    loss_spoofing = 1/ (4 * pi * obj.desired_range_m^(2));
                    scaling_val = loss_spoofing / loss_attacker * 1e-2;
                    return;
                end
            else 
                scaling_val = 1.0;
                return;
            end
        end
    
        function compute_noisy_velocity_spoof_values(obj)
            %for normal distribution
            %obj.additional_phase_variation_noise = randn(obj.chirps_to_compute,1) * obj.sigma;
    
            %for uniform distribution
            obj.additional_phase_variation_noise = (rand(obj.chirps_to_compute,1) - 0.5) * 2 * obj.sigma;
        end

        function compute_next_emulated_chirps(obj)
        %{
            Purpose: updates the chirps_emulated array to be chirps for the
                current frame. Attacking chirps are also adjusted to account for
                the attacker position relative to the victim position.
                After generating the emulated chirps for the current frame,
                the frame number is incremented so that the next time the
                function is called the next frame's chirps is automatically
                computed
        %}
            %increment the frame counter to be the next frame
            obj.frame = obj.frame + 1;
            
            %initialize the array of chirps
            obj.emulated_chirps = zeros(obj.num_samples_per_chirp, obj.chirps_to_compute);
            
            %if velocity spoof, update the additional phase_variation
            %values
            if contains(obj.attack_mode,"velocity spoof - noisy")
                obj.compute_noisy_velocity_spoof_values()
            end
            
            %time delay for current emulation range
            if obj.attack_at_target_location
                current_emulation_range = obj.Attacker.current_target_pos;
            else
                current_emulation_range = obj.desired_range_m - ...
                    obj.desired_velocity_m_s * ...
                    obj.estimated_frame_periodicity_ms * 1e-3 * ...
                    (double(obj.frame) - 1);
            end
            desired_time_delay = range2time(current_emulation_range,physconst('lightspeed')) + ...
                obj.additional_time_delay_us * 1e-6;
            propagation_delay = range2time(obj.Attacker.current_victim_pos ,physconst('lightspeed'))/2;
            time_delay = desired_time_delay - propagation_delay;
            num_samples_delay = round(time_delay * (obj.Attacker.FMCW_sample_rate_Msps * 1e6));
            
            scaling_val = obj.compute_power_scaling();
            
            for chirp = 1:int32(obj.chirps_to_compute)
                %compute the phase shift
                phase_shift = (double(chirp) - 1) * obj.phase_shift_per_chirp;
            
                %compute the waveform
                waveform = scaling_val * ...
                    (cos(pi * obj.FrequencySlope_MHz_us * 1e12 ...
                    * obj.t.^(2) + phase_shift + obj.additional_phase_variation(chirp) + ...
                    obj.additional_phase_variation_noise(chirp)) + ...
                    1i * sin(pi * obj.FrequencySlope_MHz_us * 1e12...
                    * obj.t.^(2) + phase_shift + obj.additional_phase_variation(chirp) + ...
                    obj.additional_phase_variation_noise(chirp)));
            
                %save it in the emulated_chirps array
                if obj.num_samples_sweep_time ~= size(waveform)
                    stop = true;
                end
                obj.emulated_chirps(1:obj.num_samples_sweep_time,chirp) = waveform.';
            
                if int32(num_samples_delay) > 0
                    obj.emulated_chirps(:,chirp) = ...
                        [zeros(int32(num_samples_delay),1);...
                        obj.emulated_chirps(1: end - int32(num_samples_delay),chirp)];
                elseif num_samples_delay < 0
                            obj.emulated_chirps(:,chirp) = ...
                            [obj.emulated_chirps(int32(-1 * num_samples_delay) + 1: end,chirp);...
                            zeros(int32(-1 * num_samples_delay),1)];
                end
            end
        end

        function sig = transmit_signal(obj,num_samples)
            %{
                Purpose: send out the specified number of samples
                Inputs: 
                    num_samples: the number of samples to send out
                Outputs:
                    sig: the output signal to send
            %}
            sig_assembled = false;
            next_signal_index = 1;

            %initialize the output signal
            sig = zeros(num_samples,1);

            while ~sig_assembled
                current_signal_index = next_signal_index;
                switch obj.state
                    case "Waiting for Configuration"
                        %check if a configuration has now been loaded
                        if obj.configuration_loaded == true
                            obj.state = "Waiting for Attack Start";
                        else
                            %simulate waiting for the configuration
                            [generated_sig,sig_assembled,next_signal_index] = ...
                                obj.simulate_waiting_for_config(next_signal_index,num_samples);
                            
                            %add the generated signal to the output
                            sig(current_signal_index:next_signal_index - 1) = generated_sig;
                        end
                    case "Waiting for Attack Start"
                        [generated_sig,sig_assembled,next_signal_index,frame_start_arrived] = ...
                            obj.simulate_waiting_for_attack_start(...
                            next_signal_index,num_samples);
                        
                        %add the generated signal to the output
                        sig(current_signal_index:next_signal_index - 1) = generated_sig;
                        
                        if frame_start_arrived
                            obj.state = "Attacking";

                            %initialize attack streaming parameters
                            obj.attack_streaming_params.current_chirp = 1;
                            obj.attack_streaming_params.next_sample_index = 1;
                        end
                    case "Attacking"
                        [generated_sig,sig_assembled,next_signal_index,end_of_attack] = ...
                            obj.simulate_attack_transmission(...
                            next_signal_index,num_samples);

                        %add the generated signal to the output
                        sig(current_signal_index:next_signal_index - 1) = generated_sig;
                        
                        if end_of_attack
                            obj.state = "Waiting for Attack Start";
                        end
                    otherwise
                end
            end
        end

        function [sig,sig_assembled,next_signal_index] = simulate_waiting_for_config(...
                obj,next_signal_index,num_samples)
            %{
                Purpose: simulates the attacker being in a state where is
                    is waiting for a configuration (i.e: doesn't have anything
                    from the sensing subsystem yet)
                Inputs:
                    next_signal_index: the next signal index where the
                        samples will be added
                    num_samples: the final number of samples that is
                        desired for the transmit function
                Outputs:
                    sig: the generated signal
                    sig_assembled: indicator for if the signal to transmit
                        is now fully assembled
                    next_signal_index: the next index for a signal
            %}
            
            %determine how many samples have yet to be sent
            unsent_samples = num_samples - next_signal_index;

            %update the number of samples streamed variable
            obj.num_samples_streamed = obj.num_samples_streamed + unsent_samples + 1;

            %create the output signal
            sig = zeros(unsent_samples + 1,1);

            %specify that the signal is now assembled
            sig_assembled = true;
            next_signal_index = next_signal_index + unsent_samples + 1;
        end

        function [sig,sig_assembled,next_signal_index,frame_start_arrived] = simulate_waiting_for_attack_start(...
                obj,next_signal_index,num_samples)
            %{
                Purpose: simulates the attacker being in a state where is
                    is waiting for a configuration (i.e: doesn't have anything
                    from the sensing subsystem yet)
                Inputs:
                    next_signal_index: the next signal index where the
                        samples will be added
                    num_samples: the final number of samples that is
                        desired for the transmit function
                Outputs:
                    sig: the generated signal
                    sig_assembled: indicator for if the signal to transmit
                        is now fully assembled
                    next_signal_index: the next index for a signal
                    frame_start_arrived: set to true when the start of the
                        next frame has arrived, false otherwise
            %}

            %determine how many samples have yet to be sent
            unsent_samples = num_samples - next_signal_index;

            %check to make sure that a future frame start time is available
            frame_available = obj.check_for_next_frame();

            %set default behavior of next frame start
            frame_start_arrived = false;

            if frame_available
                %compute the number of samples until the next frame start.
                %Subtract by 2 because we want the frame to start on that
                %exact sample
                samples_until_next_frame_start = ...
                    obj.attack_streaming_params.frame_start_times_samples(...
                    obj.attack_streaming_params.current_frame) - obj.num_samples_streamed - 2;

                if samples_until_next_frame_start > unsent_samples
                    %update the number of samples streamed variable
                    obj.num_samples_streamed = obj.num_samples_streamed + unsent_samples + 1;
        
                    %create the output signal
                    sig = zeros(unsent_samples + 1,1);
        
                    %specify that the signal is now assembled
                    sig_assembled = true;
                    next_signal_index = next_signal_index + unsent_samples + 1;
                else
                    %next frame start has arrived
                    %update the number of samples streamed variable
                    obj.num_samples_streamed = obj.num_samples_streamed + samples_until_next_frame_start + 1;
        
                    %create the output signal
                    sig = zeros(samples_until_next_frame_start + 1,1);
        
                    %specify that the signal is now assembled
                    sig_assembled = false;
                    next_signal_index = next_signal_index + samples_until_next_frame_start + 1;
                    
                    %set the frame_start_arrived variable
                    frame_start_arrived = true;
                end

            else
                %update the number of samples streamed variable
                obj.num_samples_streamed = obj.num_samples_streamed + unsent_samples + 1;
    
                %create the output signal
                sig = zeros(unsent_samples + 1,1);
    
                %specify that the signal is now assembled
                sig_assembled = true;
                next_signal_index = next_signal_index + unsent_samples + 1;
            end
        end
        
        function frame_available = check_for_next_frame(obj)
            %{
                Purpose: checks to see if there is a future frame start
                    time available in 
                    obj.attack_streaming_params.frame_start_times_samples
                    and updates the obj.attack_streaming_params.current
                    frame if needed
                Outputs: 
                    frame_available: true if there is a frame available,
                    false if not
            %}

            current_frame = obj.attack_streaming_params.current_frame;
            frames_loaded = obj.attack_streaming_params.next_frame_to_load - 1;
            next_frame_start = obj.attack_streaming_params.frame_start_times_samples(current_frame);
            
            %find the next future frame time (if one exists)
            while obj.num_samples_streamed > next_frame_start && current_frame < frames_loaded
                current_frame = current_frame + 1;
                next_frame_start = obj.attack_streaming_params.frame_start_times_samples(current_frame);
                
                %update the class variables as well
                obj.compute_next_emulated_chirps();
                obj.attack_streaming_params.current_frame = current_frame;
            end

            if next_frame_start > obj.num_samples_streamed
                frame_available = true;
            else
                frame_available = false;
            end

        end
        
        function [sig,sig_assembled,next_signal_index,end_of_attack] = simulate_attack_transmission(...
                obj,next_signal_index,num_samples)
            %{
            Purpose: simulates the attacker being in the state where it is
                transmitting the attacking signal
            Inputs:
                next_signal_index: the next signal index where the
                    samples will be added
                num_samples: the final number of samples that is
                    desired for the transmit function
            Outputs:
                sig: the generated signal
                sig_assembled: indicator for if the signal to transmit
                    is now fully assembled
                next_signal_index: the next index for a signal
                end_of_attack: set to true when the attack has finished
                    sending the current frame
            %}
            

            %set the default for end_of_attack
            end_of_attack = false;

            %compute the number of unsent samples out of the number of
            %requested samples
            unsent_samples = num_samples - next_signal_index;

            %determine the number of unsent samples from the current chirp
            unsent_samples_chirp = size(obj.emulated_chirps,1) - ...
                obj.attack_streaming_params.next_sample_index;
            
            %if there are more unsent samples from the current chirp
            if unsent_samples_chirp > unsent_samples
                
                %set the start and end index for the samples in the chrip
                start_index = obj.attack_streaming_params.next_sample_index;
                end_index = start_index + unsent_samples;

               
                sig = obj.emulated_chirps(start_index:end_index,...
                    obj.attack_streaming_params.current_chirp);

                %update settings as needed
                sig_assembled = true;
                next_signal_index = next_signal_index + unsent_samples + 1;
                obj.attack_streaming_params.next_sample_index = end_index + 1;
                obj.num_samples_streamed = obj.num_samples_streamed + unsent_samples + 1;
            else
                %set the start and end index for the samples in the chrip
                start_index = obj.attack_streaming_params.next_sample_index;
                
                %obtain the output sig
                sig = obj.emulated_chirps(start_index:end,...
                    obj.attack_streaming_params.current_chirp);

                %update settings as needed
                if unsent_samples_chirp == unsent_samples
                    sig_assembled = true;
                else
                    sig_assembled = false;
                end

                obj.attack_streaming_params.next_sample_index = 1;
                next_signal_index = next_signal_index + unsent_samples_chirp + 1;

                if obj.attack_streaming_params.current_chirp == obj.chirps_to_compute
                    %if the attack is now complete
                    end_of_attack = true;
                    obj.attack_streaming_params.current_chirp = 1;
                else
                    obj.attack_streaming_params.current_chirp = ...
                        obj.attack_streaming_params.current_chirp + 1;
                end

                obj.num_samples_streamed = obj.num_samples_streamed + unsent_samples_chirp + 1;

            end

        end

        function Tx_sig_attacker = get_transmitted_attacker_chirp(obj,chirp)
            %{
                Purpose: retrieves the specific attacking chirp and
                    computes the transmitted signal
                Inputs:
                    chirp - the attacking chirp number
                Outputs:
                    Tx_sig_attacker - the transmitted attacking chirp
                        signal
            %}
            attacker_sig = obj.attacker_chirps(:,chirp);
            Tx_sig_attacker = obj.transmitter(attacker_sig);
        end
    end
end