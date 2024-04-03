classdef FMCW_Emulator_revA  < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties

        %the target emulator will associate itself with a Victim and will
        %emulate radar targets based on this
        Radar

        %variables to aid in the computation of the emulated chirp
        phase_shift_per_chirp
        t                           %varialbe to hold the times at which to compute chirps

        FrequencySlope_MHz_us       %initializing a varialbe to hold the slope of the emulated chirps as some attacks adjust this
        
        %variables to be used for simulating random phase variation used in
        %the "similar velocity" mode
        sigma                               %the variance of the additional phase variation
        velocity_spoof_adjustment           %the amount to adjust the velocity by (affects phase shift per chirp)
        additional_phase_variation          %for executing velocity attacks
        additional_time_delay_us            %for executing range attacks

        %variable to hold all of the emulated chirps for a specific frame
        emulated_chirps
        
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
        emulation_mode
    end

    methods
        function obj = FMCW_Emulator_revA(Radar,desired_range_m,desired_velocity_m_s,emulation_mode)
            %{
                Purpose: the purpose of the class is to initialize an FMCW
                Radar target emulator that will replicate what a radar
                would see from a target
                Inputs:
                    Radar: A Radar class object for the emulator to
                        replicate a target from
                    desired_target_range_m: the desired range of the
                        emulated target
                    desired_target_velocity_m_s: the desired velocity of
                        the emulated target
            %}
            obj.Radar = Radar;
            obj.desired_range_m = desired_range_m;
            obj.desired_velocity_m_s = desired_velocity_m_s;
            obj.emulation_mode = emulation_mode;

            obj.compute_calculated_values();
            
            %compute the first frame of chirps
            obj.frame = 0;
            obj.compute_next_emulated_chirps();
        end
      
        function compute_calculated_values(obj)
            %{
                Purpose: computes all of the calculated parameters for the
                attacking subsystem
            %}
            
            %other parameters
            obj.t = 0:obj.Radar.FMCW_sampling_period_s:obj.Radar.sweep_time ...
                - obj.Radar.FMCW_sampling_period_s;
            obj.phase_shift_per_chirp = 4 * pi * obj.desired_velocity_m_s ...
                * obj.Radar.ChirpCycleTime_us * 1e-6 / obj.Radar.Lambda_m;
            
            if contains(obj.emulation_mode,"range spoof")
                obj.initialize_range_spoof();
            else
                obj.FrequencySlope_MHz_us = obj.Radar.FrequencySlope_MHz_us;
                obj.additional_time_delay_us = 0;
            end

            if contains(obj.emulation_mode,"velocity spoof")
                obj.initialize_velocity_spoof()
            else
                obj.additional_phase_variation = zeros(obj.Radar.NumChirps,1);
            end
        end

        function initialize_range_spoof(obj)
            if contains(obj.emulation_mode,"similar slope")
                obj.FrequencySlope_MHz_us = obj.Radar.FrequencySlope_MHz_us + obj.Radar.FrequencySlope_MHz_us * 0.007; %use 0.015 for low BW attacks
                obj.additional_time_delay_us = ...
                    0.5 * obj.Radar.Chirp_Tx_Bandwidth_MHz *...
                    (1/obj.Radar.FrequencySlope_MHz_us - 1/obj.FrequencySlope_MHz_us); %use 0.4 for low BW attacks
            end
        end

        function initialize_velocity_spoof(obj)
            if contains(obj.emulation_mode,"noisy")
                %compute any additional phase variation (mu = 0)
                num_bins_to_target = max(obj.Radar.NumChirps * 0.2,3);
                obj.sigma = num_bins_to_target * 2 * pi / obj.Radar.NumChirps;
    
                %adjust the velocity by the desired amount

                %if there is also a range spoof, don't adjust the velocity
                if contains(obj.emulation_mode,"range spoof")
                    obj.velocity_spoof_adjustment = 0;
                else
                    obj.velocity_spoof_adjustment = -3.0;
                end
                obj.phase_shift_per_chirp = 4 * pi * ...
                    (obj.desired_velocity_m_s + obj.velocity_spoof_adjustment) ...
                    * obj.Radar.ChirpCycleTime_us * 1e-6 / obj.Radar.Lambda_m;
            elseif contains(obj.emulation_mode,"similar velocity")
                %adjust the velocity by the desired amount
                obj.velocity_spoof_adjustment = 0;
                obj.phase_shift_per_chirp = 4 * pi * ...
                    (obj.desired_velocity_m_s + obj.velocity_spoof_adjustment) ...
                    * obj.Radar.ChirpCycleTime_us * 1e-6 / obj.Radar.Lambda_m;

                %compute what the phase shift values would actually be with
                %no attack
                phase_shift_at_chirps = obj.phase_shift_per_chirp * (0:obj.Radar.NumChirps - 1);
                

                %compute what the phase shift values should be with the
                %attack
                num_bins_to_target = max(obj.Radar.NumChirps * 0.10,3);
                max_phase_shift = num_bins_to_target * 2 * pi / obj.Radar.NumChirps;

                phase_shift_change_per_chirp = 2 * max_phase_shift / obj.Radar.NumChirps;
                desired_phase_shift_deltas = (-1 * max_phase_shift : phase_shift_change_per_chirp : ...
                    max_phase_shift - phase_shift_change_per_chirp) ...
                    + obj.phase_shift_per_chirp;
                desired_phase_shift_at_chirps = zeros(1,obj.Radar.NumChirps);

                for chirp = 1:obj.Radar.NumChirps - 1
                    desired_phase_shift_at_chirps(chirp + 1) = ...
                        desired_phase_shift_at_chirps(chirp) + ...
                        desired_phase_shift_deltas(chirp);
                end
                obj.additional_phase_variation = desired_phase_shift_at_chirps - phase_shift_at_chirps;
            end
        end

        function compute_noisy_velocity_spoof_values(obj)
                %for normal distribution
                %obj.additional_phase_variation = randn(obj.Radar.NumChirps,1) * obj.sigma;
    
                %for uniform distribution
                obj.additional_phase_variation = (rand(obj.Radar.NumChirps,1) - 0.5) * 2 * obj.sigma;
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
            obj.emulated_chirps = zeros(size(obj.t,2) + obj.Radar.num_samples_idle_time, obj.Radar.NumChirps);
            
            %if velocity spoof, update the additional phase_variation
            %values
            if contains(obj.emulation_mode,"velocity spoof - noisy")
                obj.compute_noisy_velocity_spoof_values()
            end
            
            %time delay for current emulation range
            current_emulation_range = obj.desired_range_m - ...
                obj.desired_velocity_m_s * ...
                obj.Radar.FramePeriodicity_ms * 1e-3 * ...
                (double(obj.frame) - 1);
            time_delay = range2time(current_emulation_range,physconst('lightspeed')) + ...
                obj.additional_time_delay_us * 1e-6;
            num_samples_delay = time_delay / obj.Radar.FMCW_sampling_period_s;
            
            for chirp = 1:int32(obj.Radar.NumChirps)
                %compute the phase shift
                phase_shift = (double(chirp) - 1) * obj.phase_shift_per_chirp;
            
                %compute the waveform
                waveform = cos(pi * obj.FrequencySlope_MHz_us * 1e12 ...
                    * obj.t.^(2) + phase_shift + obj.additional_phase_variation(chirp)) + ...
                    1i * sin(pi * obj.FrequencySlope_MHz_us * 1e12...
                    * obj.t.^(2) + phase_shift + obj.additional_phase_variation(chirp));
            
                %save it in the emulated_chirps array
                obj.emulated_chirps(obj.Radar.num_samples_idle_time + 1:end,chirp) = waveform.';
            
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
    
        function reset_emulated_chirp_computation(obj)
            obj.frame = 0;
        end
    end
end