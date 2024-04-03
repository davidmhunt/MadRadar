%% For use in SIMULINK Model

%% Notes
%{
    -This class is originally based off of the Radar Class, but has been
    tailored to include things that are specific to the attacker
%}

classdef Attacker_revB < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = public)
        %parameters for each of the subsystems in the class
        Subsystem_attacking         %a Subsystem_attacking class object
        Subsystem_tracking          %a Radar class object
        Subsystem_spectrum_sensing  %a Subsystem_spectrum_sensing class object
        
        %parameters for the position and speed of the attacker
        position_m
        velocity_m_per_s

        %parameter for the platform of the attacker
        platform            %phased.Platform object

        %parameter used by all attacker subsystems for the sampling rate
        %and operating frequency
        FMCW_sample_rate_Msps
        StartFrequency_GHz
        Bandwidth_MHz           %used by attacking subsystem for chirp bandwidth
        
        %parameters for an attacker's transmitter and receiver
            %parameters for the transmitter and receiver
            transmitter         %phased.Transmitter object
            receiver            %phased.Receiver
    
            %parameters for the Rx and Tx power when dealing with the FMCW
            %waveform, not setable parameters in the UI
            ant_aperture_m2
            ant_gain_dB
    
            tx_power_W
            tx_gain_dB
    
            rx_gain_dB
            rx_nf_dB
        
        %parameters to support attacker operation
        current_victim_pos
        current_victim_vel

        current_target_pos
        current_target_vel

    end

    methods (Access = public)
        function obj = Attacker_revB()
            %Radar construct an instance of this class
            %   Detailed explanation goes here
            obj.Subsystem_tracking = Radar_revB();
            obj.Subsystem_attacking = Subsystem_attacking(obj);
            obj.Subsystem_spectrum_sensing = Subsystem_spectrum_sensing(obj);
        end

        function configure_platform(obj)
            %{
                Prpose: configures the platform object as well as the
                platform objects for the subsystem modules as needed
            %}

            %for the attack module itself
            obj.platform = phased.Platform( ...
                'InitialPosition',obj.position_m, ...
                'Velocity',obj.velocity_m_per_s);

            %for the tracking subsystem
            obj.Subsystem_tracking.position_m = obj.position_m;
            obj.Subsystem_tracking.velocity_m_per_s = obj.velocity_m_per_s;
            obj.Subsystem_tracking.platform = obj.platform;
        end
    
        function set_desired_attack_parameters(obj, desired_range_m,desired_velocity_m_s)
            %{
                Purpose: sets the desired attack parameters for the
                attacker
            %}
            obj.Subsystem_attacking.desired_attack_range_m = desired_range_m;
            obj.Subsystem_attacking.desired_attack_velocity_m_s = desired_velocity_m_s;

        end
        
        function initialize_attacker(obj,FMCW_sample_rate_Msps,StartFrequency_GHz,Bandwidth_MHz)
            %{
                Purpose: initializes the attacker and its additional
                modules
                Inputs:
                    FMCW_sample_rate_Msps: the sample rate that the
                        attacker and its subsystems should operate at
                    StartFrequency_GHz: the frequency that the attacker
                        operates at (it is possible to set the tracking
                        subsystem to operate at a different frequency)
                    Bandwidth: the maximum bandwidth of the attacking
                        chirps
            %}
            obj.FMCW_sample_rate_Msps = FMCW_sample_rate_Msps;
            obj.StartFrequency_GHz = StartFrequency_GHz;
            obj.Bandwidth_MHz = Bandwidth_MHz;
            obj.Subsystem_spectrum_sensing.initialize_spectrum_sensing_parameters(FMCW_sample_rate_Msps);

        end
        
        function configure_transmitter_and_receiver(obj)
            %{
                Purpose: configures the transmitter and receiver objects
                    for the attacker (same as for the radar)
                Note: all other parameters must be set before configuring
                the transmitter and receiver
            %}
            lambda = freq2wavelen(obj.Subsystem_tracking.StartFrequency_GHz * 1e9);


            obj.ant_aperture_m2 = 6.06e-4;                                  % in square meter
            obj.ant_gain_dB = aperture2gain(obj.ant_aperture_m2,lambda);    % in dB
            
            obj.tx_power_W = db2pow(5)*1e-3;                                % in watts
            obj.tx_gain_dB = 9+ obj.ant_gain_dB;                            % in dB
            
            obj.rx_gain_dB = 15+ obj.ant_gain_dB;                           % in dB
            obj.rx_nf_dB = 20;                                             % in dB

            obj.transmitter = phased.Transmitter( ...
                'PeakPower',obj.tx_power_W, ...
                'Gain',obj.tx_gain_dB);
            obj.receiver = phased.ReceiverPreamp( ...
                'Gain',obj.rx_gain_dB, ...
                'NoiseFigure',obj.rx_nf_dB,...
                'SampleRate',obj.Subsystem_tracking.FMCW_sampling_rate_Hz);
        end

        function import_parameters_from_victim(obj, Victim)
        %{
            Purpose: This funciton allows the attacker to obtain its
            parameters from the victim. Basically, it allows me to use the
            attacker as a target emulator
        %}
            %imported parameters
        obj.Subsystem_attacking.StartFrequency_GHz = Victim.StartFrequency_GHz;
        obj.Subsystem_attacking.FrequencySlope_MHz_us= Victim.FrequencySlope_MHz_us;
        obj.Subsystem_attacking.IdleTime_us = Victim.IdleTime_us;                
        obj.Subsystem_attacking.RampEndTime_us = Victim.RampEndTime_us;                                  
        obj.Subsystem_attacking.Chirp_Tx_Bandwidth_MHz = Victim.Chirp_Tx_Bandwidth_MHz;          
        obj.Subsystem_attacking.ChirpCycleTime_us = Victim.ChirpCycleTime_us;
        obj.Subsystem_attacking.Lambda_m = Victim.Lambda_m;                        

        %frame parameters
        obj.Subsystem_attacking.NumChirps = Victim.NumChirps;
        obj.Subsystem_attacking.FramePeriodicity_ms = Victim.FramePeriodicity_ms;
        obj.Subsystem_attacking.ActiveFrameTime_ms = Victim.ActiveFrameTime_ms;              

        %parameters for the FMCW waveform, not setable parameters in UI
        obj.Subsystem_attacking.FMCW_sampling_rate_Hz = Victim.FMCW_sampling_rate_Hz;
        obj.Subsystem_attacking.sweep_time_s = Victim.sweep_time;
        obj.Subsystem_attacking.num_samples_per_frame = Victim.num_samples_per_frame;
        end
    
        function update_victim_pos_and_velocity(obj, attacker_pos, victim_pos, attacker_vel,victim_vel)
            %{
                Purpose: updates the attacker's knowledge of a victim's
                    relative position and velocity
                Inputs:
                    attacker_pos: the position of an attacker
                    victim_pos: the position of an victim
                    attacker_vel: the velocity of the attacker
                    victim_vel: the velocity of the victim
            %}
            obj.current_victim_pos = norm(attacker_pos - victim_pos);
            
            %using the projection to determine the veloctiy perceived by
            %the victim
            relative_pos_normalized = (attacker_pos - victim_pos)/norm(attacker_pos -victim_pos);
            relative_vel = attacker_vel - victim_vel;
            obj.current_victim_vel = -1 * dot(relative_pos_normalized,relative_vel);
        end

        function update_target_pos_and_velocity(obj, victim_pos, target_pos, victim_vel,target_vel)
            %{
                Purpose: updates the attacker's knowledge of the target
                location
                Inputs:
                    attacker_pos: the position of an attacker
                    victim_pos: the position of an victim
                    attacker_vel: the velocity of the attacker
                    victim_vel: the velocity of the victim
            %}
            obj.current_target_pos = norm(victim_pos - target_pos);

            relative_pos_normalized = (victim_pos - target_pos)/norm(victim_pos - target_pos);
            relative_vel = victim_vel - target_vel;
            obj.current_target_vel = -1 * dot(relative_pos_normalized,relative_vel);
        end

        function receive_signal(obj,sig)
            %{
                Purpose: receive the signal from the victim, amplify the
                signal, and then send it to the spectrum sensing subsystem
                to process the signal
            %}
            obj.Subsystem_spectrum_sensing.receive_signal(obj.receiver(sig));
        end

        function sig = transmit_attack(obj,num_samples)
            %{
                Purpose: get the attack signal (if there is one) from the
                    attacking subsystem and amplify it using the attacker's
                    transmitter
                Inputs:
                    num_samples: the number of samples for the attacker to
                        send out
                Outputs:
                    sig: the attacker's signal from the
                        output of the attacker's transmitter
            %}

            sig = obj.transmitter(obj.Subsystem_attacking.transmit_signal(num_samples));
        end
    end
end