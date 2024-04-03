%% For use in SIMULINK Model

%% Notes
%adjust NF here
%{
    - If the start frequency is higher than 77 GHz, there is a little bit
    of extra work that has to be done. When creating this simulation, I
    removed this functionality to simplify things. I did implement the
    functionality in previous versions though which can be found in the
    archived functions folder. See the parameters additional_BW_MHz and
    additional_sweep_time_us in the archived Radar class file and the
    FMCW_generate_chirp_sig_vals function

    - reconfigured the chrip construction. Chirps still have an idle period
    and a sampling period, but the end of the sampling period now marks the
    end of the chirp instead of there being a little bit of extra time
    after the end of the sampling period
%}

classdef Radar_revB < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = public)
        %chirp parameters
        StartFrequency_GHz
        FrequencySlope_MHz_us
        IdleTime_us                     %calculated value, not setable in UI
        TxStartTime_us              
        ADC_ValidStartTime_us           %calculated value, not setable in UI
        ADC_Samples
        ADC_SampleRate_MSps
        RampEndTime_us                  %calculated value, not setable in UI                  
        Chirp_Tx_Bandwidth_MHz          %calculated value, not setable in UI
        Chirp_Sampling_Bandwidth_MHz    %calculated value, not setable in UI
        ADC_SamlingPeriod_us            %calculated value, not setable in UI
        ChirpCycleTime_us
        Lambda_m                        %calculated value, not setable in UI
        
        %frame parameters
        NumChirps
        FramePeriodicity_ms
        ActiveFrameTime_ms              %calculated value, not setable in UI

        %radar performance specs        %calculated values, not setable in UI
        Range_Max_m
        Range_Res_m
        Ranges
        V_Max_m_per_s
        V_Res_m_per_s
        Velocities

        %parameters for the position and speed of the radar
        position_m
        velocity_m_per_s

        %parameter for the platform of the radar
        platform            %phased.Platform object
        
        %if the radar is an attacker, use this parameter to set its offset
        %in us from the defender
        timing_offset_us
        
        %% The following parameters don't appear in the UI
        %define parameters for plotting
        plotResolution_us
        tx_period_plot_color
        tx_sampling_period_plot_color
        radar_name

        %parameters for the FMCW waveform, not setable parameters in UI
        waveform                            %phased.waveform object
        chirp                               %object that holds the raw samples for an FMCW chirp
        chirps                              %object that holds raw samples for all of the FMCW chirps in a frame
        FMCW_sampling_rate_Hz
        FMCW_sampling_period_s
        downsample_factor
        sweep_time
        num_samples_per_chirp
        num_samples_idle_time
        num_samples_per_frame
        num_samples_active_frame_time
%        additional_BW_MHz                   %for if the start frequency isn't 77.0 GHz, commented out for now
%        additional_sweep_time_us            %for if the start frequency isn't 77.0 GHz, commented out for now

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

        %parameter for the Radar Signal Processor
        Radar_Signal_Processor

        %parameter's for streaming chirps/frames
        current_chirp %the current chirp that the radar is transmitting
        current_frame
        num_samples_sent
        signal_received % a boolean to confirm that each transmitted signal was received before a new signal is transmitted
        tx_sig %the transmitted signal (used when receiving the signal)
    end

    methods (Access = public)
        function obj = Radar_revB()
            %Radar construct an instance of this class
            %   Detailed explanation goes here
            obj.Radar_Signal_Processor = Radar_Signal_Processor_revA(obj);
        end

        %% [1] Functions to print out Radar parameters
        function print_chirp_parameters(obj)
            fprintf("Chirp Parameters\n")
            fprintf("\t Start Frequency: \t\t %.2f GHz\n",obj.StartFrequency_GHz)
            fprintf("\t Frequency Slope: \t\t %.2f MHz/us\n",obj.FrequencySlope_MHz_us)
            fprintf("\t Idle Time: \t\t\t %.2f us\n",obj.IdleTime_us)
            fprintf("\t Tx Start Time: \t\t %.2f us\n",obj.TxStartTime_us)
            fprintf("\t ADC Valid Start Time: \t\t %.2f us\n",obj.ADC_ValidStartTime_us)
            fprintf("\t ADC Samples: \t\t\t %d \n",obj.ADC_Samples)
            fprintf("\t ADC Sample Rate: \t\t %.2f MSps\n",obj.ADC_SampleRate_MSps)
            fprintf("\t Ramp End Time: \t\t %.2f us\n",obj.RampEndTime_us)
            fprintf("\t Chirp Tx Bandwidth: \t\t %.2f MHz\n",obj.Chirp_Tx_Bandwidth_MHz)
            fprintf("\t Chirp Sampling Bandwidth: \t %.2f MHz\n", obj.Chirp_Sampling_Bandwidth_MHz)
            fprintf("\t ADC Sampling Period: \t\t %.2f us\n",obj.ADC_SamlingPeriod_us)
            fprintf("\t Chirp Cycle Time: \t\t %.2f us\n",obj.ChirpCycleTime_us)
            fprintf("\t Chirp Wavelength: \t\t %.2f mm\n",obj.Lambda_m * 1e3)
        end

        function print_frame_parameters(obj)
            fprintf("Frame Parameters\n")
            fprintf("\t Number of Chirps \t\t %d\n",obj.NumChirps)
            fprintf("\t FramePeriodicity \t\t %0.2f ms\n",obj.FramePeriodicity_ms)
            fprintf("\t Active Frame Time \t\t %0.2f ms\n",obj.ActiveFrameTime_ms)
        end
        
        function print_performance_specs(obj)
            fprintf("Performance Specifications\n")
            fprintf("\t Max Range \t\t\t %0.2f m\n",obj.Range_Max_m)
            fprintf("\t Range Resolution \t\t %0.2f m\n",obj.Range_Res_m)
            fprintf("\t Max Velocity \t\t\t %0.2f m/s\n",obj.V_Max_m_per_s)
            fprintf("\t Velocity Resolution \t\t %0.2f m/s\n",obj.V_Res_m_per_s)
        end

        function print_FMCW_specs(obj)
            fprintf("FMCW Specifications\n")
            fprintf("\t FMCW sampling rate \t\t %0.2f MHz\n",obj.FMCW_sampling_rate_Hz * 1e-6)
            fprintf("\t Downsampling factor \t\t %d\n",obj.downsample_factor)
            fprintf("\t Sweep time \t\t\t %0.2f us\n",obj.sweep_time * 1e6)
            fprintf("\t Samples per chirp \t\t %0.2f \n",obj.ChirpCycleTime_us * 1e-6 * obj.FMCW_sampling_rate_Hz)
        end

        function print_CFAR_detection_region(obj)
            fprintf("CFAR Detection Region\n");
            fprintf("\t Range Detection Region \t\t %0.3f m to %0.3f m\n",...
                obj.Radar_Signal_Processor.distance_detection_range(1), ...
                obj.Radar_Signal_Processor.distance_detection_range(2));
            fprintf("\t Velocity Detection Region \t\t %0.3f m/s to %0.3f m/s\n",...
                obj.Radar_Signal_Processor.velocity_detection_range(1), ...
                obj.Radar_Signal_Processor.velocity_detection_range(2));
        end

        function print_radar_parameters(obj)
            obj.print_chirp_parameters();
            obj.print_frame_parameters();
            obj.print_performance_specs();
            obj.print_FMCW_specs();
            obj.print_CFAR_detection_region();
        end

        %% [2] Functions to compute calculated radar parameters
        function compute_calculated_vals(obj)
            %{
                Purpose: computes all of the Radar class parameters that
                are marked as "calculated value, not setable in UI"
            %}

            %adjust the chirp cycle period so that there is an integer
            %number of samples in the chirp cycle period
            obj.ChirpCycleTime_us = ceil(obj.ChirpCycleTime_us * obj.ADC_SampleRate_MSps)...
                / obj.ADC_SampleRate_MSps;

            %compute calculated chirp parameters
            obj.ADC_SamlingPeriod_us            = obj.ADC_Samples / obj.ADC_SampleRate_MSps;
            
            
            %since the simulation in simulink will have different sample
            %times, it was determined that it was necessary for the frame
            %rate to be an integer multiple of the chirp cycle time. In
            %practice this wouldn't be a requirement, but in order to make
            %the simulation simpler, I've gone in and made this the case
            obj.FramePeriodicity_ms = ceil(obj.FramePeriodicity_ms/(obj.ChirpCycleTime_us * 1e-3)) * obj.ChirpCycleTime_us * 1e-3;

            %for the ADC_valid time, find a time that results in an integer
            %number of samples, but is longer than 6.3us (chosen to allow
            %sufficient time for the signal to propogate)
            obj.ADC_ValidStartTime_us           = ceil(6.3 * obj.ADC_SampleRate_MSps) / obj.ADC_SampleRate_MSps;
            obj.RampEndTime_us                  = ceil((obj.ADC_SamlingPeriod_us + obj.ADC_ValidStartTime_us) *...
                                                    obj.ADC_SampleRate_MSps) / obj.ADC_SampleRate_MSps;
            obj.IdleTime_us                     = obj.ChirpCycleTime_us - obj.RampEndTime_us;
            obj.Chirp_Tx_Bandwidth_MHz          = obj.FrequencySlope_MHz_us * obj.RampEndTime_us;
            obj.Chirp_Sampling_Bandwidth_MHz    = obj.FrequencySlope_MHz_us * obj.ADC_SamlingPeriod_us;
            obj.Lambda_m                        = physconst('LightSpeed') / (obj.StartFrequency_GHz * 1e9);

            %compute calculated frame parameters
            obj.ActiveFrameTime_ms              = obj.ChirpCycleTime_us * obj.NumChirps/1000;

            %calculate radar performance parameters
            obj.Range_Max_m = physconst('LightSpeed') * obj.ADC_SampleRate_MSps*1e6 / (2*obj.FrequencySlope_MHz_us*1e6*1e6);
            obj.Range_Res_m = physconst('LightSpeed') / (2*obj.Chirp_Sampling_Bandwidth_MHz*1e6);
            obj.Ranges = 0 : obj.Range_Res_m : obj.ADC_Samples * obj.Range_Res_m - obj.Range_Res_m;

            obj.V_Max_m_per_s = obj.Lambda_m / (4* obj.ChirpCycleTime_us * 1e-6);
            obj.V_Res_m_per_s = obj.Lambda_m / (2*obj.NumChirps*obj.ChirpCycleTime_us * 1e-6);
            obj.Velocities = -(obj.V_Max_m_per_s - obj.V_Res_m_per_s): obj.V_Res_m_per_s : obj.V_Max_m_per_s;
        end

        function configure_transmitter_and_receiver(obj)
            %{
                Purpose: configures the transmitter and receiver objects
                Note: all other parameters must be set before configuring
                the transmitter and receiver
            %}
            lambda = freq2wavelen(obj.StartFrequency_GHz * 1e9);


            obj.ant_aperture_m2 = 6.06e-4;                                  % in square meter
            obj.ant_gain_dB = aperture2gain(obj.ant_aperture_m2,lambda);    % in dB
            
            obj.tx_power_W = db2pow(5)*1e-3;                                % in watts
            obj.tx_gain_dB = 9+ obj.ant_gain_dB;                            % in dB
            
            obj.rx_gain_dB = 15+ obj.ant_gain_dB;                           % in dB
            obj.rx_nf_dB = 5;                                               % in dB
            
            obj.transmitter = phased.Transmitter( ...
                'PeakPower',obj.tx_power_W, ...
                'Gain',obj.tx_gain_dB);
            obj.receiver = phased.ReceiverPreamp( ...
                'Gain',obj.rx_gain_dB, ...
                'NoiseFigure',obj.rx_nf_dB,...
                'SampleRate',obj.FMCW_sampling_rate_Hz);
        end

        function configure_waveform_and_chirp(obj)
            %{
                Purpose: configures the phased.FMCWWaveform objects and
                    computes what the sent chirp will be including the idle
                    time samples
            %}
            obj.num_samples_per_chirp = int32(obj.ChirpCycleTime_us * 1e-6 * obj.FMCW_sampling_rate_Hz);
            obj.num_samples_idle_time = int32(obj.IdleTime_us * 1e-6 * obj.FMCW_sampling_rate_Hz);
            obj.num_samples_per_frame = int32(obj.FramePeriodicity_ms * 1e-3 * obj.FMCW_sampling_rate_Hz);
            obj.num_samples_active_frame_time = int32(obj.NumChirps * obj.ChirpCycleTime_us * 1e-6 * obj.FMCW_sampling_rate_Hz);
            
            
            
            obj.waveform = phased.FMCWWaveform( ...
                'SampleRate', obj.FMCW_sampling_rate_Hz, ...
                'SweepTime', obj.sweep_time,...
                'SweepBandwidth', (obj.Chirp_Tx_Bandwidth_MHz) * 1e6, ...
                'SweepDirection', 'Up',...
                'SweepInterval', 'Positive',...
                'OutputFormat','Sweeps',...
                'NumSweeps',1);
            
            obj.chirp = [zeros(obj.num_samples_idle_time,1); obj.waveform()];

            obj.current_chirp = 1;
            obj.current_frame = 1;
            obj.num_samples_sent = 0;
            obj.signal_received = true; %set the flag to true when initializing
        end
        
        function configure_radar_signal_processor(obj)
            %{
                Purpose: Configures the radar signal processor object which
                will perform all of the radar signal processing for this
                radar
            %}
            obj.Radar_Signal_Processor.configure_radar_signal_processor();
        end
        
        %% [3] Functions for processing the signals
        function chirps = precompute_radar_chirps(obj)
            obj.chirps = repmat(obj.chirp,1,obj.NumChirps);
            chirps = obj.chirps;
        end

        function signal = get_radar_tx_signal(obj)
            %{
                Purpose: outputs the full radar signal on a chirp by chirp basis
                    ,and updates the chirp and frame trackers. When there
                    are no more chirps in the current frame, outputs zeros
                    until the next frame starts. Simulates the output from
                    the radar's transmiter
                Signal: a transmitted radar signal (ex:chirp)
            %}

            %check to make sure that the signal received flag is not still
            %false
            if obj.signal_received == false
                error("Radar/get_radar_tx_signal: attempted to sent signal without receiving first");
            else
                obj.signal_received = false; %set the flag to note that there is another signal to send
            end

            if obj.current_chirp <= obj.NumChirps
                %if the radar is still sending out chirps
                obj.tx_sig = obj.chirp;
                obj.num_samples_sent = obj.num_samples_sent + obj.num_samples_per_chirp;
            else
                %if all chirps have been transmitted, but the frame isn't
                %over
                if (obj.num_samples_per_frame - obj.num_samples_sent) > obj.num_samples_per_chirp
                    samples_to_send = obj.num_samples_per_chirp;
                    obj.num_samples_sent = obj.num_samples_sent + obj.num_samples_per_chirp;
                else
                    %this means that the end of the frame has arrived
                    samples_to_send = obj.num_samples_per_frame - obj.num_samples_sent;
                    obj.num_samples_sent = 0;
                end
                obj.tx_sig = zeros(samples_to_send,1);
            end
            signal = obj.transmitter(obj.tx_sig);
        end

        function receive_signal(obj,rx_sig)
            %{
                Purpose: captures the signals received by the radar (ex:
                    reflections or malicious signals from an attacker), and
                    loads them as needed
                Inputs: 
                    rx_sig: the raw received signal from reflected objects
                        or from an attacker
            %}

            %if the radar is currently receiving chirps
            if obj.current_chirp <= obj.NumChirps
                obj.Radar_Signal_Processor.update_radar_cube(obj.tx_sig,obj.receiver(rx_sig),obj.current_chirp);
                

                %if this is the last chirp in the frame, process the received
                %signal
                if obj.current_chirp == obj.NumChirps
                    obj.Radar_Signal_Processor.process_radar_cube();
                end

                obj.current_chirp = obj.current_chirp + 1;
            else
                %if all chirps have been transmitted, but the frame isn't
                %over
                if obj.num_samples_sent == 0
                    %this means that the end of the frame has arrived
                    obj.Radar_Signal_Processor.reset_radar_cube();
                    obj.current_chirp = 1;
                    obj.current_frame = obj.current_frame + 1;
                end
            end
            obj.signal_received = true;
        end

        
        %% [4] Functions for plotting chirp plots to aid in visualization
        function [t,f] = generate_chirp_f_over_t_vals(obj,chirp_start_time_us)
%{
            Purpose: generates (t,f) coordinates for the Tx
                    frequency over time
            Inputs:
                    chirp_start_time: time that a chirp starts at
            Outputs:
                   t,f: (t,f) coordinates for the Tx frequency over time
%}            

            %round the times to be within the level of precision
            chirp_cycle_time = round((obj.ChirpCycleTime_us + chirp_start_time_us)/ obj.plotResolution_us)*obj.plotResolution_us;
            idle_time = round((obj.IdleTime_us + chirp_start_time_us) / obj.plotResolution_us)*obj.plotResolution_us;
            tx_start_time = round(obj.TxStartTime_us/ obj.plotResolution_us)*obj.plotResolution_us;
            
            %calculate the sloping values
            t = idle_time:obj.plotResolution_us:chirp_cycle_time;
            f = obj.StartFrequency_GHz + obj.FrequencySlope_MHz_us * (t - obj.IdleTime_us - chirp_start_time_us)/1000;

            %If Tx Start Time is not zero
            if obj.TxStartTime_us ~= 0
                t_additional = idle_time - tx_start_time:obj.plotResolution_us:idle_time - obj.plotResolution_us;
                t = [t_additional, t];

                f_additional =  obj.StartFrequency_GHz * ones(1,size(t_additional,2));
                f = [f_additional, f];
            end
        end

        function [ts,f] = generate_chirp_f_over_Tsampling_vals(obj, chirp_start_time_us)
%{
            Purpose: generates (t,f) coordinates for the Tx frequency over
                time for a single chirp
            Inputs:
                    chirp_start_time: time that a chirp starts at
            Outputs:
                t,f: (t,f) coordinates for the Tx frequency over the
                    sampling period
%}
            adc_sampling_start_time = round((obj.IdleTime_us + obj.ADC_ValidStartTime_us + chirp_start_time_us)/obj.plotResolution_us)*obj.plotResolution_us;
            adc_sampling_period = round(obj.ADC_SamlingPeriod_us/obj.plotResolution_us) * obj.plotResolution_us;

            %calculate the currently sloping values
            ts = adc_sampling_start_time:obj.plotResolution_us:adc_sampling_start_time + adc_sampling_period;
            f = obj.StartFrequency_GHz + obj.FrequencySlope_MHz_us * (ts - obj.IdleTime_us - chirp_start_time_us)/1000;
        end

        function [t,f] = generate_frame_f_over_T_vals(obj,frame_start_time_ms)
            %{
            Purpose: generates (t,f) coordinates for the Tx frequency over
                    time for an entire frame (in ms)
            Outputs:
                t,f: (t,f) coordinates for the Tx frequency over the
                    period of each chirp in the frame (in ms)
            %}
            
            %pre-allocate the size of the output array

            %round the times to be within the level of precision
            chirp_cycle_time = round((obj.ChirpCycleTime_us)/ obj.plotResolution_us)*obj.plotResolution_us;
            idle_time = round((obj.IdleTime_us) / obj.plotResolution_us)*obj.plotResolution_us;
            tx_start_time = round(obj.TxStartTime_us/ obj.plotResolution_us)*obj.plotResolution_us;
            
            %calculate the sloping values
            t_size = int32((chirp_cycle_time - idle_time)/obj.plotResolution_us + 1);

            %If Tx Start Time is not zero
            if obj.TxStartTime_us ~= 0
                t_size = t_size + tx_start_time/obj.plotResolution_us;

            end
            
            t = zeros(1,t_size * obj.NumChirps);
            f = zeros(1,t_size * obj.NumChirps);
            next_start_index = 1;

            %generate the f and t arrays 
            for chirp = 1:obj.NumChirps
                chirp_start_time_us = obj.ChirpCycleTime_us * (chirp - 1) + (frame_start_time_ms * 1000);
                [t_chirp,f_chirp] = obj.generate_chirp_f_over_t_vals(chirp_start_time_us);

                if(size(t_chirp,2) ~= t_size)
                    difference = size(t_chirp,2) - t_size;
                    if(difference > 0)
                        t = [t,zeros(1,difference)];
                        f = [f,zeros(1,difference)];
                    else
                        t = t(1:end + difference); %will reduce its size
                        f = f(1:end + difference); %will reduce its size
                    end
                    
                    t(next_start_index: next_start_index + t_size + difference - 1) = t_chirp;
                    f(next_start_index: next_start_index + t_size + difference - 1) = f_chirp;
                    next_start_index = next_start_index + t_size + difference;
                else
                    t(next_start_index: next_start_index + t_size - 1) = t_chirp;
                    f(next_start_index: next_start_index + t_size - 1) = f_chirp;
                    next_start_index = next_start_index + t_size;
                end
            end
            t = t/1000;
        end

        function [t,f] = generate_frame_f_over_Tsampling_vals(obj, frame_start_time_ms)
            %{
            Purpose: generates (t,f) coordinates for the Tx frequency over
                    the sampling period for each chirp in an entire frame
                    (in ms)
            Outputs:
                t,f: (t,f) coordinates for the Tx frequency over the
                    sampling period of each chirp in the frame (in ms)
            %}
            
            %pre-allocate the size of the output array

            %round the times to be within the level of precision
            adc_sampling_period = round(obj.ADC_SamlingPeriod_us/obj.plotResolution_us) * obj.plotResolution_us;
            
            %calculate the sloping values
            t_size = int32((adc_sampling_period)/obj.plotResolution_us + 1);
            
            t = zeros(1,t_size * obj.NumChirps);
            f = zeros(1,t_size * obj.NumChirps);
            next_start_index = 1;

            %generate the f and t arrays 
            for chirp = 1:obj.NumChirps
                chirp_start_time_us = obj.ChirpCycleTime_us * (chirp - 1) + (frame_start_time_ms * 1000);
                [t_chirp,f_chirp] = obj.generate_chirp_f_over_Tsampling_vals(chirp_start_time_us);

                if(size(t_chirp,2) ~= t_size)
                    difference = size(t_chirp,2) - t_size;
                    if(difference > 0)
                        t = [t,zeros(1,difference)];
                        f = [f,zeros(1,difference)];
                    else
                        t = t(1:end + difference); %will reduce its size
                        f = f(1:end + difference);
                    end
                    
                    t(next_start_index: next_start_index + t_size + difference - 1) = t_chirp;
                    f(next_start_index: next_start_index + t_size + difference - 1) = f_chirp;
                    next_start_index = next_start_index + t_size + difference;
                else
                    t(next_start_index: next_start_index + t_size - 1) = t_chirp;
                    f(next_start_index: next_start_index + t_size - 1) = f_chirp;
                    next_start_index = next_start_index + t_size;
                end
            end
            t = t/1000;
        end

     end
end