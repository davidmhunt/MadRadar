classdef Subsystem_spectrum_sensing < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
        %variable to link the spectrum sensing object to its respective
        %attacker object
        Attacker

        %parameter to keep track of the sampling rate
        FMCW_sample_rate_Msps

        %variable to track the current state
        state % potential states: "Measuring Noise","Waiting for Chirp","Sampling Chirp","Processing"

        %struct to hold buffers for streaming the received signal
        rx_signal_buffer %buffer to store a sample of the signal once the first chirp has been received
        rx_temp_buffer % 2 row buffer to temporarially store samples in while waiting a chirp event to occue
        rx_buffer_params

        %struct to hold the detection parameters
        detection_params
        detection_start_time %to track the time that the first sample of a detection starts at
        
        %variables to keep track of timing
        num_samples_streamed %specifies the number of samples that have previously been streamed
        processing_delay_samples %specifies the number of samples to stream (simulating processing delay)
        current_processing_delay %variable to track the current number of samples that have occurred while in the "processing" state
        
        %struct to hold all of the spectogram parameters
        spectogram_params

        %structs to hold parameters for chirp and frame tracking
        chirp_tracking
        frame_tracking

        %structs for peak detection when evaluating the spectograms
        peak_detection_params

        %structs for clustering and linear model detection
        clustering_params
        
        %struct for adding a debugging capability
        Debugger

        %parameters used for plotting if desired
        plot_params
        
        %other support variables used by the sensing subsystem
        spectogram_points
%         previous_spectogram_points
        sampling_window_count

        %other variables used to log progress and validate the simulink
        %model
        received_signal
        reshaped_signal
        reshaped_signal_for_fft
        windowed_signal
        generated_spectogram
%         combined_spectogram
        detected_times
        detected_frequencies
        detected_chirps
        
        %variables used by the clustering algorithm
        idx
        corepts




    end

    methods
        function obj = Subsystem_spectrum_sensing(Attacker)
            obj.Attacker = Attacker;
        end

        function initialize_spectrum_sensing_parameters(obj,FMCW_sample_rate_Msps)
        %{
            Purpose: configures all of the spectrum_sensing parameters
                except for the debugger (done separately as this is a separate
                functionality)
            Inputs: 
                FMCW_sample_rate_Msps: the sampling rate of the spectrum
                    sensing subsystem
        %}
            %set the FMCW sampling rate
            obj.FMCW_sample_rate_Msps = FMCW_sample_rate_Msps;

            %initialize rx_buffers for energy detection
            min_recording_time_ms = 5;
            obj.configure_rx_buffers(obj.FMCW_sample_rate_Msps,min_recording_time_ms);
            obj.state = "Measuring Noise"; 

            %initialize the remaining parameters
            obj.initialize_detection_params();
            min_processing_delay_ms = 90;%20 ms for 4GHz - 100 MHz, check frame rate for 20 MHz configurations
            obj.initialize_timing_params(min_processing_delay_ms);
            obj.initialize_spectogram_params(FMCW_sample_rate_Msps);
            obj.initialize_chirp_and_frame_tracking();
            obj.initialize_plot_params(FMCW_sample_rate_Msps);
            obj.initialize_peak_detection_params();
            obj.initialize_clustering_params();

            %initialize the other miscellaneous parameters
            %initialize array to hold the current and previously measured spectrum
            obj.spectogram_points = [];
%             obj.previous_spectogram_points = [];
%             obj.combined_spectogram = [];
            obj.detected_times = [];
            obj.detected_times = [];
            obj.detected_chirps = zeros(obj.clustering_params.max_num_clusters,2);
            
            %initialize counter to keep track of how many full iterations of the while loop
            %have occurred
            obj.sampling_window_count = 0;
        end

        function configure_rx_buffers(obj, FMCW_sample_rate_Msps, min_recording_time_ms)
            %Adjust buffer size based on FMCW sampling rate
            if FMCW_sample_rate_Msps <= 50
                samples_per_buffer = 2040;
            elseif FMCW_sample_rate_Msps <= 100
                samples_per_buffer = 5000;
            elseif FMCW_sample_rate_Msps <= 500
                samples_per_buffer = 1e4;
            else
                samples_per_buffer = 2e4;
            end

            
            %initialize the rx_buffer parameters
                %specify samples per buffer
                obj.rx_buffer_params.samples_per_buffer = samples_per_buffer; %similar to value on USRPB210

                %determine number of rows needed to meet minimum recording
                %time
                row_period_s = (obj.rx_buffer_params.samples_per_buffer/FMCW_sample_rate_Msps) * 1e-6;
                obj.rx_buffer_params.num_rows = ceil((min_recording_time_ms * 1e-3) / row_period_s);

                %varialbes to track indicies when "sampling" a received
                %signal
                obj.rx_buffer_params.next_sample_index = 1; %next sample index (column index)
                obj.rx_buffer_params.current_row_index = 1; %current row index

                %variable to track the current rx_temp buffer (1st row or
                %2nd row)
                obj.rx_buffer_params.current_temp_buff_row = 1;

                %boolean to determine if the current buffer (i.e: the
                %current row) has been filled
                obj.rx_buffer_params.buffer_full = false;

            %initialize the rx_buffer
            obj.rx_signal_buffer = zeros(obj.rx_buffer_params.num_rows,obj.rx_buffer_params.samples_per_buffer);

            %initialize the rx_temp buffer - when the buffer is in "waiting
            %for chirp mode" it will continuously sample the signal until a
            %threshold is reached
            obj.rx_temp_buffer = zeros(2,obj.rx_buffer_params.samples_per_buffer);
        end

        function initialize_detection_params(obj)
            %initialize a variable to keep track of the relative noise
            %power
            obj.detection_params.relative_noise_power = 0;

            %specify an amount (in dB) that a received signal must be
            %greater than the noise level, before the sensing subsystem
            %starts to record samples
            obj.detection_params.threshold_level = -3;
        end
       
        function initialize_timing_params(obj,min_processing_time_ms)
            %{
                Purpose: initializes the parameters used to keep track of
                    timing in the sensing subsystem
                Inputs: 
                    min_processing_time_ms: the minimum processing time that the sensing
                        subsystem must while in the "processing" state before
                        transitioning to another state
            %}
            obj.num_samples_streamed = 0; 
            obj.processing_delay_samples = ceil(min_processing_time_ms * 1e-3 * obj.FMCW_sample_rate_Msps * 1e6);
            obj.current_processing_delay = 0;
        end
        
        function  initialize_spectogram_params(obj,FMCW_sample_rate_Msps)
            %{
                Purpose: initializes the default settings for the
                    spectogram_params property of the class
                Inputs: 
                    FMCW_sample_rate_Msps: the sampling rate of the
                        spectrum sensing module in MSPS
            %}

            %sample frequency every x us, 2 for lower BW, 0.5 for high BW (1GHz) 
            if obj.FMCW_sample_rate_Msps > 500
                obj.spectogram_params.freq_sampling_period_us = 0.5;
            else
                USRP = true;
                if USRP
                    obj.spectogram_params.freq_sampling_period_us = 15;
                else
                    obj.spectogram_params.freq_sampling_period_us = 2;
                end
            end 
            obj.spectogram_params.num_samples_per_sampling_window = ...
                ceil(obj.spectogram_params.freq_sampling_period_us * FMCW_sample_rate_Msps);
            
            %scale the fft size to be as big of a power of 2 as possible
            obj.spectogram_params.fft_size = 2^(nextpow2(...
                obj.spectogram_params.num_samples_per_sampling_window) - 1);

            %since the frequency sampling period may not have been
            %perfectly divisible by the FMCW sampling rate, adjust it so
            %that it is now divisible
            obj.spectogram_params.freq_sampling_period_us = ... 
                obj.spectogram_params.num_samples_per_sampling_window / FMCW_sample_rate_Msps;

            %continue setting the parameters
%             obj.spectogram_params.num_freq_spectrum_samples_per_spectogram = 25;
%             obj.spectogram_params.num_ADC_samples_per_spectogram = ...
%                 obj.spectogram_params.num_freq_spectrum_samples_per_spectogram * ...
%                 obj.spectogram_params.num_samples_per_sampling_window;
            
            %added a computation for the timing offset when computing the point values
            obj.spectogram_params.detected_time_offset = ...
                obj.spectogram_params.freq_sampling_period_us * ...
                obj.spectogram_params.fft_size / 2 /...
                obj.spectogram_params.num_samples_per_sampling_window;
            
            
            %compute the window to be used when generating the spectogram
            obj.spectogram_params.window = hann(obj.spectogram_params.fft_size);
        end
    
        function initialize_debugger(obj,debugger_enabled,Victim,frames_to_compute)
            %{
                Purpose: initializes the debugger if desired
                Inputs:
                    debugger_enabled: logical value determining if the
                        debugger is enabled (1 is enabled)
                    Victim: the vitim used in the simulation in order to
                        obtain "ground truth" information
                    frames_to_compute: how many frames will be computed in
                        the simulation
            %}
            
            obj.Debugger.enabled = debugger_enabled;

            %variables to keep track of the detected times and frequencies from the
            %individual points taken from the spectogram
            obj.Debugger.actual_num_chirps_per_frame = Victim.NumChirps;
            obj.Debugger.actual_ChirpCycleTime_us = Victim.ChirpCycleTime_us;
            obj.Debugger.detected_times = zeros(frames_to_compute * obj.Debugger.actual_num_chirps_per_frame,...
                    2 + 5 * ceil(obj.Debugger.actual_ChirpCycleTime_us / obj.spectogram_params.freq_sampling_period_us));
            obj.Debugger.detected_frequencies = zeros(frames_to_compute * obj.Debugger.actual_num_chirps_per_frame,...
                    2 + 5 * ceil(obj.Debugger.actual_ChirpCycleTime_us / obj.spectogram_params.freq_sampling_period_us));
            obj.Debugger.detected_chirp_slopes = zeros(frames_to_compute * obj.Debugger.actual_num_chirps_per_frame, 3);
            obj.Debugger.detected_chirp_intercepts = zeros(frames_to_compute * Victim.NumChirps, 3);
            
            %variables to keep track of the errors for the detected times, detected
            %frequencies, computed slope, and computed intercepts
            obj.Debugger.detected_times_errors = zeros(frames_to_compute * obj.Debugger.actual_num_chirps_per_frame,...
                    2 + 5 * ceil(obj.Debugger.actual_ChirpCycleTime_us / obj.spectogram_params.freq_sampling_period_us));
            obj.Debugger.detected_frequencies_errors = zeros(frames_to_compute * obj.Debugger.actual_num_chirps_per_frame,...
                    2 + 5 * ceil(obj.Debugger.actual_ChirpCycleTime_us / obj.spectogram_params.freq_sampling_period_us));
            obj.Debugger.detected_chirp_slopes_errors = zeros(frames_to_compute * obj.Debugger.actual_num_chirps_per_frame, 3);
            obj.Debugger.detected_chirp_intercepts_errors = zeros(frames_to_compute * obj.Debugger.actual_num_chirps_per_frame, 3);
            
            %variables to capture a summary of the errors - containing mean, variance,
            %and MSE
            obj.Debugger.detected_times_errors_summary = zeros(1,3);
            obj.Debugger.detected_frequencies_errors_summary = zeros(1,3);
            obj.Debugger.detected_chirp_slopes_errors_summary = zeros(1,3);
            obj.Debugger.detected_chirp_intercepts_errors_summary = zeros(1,3);
        end
    
        function initialize_chirp_and_frame_tracking(obj)
            %{
                Purpose: initializes the properties of the sensing
                subsystem to track specific chirp and frame parameters
            %}
            
            %initialize array to track captured chirps
            obj.chirp_tracking.captured_chirps_buffer_size = 128;        %this will be the maximum number of chirps captured when computing the average chirp duration
            obj.chirp_tracking.num_captured_chirps = 0;
            obj.chirp_tracking.captured_chirps = zeros(obj.chirp_tracking.captured_chirps_buffer_size, 2);
            obj.chirp_tracking.average_chirp_duration = 0;
            obj.chirp_tracking.average_slope = 0;
            obj.chirp_tracking.difference_threshold_us = 5;             %this is used as the threshold for detecting if a chirp has previously been detected
            obj.chirp_tracking.new_chirp_threshold_us = 5;              %this parameter is used to determine if a chirp has been seen before.

            %initialize array to track captured frames
            obj.frame_tracking.captured_frames_buffer_size = 128;
            obj.frame_tracking.num_captured_frames = 0;
            obj.frame_tracking.captured_frames = zeros(obj.frame_tracking.captured_frames_buffer_size,7); %duration, number of chirps, average slope, average chirp duration, start time, sampling window count
            obj.frame_tracking.average_frame_duration = 0;
            obj.frame_tracking.average_chirp_duration = 0;
            obj.frame_tracking.average_slope = 0;
            obj.frame_tracking.new_frame_threshold_us = 10;             
            %the above variable is the threshold used for detecting if a chirp is part 
            % of the current frame or the next frame. Basically, if a new
            %chirp has a duration that is different by this amount when 
            %compared with the average, it is designated a new chirp
        end
    
        function initialize_plot_params(obj,FMCW_sample_rate_Msps)
            %{
                Purpose: initialize the plot_params struct property of the
                spectrum sensing part of the attacker. This property holds
                useful values that will be used to plot different
                structures
            %}
            %if assuming complex sampling
            obj.plot_params.max_freq = FMCW_sample_rate_Msps; %assuming complex sampling
            obj.plot_params.freq_resolution = FMCW_sample_rate_Msps/obj.spectogram_params.fft_size;

            %if not assuming complex sampling
%             obj.plot_params.max_freq = 2 * FMCW_sample_rate_Msps; %assuming complex sampling
%             obj.plot_params.freq_resolution = FMCW_sample_rate_Msps/obj.spectogram_params.fft_size;

            obj.plot_params.frequencies = 0:obj.plot_params.freq_resolution:obj.plot_params.max_freq - obj.plot_params.freq_resolution;
            %obj.plot_params.times is initialized when the spectogram is
            %computed (allows for increased flexibility)

%             obj.plot_params.times = (0: obj.spectogram_params.freq_sampling_period_us : ...
%                 obj.spectogram_params.num_freq_spectrum_samples_per_spectogram * ...
%                 obj.spectogram_params.freq_sampling_period_us - obj.spectogram_params.freq_sampling_period_us)...
%                 + obj.spectogram_params.detected_time_offset;
            
            %compute times for the combined_spectogram
%             obj.plot_params.combined_spectogram_times = (0: obj.spectogram_params.freq_sampling_period_us : ...
%                 2 * obj.spectogram_params.num_freq_spectrum_samples_per_spectogram * ...
%                 obj.spectogram_params.freq_sampling_period_us - obj.spectogram_params.freq_sampling_period_us) + ...
%                 obj.spectogram_params.detected_time_offset;
        end
    
        function initialize_peak_detection_params(obj)
            obj.peak_detection_params.threshold = 7;    %threshold height below the peak value to detect a point in the spectogram
            obj.peak_detection_params.numPeaks = 2;     %maximum number of peaks for each spectogram window   
        end
        
        function initialize_clustering_params(obj)
            %{
                Purpose: initializes parameters used for the clustering
                algorithm
            %}
            obj.clustering_params.max_num_clusters = 10;
            obj.clustering_params.min_num_points_per_cluster = 5;
        end

        %functions for processing the received signal
        function receive_signal(obj,signal)
            %{
                Purpose: simulate the spectrum sensing subsystem receiving
                    the signal from the victim radar (already amplified by
                    the attacker's receiver)
                Inputs: the raw received signal (from the receiver of the
                    sensing subsystem)
            %}
            import_complete = false;
            next_signal_index = 1;

            %figure out a way to track the timing in this function

            while ~import_complete
                switch obj.state
                    case "Measuring Noise"
                        [next_signal_index,import_complete] = obj.load_signal_into_buffer(signal,next_signal_index);
                        if obj.rx_buffer_params.buffer_full

                            %set the relative noise power
                            obj.detection_params.relative_noise_power = obj.compute_signal_power(reshape(obj.rx_signal_buffer.',1,[]));

                            %change the state to "processing" to simulate
                            %needing to wait for a certain amount of time
                            obj.state = "Processing";
                            obj.current_processing_delay = 0;
                        end
    
                    case"Waiting for Chirp"
                        %create a new function to handle the temporary
                        %buffer behavior
                        [next_signal_index,import_complete,detected_chirp] = obj.check_for_chirp(signal,next_signal_index);
                        if detected_chirp
                            obj.state = "Sampling Chirp";
                            obj.detection_start_time = ((obj.num_samples_streamed - 2*obj.rx_buffer_params.samples_per_buffer)...
                                / (obj.FMCW_sample_rate_Msps * 1e6)) * 1e6; %convert to us

                            %update the start time by factoring in the
                            %distance that the signal propogated
                            distance_delay = range2time(obj.Attacker.current_victim_pos,physconst("LightSpeed"))/2 * 1e6;
                            obj.detection_start_time = obj.detection_start_time - distance_delay;
                        end

                    case "Sampling Chirp"
                        %the buffer should already be initialized, I should
                        %be able to use the same function to load the
                        %signal into the buffer
                        [next_signal_index,import_complete] = obj.load_signal_into_buffer(signal,next_signal_index);
                        if obj.rx_buffer_params.buffer_full
                            %process the sampled signal
                            obj.process_sampled_signal();
                            obj.send_victim_parameters_to_attack_subsystem();
                            %change the state to "processing" to simulate
                            %needing to wait for a certain amount of time
                            obj.state = "Processing";
                            obj.current_processing_delay = 0;
                        end

                    case "Processing"
                       [next_signal_index,import_complete] = obj.simulate_processing(signal,next_signal_index);
                       if obj.current_processing_delay == obj.processing_delay_samples
                           obj.state = "Waiting for Chirp";
                           obj.configure_rx_buffers(obj.FMCW_sample_rate_Msps,2);%specify 2ms for recording samples once triggered
                       end
                    otherwise
                end
            end
        end
        
        function [next_signal_index,import_complete] = load_signal_into_buffer(...
                obj,signal,next_signal_index)
        %{
            Purpose: load as much of an input signal into a buffer as
                    possible, and update the num_samples_streamed variable
                    as well
            Outputs:
                next_signal_index: the index of the next sample to be read
                    in for the signal (if the buffer filled before the full
                    signal could be read)
                import_complete: true if the full signal was read into the
                    buffer
            Inputs:
                signal: the signal to load into the buffer
                next_signal_index: the index of the next sample to be read
                    in for the signal
        %}
            %determine the number of unloaded samples in the buffer and the
            %signal
            unloaded_samples_signal = size(signal,2) - next_signal_index;
            unloaded_samples_buffer = obj.rx_buffer_params.samples_per_buffer - obj.rx_buffer_params.next_sample_index;
            
            %if the buffer has more unloaded samples
            if unloaded_samples_buffer > unloaded_samples_signal
                %compute the start and end coordinates to insert the
                %remaining signal into the buffer
                buff_start = obj.rx_buffer_params.next_sample_index;
                buff_end = buff_start + unloaded_samples_signal;

                %load the signal into the buffer
                obj.rx_signal_buffer(obj.rx_buffer_params.current_row_index,buff_start:buff_end) = signal(next_signal_index:end);
                
                %adjust settings as needed
                import_complete = true;
                next_signal_index = size(signal,2) + 1; %would throw an error, but doing for consistent behavior
                obj.rx_buffer_params.next_sample_index = buff_end + 1;

                obj.num_samples_streamed = obj.num_samples_streamed + unloaded_samples_signal + 1;
            %if the signal has more unloaded samples
            else
                buff_start = obj.rx_buffer_params.next_sample_index;
                sig_end = next_signal_index + unloaded_samples_buffer;

                obj.rx_signal_buffer(obj.rx_buffer_params.current_row_index,buff_start:end) = signal(next_signal_index:sig_end);
                
                if unloaded_samples_signal == unloaded_samples_buffer
                    import_complete = true;
                else
                    import_complete = false;
                end

                obj.rx_buffer_params.next_sample_index = 1;
                next_signal_index = sig_end + 1;

                if obj.rx_buffer_params.current_row_index == obj.rx_buffer_params.num_rows
                    %if the buffer is now filled, reset the row index and
                    %set the buffer full flag to true
                    obj.rx_buffer_params.current_row_index = 1;
                    obj.rx_buffer_params.buffer_full = true;
                else
                    %otherwise just increment the row index
                    obj.rx_buffer_params.current_row_index = obj.rx_buffer_params.current_row_index + 1;
                    obj.rx_buffer_params.buffer_full = false;
                end

                obj.num_samples_streamed = obj.num_samples_streamed + unloaded_samples_buffer + 1;
            end

        end

        function power = compute_signal_power(obj,signal)
            %{
                Purpose: computes the power of a signal
                Inputs: a 1xN complex signal
                Outputs: 
                    power = the signal power in decibels
            %}
            t_period = 1/obj.FMCW_sample_rate_Msps;
            t = 0:t_period:(size(signal,2) - 1) * t_period;
            signal_period = size(signal,2) / (obj.FMCW_sample_rate_Msps * 1e6);
            power = 10 * log10(sum(abs(signal).^2)/signal_period);
            
        end

        function [next_signal_index,import_complete] = simulate_processing( ...
                obj,signal,next_signal_index)
            %{
                Purpose: simulate a processing delay for the sensing
                    subsystem to perform computations
                Outputs:
                    next_signal_index: the index of the next sample to be read
                        in for the signal (if the buffer filled before the full
                        signal could be read)
                    import_complete: true if the full signal was read into the
                        buffer
                Inputs:
                    signal: the signal to load into the buffer
                    next_signal_index: the index of the next sample to be read
                        in for the signal
            %}
            %determine the number of unloaded samples
            unloaded_samples = size(signal,2) - next_signal_index;
            remaining_processing_delay_samples = obj.processing_delay_samples - obj.current_processing_delay;

            if remaining_processing_delay_samples > unloaded_samples
                %if the remaining delay is longer than the number of
                %unloaded samples in the signal

                %update the timing counters
                obj.num_samples_streamed = obj.num_samples_streamed + unloaded_samples + 1;
                obj.current_processing_delay = obj.current_processing_delay + unloaded_samples;

                %specify that import is complete
                next_signal_index = next_signal_index + unloaded_samples + 1;
                import_complete = true;
                
            else
                %if this is the end of the processing delay
                
                %update the timing counters
                obj.num_samples_streamed = obj.num_samples_streamed + remaining_processing_delay_samples + 1;
                obj.current_processing_delay = obj.current_processing_delay + remaining_processing_delay_samples;

                %specify that that the import is not complete
                next_signal_index = next_signal_index + remaining_processing_delay_samples + 1;
                if remaining_processing_delay_samples == unloaded_samples
                    import_complete = true;
                else
                    import_complete = false;
                end
            end
        end

        function [next_signal_index,import_complete,detected_chirp] = check_for_chirp( ...
                obj,signal,next_signal_index)
            %{
                Purpose: loads the signal into a temp buffer and checks to
                see if there is a chirp present in the signal
                Outputs:
                    next_signal_index: the index of the next sample to be read
                        in for the signal (if the buffer filled before the full
                        signal could be read)
                    import_complete: true if the full signal was read into the
                        buffer
                Inputs:
                    signal: the signal to load into the buffer
                    next_signal_index: the index of the next sample to be read
                        in for the signal
            %}

            %load the chirp into the buffer
            %determine the number of unloaded samples in the buffer and the
            %signal
            unloaded_samples_signal = size(signal,2) - next_signal_index;
            unloaded_samples_buffer = obj.rx_buffer_params.samples_per_buffer - obj.rx_buffer_params.next_sample_index;
            
            %if the buffer has more unloaded samples
            if unloaded_samples_buffer > unloaded_samples_signal
                %compute the start and end coordinates to insert the
                %remaining signal into the buffer
                buff_start = obj.rx_buffer_params.next_sample_index;
                buff_end = buff_start + unloaded_samples_signal;

                %load the signal into the buffer
                obj.rx_temp_buffer(obj.rx_buffer_params.current_temp_buff_row,buff_start:buff_end) = signal(next_signal_index:end);
                
                %adjust settings as needed
                import_complete = true;
                next_signal_index = size(signal,2) + 1; %would throw an error, but doing for consistent behavior
                obj.rx_buffer_params.next_sample_index = buff_end + 1;

                %since the buffer wasn't filled, no chirp detected
                detected_chirp = false;
                
                obj.num_samples_streamed = obj.num_samples_streamed + unloaded_samples_signal + 1;
            %if the signal has more unloaded samples, the current temp
            %buffer will be filled
            else
                %fill the temp buffer
                buff_start = obj.rx_buffer_params.next_sample_index;
                sig_end = next_signal_index + unloaded_samples_buffer;

                obj.rx_temp_buffer(obj.rx_buffer_params.current_temp_buff_row,buff_start:end) = signal(next_signal_index:sig_end);
                
                %specify whether or not the full signal has been imported
                if unloaded_samples_signal == unloaded_samples_buffer
                    import_complete = true;
                else
                    import_complete = false;
                end

                obj.rx_buffer_params.next_sample_index = 1;
                next_signal_index = sig_end + 1;

                %compute the signal power level for the current buffer
                sig_power = obj.compute_signal_power(obj.rx_temp_buffer(obj.rx_buffer_params.current_temp_buff_row,:));
                
                %if there is a chirp present, set the detected_chirp flag
                %and load the buffers into rx_buffer
                if sig_power > obj.detection_params.relative_noise_power + obj.detection_params.threshold_level
                    if obj.rx_buffer_params.current_temp_buff_row == 1
                        obj.rx_signal_buffer(1,:) = obj.rx_temp_buffer(2,:);
                        obj.rx_signal_buffer(2,:) = obj.rx_temp_buffer(1,:);
                    else
                        obj.rx_signal_buffer(1,:) = obj.rx_temp_buffer(1,:);
                        obj.rx_signal_buffer(2,:) = obj.rx_temp_buffer(2,:);
                    end
                    
                    %set the row vector to be the third row now
                    obj.rx_buffer_params.current_row_index = 3;
                    obj.rx_buffer_params.buffer_full = false;

                    %set the chirp_detected flag
                    detected_chirp = true;
                else
                    %change the current temp_buffer_row
                    if obj.rx_buffer_params.current_temp_buff_row == 1
                        obj.rx_buffer_params.current_temp_buff_row = 2;
                    else
                        obj.rx_buffer_params.current_temp_buff_row = 1;
                    end
                    detected_chirp = false;
                end

                obj.num_samples_streamed = obj.num_samples_streamed + unloaded_samples_buffer + 1;
            end
        end

        function process_sampled_signal(obj)
            %{
                Purpose: takes in a received signal, generates a spectogram
                
            %}
            obj.received_signal = reshape(obj.rx_signal_buffer.',1,[]);
            obj.generate_spectogram();
            obj.spectogram_points = obj.detect_peaks_in_spectogram(obj.generated_spectogram);

            %combine the detected points with the previously detected points
%                 if ~isempty(obj.spectogram_points)
%                     obj.combined_spectogram = [obj.previous_spectogram_points,obj.spectogram_points...
%                         + [0;obj.spectogram_params.num_freq_spectrum_samples_per_spectogram]];
%                 else
%                     obj.combined_spectogram = obj.previous_spectogram_points;
%                 end
%                 obj.previous_spectogram_points = obj.spectogram_points;
            
            %provided there were detected points in the combined
            %spectogram, continue with the rest of the program
            if ~isempty(obj.generated_spectogram)
                obj.compute_detected_times_and_frequencies(obj.spectogram_points);
                obj.compute_clusters_simplified();
                obj.fit_linear_model();
                obj.compute_victim_parameters();
            end
            obj.sampling_window_count = obj.sampling_window_count + 1;
        end

        function generate_spectogram(obj)
            %{
                Purpose: takes in the received signal (not reshaped yet),
                    reshapes it for fft processing, performs windowing, and
                    generates the spectogram. The updated spectogram is stored
                    in the generated_spectogram property of the
                    spectrum_sensing class
            %}

            %remove a few extra samples at the end of the receive rx buffer
            %(if needed) so that it can be reshaped for fft processing
            received_samples = size(obj.received_signal,2);

            %determine the number of spectrum samples that will be taken
            num_spectrum_samples = floor(received_samples/...
                obj.spectogram_params.num_samples_per_sampling_window);
            
            
            num_samples_per_spectogram = num_spectrum_samples * ...
                obj.spectogram_params.num_samples_per_sampling_window;
            obj.received_signal = obj.received_signal(1:num_samples_per_spectogram);

            %set the plot_params.times variable now that the spectogram
            %size has been determined
            obj.plot_params.times = (0: obj.spectogram_params.freq_sampling_period_us : ...
                num_spectrum_samples * obj.spectogram_params.freq_sampling_period_us...
                - obj.spectogram_params.freq_sampling_period_us)...
                + obj.spectogram_params.detected_time_offset;

            %reshape the received signal for fft processing
            obj.reshaped_signal = reshape(obj.received_signal,...
                obj.spectogram_params.num_samples_per_sampling_window,[]);
            
            %next, shave off the last few samples so that we get the desired fft size
            obj.reshaped_signal_for_fft = obj.reshaped_signal(1:obj.spectogram_params.fft_size,:);
            
            %window the signal
            obj.windowed_signal = obj.reshaped_signal_for_fft .* obj.spectogram_params.window;
            
            %perform an fft
            obj.generated_spectogram = fft(obj.windowed_signal);

            %convert the spectrum to dB
            obj.generated_spectogram = 10*log10(abs(obj.generated_spectogram));
            
            %clip off the negative frequencies - only if not using complex
            %sampling
%            obj.generated_spectogram = obj.generated_spectogram(1:obj.spectogram_params.fft_size/2,:);
        end

        function spectogram_points = detect_peaks_in_spectogram(obj,generated_spectogram)
            %{
                Purpose: for a given generated spectogram, detect the peaks
                    in the spectogram and output an array of indicies for the
                    detected peaks in the spectogram
                Inputs:
                    generated_spectogram: the generated spectogram to
                        identify peaks in
                Outputs:
                    spectogram_points: the indicies of the detected peaks
                        in the spectogram array, first row is the location
                        corresponding to frequency, second row is the index
                        corresponding to time
            %}
            spectogram_points = [];  %array to hold the time, freq locations of valid spectogram points
            
            %determine the maximum value in the spectogram to set the
            %threshold around
            threshold = max(generated_spectogram,[],'all') - obj.peak_detection_params.threshold;


%             for i = 1:size(generated_spectogram,2)
%                 [peaks,locations] = findpeaks(generated_spectogram(:,i),"NPeaks",obj.peak_detection_params.numPeaks);
%                 locations = locations(peaks > threshold);
%                 if ~isempty(locations)
%                     locations = [locations.'; i * ones(1,size(locations,1))];
%                     spectogram_points = [spectogram_points,locations];
%                 end
%             end

            for i = 1:size(generated_spectogram,2)
                [M,I] = max(generated_spectogram(:,i));
                if M > threshold
                    locations = [I; i ];
                    spectogram_points = [spectogram_points,locations];
                end
            end
        end

        function compute_detected_times_and_frequencies(obj,spectogram_points)
            %{
                Purpose: takes the combined spectogram of array indicies in
                    the generated spectogram and converts the indicies to times
                    and frequencies
                Inputs: 
                    spectogram_points: the array of indicies of peaks in
                    the generated spectogram stored in the combined
                    spectogram object
            %}
            obj.detected_frequencies = obj.plot_params.frequencies(spectogram_points(1,:));
            obj.detected_times = obj.plot_params.times(spectogram_points(2,:));
        end
        
        function compute_clusters(obj)
            %{
                ARCHIVED FUNCTION - NO LONGER USED
                Purpose: performs the DBSCAN algorithm to identify each
                individual chirp
            %}

            [obj.idx,obj.corepts] = dbscan([obj.detected_times.',obj.detected_frequencies.'],10,2);
        end

        function compute_clusters_simplified(obj)
            %{
                Purpose: performs the simplified algorithm to identify each
                individual chirp
            %}

            %initialize the idx and corepts outputs
            num_points = size(obj.detected_times,2);
            obj.idx = zeros(num_points,1);
            obj.corepts = zeros(num_points,1);

            %initialize clustering parameters
            min_pts_per_chirp = 5;

            %progress through each point
            chirp = 1;
            
            %declare support variables
            chirp_start_idx = 1;
            num_points_in_chirp = 1; %initialized to one because we start on the 2nd points

            for i = 2:num_points
                %determine if the current point is part of a new chirp or
                %not
                if((obj.detected_frequencies(i) - obj.detected_frequencies(i-1) > -1)...
                        && (obj.detected_frequencies(i) >= 2) )
                    num_points_in_chirp = num_points_in_chirp + 1;
                else
                    %part of a new chirp, make sure there are enough points
                    %to classify it as a new chirp
                    if num_points_in_chirp >= min_pts_per_chirp
                        %update the output arrays
                        obj.idx(chirp_start_idx:i-1) = chirp;
                        obj.corepts(chirp_start_idx:i-1) = 1;

                        %reset support variables for tracking new chirp
                        chirp_start_idx = i;
                        num_points_in_chirp = 1;
                        chirp = chirp + 1;
                    else
                        %update the output arrays to specify that points
                        %aren't a chirp
                        obj.idx(chirp_start_idx:i-1) = -1;
                        obj.corepts(chirp_start_idx:i-1) = 0;

                        %reset support variables for tracking new chirp
                        chirp_start_idx = i;
                        num_points_in_chirp = 1;
                        %don't update the chirp number
                    end
                end
            end
            %check the last chirp
            if num_points_in_chirp >= min_pts_per_chirp
                %update the output arrays
                obj.idx(chirp_start_idx:end) = chirp;
                obj.corepts(chirp_start_idx:end) = 1;
            else
                %update the output arrays to specify that points
                %aren't a chirp
                obj.idx(chirp_start_idx:end) = -1;
                obj.corepts(chirp_start_idx:end) = 0;
            end
        end
        
        function fit_linear_model(obj)
            %{
                Purpose: fits a linear model for each valid cluster
                identified as a chirp
            %}
            obj.detected_chirps = zeros(max(obj.idx),2);
            for cluster_id = 1:max(obj.idx)
                %solve for linear model using linear algrebra
                %y is frequencies, x is time
                if sum(obj.idx == cluster_id) > obj.clustering_params.min_num_points_per_cluster
                    X = [ones(size(obj.detected_times(obj.idx == cluster_id),2),1),obj.detected_times(obj.idx == cluster_id).'];
                    Y = obj.detected_frequencies(obj.idx == cluster_id).';
                    B = inv(X.' * X) * X.' * Y;
                    obj.detected_chirps(cluster_id,1:2) = [-B(1)/B(2),B(2)];
                    slope = obj.detected_chirps(cluster_id,2);
                    intercept = obj.detected_chirps(cluster_id,1);
                end
            end
            obj.remove_invalid_clusters();

%             %option for a simplified linear model
%                     %Now, we experiment with only taking 15 points to determine the slope and
%                     %intercept
%                     num_points_for_linearizing = 15;
%                     detected_chirps_simplified = zeros(max(idx),2);
%                     for cluster_id = 1:max(idx)
%                         if sum(idx == cluster_id) >= num_points_for_linearizing
%                             cluster_times = detected_times(idx == cluster_id);
%                             cluster_freqs = detected_frequencies(idx == cluster_id);
%                             X_simplified = [ones(num_points_for_linearizing,1),cluster_times(1:num_points_for_linearizing).'];
%                             Y_simplified = cluster_freqs(1:num_points_for_linearizing).';
%                             B_simplified = inv(X_simplified.' * X_simplified) * X_simplified.' * Y_simplified;
%                             detected_chirps_simplified(cluster_id,1:2) = [-B_simplified(1)/B_simplified(2),B_simplified(2)];
%                             slope_simplified = detected_chirps_simplified(cluster_id,2);
%                             intercept_simplified = detected_chirps_simplified(cluster_id,1);
%                         end
%                     end
%                     %detected_chirps_simplified - detected_chirps;
%                     %shows that decreasing the number of points affected the computed intercept
%                     %point
        end
        
        function remove_invalid_clusters(obj)
            invalid_points = 0;
            for i = 1:size(obj.detected_chirps,1)
                index = i + invalid_points;
                if index <= size(obj.detected_chirps,1)
                    if obj.detected_chirps(index,1) == 0
                        invalid_points = invalid_points + 1;
                        obj.detected_chirps = ...
                            [obj.detected_chirps(1:index - 1,:);...
                                obj.detected_chirps(index + 1:end,:)];
                    end
                end
            end
        end

        function compute_victim_parameters(obj)

            %compute the actual time that the chirp intercept occured at
            obj.detected_chirps = obj.detected_chirps + [obj.detection_start_time, 0];
            
            %update captured chirps tracking
            for i = 1:size(obj.detected_chirps,1)
                obj.chirp_tracking.captured_chirps(obj.chirp_tracking.num_captured_chirps + 1,:) = ...
                    obj.detected_chirps(i,:);
                obj.chirp_tracking.num_captured_chirps = obj.chirp_tracking.num_captured_chirps + 1;
                obj.debugger_save_detection_points(i);
            end

            %compute average chirp statistics
            obj.chirp_tracking.average_slope = sum(obj.chirp_tracking.captured_chirps(1:obj.chirp_tracking.num_captured_chirps,2))/...
                        double(obj.chirp_tracking.num_captured_chirps);
            obj.chirp_tracking.average_chirp_duration = (obj.chirp_tracking.captured_chirps(obj.chirp_tracking.num_captured_chirps,1)...
                            - obj.chirp_tracking.captured_chirps(1,1))/double(obj.chirp_tracking.num_captured_chirps - 1);

            %increment the frame counter
                obj.frame_tracking.num_captured_frames = obj.frame_tracking.num_captured_frames + 1;

            %save frame information
                %save the duration, number of chirps, average slope, average chirp duration, start time, and sampling window count
                
                obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,2) = obj.chirp_tracking.num_captured_chirps;                          % number of chirps
                obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,3) = obj.chirp_tracking.average_slope;                                % average slope
                obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,4) = obj.chirp_tracking.average_chirp_duration;                       % average_chirp_duration
                obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,5) = obj.chirp_tracking.captured_chirps(1,1);                
                
                %if the attacking subsystem now has a configuration loaded,
                %perform precise timing adjustment on the frame start time
                if obj.Attacker.Subsystem_attacking.configuration_loaded
                    obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,5) =...
                        obj.compute_precise_chirp_start_time(...
                        obj.chirp_tracking.captured_chirps(1,1));
                end
                
                %compute the predicted time for the next chirp
                %to occur on.
                if obj.frame_tracking.num_captured_frames > 1
                    %compute frame duration
                    obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,1) =...
                        obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,5) - ...
                        obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames - 1,5);
                    %compute the average frame duration
                    obj.frame_tracking.average_frame_duration = (obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,5) -...
                        obj.frame_tracking.captured_frames(1,5)) / (obj.frame_tracking.num_captured_frames - 1);
                    %predict next frame
                    obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,7) = ...
                                obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,5)...
                                + obj.frame_tracking.average_frame_duration;
                end
                    
            %compute the overall averages for chirp slope
            %and duration
            obj.frame_tracking.average_chirp_duration =  ...
                sum(obj.frame_tracking.captured_frames(:,4) .* (obj.frame_tracking.captured_frames(:,2) - 1))/ ...
                sum((obj.frame_tracking.captured_frames(obj.frame_tracking.captured_frames(:,2) ~=0,2) - 1));
            obj.frame_tracking.average_slope = ...
                sum(obj.frame_tracking.captured_frames(:,3) .* (obj.frame_tracking.captured_frames(:,2) - 1))/ ...
                sum((obj.frame_tracking.captured_frames(obj.frame_tracking.captured_frames(:,2) ~=0,2) - 1));
                
            %reset captured chirps tracking
            obj.chirp_tracking.num_captured_chirps = 0;
            obj.chirp_tracking.captured_chirps = zeros(obj.chirp_tracking.captured_chirps_buffer_size, 2);
            obj.chirp_tracking.average_chirp_duration = 0;
            obj.chirp_tracking.average_slope = 0;
        end
        
        function send_victim_parameters_to_attack_subsystem(obj)
            %{
                Purpose: sends the victim parameters to the attacking
                subsystem so that it can then attack the victim
            %}

            if obj.frame_tracking.num_captured_frames > 3
                
                %determine how many chirps the attacker should transmit,
                %keeping in mind that it may not have captured all of the
                %chirps
                chirps_to_compute = 0;
                num_captured_chirps = obj.frame_tracking.captured_frames(...
                    obj.frame_tracking.num_captured_frames,2);
                chirps_to_compute = 256;
                
                %send information to the attacking subsystem
                obj.Attacker.Subsystem_attacking.compute_calculated_values(...
                    obj.frame_tracking.average_chirp_duration, ...
                    obj.frame_tracking.average_slope,...
                    obj.frame_tracking.average_frame_duration * 1e-3, ...
                    chirps_to_compute)

                %get the time of the next frame
                next_frame_start_ms = obj.frame_tracking.captured_frames(obj.frame_tracking.num_captured_frames,7) * 1e-3;
                obj.Attacker.Subsystem_attacking.load_frame_start_time(next_frame_start_ms);
            end
        end
        
        function precise_start_time = compute_precise_chirp_start_time(...
                obj,estimated_start_time)
            %{
                Purpose: for the given set of detected chirps, computes a
                    more precise chirp start time using a cross-correlation
                    operation and returns a more precise start time for the
                    chirp
                Inputs:
                    estimated_start_time: the estimated start time of the
                        desired chirp
                Outputs:
                    precise_start_time: the more precise start time for the
                    specified chirp in us
                Note: Function requires the use of the chirp_tracking
                object and so must be called while this is populated
            %}
            
            %full window size to use for cross-correlation
            max_lag = 400;
            observation_window_time_us = 10;
            observation_window_samples = 2 * ceil(0.5 * observation_window_time_us *...
                obj.FMCW_sample_rate_Msps); %make sure it is an even number
            
            time_before_start_us = min(1,estimated_start_time - obj.detection_start_time - 0.1);
            time_before_start_samples = ceil(time_before_start_us *...
                obj.FMCW_sample_rate_Msps);

            %obtain a specific sample for the start of the chirp that we
            %seek to estimate
            chirp_start_time = estimated_start_time;

            chirp_start_sample = round((chirp_start_time - obj.detection_start_time) ...
                * obj.FMCW_sample_rate_Msps);

            start_index = int32(chirp_start_sample - time_before_start_samples);
            end_index = int32(chirp_start_sample + (observation_window_samples - time_before_start_samples));

            estimated_chirp = obj.received_signal(start_index:end_index);

            %obtain the same-sized sample of the already computed estimated
            %chirp
            if time_before_start_samples >= 0
                padded_zeros = zeros(time_before_start_samples,1);
                end_index = int32(observation_window_samples - time_before_start_samples);
                computed_chirp = [padded_zeros;...
                    obj.Attacker.Subsystem_attacking.chirp_waveform(1:end_index + 1).'];
            else
                start_index = int32(1 - time_before_start_samples);
                end_index = int32(observation_window_samples - time_before_start_samples);
                %check to make sure that the end_index isn't greater than
                %size of the chirp waveform
                if end_index > (size(obj.Attacker.Subsystem_attacking.chirp_waveform,2) - 1)
                    end_index = size(obj.Attacker.Subsystem_attacking.chirp_waveform,2) - 1;
                    %adjust the estimated_chirp sample accordingly
                    estimated_chirp = estimated_chirp(1:end_index-start_index + 2);
                end
                computed_chirp = obj.Attacker.Subsystem_attacking.chirp_waveform(start_index:end_index +1).';
            end
            

            %perform cross correlation and convert into a timing offset
            [C,lag] = xcorr(estimated_chirp,computed_chirp,"normalized",max_lag);
%             clf;
%             plot(lag,abs(C));
%             xlabel("Delay in samples")
%             title("Cross Correlation")

            [M,I] = max(abs(C));
            delay_samps = lag(I);

            delay_us = delay_samps / obj.FMCW_sample_rate_Msps;
            
            precise_start_time_samples = chirp_start_sample + delay_samps;
            precise_start_time = precise_start_time_samples / ...
                obj.FMCW_sample_rate_Msps + obj.detection_start_time;
            
        end

        function precise_start_time = compute_precise_chirp_start_time_fft(...
                obj,estimated_start_time)
            %{
                Purpose: for the given set of detected chirps, computes a
                    more precise chirp start time using a cross-correlation
                    operation and returns a more precise start time for the
                    chirp
                Inputs:
                    estimated_start_time: the estimated start time of the
                        desired chirp
                    chirp: the number of the chirp to obtain precise timing
                        on
                Outputs:
                    precise_start_time: the more precise start time for the
                    specified chirp in us
                Note: Function requires the use of the chirp_tracking
                object and so must be called while this is populated
            %}
            
            %full window size to use for cross-correlation
            observation_window_time_us = 10;
            observation_window_samples = 2 * ceil(0.5 * observation_window_time_us *...
                obj.FMCW_sample_rate_Msps); %make sure it is an even number
            
            fft_size = 2^(nextpow2(observation_window_samples) - 1);
            %if assuming complex sampling
            max_freq = obj.FMCW_sample_rate_Msps; %assuming complex sampling
            freq_resolution = obj.FMCW_sample_rate_Msps/fft_size;

            frequencies = -max_freq/2:freq_resolution:max_freq/2 - freq_resolution;

            %obtain a specific sample for the start of the chirp that we
            %seek to estimate
            chirp_start_time = estimated_start_time;

            chirp_start_sample = round((chirp_start_time - obj.detection_start_time) ...
                * obj.FMCW_sample_rate_Msps);

            start_index = int32(chirp_start_sample);
            end_index = int32(chirp_start_sample + (fft_size) - 1);

            estimated_chirp = obj.received_signal(start_index:end_index).';

            %obtain the same-sized sample of the already computed estimated
            %chirp
            computed_chirp = obj.Attacker.Subsystem_attacking.chirp_waveform(1:fft_size).';

            %perform cross correlation and convert into a timing offset
            y = fftshift(fft(dechirp(estimated_chirp,computed_chirp)));
%             clf;
%             plot(lag,abs(C));
%             xlabel("Delay in samples")
%             title("Cross Correlation")

            [M,I] = max(abs(y));
            if_freq = frequencies(I);

            delay_us = if_freq / obj.frame_tracking.average_slope;

            delay_samps = delay_us * obj.FMCW_sample_rate_Msps;
            
            precise_start_time_samples = chirp_start_sample + delay_samps;
            precise_start_time = precise_start_time_samples / ...
                obj.FMCW_sample_rate_Msps + obj.detection_start_time;
            
        end
        
        %debugger functions - These should eventually get updated, but for
        %now they are sufficient as is
        function debugger_save_detection_points(obj,detected_chirp_index)
        %{
            Purpose: updates the debugger object (if enabled)
            Inputs:
                detected_chirp_index - the cluster I.D (in the idx) for the whose chirp whose values are being stored
        %}
            if obj.Debugger.enabled
                
                %the first frame that the sensing subsystem detects will
                %actually be the 2nd frame that it receives (the first was
                %used to compute the relative noise floor)

                current_frame = obj.frame_tracking.num_captured_frames + 1;
                debugger_index = current_frame * obj.Debugger.actual_num_chirps_per_frame + obj.chirp_tracking.num_captured_chirps;
                timing_offset = obj.detection_start_time;
        
                %save the detected times
                obj.Debugger.detected_times(debugger_index,1: 2 + size(obj.idx(obj.idx == detected_chirp_index),1)) = ...
                    [obj.frame_tracking.num_captured_frames + 1,obj.chirp_tracking.num_captured_chirps, ...
                    timing_offset + obj.detected_times(obj.idx == detected_chirp_index)];
                %save the detected frequencies
                obj.Debugger.detected_frequencies(debugger_index,1:2 + size(obj.idx(obj.idx == detected_chirp_index),1)) = ...
                    [current_frame + 1,obj.chirp_tracking.num_captured_chirps, obj.detected_frequencies(obj.idx == detected_chirp_index)];
        
                obj.Debugger.detected_chirp_slopes(debugger_index,:) = ...
                    [current_frame + 1,obj.chirp_tracking.num_captured_chirps,obj.detected_chirps(detected_chirp_index,2)];
                obj.Debugger.detected_chirp_intercepts(debugger_index,:) = ...
                    [current_frame + 1,obj.chirp_tracking.num_captured_chirps,obj.detected_chirps(detected_chirp_index,1)];
            end
        end
        
        function debugger_compute_errors(obj,victim)
        %{
            Purpose: compares the measured values with what the actual values
            should be
            Inputs:
                Debugger - the previous debugger object
                victim - the victim object containing relevant frame and chirp
                    parameters
            Outputs: 
                Debugger_updated - the updated debugger object
        %}
            if obj.Debugger.enabled
                %remove any rows in the detected arrays with zeros
                %(undetected for one reason on another)
                obj.Debugger.detected_times = obj.Debugger.detected_times(obj.Debugger.detected_times(:,1) ~= 0,:);
                obj.Debugger.detected_frequencies = obj.Debugger.detected_frequencies(obj.Debugger.detected_frequencies(:,1) ~= 0,:);
                obj.Debugger.detected_chirp_slopes = obj.Debugger.detected_chirp_slopes(obj.Debugger.detected_chirp_slopes(:,1) ~= 0,:);
                obj.Debugger.detected_chirp_intercepts = obj.Debugger.detected_chirp_intercepts(obj.Debugger.detected_chirp_intercepts(:,1) ~= 0,:);
                
                %update the sizes of the error trackers as the size may have
                %changed slightly when computing:
                obj.Debugger.detected_times_errors = zeros(size(obj.Debugger.detected_times));
                obj.Debugger.detected_frequencies_errors = zeros(size(obj.Debugger.detected_frequencies));
                obj.Debugger.detected_chirp_slopes_errors = zeros(size(obj.Debugger.detected_chirp_slopes));
                obj.Debugger.detected_chirp_intercepts_errors = zeros(size(obj.Debugger.detected_chirp_intercepts));
               
                
                %compute the errors
                true_chirp_intercepts = ...
                    (obj.Debugger.detected_chirp_intercepts(:,1) - 1) * victim.FramePeriodicity_ms * 1e3 +...
                    (obj.Debugger.detected_chirp_intercepts(:,2) - 1) * victim.ChirpCycleTime_us + ...
                    victim.IdleTime_us;
        
                %intercept errors
                obj.Debugger.detected_chirp_intercepts_errors(:,1:2) = obj.Debugger.detected_chirp_intercepts(:,1:2);
                obj.Debugger.detected_chirp_intercepts_errors(:,3) = (obj.Debugger.detected_chirp_intercepts(:,3) - true_chirp_intercepts); 
        
                %slope errors
                obj.Debugger.detected_chirp_slopes_errors(:,1:2) = obj.Debugger.detected_chirp_slopes(:,1:2);
                obj.Debugger.detected_chirp_slopes_errors(:,3) = (obj.Debugger.detected_chirp_slopes(:,3) - victim.FrequencySlope_MHz_us);
        
                %error for time points
                obj.Debugger.detected_times_errors(:,1:2) = obj.Debugger.detected_times(:,1:2);
                for i = 1:size(obj.Debugger.detected_times,1)
                    num_detected_times = sum(obj.Debugger.detected_times(i,3:end) ~= 0);
                    true_times = (obj.Debugger.detected_frequencies(i,3:num_detected_times + 2) / ...
                        victim.FrequencySlope_MHz_us) + true_chirp_intercepts(i);
                    obj.Debugger.detected_times_errors(i,3:num_detected_times + 2) = ...
                        obj.Debugger.detected_times(i,3:num_detected_times + 2) - true_times;
                end
                
                %error for frequency points
                obj.Debugger.detected_frequencies_errors(:,1:2) = obj.Debugger.detected_frequencies(:,1:2);
                for i = 1:size(obj.Debugger.detected_frequencies,1)
                    num_detected_frequencies = sum(obj.Debugger.detected_frequencies(i,3:end) ~= 0);
                    true_frequencies = victim.FrequencySlope_MHz_us * ...
                        (obj.Debugger.detected_times(i,3:num_detected_frequencies + 2) - true_chirp_intercepts(i));
                    obj.Debugger.detected_frequencies_errors(i,3:num_detected_frequencies + 2) = ...
                        obj.Debugger.detected_frequencies(i,3:num_detected_frequencies + 2) - true_frequencies;
                end
        
            end
        end
        
        function [summary_table] = debugger_summarize_errors(obj)
            %{
                Purpose: computes the sample mean, sample variance, and mean
                squared error for the slope, intercept, timing, and frequency
                errors
            %}
            %chirp intercepts
            [mean,variance,MSE] = obj.compute_summary_statistics(...
                obj.Debugger.detected_chirp_intercepts_errors, obj.Debugger.detected_chirp_intercepts);
            obj.Debugger.detected_chirp_intercepts_errors_summary(1:3) = [mean,variance,MSE];
            %slope
            [mean,variance,MSE] = obj.compute_summary_statistics(...
                obj.Debugger.detected_chirp_slopes_errors, obj.Debugger.detected_chirp_slopes);
            obj.Debugger.detected_chirp_slopes_errors_summary(1:3) = [mean,variance,MSE];
             %frequency measurement
            [mean,variance,MSE] = obj.compute_summary_statistics(...
                obj.Debugger.detected_frequencies_errors, obj.Debugger.detected_frequencies);
            obj.Debugger.detected_frequencies_errors_summary(1:3) = [mean,variance,MSE];
             %time measurement
            [mean,variance,MSE] = obj.compute_summary_statistics(...
                obj.Debugger.detected_times_errors, obj.Debugger.detected_times);
            obj.Debugger.detected_times_errors_summary(1:3) = [mean,variance,MSE];
        
            %output the table as well
            summary_table = array2table( ...
                [obj.Debugger.detected_times_errors_summary;...
                obj.Debugger.detected_frequencies_errors_summary;...
                obj.Debugger.detected_chirp_slopes_errors_summary;...
                obj.Debugger.detected_chirp_intercepts_errors_summary], ...
                "VariableNames",[ "Mean","Variance","MSE"], ...
                "RowNames",["Detected Times (us)","Detected Frequencies","Computed Slope","Computed Intercept (us)"]);
        end
        
        function [sample_mean,sample_variance,MSE] = compute_summary_statistics(obj,errors_array, debugger_array)
            %{
                Purpose: computes the sample mean, sample variance, and mean
                    squared error for the various errors captured by the debugger
                Inputs: 
                    Errors_array: an array containing the errors computed from the
                        debugger
                    Debugger_array: the array containing the measured values from
                        the debugger
            %}
            valid_intercepts = debugger_array ~= 0;
            valid_intercepts(:,1:2) = 0;
            sample_mean = ...
                sum(errors_array(valid_intercepts),'all') /...
                sum(valid_intercepts,'all');
            sample_variance = ...
                sum((errors_array(valid_intercepts) - sample_mean).^2,'all') /...
                (sum(valid_intercepts,'all')-1);
            MSE = ...
                sum(errors_array(valid_intercepts).^2,'all') /...
                (sum(valid_intercepts,'all'));
        end

        
        %functions to aid in plotting
        function plot_received_spectogram(obj,max_time_to_plot_us)
            %{
                Purpose: Plot the last spectogram computed by the sensing
                subsystem
            %}
            idx_to_plot = obj.plot_params.times < max_time_to_plot_us;
            surf(obj.plot_params.times(idx_to_plot),...
                obj.plot_params.frequencies,...
                obj.generated_spectogram(:,idx_to_plot),"LineStyle","none");
            resp_max = 10*log10(abs(max(obj.generated_spectogram(:,idx_to_plot),[],"all")));
            clim([resp_max-30, resp_max]);
            ylim([0,max(obj.plot_params.frequencies)])
            xlim([0,max(obj.plot_params.times(idx_to_plot))])
            font_size = 16;
            colorbar(gca,"FontSize",font_size)
            colorbar(gca,"Visible","off")
            legend('off')
            set(gcf,'Position',[0 0 450 350])
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            title_str = sprintf('Spectogram');
            title(title_str,"FontSize",font_size);
            xlabel('Time(us)',"FontSize",font_size)
            ylabel('Frequency (MHz)',"FontSize",font_size)
            view([0,90.0])
            print('-r300',"generated_plots/spectrogram",'-dsvg')
            print('-r300',"generated_plots/spectrogram",'-dpng')
        end
    
        function plot_clusters(obj,max_time_to_plot_us)
            %{
                Purpose: generates a plot of the detected chirps
            %}
            idx_to_plot = obj.detected_times < max_time_to_plot_us;
            gscatter(obj.detected_times(idx_to_plot), ...
                obj.detected_frequencies(idx_to_plot), ...
                obj.idx(idx_to_plot));
            font_size = 16;
%             colorbar(gca,"FontSize",font_size)
            legend('off')
            set(gcf,'Position',[0 0 450 350])
            ax = gca;
            ax.FontSize = font_size;
            ax.LineWidth = 2.0;
            title_str = sprintf('Identified Chirps');
            title(title_str,"FontSize",font_size);
            xlabel('Time(us)',"FontSize",font_size)
            ylabel('Frequency (MHz)',"FontSize",font_size)
            view([0,90.0])
            print('-r300',"generated_plots/identified_chirps",'-dsvg')
            print('-r300',"generated_plots/identified_chirps",'-dpng')
        end
    end
end