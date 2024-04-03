%% For use in SIMULINK Model

%% Notes
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

classdef Radar_Signal_Processor_revA < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = public)
        %Radar object associated with this Radar Signal Processing Object
        Radar

        %parameters for the lowpass filter design
        Fp                      %start frequency of pass band
        Fst                     %start of stop pand 
        Ap                      %ripple to allow in the pass band
        Ast                     %attenuation in the stop band
        Fs                      %sampling frequency
        high_pass_filter         %low pass filter object

        %instead of a lowpass filter, using a decimator (used int he
        %simulink model
        FIRDecimator        %dsp.FIRDecimator object
        decimation_factor   %the decimation factor to use

        %parameters used to select the samples corresponding to the sample
        %period
        num_samples_prior_to_sample_period
        num_samples_in_sampling_period

        %parameters for the Range-Doppler Response
        RangeDopplerResponse        %phased.RangeDopplerResponse object
        
        %parameters for the CA - CFAR detector
        CFARDetector2D              %phased.CFARDetector2D object
        guard_region
        training_region
        PFAR
        CUT_indicies
        distance_detection_range
        velocity_detection_range

        %parameters for the DBScan algorithm
        Epsilon
        minpts

        %parameters for the phased.RangeEstimator object
        RangeEstimator                  %phased.RangeEstimator object
        NumEstimatesSource_rng
        NumEstimates_rng
        ClusterInputPort_rng

        %parameters for the phased.DopplerEstimator object
        DopplerEstimator                %phased.DopplerEstimator object
        NumEstimatesSource_dplr
        NumEstimates_dplr
        ClusterInputPort_dplr

        %variables for radar signal processor operation
        radar_cube

        %variables to store results from each frame
        capture_movies = false;
        simplify_plots = false;
        F_rngdop = struct('cdata',[],'colormap',[])
        F_clusters = struct('cdata',[],'colormap',[])
        range_estimates
        velocity_estimates
        tgt_range_lims %so that the video can zoom in on where the target should be
        tgt_velocity_lims %so that the video can zoom in on where the target should be
        tgt_vel_exclusion_region %to remove undesired detections from the visualization

    end

    methods (Access = public)
        function obj = Radar_Signal_Processor_revA(Radar)
            %{
                Purpose: Initialize a new instance of the Radar Signal
                Processing Class that will perform all processing for the
                Radar Class
                Inputs:
                    Radar: a Radar class object that will be associated
                    with this particular radar signal processor
            %}
            obj.Radar = Radar;
        end


        %% [2] Functions to compute calculated radar parameters

        function configure_radar_signal_processor(obj)
            %{
                Purpose: this object configures all of the Radar Signal
                Processing components by calling the other configuration
                functions
            %}
            obj.configure_highpass_filter();
            obj.configure_decimator();
            obj.configure_select_sampled_IF_sig();
            obj.configure_RangeDopplerResponse();
            obj.configure_Range_and_Doppler_Estimators();
            obj.configure_CFAR_detector();
            obj.configure_DB_scan();
            obj.reset_radar_cube();
        end
        
        function configure_highpass_filter(obj)
            %{
                Purpose: function configures the lowpass filter that will
                be used to simulate how the ADC and mixer attenuate higher
                frequency components in a signal
            %}
            
            pass_band_start_freq_Hz = 10 * obj.Radar.Range_Res_m * 2 * ...
                obj.Radar.FrequencySlope_MHz_us * 1e12 / physconst('LightSpeed');
            stop_band_start_freq_Hz = 5 * obj.Radar.Range_Res_m * 2 * ...
                obj.Radar.FrequencySlope_MHz_us * 1e12 / physconst('LightSpeed');

            obj.Fp = pass_band_start_freq_Hz;                                  %start frequency of pass band
            obj.Fst = stop_band_start_freq_Hz;                                 %start of stop band 
            obj.Ap = 0.5;                                       %ripple to allow in the pass band
            obj.Ast = 40;                                       %attenuation in the stop band
            obj.Fs = obj.Radar.FMCW_sampling_rate_Hz;           %sampling frequency of the Radar object
            
            d = fdesign.highpass('Fst,Fp,Ast,Ap',obj.Fst,obj.Fp,obj.Ast,obj.Ap,obj.Fs);
            obj.high_pass_filter = design(d,'butter','MatchExactly','passband');
%             Hd = design(d,'equiripple');
%             fvtool(obj.high_pass_filter)
        end

        function configure_decimator(obj)
            %{
                Purpose: function configures the decimator that will
                be used to simulate how the ADC and mixer attenuate higher
                frequency components in a signal
            %}
            
            obj.FIRDecimator = dsp.FIRDecimator(obj.decimation_factor,designMultirateFIR(1,ceil(obj.decimation_factor/2)));
            %obj.FIRDecimator = dsp.FIRDecimator(obj.decimation_factor,'Auto');
        end

        function configure_select_sampled_IF_sig(obj)
            %{
                Purpose: function configures the variables needed to select
                the sampled IF signal from the fully dechirped signal
            %}
            obj.num_samples_prior_to_sample_period = ...
                int32((obj.Radar.ADC_ValidStartTime_us + obj.Radar.IdleTime_us) *...
                obj.Radar.ADC_SampleRate_MSps);
            obj.num_samples_in_sampling_period = obj.Radar.ADC_Samples;
        end
    
        function configure_RangeDopplerResponse(obj)
            %{
                Purpose: configures the phased.RangeDopplerResponse object
                that is used to compute the range-doppler response
            %}
            obj.RangeDopplerResponse = phased.RangeDopplerResponse(...
                "RangeMethod","FFT",...
                "PropagationSpeed",physconst('Lightspeed'),...
                'SampleRate',obj.Radar.ADC_SampleRate_MSps * 1e6, ...
                'SweepSlope',obj.Radar.FrequencySlope_MHz_us * 1e12,...
                'DechirpInput',false,...
                'RangeFFTLengthSource','Property',...
                'RangeFFTLength',obj.Radar.ADC_Samples,...
                'RangeWindow',"Hann",...
                'ReferenceRangeCentered',false,...
                'ReferenceRange',0.0,...
                'PRFSource','Property',...
                'PRF',1/(obj.Radar.ChirpCycleTime_us * 1e-6),...
                'DopplerFFTLengthSource','Property',...
                'DopplerFFTLength',obj.Radar.NumChirps,...
                'DopplerWindow','Hann',...
                'DopplerOutput','Speed',...
                'OperatingFrequency',obj.Radar.StartFrequency_GHz * 1e9);
        end

        function configure_Range_and_Doppler_Estimators(obj)
            %{
                Purpose: Configures the Phased.RangeEstimator and
                Phased.DopplerEstimator objects that will be used to
                convert cluster ID's into range estimates
            %}

            %set key parameters for the range estimator
            obj.NumEstimatesSource_rng = 'Property';
            obj.NumEstimates_rng = 30; %previously 5
            obj.ClusterInputPort_rng = true;

            %initialize the range estimator
            obj.RangeEstimator = phased.RangeEstimator(...
                "NumEstimatesSource",obj.NumEstimatesSource_rng,...
                "NumEstimates", obj.NumEstimates_rng,...
                "ClusterInputPort",obj.ClusterInputPort_rng);

            %set key parameters for the range estimator
            obj.NumEstimatesSource_dplr = 'Property';
            obj.NumEstimates_dplr = 30; %previously 5
            obj.ClusterInputPort_dplr = true;

            %initialize the range estimator
            obj.DopplerEstimator = phased.DopplerEstimator(...
                "NumEstimatesSource",obj.NumEstimatesSource_dplr,...
                "NumEstimates", obj.NumEstimates_dplr,...
                "ClusterInputPort",obj.ClusterInputPort_dplr);
        end
        
        function configure_CFAR_detector(obj)
            %set PFAR to be 1e-8 for now
            obj.PFAR = 1e-7;
            
            %calculate training region size
            %high BW configurations
%             range_training_size = min(8,ceil(obj.Radar.ADC_Samples * 0.05)); 
%             velocity_training_size = max(3,ceil(obj.Radar.NumChirps * 0.05));
            
            % low bw configurations
            range_training_size = 2;
            velocity_training_size = 3;
            
            %put guard and training region sizes into arrays for the CFAR detector
            obj.guard_region = [2,3]; %another option is [2,1]
            obj.training_region = [range_training_size,velocity_training_size];
            
            %compute the max and min indicies for the cells under test
            max_detected_range_index = obj.Radar.ADC_Samples - obj.guard_region(1) - obj.training_region(1);
            min_detected_range_index = 1 + obj.guard_region(1) + obj.training_region(1);
            max_detected_velocity_index = obj.Radar.NumChirps - obj.guard_region(2) - obj.training_region(2);
            min_detected_velocity_index = 1 + obj.guard_region(2) + obj.training_region(2);
            
            %generate an array of all the indicies to perform a CFAR operation on
            range_indicies = (min_detected_range_index:max_detected_range_index).';
            velocity_indicies = min_detected_velocity_index:max_detected_velocity_index;
            
            CUT_range_indicies = repmat(range_indicies,1, size(velocity_indicies, 2));
            CUT_range_indicies = reshape(CUT_range_indicies,[],1);
            
            CUT_velocity_indicies = repmat(velocity_indicies,size(range_indicies,1),1);
            CUT_velocity_indicies = reshape(CUT_velocity_indicies,[],1);
            
            obj.CUT_indicies = [CUT_range_indicies CUT_velocity_indicies].';      
            
            %compute the observable range for the detector
            max_detected_range = obj.Radar.Ranges(max_detected_range_index);
            min_detected_range = obj.Radar.Ranges(min_detected_range_index);
            max_detected_velocity = obj.Radar.Velocities(max_detected_velocity_index);
            min_detected_velocity = obj.Radar.Velocities(min_detected_velocity_index);
            
            obj.distance_detection_range = [min_detected_range max_detected_range];
            obj.velocity_detection_range = [min_detected_velocity max_detected_velocity];

            obj.CFARDetector2D = phased.CFARDetector2D(...
                'Method','CA',...
                'GuardBandSize',obj.guard_region,...
                'TrainingBandSize',obj.training_region,...
                'ThresholdFactor','Auto',...
                'ProbabilityFalseAlarm',obj.PFAR,...
                'OutputFormat','Detection index',...
                'NumDetectionsSource','Auto');
        end
        
        function configure_DB_scan(obj)
        %{
            Purpose: Set's the values for Epsilon and the minpts for the
            DBScan algorithm used to identify radar clusters
        %}

            obj.Epsilon = 3; %previously 2
            obj.minpts = 2; %was previously 3
        end       
        
        function reset_radar_cube(obj)
            %{
                Purpose: initializes the radar cube
            %}
            obj.radar_cube = zeros(obj.Radar.ADC_Samples,obj.Radar.NumChirps);
        end
        
        function configure_movie_capture(obj,frames_to_capture,capture_movies, ...
                range_lims,vel_lims, vel_exclusion_region,enable_simplified_plots)
            %{
                Purpose: this function configures parameters to save each
                    frame's range-doppler,clustering,range-detections, and
                    velocity detections so that they can be saved and reviewed
                    after the simulation
                Inputs:
                    frames_to_capture: the number of frames to capture
                    capture_movies: a bool on whether or not to capture
                        movies of the range doppler and clustering outputs
                    tgt_range: range of target so movie can zoom in on it
                    tgt_vel: velocity of target so movie can zoom in on it
            %}
            obj.F_rngdop(frames_to_capture) = struct('cdata',[],'colormap',[]);
            obj.F_clusters(frames_to_capture) = struct('cdata',[],'colormap',[]);
            obj.capture_movies = capture_movies;
            obj.tgt_range_lims = range_lims;
            obj.tgt_velocity_lims = vel_lims;
            obj.tgt_vel_exclusion_region = vel_exclusion_region;
            obj.simplify_plots = enable_simplified_plots;
        end
        
        %% [3] Functions for processing the signals
        
        function update_radar_cube(obj,Tx_sig,Rx_sig,chirp)
            %{
                Purpose: updates the radar cube at the specific chirp index
                    using the provided Tx and Rx signals (also computes the IF
                    and sampled signal as well)
                Input:
                    Tx_sig: the transmitted signal (chirp)
                    Rx_sig: the received signal (reflected or otherwise)
                    chirp: the index in the radar cube to update
            %}
            
            %assemble the radar cube
            obj.radar_cube(:,chirp) = obj.FMCW_dechirp_and_decimate(Tx_sig,Rx_sig);
        end

        function sampled_IF_sig = FMCW_dechirp_and_decimate(obj,Tx_sig,Rx_sig)
            %{
                Purpose: simulates a received signal going through a mixer
                    and then getting sampled by the defender's ADC at its ADC
                    sampling frequency. 
                Inputs:
                    Tx_sig: the waveform of the signal transmitted by
                        the defender
                    Rx_sig: the signal that's received by the
                        defender after reflecting off of any targets. Includes
                        any attacker interference as well
                Outputs:
                    sampled_IF_sig: the sampled IF signal that would be
                        recorded by the DCA1000. Represents all of the samples
                        recorded for the given defender chirp
            %}
            %dechirp the received signal
            sampled_IF_sig = dechirp(Rx_sig,Tx_sig);

            %remove DC terms
%             sampled_IF_sig = filter(obj.high_pass_filter,sampled_IF_sig);

            %run the sampled IF signal through a decimator
            sampled_IF_sig = obj.FIRDecimator(sampled_IF_sig);

            %select only the portion of the dechirped signal that was in
            %the sampling period
            sampled_IF_sig = obj.select_sampled_IF_sig(sampled_IF_sig);
            
        end

        function sampled_IF_sig = select_sampled_IF_sig(obj,IF_sig)
            %{
                Purpose: this function selects the portion of the IF signal
                that occurs during the sampling period
            %}
            sampled_IF_sig = IF_sig(...
                obj.num_samples_prior_to_sample_period + 1:...
                obj.num_samples_prior_to_sample_period + obj.num_samples_in_sampling_period);
        end
        
        function process_radar_cube(obj)
            %{
                Purpose: processes the radar cube and saves videos of the
                range-doppler plot, clusters, and also a list of the
                detections
            %}

            %range-doppler response
            [resp, rnggrid,dopgrid] = obj.RangeDopplerResponse(obj.radar_cube);
            resp_max =  10*log10(abs(max(resp,[],"all").^(2)));
        
            %CA CFAR 2-D
            detections = obj.CFARDetector2D(abs(resp).^2,obj.CUT_indicies);
            detected_velocities = dopgrid(detections(2,:));
            detected_ranges = rnggrid(detections(1,:));
            
            %filtering out non-zero velocity components
            %identify non-zero velocity objects
            valid_idxs = (detected_velocities < obj.tgt_vel_exclusion_region(1)) ...
                        | (detected_velocities > obj.tgt_vel_exclusion_region(2));
            detected_velocities = detected_velocities(valid_idxs);
            detected_ranges = detected_ranges(valid_idxs);
            detections = detections(:,valid_idxs);
            
            fig_position = [0,0,440,350];
            font_size = 18;
            font_size_simplified = 17;
            font_size_colorbar = 15;
            color_upper_limit = -5; %-5 for sim, -10 for USRP
            color_lower_lim = 20;%use 49 for USRP case studies, 30 for Simulations
            c_map = hot();
%             c_map = parula(6);

            %tune colors to make it easier to see lower power levels
%             c_map(1,:) = c_map(1,:) * 0.75;
%             c_map(2,:) = c_map(3,:) * 0.85;
%             c_map(3,:) = c_map(3,:) * 1.1;
%             c_map(4,:) = c_map(4,:) * 1.1;
            
        
            %estimate the range and the velocities
            if ~isempty(detections)

                %DBSCAN Clustering
                idx = dbscan(detections.',obj.Epsilon,obj.minpts);

                obj.range_estimates(obj.Radar.current_frame,:) = obj.RangeEstimator(resp,rnggrid,detections,idx.');
                obj.velocity_estimates(obj.Radar.current_frame,:) = obj.DopplerEstimator(resp,dopgrid,detections,idx.');
                
                %plot the range-doppler and clustering responses if the
                %capture_movies flag is set
                if obj.capture_movies
                    %plot range doppler
                    plotResponse(obj.RangeDopplerResponse,obj.radar_cube);
                    
                    %zoom the range doppler plot
                    colormap(c_map);
                    clim([resp_max-color_lower_lim, resp_max + color_upper_limit]); %previously resp_max - 30
                    xlim(obj.tgt_velocity_lims)
                    ylim(obj.tgt_range_lims)
                    
                    set(gcf,'Position',fig_position)
                    title("Range-Doppler","FontSize",font_size)
                    ax = gca;
                    ax.FontSize = font_size;
                    ax.LineWidth = 2.0;

                    if obj.simplify_plots
                        ax.FontSize = font_size_simplified;
                        xlabel("Velocity (m/s)","FontSize",font_size_simplified)
                        ylabel("Range (m)","FontSize",font_size_simplified)
                    else
                        ax.FontSize = font_size;
                        xlabel("Velocity (m/s)","FontSize",font_size)
                        ylabel("Range (m)","FontSize",font_size)
                    end
                    h = colorbar;
                    h.FontSize = font_size_colorbar;
                    h.Label.String = "Relative Power (dB)";
                    h_label = h.Label;
                    drawnow
                    obj.F_rngdop(obj.Radar.current_frame) = getframe(gcf);
    
                    %plot the clusters
                    gscatter(detected_velocities,detected_ranges,idx);
                    xlim(obj.tgt_velocity_lims)
                    ylim(obj.tgt_range_lims) 
                    
                    legend('off')
                    set(gcf,'Position',fig_position)
                    title("Detections","FontSize",font_size)
                    ax = gca;
                    ax.LineWidth = 2.0;

                    
                    if obj.simplify_plots
                        ax.FontSize = font_size_simplified;
                        xlabel("Velocity (m/s)","FontSize",font_size_simplified)
                        ylabel("Range (m)","FontSize",font_size_simplified)
                    else
                        ax.FontSize = font_size;
                        xlabel("Velocity (m/s)","FontSize",font_size)
                        ylabel("Range (m)","FontSize",font_size)
                    end
                    drawnow
                    obj.F_clusters(obj.Radar.current_frame) = getframe(gcf);
                end
            else
                obj.range_estimates(obj.Radar.current_frame,:) = NaN(1,obj.NumEstimates_rng);
                obj.velocity_estimates(obj.Radar.current_frame,:) = NaN(1,obj.NumEstimates_dplr);
                
                if obj.capture_movies
                    %plot range doppler
                    plotResponse(obj.RangeDopplerResponse,obj.radar_cube);
                    
                    colormap(c_map);
                    clim([resp_max-color_lower_lim, resp_max + color_upper_limit]);
                    xlim(obj.tgt_velocity_lims)
                    ylim(obj.tgt_range_lims)

                    set(gcf,'Position',fig_position)
                    title("Range-Doppler","FontSize",font_size)
                    ax = gca;
                    ax.FontSize = font_size;
                    ax.LineWidth = 2.0;

                    
                    if obj.simplify_plots
                        ax.FontSize = font_size_simplified;
                        xlabel("Velocity (m/s)","FontSize",font_size_simplified)
                        ylabel("Range (m)","FontSize",font_size_simplified)
                    else
                        xlabel("Velocity (m/s)","FontSize",font_size)
                        ylabel("Range (m)","FontSize",font_size)
                        ax.FontSize = font_size;
                    end
                    h = colorbar;
                    h.FontSize = font_size_colorbar;
                    h.Label.String = "Relative Power (dB)";
                    h_label = h.Label;
                    drawnow
                    obj.F_rngdop(obj.Radar.current_frame) = getframe(gcf);
    
                    %plot the clusters
                    clf;
                    detected_velocities = [0];
                    detected_ranges = [0];
                    idx = [1];
                    gscatter(detected_velocities,detected_ranges,idx);
                    xlim(obj.tgt_velocity_lims)
                    ylim(obj.tgt_range_lims)

                    cla;
                    legend('off')
                    set(gcf,'Position',fig_position)
                    title("Detections","FontSize",font_size)
                    xlabel("Velocity (m/s)","FontSize",font_size)
                    ylabel("Range (m)","FontSize",font_size)
                    ax = gca;
                    ax.LineWidth = 2.0;

                    if obj.simplify_plots
                        ax.FontSize = font_size_simplified;
                        xlabel("Velocity (m/s)","FontSize",font_size_simplified)
                        ylabel("Range (m)","FontSize",font_size_simplified)
                    else
                        xlabel("Velocity (m/s)","FontSize",font_size)
                        ylabel("Range (m)","FontSize",font_size)
                        ax.FontSize = font_size;
                    end
                    drawnow;
                    obj.F_clusters(obj.Radar.current_frame) = getframe(gcf);
                end
            end
        end
    
        function play_range_doppler_movie(obj)
           %{
                Purpose: play the range doppler movie at 10fps
           %}
            fig = figure;
            movie(fig,obj.F_rngdop,1,10);
        end

        function play_clustering_movie(obj)
           %{
                Purpose: play the clustering movie at 10fps
           %}
            fig = figure;
            movie(fig,obj.F_clusters,1,10);
        end
    end
end