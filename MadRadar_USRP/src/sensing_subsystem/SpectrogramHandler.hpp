#ifndef SPECTROGRAMHANDLER
#define SPECTROGRAMHANDLER

    //include header files
    #include "pocketfft/pocketfft_hdronly.h"

    //C standard libraries
    #include <iostream>
    #include <cstdlib>
    #include <string>
    #include <complex.h>
    #include <vector>
    #include <csignal>
    #include <thread>

    //to track processing times
    #include <chrono>

    #define _USE_MATH_DEFINES
    #include <cmath>
    #include <algorithm>

    //including buffer handler
    #include "../BufferHandler.hpp"

    //include the JSON handling capability
    #include <nlohmann/json.hpp>

    //include the required headers from EIGEN
    #include "eigen3/Eigen/Dense"

    //include the custom cross correlation class
    #include "CrossCorr.hpp"


    using namespace Buffers;
    using namespace pocketfft;
    using json = nlohmann::json;
    using namespace Eigen;
    using CrossCorr_namespace::CrossCorr;

    //Variable for tracking process times
    using namespace std::chrono;

    namespace SpectrogramHandler_namespace {

        template<typename data_type>
        class SpectrogramHandler
        {
        private:

            json config;
            
            //size parameters
            size_t samples_per_sampling_window;
            size_t fft_size;
            size_t num_rows_rx_signal; //for the received signal
            size_t samples_per_buffer_rx_signal; //the spb for the received signal
            size_t num_rows_spectrogram; //for the reshaped array (in preparation for spectogram)
            size_t num_samples_rx_signal;
            size_t num_samples_per_spectrogram; //for the reshaped spectrogram

            //fft parameters
            shape_t shape;
            stride_t stride;
            shape_t axes;

            //peak_detection_parameters
            data_type peak_detection_threshold;
            data_type spectrogram_absolute_max_val;

            //frequency and timing variables
            double FMCW_sampling_rate_Hz;
            double frequency_resolution;
            double frequency_sampling_period;
            double detected_time_offset;
        public: //public to allow for being saved offline
            std::vector<double> frequencies;
            std::vector<double> times;
        private:
            //clustering parameters
            size_t min_points_per_chirp;
            int max_cluster_index;

            //timing parameters
            double detection_start_time_us;
            const double c = 2.99792458e8;
            size_t frame_tracking_num_captured_frames;
            double frame_tracking_average_frame_duration_us;
            double frame_tracking_average_chirp_duration_us;
            double frame_tracking_average_chirp_slope_MHz_us;



            //precise timing parameters
            size_t xcorr_max_lag;
            double xcorr_observation_window_time_us;
            size_t xcorr_observation_window_samples;
        
        public: //make the buffers and status flags public
            //cross correlation class
            CrossCorr<data_type> cross_corr;
            
            //status flags
            bool attack_in_progress; //status for if an attack is in progress
            bool victim_waveform_loaded;

            //public variable for frame tracking
            size_t max_chirps_to_capture;
            size_t max_frames_to_capture;
            double min_frame_periodicity_s;
            double max_waiting_time;

            //buffers used

                //rx signal buffer
                Buffer_2D<std::complex<data_type>> rx_buffer;

                //reshaped and window buffer (ready for FFT)
                Buffer_2D<std::complex<data_type>> reshaped__and_windowed_signal_for_fft;

                //hanning window
                Buffer_1D<std::complex<data_type>> hanning_window;

                //computed FFT vector
                Buffer_2D<std::complex<data_type>> computed_fft;

                //generated spectrogram
                Buffer_2D<data_type> generated_spectrogram;

                //spectrogram points
                Buffer_1D<data_type> spectrogram_points_values;
                Buffer_1D<size_t> spectrogram_points_indicies;

                //frequency and timing bins
                Buffer_1D<double> detected_times;
                Buffer_1D<double> detected_frequencies;

                //cluster indicies
                Buffer_1D<int> cluster_indicies;

                //buffers for detected chirps
                Buffer_1D<double> detected_slopes_MHz_us;
                Buffer_1D<double> detected_intercepts_us;

                //parameter estimation buffers
                Parameter_Estimation_Buffer<double> slope_MHz_us_estimator_buffer;
                Parameter_Estimation_Buffer<double> chirp_period_us_estimator_buffer;
                Parameter_Estimation_Buffer<double> frame_period_us_estimator_buffer;

                //buffer for tracking victim frames
                Buffer_2D<double> captured_frames; //colums as follows: duration, number of chirps, average slope, average chirp duration, start time, next predicted frame start time

                //buffer containing the victim's estimated waveform
                Buffer_1D<std::complex<data_type>> computed_victim_chirp;

        private:

            //parameter randomization detection
            bool randomization_detected;

            //debug status
            bool debug;
        public:
            //save file path
            std::string save_file_path;

        public: //constructors, destructions, and init() function are public

            /**
             * @brief Construct a new Spectrogram Handler object - DOES NOT INITIALIZE SPECTROGRAM HANDLER
             * 
             */
            SpectrogramHandler(){}
            
            /**
             * @brief Construct a new Spectrogram Handler object using a config file
             * 
             * @param json_config a json object with configuration information 
             */
            SpectrogramHandler(json & json_config){
                init(json_config);
            }

            /**
             * @brief Construct a new Spectrogram Handler object- COPY CONSTRUCTOR
             * 
             * @param rhs existing SpectrogramHandler object
             */
            SpectrogramHandler(const SpectrogramHandler & rhs) : config(rhs.config),
                                                                samples_per_sampling_window(rhs.samples_per_sampling_window),
                                                                fft_size(rhs.fft_size),
                                                                num_rows_rx_signal(rhs.num_rows_rx_signal),
                                                                samples_per_buffer_rx_signal(rhs.samples_per_buffer_rx_signal),
                                                                num_rows_spectrogram(rhs.num_rows_spectrogram),
                                                                num_samples_rx_signal(rhs.num_samples_rx_signal),
                                                                num_samples_per_spectrogram(rhs.num_samples_per_spectrogram),
                                                                shape(rhs.shape),
                                                                stride(rhs.stride),
                                                                axes(rhs.axes),
                                                                peak_detection_threshold(rhs.peak_detection_threshold),
                                                                spectrogram_absolute_max_val(rhs.spectrogram_absolute_max_val),
                                                                FMCW_sampling_rate_Hz(rhs.FMCW_sampling_rate_Hz),
                                                                frequency_resolution(rhs.frequency_resolution),
                                                                frequency_sampling_period(rhs.frequency_sampling_period),
                                                                detected_time_offset(rhs.detected_time_offset),
                                                                frequencies(rhs.frequencies),
                                                                times(rhs.times),
                                                                min_points_per_chirp(rhs.min_points_per_chirp),
                                                                max_cluster_index(rhs.max_cluster_index),
                                                                detection_start_time_us(rhs.detection_start_time_us),
                                                                c(rhs.c),
                                                                frame_tracking_num_captured_frames(rhs.frame_tracking_num_captured_frames),
                                                                frame_tracking_average_frame_duration_us(rhs.frame_tracking_average_frame_duration_us),
                                                                frame_tracking_average_chirp_duration_us(rhs.frame_tracking_average_chirp_duration_us),
                                                                frame_tracking_average_chirp_slope_MHz_us(rhs.frame_tracking_average_chirp_slope_MHz_us),
                                                                xcorr_max_lag(rhs.xcorr_max_lag),
                                                                xcorr_observation_window_time_us(rhs.xcorr_observation_window_time_us),
                                                                xcorr_observation_window_samples(rhs.xcorr_observation_window_samples),
                                                                cross_corr(rhs.cross_corr),
                                                                attack_in_progress(rhs.attack_in_progress),
                                                                victim_waveform_loaded(rhs.victim_waveform_loaded),
                                                                max_chirps_to_capture(rhs.max_chirps_to_capture),
                                                                max_frames_to_capture(rhs.max_frames_to_capture),
                                                                min_frame_periodicity_s(rhs.min_frame_periodicity_s),
                                                                max_waiting_time(rhs.max_waiting_time),
                                                                rx_buffer(rhs.rx_buffer),
                                                                reshaped__and_windowed_signal_for_fft(rhs.reshaped__and_windowed_signal_for_fft),
                                                                hanning_window(rhs.hanning_window),
                                                                computed_fft(rhs.computed_fft),
                                                                generated_spectrogram(rhs.generated_spectrogram),
                                                                spectrogram_points_values(rhs.spectrogram_points_values),
                                                                spectrogram_points_indicies(rhs.spectrogram_points_indicies),
                                                                detected_times(rhs.detected_times),
                                                                detected_frequencies(rhs.detected_frequencies),
                                                                cluster_indicies(rhs.cluster_indicies),
                                                                detected_slopes_MHz_us(rhs.detected_slopes_MHz_us),
                                                                detected_intercepts_us(rhs.detected_intercepts_us),
                                                                slope_MHz_us_estimator_buffer(rhs.slope_MHz_us_estimator_buffer),
                                                                chirp_period_us_estimator_buffer(rhs.chirp_period_us_estimator_buffer),
                                                                frame_period_us_estimator_buffer(rhs.frame_period_us_estimator_buffer),
                                                                captured_frames(rhs.captured_frames),
                                                                computed_victim_chirp(rhs.computed_victim_chirp),
                                                                randomization_detected(rhs.randomization_detected),
                                                                debug(rhs.debug),
                                                                save_file_path(rhs.save_file_path)
                                                                {}
            
            /**
             * @brief Assignment operator
             * 
             * @param rhs existing SpectrogramHandler object
             * @return SpectrogramHandler& 
             */
            SpectrogramHandler & operator=(const SpectrogramHandler & rhs){
                if(this != &rhs)
                {
                    config = rhs.config;
                    samples_per_sampling_window = rhs.samples_per_sampling_window;
                    fft_size = rhs.fft_size;
                    num_rows_rx_signal = rhs.num_rows_rx_signal;
                    samples_per_buffer_rx_signal = rhs.samples_per_buffer_rx_signal;
                    num_rows_spectrogram = rhs.num_rows_spectrogram;
                    num_samples_rx_signal = rhs.num_samples_rx_signal;
                    num_samples_per_spectrogram = rhs.num_samples_per_spectrogram;
                    shape = rhs.shape;
                    stride = rhs.stride;
                    axes = rhs.axes;
                    peak_detection_threshold = rhs.peak_detection_threshold;
                    spectrogram_absolute_max_val = rhs.spectrogram_absolute_max_val;
                    FMCW_sampling_rate_Hz = rhs.FMCW_sampling_rate_Hz;
                    frequency_resolution = rhs.frequency_resolution;
                    frequency_sampling_period = rhs.frequency_sampling_period;
                    detected_time_offset = rhs.detected_time_offset;
                    frequencies = rhs.frequencies;
                    times = rhs.times;
                    min_points_per_chirp = rhs.min_points_per_chirp;
                    max_cluster_index = rhs.max_cluster_index;
                    detection_start_time_us = rhs.detection_start_time_us;
                    c = rhs.c;
                    frame_tracking_num_captured_frames = rhs.frame_tracking_num_captured_frames;
                    frame_tracking_average_frame_duration_us = rhs.frame_tracking_average_frame_duration_us;
                    frame_tracking_average_chirp_duration_us = rhs.frame_tracking_average_chirp_duration_us;
                    frame_tracking_average_chirp_slope_MHz_us = rhs.frame_tracking_average_chirp_slope_MHz_us;
                    xcorr_max_lag = rhs.xcorr_max_lag;
                    xcorr_observation_window_time_us = rhs.xcorr_observation_window_time_us;
                    xcorr_observation_window_samples = rhs.xcorr_observation_window_samples;
                    cross_corr = rhs.cross_corr;
                    attack_in_progress = rhs.attack_in_progress;
                    victim_waveform_loaded = rhs.victim_waveform_loaded;
                    max_chirps_to_capture = rhs.max_chirps_to_capture;
                    max_frames_to_capture = rhs.max_frames_to_capture;
                    min_frame_periodicity_s = rhs.min_frame_periodicity_s;
                    max_waiting_time = rhs.max_waiting_time;
                    rx_buffer = rhs.rx_buffer;
                    reshaped__and_windowed_signal_for_fft = rhs.reshaped__and_windowed_signal_for_fft;
                    hanning_window = rhs.hanning_window;
                    computed_fft = rhs.computed_fft;
                    generated_spectrogram = rhs.generated_spectrogram;
                    spectrogram_points_values = rhs.spectrogram_points_values;
                    spectrogram_points_indicies = rhs.spectrogram_points_indicies;
                    detected_times = rhs.detected_times;
                    detected_frequencies = rhs.detected_frequencies;
                    cluster_indicies = rhs.cluster_indicies;
                    detected_slopes_MHz_us = rhs.detected_slopes_MHz_us;
                    detected_intercepts_us = rhs.detected_intercepts_us;
                    slope_MHz_us_estimator_buffer = rhs.slope_MHz_us_estimator_buffer;
                    chirp_period_us_estimator_buffer = rhs.chirp_period_us_estimator_buffer;
                    frame_period_us_estimator_buffer = rhs.frame_period_us_estimator_buffer;
                    captured_frames = rhs.captured_frames;
                    computed_victim_chirp = rhs.computed_victim_chirp;
                    randomization_detected = rhs.randomization_detected;
                    debug = rhs.debug;
                    save_file_path = rhs.save_file_path;
                }

                return *this;
            }

            ~SpectrogramHandler() {};
            
            /**
             * @brief Initialize the SpectrogramHandler object
             * 
             * @param config_data json object with configuratoin data
             */
            void init(json & config_data){
                config = config_data;
                if (check_config())
                {
                    initialize_spectrogram_params();
                    initialize_fft_params();
                    initialize_buffers();
                    initialize_hanning_window();
                    initialize_freq_and_timing_bins();
                    initialize_clustering_params();
                    initialize_chirp_and_frame_tracking();
                    initialize_precise_timing_estimates();
                    initialize_parameter_randomization_detection();
                    initialize_debug();
                    initialize_save_file_path();
                }
            }

        private: //functions to support initialization should be private

            /**
             * @brief Check the json config file to make sure all necessary parameters are included
             * 
             * @return true - JSON is all good and has required elements
             * @return false - JSON is missing certain fields
             */
            bool check_config(){
                bool config_good = true;
                //check sampling rate
                if(config["USRPSettings"]["Multi-USRP"]["sampling_rate"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: no sampling_rate in JSON" <<std::endl;
                    config_good = false;
                }
                
                //check the samples per buffer
                if(config["USRPSettings"]["RX"]["spb"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: Rx spb not specified" <<std::endl;
                    config_good = false;
                }

                //check the minimum recording time
                if(config["SensingSubsystemSettings"]["min_recording_time_ms"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: min recording time not specified" <<std::endl;
                    config_good = false;
                }

                //check the spectrogram frequency sampling period
                if(config["SensingSubsystemSettings"]["spectrogram_freq_sampling_period_s"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: spectrogram_freq_sampling_period_s not specified" <<std::endl;
                    config_good = false;
                }
                
                //spectrogram peak detection threshold
                if(config["SensingSubsystemSettings"]["spectogram_peak_detection_threshold_dB"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: spectogram peak detection threshold not specified" <<std::endl;
                    config_good = false;
                }

                //spectrogram min_points_per_chirp
                if(config["SensingSubsystemSettings"]["min_points_per_chirp"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: min number of points per chirp not specified" <<std::endl;
                    config_good = false;
                }

                //precise timing estimate observation window time
                if(config["SensingSubsystemSettings"]["precise_timing_estimate_observation_window_time_us"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: observation window time for precise timing not specified" <<std::endl;
                    config_good = false;
                }  

                //cross correlation maximum lag
                if(config["SensingSubsystemSettings"]["precise_timing_estimate_max_xcorr_lag_samples"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: max xcorr lag samples for precise timing not specified" <<std::endl;
                    config_good = false;
                }

                //number of victim frames to capture
                if(config["SensingSubsystemSettings"]["num_victim_frames_to_capture"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: num_victim_frames_to_capture not specified" <<std::endl;
                    config_good = false;
                }

                //maximum victim chirps to capture when performing parameter estimatino
                if(config["SensingSubsystemSettings"]["max_victim_chirps_to_capture"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: max_victim_chirps_to_capture not specified" <<std::endl;
                    config_good = false;
                }

                if(config["SensingSubsystemSettings"]["min_frame_periodicity_ms"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: min_frame_periodicity_ms not specified" <<std::endl;
                    config_good = false;
                }

                if(config["SensingSubsystemSettings"]["debug"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: debug not specified" <<std::endl;
                    config_good = false;
                }

                if(config["SensingSubsystemSettings"]["save_file_path"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: save_file_path not specified" <<std::endl;
                    config_good = false;
                }

                if(config["SensingSubsystemSettings"]["max_waiting_time_ms"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: max_waiting_time_ms not specified" <<std::endl;
                    config_good = false;
                }

                if(config["SensingSubsystemSettings"]["randomization_detection"]["frame_period_us"]["threshold"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: frame period randomization detection threshold not specified" <<std::endl;
                    config_good = false;
                }

                if(config["SensingSubsystemSettings"]["randomization_detection"]["frame_period_us"]["num_samples"].is_null()){
                    std::cerr << "SpectrogramHandler::check_config: frame period randomization detection num_samples not specified" <<std::endl;
                    config_good = false;
                }

                return config_good;
            }
            
            /**
             * @brief Initialize spectrogram parameters based on the configuration file
             * 
             */
            void initialize_spectrogram_params(){

                //specify the sampling rate
                FMCW_sampling_rate_Hz = config["USRPSettings"]["Multi-USRP"]["sampling_rate"].get<double>();
                samples_per_buffer_rx_signal = config["USRPSettings"]["RX"]["spb"].get<size_t>();

                //determine the frequency sampling period based on the sampling rate                
                double freq_sampling_period = config["SensingSubsystemSettings"]["spectrogram_freq_sampling_period_s"].get<double>();

                //determine the number of samples per sampling window
                samples_per_sampling_window = static_cast<size_t>(std::ceil(FMCW_sampling_rate_Hz * freq_sampling_period));

                //determine the fft size
                fft_size = static_cast<size_t>(
                                std::pow(2,std::floor(
                                    std::log2(static_cast<data_type>(samples_per_sampling_window)))));

                //recompute the actual frequency sampling window using the number of samples 
                // per sampling window
                freq_sampling_period = static_cast<double>(samples_per_sampling_window) /
                                            FMCW_sampling_rate_Hz;
                
                //determine the number of rows in the rx signal buffer
                double row_period = static_cast<double>(samples_per_buffer_rx_signal)/FMCW_sampling_rate_Hz;
                double min_recording_time_ms = config["SensingSubsystemSettings"]["min_recording_time_ms"].get<double>();
                num_rows_rx_signal = static_cast<size_t>(std::ceil((min_recording_time_ms * 1e-3)/row_period));

                //determine the number of samples per rx signal
                num_samples_rx_signal = num_rows_rx_signal * samples_per_buffer_rx_signal;

                //determine the number of rows in the spectrogram
                num_rows_spectrogram = (num_samples_rx_signal / samples_per_sampling_window);
                num_samples_per_spectrogram = num_rows_spectrogram * samples_per_sampling_window;

                //set the peak detection threshold for the spectogram
                peak_detection_threshold = config["SensingSubsystemSettings"]["spectogram_peak_detection_threshold_dB"].get<data_type>();
                spectrogram_absolute_max_val = 0;

                if (debug)
                {
                    std::cout << "SpectrogramHandler::initialize_spectrogram_params():" <<
                        " fft_size = " << fft_size 
                        << ", num_rows: " << num_rows_spectrogram 
                        << ", samples_per_sampling_window: " << samples_per_sampling_window << std::endl;
                }
                
            }


            /**
             * @brief initialize the parameters needed by the fft computation
             * 
             */
            void initialize_fft_params(){
                
                shape = {fft_size};
                stride = {sizeof(std::complex<data_type>)};
                axes = {0};
            }

            /**
             * @brief initialize all buffers used by the spectrogram handler
             * 
             */
            void initialize_buffers(){
                
                //rx_buffer
                rx_buffer = Buffer_2D<std::complex<data_type>>(num_rows_rx_signal,samples_per_buffer_rx_signal);

                //reshaped and sampled signal
                reshaped__and_windowed_signal_for_fft = Buffer_2D<std::complex<data_type>>(num_rows_spectrogram,fft_size);
                
                //window to apply
                hanning_window = Buffer_1D<std::complex<data_type>>(fft_size);
                
                //fft/spectrogram generation
                computed_fft = Buffer_2D<std::complex<data_type>>(num_rows_spectrogram,fft_size);
                generated_spectrogram = Buffer_2D<data_type>(num_rows_spectrogram,fft_size);

                //getting the points from the spectrogram
                spectrogram_points_values = Buffer_1D<data_type>(num_rows_spectrogram);
                spectrogram_points_indicies = Buffer_1D<size_t>(num_rows_spectrogram);

                //tracking detected times
                detected_times = Buffer_1D<double>(num_rows_spectrogram);
                detected_frequencies = Buffer_1D<double>(num_rows_spectrogram);

                //clustering indicies
                cluster_indicies = Buffer_1D<int>(num_rows_spectrogram);

                //detected slopes and intercepts
                detected_slopes_MHz_us = Buffer_1D<double>(num_rows_spectrogram);
                detected_intercepts_us = Buffer_1D<double>(num_rows_spectrogram);

                //captured frames
                max_frames_to_capture = 
                    config["SensingSubsystemSettings"]["num_victim_frames_to_capture"].get<size_t>();
                captured_frames = Buffer_2D<double>(max_frames_to_capture,6);

                //parameter estimation buffers
                max_chirps_to_capture = 
                    config["SensingSubsystemSettings"]["max_victim_chirps_to_capture"].get<size_t>();
                
                slope_MHz_us_estimator_buffer = Parameter_Estimation_Buffer<double>(max_chirps_to_capture);
                chirp_period_us_estimator_buffer = Parameter_Estimation_Buffer<double>(max_chirps_to_capture);
                frame_period_us_estimator_buffer = Parameter_Estimation_Buffer<double>(max_frames_to_capture);  
            }

            /**
             * @brief Initialize the frequency and timing bins
             * 
             */
            void initialize_freq_and_timing_bins(){
                //initialize the frequency parameters and buffers
                frequency_resolution = FMCW_sampling_rate_Hz * 1e-6 /
                            static_cast<double>(fft_size);

                frequencies = std::vector<double>(fft_size,0);

                for (size_t i = 0; i < fft_size; i++)
                {
                    frequencies[i] = frequency_resolution * static_cast<double>(i);
                }

                //initialize the timing parameters and buffers
                //compute the timing offset
                frequency_sampling_period = 
                        static_cast<double>(samples_per_sampling_window)/
                            (FMCW_sampling_rate_Hz * 1e-6);
                
                detected_time_offset = frequency_sampling_period * 
                            static_cast<double>(fft_size) / 2 /
                            static_cast<double>(samples_per_sampling_window);
                
                //create the times buffer
                times = std::vector<double>(num_rows_spectrogram,0);
            
                for (size_t i = 0; i < num_rows_spectrogram; i++)
                {
                    times[i] = (frequency_sampling_period *
                                static_cast<double>(i)) + detected_time_offset;
                }
            }

            /**
             * @brief Initialize the clustering parameters
             * 
             */
            void initialize_clustering_params(){

                // get the minimum number of points per chirp from the JSON file
                min_points_per_chirp = config["SensingSubsystemSettings"]["min_points_per_chirp"].get<size_t>();

                //initialize the maximum cluster index to be zero
                max_cluster_index = 0;
            }


            /**
             * @brief Computes a hanning window of a given size for use in the spectogram generation
             * 
             */
            void initialize_hanning_window() {
                data_type M = static_cast<data_type>(fft_size);
                for (size_t i = 0; i < fft_size; i++)
                {
                    data_type n = static_cast<data_type>(i);
                    //data_type x = 2 * M_PI * n / (M - 1);
                    //data_type cos_x = cos(x);

                    data_type hann = 0.5 * (1 - cos(2 * M_PI * n / (M - 1)));

                    hanning_window.buffer[i] = std::complex<data_type>(hann);
                } 
            }

            /**
             * @brief initialize chirp and frame tracking capabilities
             * 
             */
            void initialize_chirp_and_frame_tracking(){

                //frame tracking
                frame_tracking_num_captured_frames = 0;
                frame_tracking_average_frame_duration_us = 0;
                frame_tracking_average_chirp_duration_us = 0;
                frame_tracking_average_chirp_slope_MHz_us = 0;

                //min frame periodicity
                min_frame_periodicity_s = config["SensingSubsystemSettings"]["min_frame_periodicity_ms"].get<double>() * 1e-3;
                max_waiting_time = config["SensingSubsystemSettings"]["max_waiting_time_ms"].get<double>() * 1e-3;

                //set attack in progress status flag
                attack_in_progress = false;
            }

            /**
             * @brief Initialize parameter randomization detected including the threshold and number of samples for each parameter estimation buffer
             * 
             */
            void initialize_parameter_randomization_detection(){

                //set randomization_detected flag to false
                randomization_detected = false;
                
                //initialize frame period randomization detection
                double threshold = config["SensingSubsystemSettings"]["randomization_detection"]["frame_period_us"]["threshold"].get<double>();
                size_t num_samples = config["SensingSubsystemSettings"]["randomization_detection"]["frame_period_us"]["num_samples"].get<size_t>();

                frame_period_us_estimator_buffer.configure_randomization_detection(threshold,num_samples);

                //TODO: add support for detecting randomization in the slope and chirp period
            }

            /**
             * @brief Initialize precise timing estimation parameters including the victim waveform and xcorrelation parameters
             * 
             */
            void initialize_precise_timing_estimates(){
                
                //set victim_waveform_loaded flag to false
                victim_waveform_loaded = false;

                //computed chirp vector is already initialized as empty
                computed_victim_chirp = Buffer_1D<std::complex<data_type>>();

                //get the maximum lag amount and observation window time from the JSON
                xcorr_max_lag = config["SensingSubsystemSettings"]["precise_timing_estimate_max_xcorr_lag_samples"].get<size_t>();
                xcorr_observation_window_time_us = config["SensingSubsystemSettings"]["precise_timing_estimate_observation_window_time_us"].get<double>();

                //compute the observation window time in samples
                xcorr_observation_window_samples = static_cast<size_t>(
                    roundeven(xcorr_observation_window_time_us * 1e-6 * FMCW_sampling_rate_Hz)
                );

                //initialize the cross correlation class
                cross_corr.set_max_lag(xcorr_max_lag);
            }

            /**
             * @brief Initializes the debugging status to print/save extra information if desired
             * 
             */
            void initialize_debug(){
                debug = config["SensingSubsystemSettings"]["debug"].get<bool>();
            }

            /**
             * @brief initialize the save file path for saving sensing subsystem state and
             * parameter estimations to a file
             * 
             */
            void initialize_save_file_path(){
                save_file_path = config["SensingSubsystemSettings"]["save_file_path"].get<std::string>();
            }

        public: //functions to reset, set detection start times, and process the receive signal are public

            /**
             * @brief re-initialize all parameters (used if performing multiple experiments)
             * 
             */
            void reset(){
                initialize_spectrogram_params();
                initialize_fft_params();
                initialize_buffers();
                initialize_hanning_window();
                initialize_freq_and_timing_bins();
                initialize_clustering_params();
                initialize_chirp_and_frame_tracking();
                initialize_precise_timing_estimates();
                initialize_parameter_randomization_detection();
                initialize_debug();
                initialize_save_file_path();
            }
            
            /**
             * @brief Set the detection start time us object
             * 
             * @param start_time_us the time that the first sample in the rx_buffer occured at (us)
             * @param victim_distance_m the range of the victim (m)
             */
            void set_detection_start_time_us(double start_time_us, double victim_distance_m = 0){
                double distance_delay_us = (victim_distance_m / c) * 1e6;
                detection_start_time_us = start_time_us - distance_delay_us;
            }

            /**
             * @brief Process the received signal
             * 
             */
            bool process_received_signal(){

                //mark the start time
                auto start = high_resolution_clock::now();
                
                //perform processing
                load_and_prepare_for_fft();
                compute_ffts();
                detect_peaks_in_spectrogram();
                compute_clusters();

                if (max_cluster_index > 0)
                {
                    compute_linear_model();
                    compute_victim_parameters();

                    //detect parameter randomization
                    update_parameter_randomization();

                    //mark stop time
                    auto stop = high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>(stop - start);
                    
                    //if debug is enabled print the time taken to compute the waveform
                    if(debug){
                        std::cout << "SpectrogramHandler::process_received_signal: Time taken to process "   
                        << duration.count() << " microseconds" << std::endl;
                    }

                    return true;
                }
                else{
                    std::cout << "SpectrogramHandler::process_received_signal: no chirps detected" << std::endl;
                    return false;
                }
                
                
            }

        private: //functions to support processing the received signal are private

            /**
             * @brief Loads a received signal, reshapes, and prepares it 
             * for fft processing. Signal is saved in 
             * reshaped__and_windowed_signal_for_fft buffer
             * 
             */
            void load_and_prepare_for_fft(){
                
                //get dimmensions of rx_buffer array
                size_t m = rx_buffer.buffer.size(); //rows
                size_t n = rx_buffer.buffer[0].size(); //cols

                //initialize variables for reshaping
                size_t from_r;
                size_t from_c;

                //initialize variable to determine the coordinate in the received signal for a row/col index in reshpaed signal
                size_t k;

                for (size_t i = 0; i < num_rows_spectrogram; i++)
                {
                    for (size_t j = 0; j < fft_size; j++)
                    {
                        //for a given row,col index in the reshaped signal, determine the coordinate in the rx_buffer
                        size_t k = i * samples_per_sampling_window + j;

                        //indicies in rx_buffer
                        from_r = k/n;
                        from_c = k % n;

                        if (from_r >= m)
                        {
                            reshaped__and_windowed_signal_for_fft.buffer[i][j] = 0;
                        }
                        else{
                            reshaped__and_windowed_signal_for_fft.buffer[i][j] = rx_buffer.buffer[from_r][from_c] * hanning_window.buffer[j];
                        }
                    } 
                }
                return;
            }

            /**
             * @brief Compute the fft for the desired rows in the reshaped and windowed
             * signal buffer
             * 
             * @param start_idx the index of the row in the reshaped_and_windowed_signal buffer
             * to start computing ffts for
             * @param end_idx the index of the row in the reshaped_and_windowed_signal buffer to
             * end computing ffts for
             */
            void compute_ffts(size_t start_idx = 0, size_t end_idx = 0){
              
              //if the end_idx is zero (default condition), set it to be the num_rows_spectrogram
              if (end_idx == 0)
              {
                end_idx = num_rows_spectrogram;
              }
              
              
              //compute the fft and generate the spectrogram for the given rows
              for (size_t i = start_idx; i < end_idx; i++)
                {
                    c2c(shape, stride, stride, axes, FORWARD,
                        reshaped__and_windowed_signal_for_fft.buffer[i].data(),
                        computed_fft.buffer[i].data(), (data_type) 1.);

                    //convert to dB
                    for (size_t j = 0; j < fft_size; j++)
                    {
                        generated_spectrogram.buffer[i][j] = 10 * std::log10(std::abs(
                            computed_fft.buffer[i][j]
                        ));
                    }  
                }
            }

            /**
             * @brief CURRENTLY BROKEN compute ffts using multiple threads
             * (calls compute_ffts() from multiple threads)
             * 
             * @param num_threads the number of threads to use for FFT computations
             */
            void compute_ffts_multi_threaded(size_t num_threads = 1){
                //initialize a vector of threads
                std::vector<std::thread> threads;
                size_t rows_per_thread = num_rows_spectrogram / num_threads;
                size_t start_row;
                size_t end_row;

                //spawn multiple threads
                for (size_t thread_num = 0; thread_num < num_threads; thread_num++)
                {   
                    //determine the start and end row for each thread
                    start_row = thread_num * rows_per_thread;

                    if (thread_num == (num_threads - 1)){
                        end_row = num_rows_spectrogram;
                    }
                    else{
                        end_row = (thread_num + 1) * rows_per_thread;
                    }

                    //spawn the thread
                    threads.push_back(std::thread([&] () {
                        compute_ffts(start_row,end_row);
                    }));
                }

                //join the threads
                for (auto& t : threads){
                    t.join();
                }
            }
        
            /**
             * @brief Compute the maximum value and its index in the given signal
             * 
             * @param signal the signal to determine the maximum value of
             * @return std::tuple<data_type,size_t> the maximum value and index of the maximum value in the signal
             */
            std::tuple<data_type,size_t> compute_max_val(std::vector<data_type> & signal){
                //set asside a variable for the value and index of the max value
                data_type max = signal[0];
                size_t idx = 0;

                for (size_t i = 0; i < signal.size(); i++)
                {
                    if (signal[i] > max)
                    {
                        max = signal[i];
                        idx = i;
                    }
                }
                return std::make_tuple(max,idx);
            }

            /**
             * @brief Detect the peaks in the computed spectrogram 
             * and saves the results in the detected_times and detected_frequenies array
             * 
             */
            void detect_peaks_in_spectrogram(){
                //initialize variable to store results from compute_max_val
                std::tuple<data_type,size_t> max_val_and_idx;
                data_type max_val;
                size_t idx;

                //clear the detected times and frequencies buffers
                detected_times.clear();
                detected_frequencies.clear();

                if(not attack_in_progress)
                {
                    //variable to track the absolute maximum value detected in the spectrogram
                    spectrogram_absolute_max_val = generated_spectrogram.buffer[0][0];

                    //get the maximum_value from each computed_spectrogram
                    for (size_t i = 0; i < num_rows_spectrogram; i++)
                    {
                        max_val_and_idx = compute_max_val(generated_spectrogram.buffer[i]);
                        max_val = std::get<0>(max_val_and_idx);
                        idx = std::get<1>(max_val_and_idx);
                        spectrogram_points_values.buffer[i] = max_val;
                        spectrogram_points_indicies.buffer[i] = idx;

                        //update the max value
                        if (max_val > spectrogram_absolute_max_val)
                        {
                            spectrogram_absolute_max_val = max_val;
                        }
                    }
                }
                
                
                data_type threshold = spectrogram_absolute_max_val - peak_detection_threshold;
                //go through the spectrogram_points and zero out the points below the threshold
                for (size_t i = 0; i < num_rows_spectrogram; i ++){
                    if (spectrogram_points_values.buffer[i] > threshold)
                    {
                        detected_times.push_back(times[i]);
                        detected_frequencies.push_back(
                            frequencies[
                                spectrogram_points_indicies.buffer[i]]);
                    }
                }
                return;
            }

            /**
             * @brief identify the clusters from the detected times and frequencies
             * 
             */
            void compute_clusters(){

                //declare support variables
                int chirp = 1;
                size_t chirp_start_idx = 0;
                size_t num_points_in_chirp = 1;
                size_t set_val; // use

                //variable to track the total number of detected points
                size_t num_detected_points = detected_frequencies.num_samples;

                //go through the points and determine the clusters
                for (size_t i = 1; i < num_detected_points; i++)
                {
                    //to be part of the same chirp, the frequency must increase, and the detected point
                    //must be with 5 us of the previous point

                    //TODO: Make these parameters tunable
/*
                    if (((detected_frequencies.buffer[i] - detected_frequencies.buffer[i-1]) > 1) ||
                        (((detected_times.buffer[i] - detected_times.buffer[i-1] < 5))) &&
                        ((detected_frequencies.buffer[i] - detected_frequencies.buffer[i-1]) >= -1))
                    {
*/
                    //new approach: frequency must increase, and be above some minimum value

                    if (((detected_frequencies.buffer[i] - detected_frequencies.buffer[i-1]) >= -1) &&
                        (detected_frequencies.buffer[i] >= 2))
                    {
                        num_points_in_chirp += 1;
                    }
                    else{
                        if(num_points_in_chirp >= min_points_per_chirp){
                            cluster_indicies.set_val_at_indicies(chirp,chirp_start_idx,i);
                            
                            //start tracking the next chirp
                            chirp += 1;
                        }
                        else{
                            cluster_indicies.set_val_at_indicies(-1,chirp_start_idx,i);
                        }

                        //reset support variables for tracking new chirp
                        chirp_start_idx = i;
                        num_points_in_chirp = 1;
                    }
                }

                //check the last point
                if (num_points_in_chirp >= min_points_per_chirp)
                {
                    cluster_indicies.set_val_at_indicies(chirp,chirp_start_idx,num_detected_points);

                    //set the maximum cluster index
                    max_cluster_index = chirp;
                }
                else{
                    cluster_indicies.set_val_at_indicies(-1,chirp_start_idx,num_detected_points);
                    //set the maximum cluster index
                    max_cluster_index = chirp - 1;
                }

                //set the remaining samples in the cluster array to zero
                for (size_t i = num_detected_points; i < num_rows_spectrogram; i++)
                {
                    cluster_indicies.buffer[i] = 0;
                }
            }

            /**
             * @brief Compute the linear model from the clustered times and frequencies
             * 
             */
            void compute_linear_model(){

                //clear the detected slopes and intercepts arrays
                detected_slopes_MHz_us.clear();
                detected_intercepts_us.clear();
                
                //initialize the b vector
                Eigen::Vector<double,2> b;
                size_t n = 0;

                //initialize variables to find indicies for each cluster index
                std::vector<size_t> indicies;
                size_t search_start = 0;


                for (int i = 1; i <= max_cluster_index; i++)
                {
                    indicies = cluster_indicies.find_indicies_with_value(i,search_start,true);
                    n = indicies.size();

                    //initialize Y and X matricies
                    Eigen::Matrix<double,Dynamic,2> X(n,2);
                    Eigen::Vector<double,Dynamic> Y(n);

                    for (size_t j = 0; j < n; j++)
                    {
                        Y(j) = detected_frequencies.buffer[indicies[j]];
                        X(j,0) = 1;
                        X(j,1) = detected_times.buffer[indicies[j]];
                    }

                    //solve the linear equation
                    b = (X.transpose() * X).ldlt().solve(X.transpose() * Y);
                    detected_slopes_MHz_us.push_back(b(1));
                    detected_intercepts_us.push_back(-b(0)/b(1) + detection_start_time_us);
                }
            }

            /**
             * @brief Compute the victim parameters from the detected signal
             * 
             */
            void compute_victim_parameters(){
                
                //increment frame counter
                frame_tracking_num_captured_frames += 1;
                
                //TODO: revise this process to store a series of tracked chirps and frames so as to be able to filter out outliers as needed
                if(not attack_in_progress){
          
                    //save the estimates to the parameter estimator buffers
                    slope_MHz_us_estimator_buffer.load_estimates(detected_slopes_MHz_us.buffer);
                    chirp_period_us_estimator_buffer.load_estimates_from_intercepts(detected_intercepts_us.buffer);

                    //TODO: move these somewhere else to make more efficient
                    chirp_period_us_estimator_buffer.remove_outliers(200);
                    slope_MHz_us_estimator_buffer.remove_outliers(200);
                    

                    //compute average chirp slope
                    frame_tracking_average_chirp_slope_MHz_us = slope_MHz_us_estimator_buffer.get_mean();
                    
                    //compute average chirp period
                    frame_tracking_average_chirp_duration_us = chirp_period_us_estimator_buffer.get_mean();
                }
                
                //estimated frame start time
                if (victim_waveform_loaded)
                {
                    captured_frames.buffer[frame_tracking_num_captured_frames - 1][4] =
                        compute_precise_frame_start_time(detected_intercepts_us.buffer[0]);
                }
                else{
                    captured_frames.buffer[frame_tracking_num_captured_frames - 1][4] = detected_intercepts_us.buffer[0];
                }

                //TODO:move to make more efficient
                frame_period_us_estimator_buffer.remove_outliers(10);
                
                //ocmpute frame duration, average frame duration, and predict next frame
                if(frame_tracking_num_captured_frames > 1)
                {
                    //compute and save frame duration
                    frame_period_us_estimator_buffer.load_estimate_from_intercept(
                        captured_frames.buffer[frame_tracking_num_captured_frames - 1][4],
                        captured_frames.buffer[frame_tracking_num_captured_frames - 2][4]
                    );

                    //compute average frame duration
                    frame_tracking_average_frame_duration_us = frame_period_us_estimator_buffer.get_mean();

                    //predict next frame - predict the time of the second chirp so that the attack doesn't interfere with the 1st chirp
                    captured_frames.buffer[frame_tracking_num_captured_frames - 1][5] = 
                        captured_frames.buffer[frame_tracking_num_captured_frames - 1][4]
                        + 1 * frame_tracking_average_frame_duration_us + frame_tracking_average_chirp_duration_us;
                }
                else
                {
                    captured_frames.buffer[frame_tracking_num_captured_frames - 1][0] = 0;
                    captured_frames.buffer[frame_tracking_num_captured_frames - 1][5] = 0;
                }
            }

            void update_parameter_randomization(){
                
                //check frame parameter randomization
                if (frame_period_us_estimator_buffer.get_randomization_detected())
                {
                    randomization_detected = true;
                }
                
            }

            double compute_precise_frame_start_time(double estimated_start_time_us){
                
                /*
                *determine how many samples before the estimated chirp start time
                *to include in the window used to perform the cross correlation
                *NOTE: This is also useful in the case that the estimated start time
                *occurs before the first sample
                */
            
               double time_before_start_us;
               if ((estimated_start_time_us - detection_start_time_us) > 0.1)
               {
                    time_before_start_us = (1 < (estimated_start_time_us - detection_start_time_us - 0.1)) ? 1 : 
                    (estimated_start_time_us - detection_start_time_us - 0.1);
               }
               else
               {
                    time_before_start_us = 0.1;
               }

               int time_before_start_samples = std::ceil(time_before_start_us * 1e-6 * FMCW_sampling_rate_Hz);

               //Obtain a specific sample for the start of the chirp
               int chirp_start_sample = static_cast<int>(
                    std::round(
                        (estimated_start_time_us - detection_start_time_us) * 1e-6 *
                        FMCW_sampling_rate_Hz
                    )
               );

               //identify the start and end indicies within received signal that fall in the observation window
                int start_index = chirp_start_sample - time_before_start_samples;

                //get the received signal that falls within the xcorr window (based on the estimated receive time)
                std::vector<std::complex<data_type>> received_signal(xcorr_observation_window_samples,std::complex<data_type>(0,0));
                get_received_signal_in_xcorr_observation_window(start_index,received_signal);

                //compute the start index for the computed chirp waveform
                start_index = -1 * time_before_start_samples;
                std::vector<std::complex<data_type>> computed_signal(xcorr_observation_window_samples,std::complex<data_type>(0,0));
                get_computed_chirp_in_xcorr_observation_window(start_index,computed_signal);

                //compute the cross correlation
                cross_corr.compute(received_signal,computed_signal);

                //get the delay in samples
                int delay_in_samples = cross_corr.delay_samples;
                double delay_in_us = cross_corr.compute_delay_us(FMCW_sampling_rate_Hz * 1e-6);

                //return the computed precise start time
                double precise_start_time = 
                    (static_cast<double>(
                        chirp_start_sample + delay_in_samples
                    ) / FMCW_sampling_rate_Hz) * 1e6 + detection_start_time_us;

               return precise_start_time;
            }
        
            /**
             * @brief Helper function that loads the samples present in xcorr observation window
             * for the received signal into an already initialized vector of zeroes
             * 
             * @param start_index the start index in the received sample that the xcorr observation window starts. If this is negative, zeros will be used for negative indicies
             * @param received_signal a reference to an already initialized (with zeros) vector of size xcorr_observation_window_samples. The function will load the relevant samples into this vector
             */
            void get_received_signal_in_xcorr_observation_window(int start_index, std::vector<std::complex<data_type>> & received_signal){
                
                //initialize variables to perform counting
                int from_idx;
                int from_r;
                int from_c;

                //parameters on the 2D buffer used to store the received signal
                int num_cols = rx_buffer.num_cols;

                //load the data in to the buffer
                for (int i = 0; i < xcorr_observation_window_samples; i++)
                {
                    from_idx = start_index + i;

                    if(from_idx >= 0){
                        from_r = from_idx / num_cols;
                        from_c = from_idx % num_cols;
                        received_signal[i] = rx_buffer.buffer[from_r][from_c];
                    }
                }
            }

            /**
             * @brief Helper function that loads the samples present in xcorr observation window
             * for the computed victim chirp into an already initialized vector of zeroes
             * 
             * @param start_index the start index in the received sample that the xcorr observation window starts. If this is negative, zeros will be used for negative indicies
             * @param computed_signal a reference to an already initialized (with zeros) vector of size xcorr_observation_window_samples. The function will load the relevant samples into this vector. Zeros will also be used at the end if the function attempts to access indicies in the computed_victim_chirp buffer that are greater than the number of samples in the buffer
             */
            void get_computed_chirp_in_xcorr_observation_window(int start_index, std::vector<std::complex<data_type>> & computed_signal)
            {   
                //initialize variables to perform counting
                int from_idx;
                int max_from_idx = computed_victim_chirp.num_samples;

                //load the computed signal into the buffer
                for (int i = 0; i < xcorr_observation_window_samples; i++)
                {
                    from_idx = start_index + i;

                    if ((from_idx >= 0) && (from_idx < max_from_idx))
                    {
                        computed_signal[i] = computed_victim_chirp.buffer[from_idx];
                    }   
                }
                

            }

        public: //other useful functions to interface with the spectrogram handler are public

            /**
             * @brief Load a computed victim chirp (generated by the attacking subsystem
             * using the sensing subsystem's estimates) into the spectrogram handler 
             * to be used for precise timing estimates
             * 
             * @param computed_signal a vector containing complex I-Q samples of a single
             * victim chirp
             */
            void load_computed_victim_chirp(std::vector<std::complex<data_type>> & computed_signal){
                
                //load the computed signal into the computed_victim_chirp buffer
                computed_victim_chirp.load_data_into_buffer(computed_signal);

                //set the victim_waveform_loaded flag
                victim_waveform_loaded = true;
            }

            /**
             * @brief get the frame start time of the most recently recorded frame
             * 
             * @return double the start time of the most recent frame in seconds
             */
            double get_last_frame_start_time_s(){
                return captured_frames.buffer[frame_tracking_num_captured_frames - 1][4];
            }

            /**
             * @brief Get the start time prediction for the next frame in ms
             * 
             * @return double the next start time in ms
             */
            double get_next_frame_start_time_prediction_ms(){
                return (captured_frames.buffer[frame_tracking_num_captured_frames - 1][5]) * 1e-3;
            }

            double get_average_chirp_duration_us(){
                return frame_tracking_average_chirp_duration_us;
            }

            double get_average_chirp_slope_MHz_us(){
                return frame_tracking_average_chirp_slope_MHz_us;
            }

            double get_average_frame_duration_ms(){
                return frame_tracking_average_frame_duration_us * 1e-3;
            }

            bool get_parameter_randomization_detection_status(){
                return randomization_detected;
            }

            /**
             * @brief Set the attack in progress flag (on true, spectrogram handler changes behavior to avoid interference with attacking subsystem)
             * 
             * @param status status of the attacking subsystem (true means attack is ongoing)
             */
            void set_attack_in_progress(bool status){
                attack_in_progress = status;
            }

            /**
             * @brief Print a summary of the estimated parameters (slope, and timing)
             * 
             */
            void print_summary_of_estimated_parameters(){
                std::cout << "SpectrogramHandler::print_summary_of_estimated_parameters: average frame duration: " <<
                    frame_period_us_estimator_buffer.get_mean() * 1e-3 << "ms" <<
                    "(variance: " << frame_period_us_estimator_buffer.get_variance() << ")" << std::endl;
                std::cout << "SpectrogramHandler::print_summary_of_estimated_parameters: average chirp duration: " <<
                    chirp_period_us_estimator_buffer.get_mean() << "us" <<
                    "(variance: " << chirp_period_us_estimator_buffer.get_variance() << ")" << std::endl;
                std::cout << "SpectrogramHandler::print_summary_of_estimated_parameters: average chirp slope: " <<
                    slope_MHz_us_estimator_buffer.get_mean() << "MHz/us" <<
                    "(variance: " << slope_MHz_us_estimator_buffer.get_variance() << ")" << std::endl;
                std::string randomization_status = randomization_detected ? "true" : "false";
                std::cout << "SpectrogramHandler::print_summary_of_estimated_parameters: Parameter randomization detected: " <<
                    randomization_status << std::endl;
            }

            /**
             * @brief saves the estimated frame duration (ms), chirp duration (us), and chirp slope (MHz/us)
             * to a file called cpp_estimated_parameters.bin
             * 
             * @param use_custom_numbering set to true if it is desired to number the results for a given trial
             * (defaults to false)
             * @param file_num the number for the given results file (defaults to 0)
             */
            void save_estimated_parameters_to_file(bool number_results_file = false,
                size_t file_num = 0){
                Buffer_1D<double> estimated_parameters(3,false);
                 
                // save the frame duration, chirp duration, and chirp slope
                estimated_parameters.buffer[0] = frame_period_us_estimator_buffer.get_mean() * 1e-3; // ms
                estimated_parameters.buffer[1] = chirp_period_us_estimator_buffer.get_mean(); // us
                estimated_parameters.buffer[2] = slope_MHz_us_estimator_buffer.get_mean(); // MHz/us

                //save the results to a file
                std::string file_name;
                if (number_results_file && file_num > 0){
                    file_name = "cpp_estimated_parameters_" + std::to_string(file_num) + ".bin";
                }else{
                    file_name = "cpp_estimated_parameters.bin";
                }
                std::string path = save_file_path + file_name;
                estimated_parameters.set_write_file(path,true);
                estimated_parameters.save_to_file();
            }
        };
    }


#endif