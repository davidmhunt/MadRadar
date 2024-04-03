#ifndef RADARCLASS
#define RADARCLASS

    //c standard library
    #include <iostream>
    #include <cstdlib>
    #include <string>
    #include <complex>
    #include <csignal>
    #include <random>

    //JSON class
    #include <nlohmann/json.hpp>

    //source libraries
    #include "JSONHandler.hpp"
    #include "USRPHandler.hpp"
    #include "BufferHandler.hpp"

    using json = nlohmann::json;
    using USRPHandler_namespace::USRPHandler;
    using Buffers::FMCW_Buffer;
    using Buffers::Buffer_1D;

    namespace RADAR_namespace{

        /**
         * @brief RADAR Class - class to run all RADAR experiments out of
         * 
         * @tparam data_type 
         */
        template<typename data_type>
        class RADAR {
            //variables
            private:
                json config;
                USRPHandler<data_type> usrp_handler;
                FMCW_Buffer<data_type> tx_buffer;
                FMCW_Buffer<data_type> rx_buffer;
                size_t samples_per_chirp;

                //Variables to keep track of frame start times
                
                //timing arguments
                double stream_start_time_s;
                std::vector<uhd::time_spec_t> frame_start_times;
                
                //FMCW arguments
                size_t num_frames;
                double frame_periodicity_s;

                //parameter randomization arguments
                bool randomization_enabled;
                std::random_device randomization_device;
                std::mt19937 randomization_generator;
                std::normal_distribution<double> randomization_distribution;

                
                //status flags
                bool radar_initialized;
                bool debug;

            //functions
            public:
                
                /**
                 * @brief Construct a new RADAR object - DEFAULT CONSTRUTOR, DOES NOT INITIALIZE USRP
                 * 
                 */
                RADAR(){}
                
                /**
                 * @brief Construct a new RADAR object,
                 * loads the configuration, and initializes the usrp_handler 
                 * 
                 * @param config_data a json config object
                 * @param fmcw_config_data a json config object with the FMCW experiment configuration
                 * 
                 */
                RADAR(json config_data):
                    config(config_data),
                    usrp_handler(config_data),
                    radar_initialized(false){
                    
                    if (! check_config())
                    {
                        std::cerr << "RADAR: JSON config did not check out" << std::endl;
                    }
                    else
                    {
                        init_debug_status();
                    }
                }

                /**
                 * @brief Copy Constructor
                 * 
                 * @param rhs pointer to existing radar object
                 */
                RADAR(const RADAR & rhs) : config(rhs.config),
                                            usrp_handler(rhs.usrp_handler),
                                            tx_buffer(rhs.tx_buffer),
                                            rx_buffer(rhs.rx_buffer),
                                            samples_per_chirp(rhs.samples_per_chirp),
                                            stream_start_time_s(rhs.stream_start_time_s),
                                            frame_start_times(rhs.frame_start_times),
                                            num_frames(rhs.num_frames),
                                            frame_periodicity_s(rhs.frame_periodicity_s),
                                            randomization_enabled(rhs.randomization_enabled),
                                            randomization_device(rhs.randomization_device),
                                            randomization_generator(rhs.randomization_generator),
                                            randomization_distribution(rhs.randomization_distribution),
                                            radar_initialized(rhs.radar_initialized),
                                            debug(rhs.debug)
                                            {}

                /**
                 * @brief Assignment Operator
                 * 
                 * @param rhs existing RADAR object
                 * @return RADAR& 
                 */
                RADAR & operator=(const RADAR & rhs){
                    if (this != &rhs)
                    {
                        config = rhs.config;
                        usrp_handler = rhs.usrp_handler;
                        tx_buffer = rhs.tx_buffer;
                        rx_buffer = rhs.rx_buffer;
                        samples_per_chirp = rhs.samples_per_chirp;
                        stream_start_time_s = rhs.stream_start_time_s;
                        frame_start_times = rhs.frame_start_times;
                        num_frames = rhs.num_frames;
                        frame_periodicity_s = rhs.frame_periodicity_s;
                        randomization_enabled = rhs.randomization_enabled;
                        //randomization device doesn't copy
                        randomization_generator = rhs.randomization_generator;
                        randomization_distribution = rhs.randomization_distribution;
                        radar_initialized = rhs.radar_initialized;
                        debug = rhs.debug;
                    }
                    return *this;
                }

                /**
                 * @brief Destroy the RADAR object
                 * 
                 */
                ~RADAR() {}

                /**
                 * @brief Initialize the RADAR object
                 * 
                 * @param config_data json object with configuraiton information
                 */
                void init(json & config_data){
                    config = config_data;
                    usrp_handler.init(config_data);
                    radar_initialized = false;

                    //check the configuration and initialize the RADAR object
                    if (! check_config())
                    {
                        std::cerr << "RADAR: JSON config did not check out" << std::endl;
                    }
                    else
                    {
                        init_debug_status();
                        init_randomization();
                    }
                }

                /**
                 * @brief Check the json config file to make sure all necessary parameters are included
                 * 
                 * @return true - JSON is all good and has required elements
                 * @return false - JSON is missing certain fields
                 */
                bool check_config(){
                    bool config_good = true;
                    //check for tx file name
                    if (config["RadarSettings"]["tx_file_folder_path"].is_null()){
                        std::cerr << "Radar::check_config: tx_file_folder_path not specified in JSON";
                        config_good = false;
                    }
                    
                    //check for rx file name
                    if (config["RadarSettings"]["rx_file_folder_path"].is_null()){
                        std::cerr << "Radar::check_config: rx_file_folder_path not specified in JSON";
                        config_good = false;
                    }

                    //check for number of chirps
                    if (config["RadarSettings"]["num_chirps"].is_null()){
                        std::cerr << "RADAR::check_config num chirps not specified in JSON";
                        config_good = false;
                    }

                    //verify that a stream start time has been specified
                    if (config["USRPSettings"]["Multi-USRP"]["stream_start_time"].is_null()){
                        std::cerr << "RADAR::check_config stream_start_time not specified in JSON";
                        config_good = false;
                    }

                    //check for the number of frames
                    if (config["RadarSettings"]["num_frames"].is_null()){
                        std::cerr << "RADAR::check_config num_frames not specified in JSON";
                        config_good = false;
                    }

                    //check for the frame periodicity
                    if (config["RadarSettings"]["frame_periodicity_ms"].is_null()){
                        std::cerr << "RADAR::check_config frame_periodicity_ms not specified in JSON";
                        config_good = false;
                    }

                    //check for the frame periodicity
                    if (config["RadarSettings"]["debug"].is_null()){
                        std::cerr << "RADAR::check_config debug not specified in JSON";
                        config_good = false;
                    }

                    //check for parameter randomization enabled
                    if (config["RadarSettings"]["parameter_randomization"]["enabled"].is_null()){
                        std::cerr << "RADAR::check_config parameter_randomization enabled not specified in JSON";
                        config_good = false;
                    }

                    //check for parameter randomization enabled
                    if (config["RadarSettings"]["parameter_randomization"]["frame_periodicity_3_sigma_us"].is_null()){
                        std::cerr << "RADAR::check_config parameter_randomization frame_periodicity_3_sigma_us not specified in JSON";
                        config_good = false;
                    }
                    
                    return config_good;
                }

                void init_debug_status(){
                    debug = config["RadarSettings"]["debug"].get<bool>();
                }
                
                /**
                 * @brief get the tx chirp from its file, set the samples_per_chirp_variable, and
                 *  return the chirp as a vector
                 * 
                 * @param multiple_runs (default false) on true, denotes that there are multiple runs 
                 * being performed for the radar
                 * 
                 * @param run_number (default 0) when multiple runs is true, loads the tx chirp file
                 * corresponding to that number of run
                 * 
                 * @return std::vector<data_type> a single tx chirp as a vector
                 */
                std::vector<std::complex<data_type>> get_tx_chirp(bool multiple_runs = false,
                    size_t run_number = 0){
                    //initialize a vector to store the chirp data
                    Buffer_1D<std::complex<data_type>> tx_chirp_buffer(debug);

                    std::string tx_file_path = config["RadarSettings"]["tx_file_folder_path"].get<std::string>();

                    std::string file_name;
                    if (multiple_runs){
                        file_name = "MATLAB_chirp_full_" + std::to_string(run_number) + ".bin";
                    }else{
                        file_name = "MATLAB_chirp_full.bin";
                    }

                    //create a buffer to load the chirp data into it
                    tx_chirp_buffer.set_read_file(tx_file_path + file_name,true);
                    tx_chirp_buffer.import_from_file();

                    if(debug){
                        std::cout << "Radar::get_tx_chirp: detected samples per chirp: " << tx_chirp_buffer.num_samples << std::endl;
                        //tx_chirp_buffer.print_preview();
                    }

                    samples_per_chirp = tx_chirp_buffer.num_samples;

                    return tx_chirp_buffer.buffer;
                }

                void init_randomization(void){

                    //get randomization enabled status
                    randomization_enabled = config["RadarSettings"]["parameter_randomization"]["enabled"].get<bool>();
                    
                    if(randomization_enabled){
                        //get the 3 sigma value in us
                        double frame_periodicity_3_sigma_us = config["RadarSettings"]["parameter_randomization"]["frame_periodicity_3_sigma_us"].get<double>();

                        //determine variance
                        double stdev = frame_periodicity_3_sigma_us / 3;

                        //initialize the random device
                        randomization_generator = std::mt19937(randomization_device());
                        randomization_distribution = std::normal_distribution<double>(0,stdev);

                    }
                }

                /**
                 * @brief initialize the Tx Buffer for USRP operations.
                 * 
                 * @param desired_num_chirps desired number of chirps to load into the tx buffer
                 * @param desired_samples_per_buffer desired samples per buffer (defaults to max 
                 * samples per buffer for USRP tx device)
                 * @param multiple_runs (default false) on true, denotes that there are multiple runs 
                 * being performed for the radar
                 * 
                 * @param run_number (default 0) when multiple runs is true, loads the tx chirp file
                 * corresponding to that number of run
                 * 
                 */
                void init_tx_buffer(
                    size_t desired_num_chirps,
                    size_t desired_samples_per_buffer = 0,
                    bool multiple_runs = false,
                    size_t run_number = 0){

                    //get the tx chirp buffer
                    std::vector<std::complex<data_type>> tx_chirp = get_tx_chirp(multiple_runs,run_number);

                    //specify samples per buffer behavior
                    size_t samples_per_buffer;

                    if (desired_samples_per_buffer == 0)
                    {
                        samples_per_buffer = usrp_handler.tx_samples_per_buffer;
                    }
                    else
                    {
                        samples_per_buffer = desired_samples_per_buffer;
                    }
                    
                    
                    //configure the tx_buffer to be the correct size
                    tx_buffer.configure_fmcw_buffer(
                        samples_per_buffer,
                        samples_per_chirp,
                        desired_num_chirps
                    );

                    if(debug){
                        std::cout << "Radar::init_tx_buffer: Num Rows: " << tx_buffer.num_rows << " Excess Samples: " << tx_buffer.excess_samples << std::endl;
                    }

                    //load tx chirp into the tx buffer
                    tx_buffer.load_chirp_into_buffer(tx_chirp);
                }
                
                /**
                 * @brief initializes the rx buffer for USRP operation
                 * 
                 * @param desired_num_chirps desired number of chirps
                 * @param desired_samples_per_buffer desired number of samples per buffer (defaults 
                 * to max for USRP device)
                 * @param multiple_runs (default false) on true, denotes that there are multiple runs 
                 * being performed for the radar
                 * 
                 * @param run_number (default 0) when multiple runs is true, set the rx chirp file
                 * corresponding to that number of run
                 */
                void init_rx_buffer(size_t desired_num_chirps,
                    size_t desired_samples_per_buffer = 0,
                    bool multiple_runs = false,
                    size_t run_number = 0){
                    
                    std::string rx_file_path = config["RadarSettings"]["rx_file_folder_path"].get<std::string>();

                    std::string file_name;
                    if (multiple_runs){
                        file_name = "cpp_rx_data_" + std::to_string(run_number) + ".bin";
                    }else{
                        file_name = "cpp_rx_data.bin";
                    }
                    
                    //create a buffer to load the chirp data into it
                    rx_buffer.set_write_file(rx_file_path + file_name,true);

                    //specify samples per buffer behavior
                    size_t samples_per_buffer;

                    if (desired_samples_per_buffer == 0)
                    {
                        samples_per_buffer = usrp_handler.rx_samples_per_buffer;
                    }
                    else
                    {
                        samples_per_buffer = desired_samples_per_buffer;
                    }

                    rx_buffer.configure_fmcw_buffer(
                        samples_per_buffer,
                        samples_per_chirp,
                        desired_num_chirps
                    );

                    if(debug){
                        std::cout << "Radar::init_rx_buffer: Num Rows: " << rx_buffer.num_rows 
                            << " Excess Samples: " << rx_buffer.excess_samples << std::endl;
                    }
                }
                
                /**
                 * @brief initialize the tx and rx buffers for RADAR radar operation
                 * 
                 * @param multiple_runs (default false) on true, denotes that there are multiple runs 
                 * being performed for the radar
                 * 
                 * @param run_number (default 0) when multiple runs is true, loads the tx chirp file
                 * corresponding to that number of run
                 * 
                 * @param evaluate_spoofing_enabled (default false) when multiple runs is true, saves a separate rx buffer for each trial to support spoofing evaluation
                 * 
                 * @param evaluate_parameter_estimation_enabled (default false) when multiple runs is enabled, loads a separate tx buffer for each trial to support evaluation of parameter estimation
                 */
                void init_buffers_for_radar(bool multiple_runs = false,
                    size_t run_number = 0, 
                    bool evaluate_spoofing_enabled = false, 
                    bool evaluate_parameter_estimation_enabled = false){
                    
                    size_t num_chirps = config["RadarSettings"]["num_chirps"].get<size_t>();
                    
                    if (multiple_runs)
                    {
                        init_tx_buffer(num_chirps,0,evaluate_parameter_estimation_enabled,run_number);
                        init_rx_buffer(num_chirps,0,evaluate_spoofing_enabled,run_number);
                    }
                    else
                    {
                        init_tx_buffer(num_chirps,0,false,run_number);
                        init_rx_buffer(num_chirps,0,false,run_number);
                    }
                }          
                

                /**
                 * @brief Computes the frame start times in advance, and applies an offset if one is necessary
                 * 
                 */
                void init_frame_start_times(void){
                    
                    //set stream start time
                    stream_start_time_s = config["USRPSettings"]["Multi-USRP"]["stream_start_time"].get<double>();
                    

                    //set num_frames
                    num_frames = config["RadarSettings"]["num_frames"].get<size_t>();

                    //set frame_periodicity
                    frame_periodicity_s = config["RadarSettings"]["frame_periodicity_ms"].get<double>() * 1e-3;

                    //initialize the frame start times vector
                    frame_start_times = std::vector<uhd::time_spec_t>(num_frames);

                    if(debug){
                        std::cout << "RADAR::init_frame_start_times: computed start times: " << std::endl;
                    }
                    for (size_t i = 0; i < num_frames; i++)
                    {
                        frame_start_times[i] = uhd::time_spec_t(
                                        stream_start_time_s + 
                                        compute_additional_randomization_s() +
                                        (frame_periodicity_s * static_cast<double>(i))
                                    );
                        
                        if(debug){
                            std::cout << frame_start_times[i].get_real_secs() << ", ";
                        }
                    }
                    std::cout << std::endl;
                    
                }

                /**
                 * @brief Compute additional frame start time randomization to subtly perturb the frame start times
                 * 
                 * @return double a slight randomization perturbation in s
                 */
                double compute_additional_randomization_s(){
                    if(randomization_enabled){
                        double additional_randomization_us = randomization_distribution(randomization_generator);
                        return  additional_randomization_us * 1e-6;
                    }else{
                        return 0.0;
                    }
                }

                /**
                 * @brief 
                 * 
                 * @param multiple_runs (default false) on true, denotes that there are multiple runs 
                 * being performed for the radar
                 * 
                 * @param run_number (default 0) when multiple runs is true, loads the tx chirp file
                 * corresponding to that number of run
                 * 
                 * @param evaluate_spoofing_enabled (default false) when multiple runs is true, saves a separate rx buffer for each trial to support spoofing evaluation
                 * 
                 * @param evaluate_parameter_estimation_enabled (default false) when multiple runs is enabled, loads a separate tx buffer for each trial to support evaluation of parameter estimation
                 */
                void initialize_radar(bool multiple_runs = false,
                size_t run_number = 0,
                bool evaluate_spoofing_enabled = false, 
                bool evaluate_parameter_estimation_enabled = false){
                    
                    //initialize the buffers
                    init_buffers_for_radar(multiple_runs,run_number,evaluate_spoofing_enabled,evaluate_parameter_estimation_enabled);

                    //compute the frame start times
                    init_frame_start_times();

                    radar_initialized = true;
                }

                /**
                 * @brief Configures the buffers for radar operation,
                 * loads the buffers into the USRP device, and runs the radar
                 * for the desired number of frames
                 * 
                 */
                void run_RADAR(void){

                    if (! radar_initialized)
                    {
                        std::cerr << "RADAR::run_radar: radar is not initialized, but run called" << std::endl;
                    }
                    else
                    {
                        //stream the frames
                        usrp_handler.stream_frames(frame_start_times,& tx_buffer,& rx_buffer);
                        radar_initialized = false;
                    }
                     
                }
        };
    }

#endif