#ifndef USRPHANDLER
#define USRPHANDLER
    //C standard libraries
    #include <iostream>
    #include <cstdlib>
    #include <string>
    #include <sstream>
    #include <chrono>
    #include <complex>
    #include <csignal>
    #include <thread>
    #include <mutex>
    #include <stdexcept>

    //uhd specific libraries
    #include <uhd/exception.hpp>
    #include <uhd/types/tune_request.hpp>
    #include <uhd/usrp/multi_usrp.hpp>
    #include <uhd/utils/safe_main.hpp>
    #include <uhd/utils/thread.hpp>
    #include <boost/format.hpp>
    #include <boost/lexical_cast.hpp>
    #include <boost/program_options.hpp>

    //includes for JSON editing
    #include <nlohmann/json.hpp>

    //includes for uhd specific coding
    //including uhd specific libraries
    #include "uhd/types/device_addr.hpp"
    #include "uhd/device.hpp"

    //user generated header files
    #include "BufferHandler.hpp"
    #include "sensing_subsystem/EnergyDetector.hpp"
    #include "sensing_subsystem/SpectrogramHandler.hpp"

    using json = nlohmann::json;
    using Buffers::Buffer_2D;
    using Buffers::Buffer_1D;
    using EnergyDetector_namespace::EnergyDetector;
    using SpectrogramHandler_namespace::SpectrogramHandler;

    namespace USRPHandler_namespace {
        
        /**
         * @brief A USRP Handler class to handle streaming with USRP Devises
         * 
         * @tparam data_type the cpu_format used for the USRP device, will be std::complex<data_type>
         */
        template<typename data_type>
        class USRPHandler {
            
            private:
                //status variables used by streamers
                bool overflow_detected; //true if no overflow message sent
                bool rx_first_buffer; //true only on the first rx buffer, for status purposes
                std::atomic<bool> tx_stream_complete; //for coordination between tx_stream and tx_metadata sream

                //varialbes to track the channels
                size_t rx_channel;
                size_t tx_channel;

                //mutex to ensure cout is thread safe
                std::mutex cout_mutex;

                //debug settings
                bool simplified_metadata;
                bool debug;

            public:
                //usrp device
                uhd::usrp::multi_usrp::sptr usrp;
                
                //timing arguments
                uhd::time_spec_t rx_stream_start_offset;
                
                //stream arguments
                uhd::stream_args_t tx_stream_args;
                uhd::tx_streamer::sptr tx_stream;
                uhd::tx_metadata_t tx_md;
                uhd::async_metadata_t tx_async_md;
                size_t tx_samples_per_buffer;
                uhd::stream_args_t rx_stream_args;
                uhd::rx_streamer::sptr rx_stream;
                uhd::rx_metadata_t rx_md;
                size_t rx_samples_per_buffer;

                //Tx/Rx enabled status
                bool tx_enabled;
                bool rx_enabled;

                //json config file
                json config;

        //CLASS FUNCTIONS
            //initialization functions
                
                /**
                 * @brief Construct a new USRPHandler object
                 * 
                 */
                USRPHandler(){}

                /**
                 * @brief Construct a new USRPHandler object
                 * 
                 * @param config_data a json configuration object
                 */
                USRPHandler(json & config_data){
                    init(config_data);
                }

                /**
                 * @brief Copy Constructor
                 * 
                 * @param rhs reference to an existing USRPHandler Object
                 */
                USRPHandler(const USRPHandler & rhs) : overflow_detected(rhs.overflow_detected),
                                                        rx_first_buffer(rhs.rx_first_buffer),
                                                        tx_stream_complete(false), //tx_stream_complete is atomic, not copyable
                                                        rx_channel(rhs.rx_channel),
                                                        tx_channel(rhs.tx_channel),
                                                        cout_mutex(),//std::mutex is not copyable
                                                        simplified_metadata(rhs.simplified_metadata),
                                                        debug(rhs.debug),
                                                        usrp(rhs.usrp),
                                                        rx_stream_start_offset(rhs.rx_stream_start_offset),
                                                        tx_stream_args(rhs.tx_stream_args),
                                                        tx_stream(rhs.tx_stream),
                                                        tx_md(rhs.tx_md),
                                                        tx_async_md(rhs.tx_async_md),
                                                        tx_samples_per_buffer(rhs.tx_samples_per_buffer),
                                                        rx_stream_args(rhs.rx_stream_args),
                                                        rx_stream(rhs.rx_stream),
                                                        rx_md(rhs.rx_md),
                                                        rx_samples_per_buffer(rhs.rx_samples_per_buffer),
                                                        tx_enabled(rhs.tx_enabled),
                                                        rx_enabled(rhs.rx_enabled),
                                                        config(rhs.config)
                                                        {}
                
                USRPHandler & operator=(const USRPHandler & rhs) {
                    if (this != &rhs) {
                        overflow_detected = rhs.overflow_detected;
                        rx_first_buffer = rhs.rx_first_buffer;
                        tx_stream_complete = false; //tx_stream_complete is atomic, not copyable, default to false
                        rx_channel = rhs.rx_channel;
                        tx_channel = rhs.tx_channel;
                        simplified_metadata = rhs.simplified_metadata;
                        debug = rhs.debug;
                        usrp = rhs.usrp;
                        rx_stream_start_offset = rhs.rx_stream_start_offset;
                        tx_stream_args = rhs.tx_stream_args;
                        tx_stream = rhs.tx_stream;
                        tx_md = rhs.tx_md;
                        tx_async_md = rhs.tx_async_md;
                        tx_samples_per_buffer = rhs.tx_samples_per_buffer;
                        rx_stream_args = rhs.rx_stream_args;
                        rx_stream = rhs.rx_stream;
                        rx_md = rhs.rx_md;
                        rx_samples_per_buffer = rhs.rx_samples_per_buffer;
                        tx_enabled = rhs.tx_enabled;
                        rx_enabled = rhs.rx_enabled;
                        config = rhs.config;
                    }
                    return *this;
                }

                /**
                 * @brief Destroy the USRPHandler object
                 * 
                 */
                ~USRPHandler() {};


                /**
                 * @brief Initialize the USRPHandler
                 * 
                 * @param config_data json object with configuration information
                 */
                void init(json & config_data){
                    config = config_data;
                    configure_debug();
                    init_multi_usrp();
                }

                /**
                 * @brief configure debug settings for the USRP handler
                 * 
                 */
                void configure_debug(void){
                    if (config["USRPSettings"]["AdditionalSettings"]["simplified_streamer_metadata"].is_null() == false){
                        simplified_metadata = config["USRPSettings"]["AdditionalSettings"]["simplified_streamer_metadata"].get<bool>();
                        std::cout << "USRPHandler::configure_debug: simplified metadata: " << simplified_metadata << std::endl << std::endl;
                    }
                    else{
                        std::cerr << "USRPHandler::configure_debug: couldn't find simplified_streamer_metadata in JSON" <<std::endl;
                    }

                    if (config["USRPSettings"]["AdditionalSettings"]["debug"].is_null() == false){
                        debug = config["USRPSettings"]["AdditionalSettings"]["debug"].get<bool>();
                        std::cout << "USRPHandler::configure_debug: debug: " << debug << std::endl << std::endl;
                    }
                    else{
                        std::cerr << "USRPHandler::configure_debug: couldn't find debug in JSON" <<std::endl;
                    }
                }

                /**
                 * @brief find all USRP devices connected to a host and return
                 * a vector of the available devices
                 * 
                 * @return uhd::device_addrs_t a vector of uhd::device_addr_t objects
                 * for each found device
                 */
                uhd::device_addrs_t find_devices (void){
                    uhd::device_addr_t hint;
                    uhd::device_addrs_t dev_addrs = uhd::device::find(hint);
                    if (dev_addrs.size() > 0 && debug){
                        std::cout << "USRPHandler::find_devices: Number of devices found: " << dev_addrs.size() << "\n";
                        for (size_t i = 0; i < dev_addrs.size(); i++)
                        {
                            std::cout << "USRPHandler::find_devices: Device Information: \n" << dev_addrs[i].to_pp_string() <<std::endl;
                        }
                    }
                    else{
                        std::cerr << "USRPHandler::find_devices: No devices found\n";
                    }
                    return dev_addrs;
                }

                uhd::device_addr_t find_device_serial (std::string serial){
                    uhd::device_addr_t hint;
                    uhd::device_addr_t device;
                    hint["serial"] = serial;
                    uhd::device_addrs_t dev_addrs = uhd::device::find(hint);
                    if (dev_addrs.size() > 0){
                        if(debug){
                            std::cout << "USRPHandler::find_device_serial: Number of devices found: " << dev_addrs.size() << "\n";
                            std::cout << "USRPHandler::find_device_serial: Device Information: \n" << dev_addrs[0].to_pp_string() <<std::endl;
                        }
                        device = dev_addrs[0];
                    }
                    else{
                        std::cerr << "USRPHandler::find_device_serial: No devices found\n";
                    }
                    return device;
                }

                uhd::device_addr_t find_device_addr (std::string addr){
                    uhd::device_addr_t hint;
                    uhd::device_addr_t device;
                    hint["addr"] = addr;
                    uhd::device_addrs_t dev_addrs = uhd::device::find(hint);
                    if (dev_addrs.size() > 0){
                        if(debug){
                            std::cout << "USRPHandler::find_device_addr: Number of devices found: " << dev_addrs.size() << "\n";
                            std::cout << "USRPHandler::find_device_addr: Device Information: \n" << dev_addrs[0].to_pp_string() <<std::endl;
                        }
                        device = dev_addrs[0];
                    }
                    else{
                        std::cerr << "USRPHandler::find_device_addr: No devices found\n";
                    }
                    return device;
                }

                /**
                 * @brief Create a USRP device object
                 * 
                 */
                void create_USRP_device(void){
                     // create a usrp device
                    if (config["USRPSettings"]["Multi-USRP"]["use_serial"].is_null() == false &&
                        config["USRPSettings"]["Multi-USRP"]["use_serial"].get<bool>() == true &&
                        config["USRPSettings"]["Multi-USRP"]["serial"].is_null() == false &&
                        config["USRPSettings"]["Multi-USRP"]["serial"].get<std::string>().empty() == false)
                    {
                        
                        std::string serial = config["USRPSettings"]["Multi-USRP"]["serial"].get<std::string>();

                        if (debug)
                        {
                            std::cout << "USRPHandler::create_USRP_device:Creating the usrp device with serial: " <<  serial << std::endl;
                        }
                        
                        uhd::device_addr_t device = find_device_serial(serial);
                        
                        usrp = uhd::usrp::multi_usrp::make(device);
                        
                        uhd::dict<std::string, std::string> dev_info = usrp -> get_usrp_tx_info();
                    }
                    else if (config["USRPSettings"]["Multi-USRP"]["use_addr"].is_null() == false &&
                        config["USRPSettings"]["Multi-USRP"]["use_addr"].get<bool>() == true &&
                        config["USRPSettings"]["Multi-USRP"]["addrs"].is_null() == false &&
                        config["USRPSettings"]["Multi-USRP"]["addrs"].get<std::string>().empty() == false)
                    {
                        std::string addr = config["USRPSettings"]["Multi-USRP"]["addrs"].get<std::string>();
                        
                        if (debug)
                        {
                            std::cout << "USRPHandler::create_USRP_device:Creating the usrp device with addrs: " <<  addr << std::endl;
                        }
                        
                        uhd::device_addr_t device = find_device_addr(addr);
                        
                        usrp = uhd::usrp::multi_usrp::make(device);
                        
                        uhd::dict<std::string, std::string> dev_info = usrp -> get_usrp_tx_info();
                    }
                    
                    else //if no device was specified, search for and use the first USRP device
                    {
                        std::cout << std::endl;
                        if (debug)
                        {
                            std::cout << "USRPHandler::create_USRP_device: No usrp device arguments specified, searching for device...\n";
                        }
                        
                        uhd::device_addrs_t dev_addrs = find_devices();
                        usrp = uhd::usrp::multi_usrp::make(dev_addrs[0]);
                    }
                }

                /**
                 * @brief Wait for the allotted setup time to allow for the USRP device setup
                 * to complete
                 * 
                 */
                void wait_for_setup_time(void){
                    if (config["USRPSettings"]["AdditionalSettings"]["setup_time"].is_null() == false){
                        float setup_time = config["USRPSettings"]["AdditionalSettings"]["setup_time"].get<float>();
                        
                        if (debug)
                        {
                            std::cout << "USRPHandler::wait_for_setup_time: waiting for " << 
                                setup_time * 1000 << " ms for setup" <<std::endl << std::endl;
                        }
                        
                        std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(1000 * setup_time)));
                    }
                    else{
                        std::cerr << "USRPHandler::wait_for_setup_time: no setup time allotted in JSON";
                    }
                }

                /**
                 * @brief Set the clock reference based on the "ref" specified in the 
                 * json configuration object.
                 * 
                 */
                void set_ref(void){
                    //set clock reference
                    if (config["USRPSettings"]["Multi-USRP"]["ref"].is_null() == false)
                    {
                        std::string ref = config["USRPSettings"]["Multi-USRP"]["ref"].get<std::string>();
                        if (ref.empty() == false){
                            usrp->set_clock_source(ref);
                        }
                        else{
                            std::cout << "USRPHandler::set_ref: ref defined, but empty string\n\n";
                        }
                    }
                    else {
                        std::cout << "USRPHandler::set_ref: ref not defined in JSON\n\n";
                    }
                }

                /**
                 * @brief Set the tx and Rx subdevices per the JSON config object
                 * 
                 */
                void set_subdevices(void){
                    //set RX Subdevice
                    if (config["USRPSettings"]["RX"]["subdev"].is_null() == false){
                        std::string rx_subdev = config["USRPSettings"]["RX"]["subdev"].get<std::string>();
                        if (rx_subdev.empty() == false){
                            
                            //set the subdevice
                            usrp->set_rx_subdev_spec(rx_subdev);

                            //print confirmation of subdevice setting successfully
                            uhd::usrp::subdev_spec_t rx_subdev_spec = usrp -> get_rx_subdev_spec();

                            if(debug)
                            {
                                std::cout << "USRPHandler::set_subdevices: Setting Rx Subdevice: " 
                                    << rx_subdev << std::endl;

                                std::cout << "USRPHandler::set_subdevices: Rx Subdevice: " 
                                    << rx_subdev_spec.to_pp_string() <<std::endl;
                            }
                        }
                        else{
                            std::cout << "USRPHandler::set_subdevices: failed to set Rx Subdevice" <<std::endl;
                        }
                    }

                    //set TX subdevice
                    if (config["USRPSettings"]["TX"]["subdev"].is_null() == false){
                        std::string tx_subdev = config["USRPSettings"]["TX"]["subdev"].get<std::string>();
                        if (tx_subdev.empty() == false){
                            usrp->set_tx_subdev_spec(tx_subdev);
                            uhd::usrp::subdev_spec_t tx_subdev_spec = usrp -> get_tx_subdev_spec();
                            if (debug)
                            {
                                std::cout << "USRPHandler::set_subdevices: Setting Tx Subdevice: " 
                                    << tx_subdev << std::endl;
                                std::cout << "USRPHandler::set_subdevices: Tx Subdevice: " 
                                    << tx_subdev_spec.to_pp_string() <<std::endl;
                            }
                            
                        }
                        else{
                            std::cout << "USRPHandler::set_subdevices: failed to set Tx Subdevice" <<std::endl;
                        }
                    }
                    if (debug)
                    {
                        std::cout << "USRPHandler::set_subdevices: Using Device: " << usrp->get_pp_string() << std::endl;
                    }
                    
                }

                /**
                 * @brief Set the Tx and Rx sampling rates for the corresponding
                 * Tx and Rx channels as specified in the JSON file
                 * 
                 */
                void set_sample_rate(void){
                    //rx rate
                    if (config["USRPSettings"]["Multi-USRP"]["sampling_rate"].is_null() == false &&
                    config["USRPSettings"]["RX"]["channel"].is_null() == false){
                        float rx_rate = config["USRPSettings"]["Multi-USRP"]["sampling_rate"].get<float>();
                        rx_channel = config["USRPSettings"]["RX"]["channel"].get<size_t>();
                        if (rx_rate <= 0.0) {
                            std::cerr << "USRPHandler::set_sample_rate: Please specify a valid RX sample rate" << std::endl;
                        }
                        usrp->set_rx_rate(rx_rate, rx_channel);
                        if(debug)
                        {
                            std::cout << "USRPHandler::set_sample_rate: Setting RX Rate: " << 
                                        rx_rate/1e6 << " Msps..." << std::endl;
                            std::cout << "USRPHandler::set_sample_rate: Actual RX Rate: " << 
                                        usrp->get_rx_rate(rx_channel) / 1e6 << " Msps..." << std::endl;
                        }
                    }
                    else {
                        std::cerr << "USRPHandler::set_sample_rate: No Rx sample rate in JSON";
                    }
                    //tx rate
                    if (config["USRPSettings"]["Multi-USRP"]["sampling_rate"].is_null() == false &&
                    config["USRPSettings"]["TX"]["channel"].is_null() == false){
                        float tx_rate = config["USRPSettings"]["Multi-USRP"]["sampling_rate"].get<float>();
                        tx_channel = config["USRPSettings"]["TX"]["channel"].get<size_t>();
                        if (tx_rate <= 0.0) {
                            std::cerr << "USRPHandler::set_sample_rate: Please specify a valid TX sample rate" << std::endl;
                        }
                        usrp->set_tx_rate(tx_rate, tx_channel);
                        if(debug){
                            std::cout << "USRPHandler::set_sample_rate: Setting TX Rate: " << 
                                        tx_rate/1e6 << " Msps..." << std::endl;
                            std::cout << "USRPHandler::set_sample_rate: Actual TX Rate: " << 
                                        usrp->get_tx_rate(tx_channel) / 1e6 << " Msps..." << std::endl <<std::endl;
                        }
                    }
                    else {
                        std::cerr << "USRPHandler::set_sample_rate: No Tx sample rate in JSON";
                    }
                }

                /**
                 * @brief Set the RF center frequency and LO offset 
                 * for the respective Tx and Rx channels as specified in the JSON configuration
                 * object
                 * 
                 */
                void set_center_frequency(void){
                    if (config["USRPSettings"]["Multi-USRP"]["center_freq"].is_null() == false &&
                        config["USRPSettings"]["Multi-USRP"]["lo-offset"].is_null() == false)
                    {
                        //get desired center frequency and LO offset
                        float center_freq = config["USRPSettings"]["Multi-USRP"]["center_freq"].get<float>();
                        float lo_offset = config["USRPSettings"]["Multi-USRP"]["lo-offset"].get<float>();

                        //setting Rx
                        if(debug){
                            std::cout << "Setting Rx Freq: " << center_freq/1e6 << " MHz" << std::endl;
                            std::cout << "Setting Rx Lo-offset: : " << lo_offset/1e6 << " MHz" << std::endl;
                        }
                        uhd::tune_request_t rx_tune_request;
                        rx_tune_request = uhd::tune_request_t(center_freq,lo_offset);
                        if (config["USRPSettings"]["AdditionalSettings"]["int-n"].get<bool>() == true){
                            rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
                        }
                        usrp->set_rx_freq(rx_tune_request);
                        if(debug){
                            std::cout << "Actual Rx Freq: " << usrp->get_rx_freq()/1e6 << " MHz" <<std::endl <<std::endl;
                        }

                        //setting Tx
                        if(debug){
                            std::cout << "Setting Tx Freq: " << center_freq/1e6 << " MHz" << std::endl;
                            std::cout << "Setting Tx Lo-offset: : " << lo_offset/1e6 << " MHz" << std::endl;
                        }
                        uhd::tune_request_t tx_tune_request;
                        tx_tune_request = uhd::tune_request_t(center_freq,lo_offset);
                        if (config["USRPSettings"]["AdditionalSettings"]["int-n"].get<bool>() == true){
                            tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
                        }
                        usrp->set_tx_freq(tx_tune_request);
                        if(debug){
                            std::cout << "Actual Tx Freq: " << usrp->get_tx_freq()/1e6 << " MHz" <<std::endl <<std::endl;
                        }
                    }
                    else{
                        std::cerr << "USRPHandler::set_center_frequency: center frequency and lo-offset not defined in JSON config file\n";
                    }
                }

                /**
                 * @brief Set the Tx and Rx gain as specified in the JSON
                 * configuration files
                 * 
                 */
                void set_rf_gain(void){
                    //set Rx Gain
                    if (config["USRPSettings"]["RX"]["gain"].is_null() == false)
                    {
                        double rx_gain = config["USRPSettings"]["RX"]["gain"].get<double>();
                        usrp -> set_rx_gain(rx_gain,rx_channel);
                        if(debug){
                            std::cout << "USRPHandler::set_rf_gain: Setting Rx Gain: " << rx_gain << 
                                        " dB" <<std::endl;
                            std::cout << "USRPHandler::set_rf_gain: Actual Rx Gain: " << usrp -> get_rx_gain(rx_channel) << 
                                        " dB" <<std::endl;
                        }
                    }
                    
                    //set Tx Gain
                    if (config["USRPSettings"]["TX"]["gain"].is_null() == false)
                    {
                        double tx_gain = config["USRPSettings"]["TX"]["gain"].get<double>();
                        usrp -> set_tx_gain(tx_gain,tx_channel);
                        if(debug){
                            std::cout << "USRPHandler::set_rf_gain: Setting Tx Gain: " << tx_gain << 
                                        " dB" <<std::endl;
                            std::cout << "USRPHandler::set_rf_gain: Actual Tx Gain: " << usrp -> get_tx_gain(tx_channel) << 
                                        " dB" <<std::endl <<std::endl;
                        }
                    }
                }
                
                /**
                 * @brief Set the if filter bw for the TX and RX chains
                 * 
                 */
                void set_if_filter_bw(void){
                    if(config["USRPSettings"]["Multi-USRP"]["IF_filter_bw"].is_null() == false){
                        double if_filter_bw = config["USRPSettings"]["Multi-USRP"]["IF_filter_bw"].get<double>();

                        //set Rx IF filter BW
                        usrp->set_rx_bandwidth(if_filter_bw);
                        if(debug){
                            std::cout << "USRPHandler::set_if_filter_bw: Setting Rx Bandwidth: " <<
                                            if_filter_bw/1e6 << " MHz" <<std::endl;
                            std::cout << "USRPHandler::set_if_filter_bw: Actual Rx Bandwidth: " <<
                                            usrp->get_rx_bandwidth()/1e6 << " MHz" <<std::endl;
                        }
                        //set Tx IF filter BW
                        usrp->set_tx_bandwidth(if_filter_bw);
                        if(debug){
                        std::cout << "USRPHandler::set_if_filter_bw: Setting Tx Bandwidth: " <<
                                        if_filter_bw/1e6 << " MHz" <<std::endl;
                        std::cout << "USRPHandler::set_if_filter_bw: Actual Tx Bandwidth: " <<
                                        usrp->get_tx_bandwidth()/1e6 << " MHz" <<std::endl <<std::endl;
                        }
                    }
                }
                
                /**
                 * @brief Set the Tx and Rx Antennas per the JSON configuration
                 * object
                 * 
                 */
                void set_antennas(void){
                    //set Rx Antenna
                    if(config["USRPSettings"]["RX"]["ant"].is_null() == false){
                        std::string rx_ant = config["USRPSettings"]["RX"]["ant"].get<std::string>();
                        usrp->set_rx_antenna(rx_ant,rx_channel);
                        if(debug){
                            std::cout << "USRPHandler::set_antennas: Setting Rx Antenna: " <<
                                        rx_ant << std::endl;
                            std::cout << "USRPHandler::set_antennas: Actual Rx Antenna: " <<
                                        usrp->get_rx_antenna(rx_channel) << std::endl;
                        }
                    }

                    //set Tx Antenna
                    if(config["USRPSettings"]["TX"]["ant"].is_null() == false){
                        std::string tx_ant = config["USRPSettings"]["TX"]["ant"].get<std::string>();
                        usrp->set_tx_antenna(tx_ant,tx_channel);
                        if(debug){
                            std::cout << "USRPHandler::set_antennas: Setting Tx Antenna: " <<
                                        tx_ant << std::endl;
                            std::cout << "USRPHandler::set_antennas: Actual Tx Antenna: " <<
                                        usrp->get_tx_antenna(tx_channel) << std::endl <<std::endl;
                        }
                    }
                }

                /**
                 * @brief Check if the clocking freference is locked correctly
                 * 
                 */
                void check_lo_locked(void){
                    if(config["USRPSettings"]["AdditionalSettings"]["skip-lo"].get<bool>() == false){
                        // Check Ref and LO Lock detect
                        if(config["USRPSettings"]["Multi-USRP"]["ref"].is_null() == false){
                            std::string ref = config["USRPSettings"]["Multi-USRP"]["ref"].get<std::string>();
                            
                            //check Tx sensor
                            std::vector<std::string> sensor_names;
                            sensor_names = usrp->get_tx_sensor_names(0);
                            if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
                                != sensor_names.end()) {
                                uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", 0);
                                if(debug){
                                    std::cout << "USRPHandler::check_lo_locked: Checking TX: " << lo_locked.to_pp_string()
                                            << std::endl;
                                }
                                UHD_ASSERT_THROW(lo_locked.to_bool());
                            }

                            //checking Rx sensor
                            sensor_names = usrp->get_rx_sensor_names(0);
                            if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
                                != sensor_names.end()) {
                                uhd::sensor_value_t lo_locked = usrp->get_rx_sensor("lo_locked", 0);
                                if(debug){
                                    std::cout << "USRPHandler::check_lo_locked: Checking RX: " << lo_locked.to_pp_string()
                                            << std::endl;
                                }
                                UHD_ASSERT_THROW(lo_locked.to_bool());
                            }

                            //checking for mimo or external references
                            sensor_names = usrp->get_mboard_sensor_names(0);
                            if ((ref == "mimo")
                                and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked")
                                    != sensor_names.end())) {
                                uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", 0);
                                if(debug){
                                    std::cout << boost::format("USRPHandler::check_lo_locked:Checking MIMO: %s ...") % mimo_locked.to_pp_string()
                                            << std::endl;
                                }
                                UHD_ASSERT_THROW(mimo_locked.to_bool());
                            }
                            if ((ref == "external")
                                and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                                    != sensor_names.end())) {
                                uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", 0);
                                if(debug){
                                    std::cout << boost::format("USRPHandler::check_lo_locked:Checking External: %s ...") % ref_locked.to_pp_string()
                                            << std::endl;
                                }
                                UHD_ASSERT_THROW(ref_locked.to_bool());
                            }
                        }
                        else{
                            std::cout << "USRPHandler::check_lo_locked: ref is not defined in JSON\n\n";
                        }
                    }
                    else{
                        if(debug){
                            std::cout << "USRPHandler::check_lo_locked: skipping lo-locked check\n\n";
                        }
                    }
                }

                /**
                 * @brief Updates the tx_enabled and rx_enabled settings from the config file
                 * 
                 */
                void update_tx_rx_enabled_status(void){
                    
                    //tx_enabled
                    if(config["USRPSettings"]["TX"]["enabled"].is_null() == false){
                        tx_enabled = config["USRPSettings"]["TX"]["enabled"].get<bool>();
                    }
                    else{
                        std::cerr << "USRPHandler::update_tx_rx_enabled_status: couldn't find tx_enabled settingin JSON" <<std::endl;
                    }

                    //rx_enabled
                    if(config["USRPSettings"]["RX"]["enabled"].is_null() == false){
                        rx_enabled = config["USRPSettings"]["RX"]["enabled"].get<bool>();
                    }
                    else{
                        std::cerr << "USRPHandler::update_tx_rx_enabled_status: couldn't find rx_enabled settingin JSON" <<std::endl;
                    }
                }
                
                /**
                 * @brief Initializes a multi-usrp device with Tx and Rx chains per the
                 * JSON configuration file
                 * 
                 */
                void init_multi_usrp(void){
                    //create USRP device
                    create_USRP_device();
                    
                    //set clock reference
                    set_ref();

                    //set subdevices
                    set_subdevices();

                    //set sampling rate
                    set_sample_rate();

                    //set the center frequency
                    set_center_frequency();

                    //set the RF gain
                    set_rf_gain();

                    //set the IF filter bandwidth
                    set_if_filter_bw();
                    
                    //set the Tx and Rx antennas
                    set_antennas();

                    //wait for the specified setup time
                    wait_for_setup_time();

                    //check to confirm that the LO is locked
                    check_lo_locked();

                    //initialize the rx timing offset
                    init_Rx_timing_offset(); 

                    //update the tx/rx enabled status
                    update_tx_rx_enabled_status();
                    
                    //initialize the stream arguments
                    init_stream_args();
                }      

                void init_Rx_timing_offset(void){
                    //set the rx stream start offset
                        /*NOTE: this was added because on some USRP devices, there appears to
                        be a difference between when the receiver starts and the transmitter starts
                        even when both devices are given the same start time
                        */
                    if(config["USRPSettings"]["RX"]["offset_us"].is_null() == false){
                        rx_stream_start_offset = uhd::time_spec_t(
                            config["USRPSettings"]["RX"]["offset_us"].get<double>() * 1e-6);
                    }
                    else{
                        std::cerr << "USRPHandler::init_Rx_frame_offset: couldn't find rx stream start time offset in JSON" <<std::endl;
                    }
                }

                /**
                 * @brief initialize the stream arguments for the transmitter and receiver
                 * including the wire format and cpu format
                 * 
                 */
                void init_stream_args(void){
                    std::string cpu_format = config["USRPSettings"]["Multi-USRP"]["cpufmt"].get<std::string>();
                    std::string wirefmt = config["USRPSettings"]["Multi-USRP"]["wirefmt"].get<std::string>();

                    //get the samples per buffer
                    tx_samples_per_buffer = config["USRPSettings"]["TX"]["spb"].get<size_t>();
                    rx_samples_per_buffer = config["USRPSettings"]["RX"]["spb"].get<size_t>();
                    
                    //configure tx stream args
                    std::vector<size_t> tx_channels(1,tx_channel);
                    tx_stream_args = uhd::stream_args_t(cpu_format,wirefmt);
                    tx_stream_args.channels = tx_channels;
                    tx_stream = usrp -> get_tx_stream(tx_stream_args);
                    if (tx_samples_per_buffer == 0)
                    {
                        tx_samples_per_buffer = tx_stream -> get_max_num_samps();
                    }
                    
                    //configure rx stream args
                    std::vector<size_t> rx_channels(1,rx_channel);
                    rx_stream_args = uhd::stream_args_t(cpu_format,wirefmt);
                    rx_stream_args.channels = rx_channels;
                    rx_stream = usrp -> get_rx_stream(rx_stream_args);
                    if(rx_samples_per_buffer ==0)
                    {
                    rx_samples_per_buffer = rx_stream -> get_max_num_samps();
                    }

                    //print the result
                    if(debug){
                        std::cout << "USRPHandler::init_stream_args: tx_spb: " << tx_samples_per_buffer << 
                            " rx_spb: " << rx_samples_per_buffer <<std::endl;
                        std::cout << "USRPHandler::init_stream_args: initialized stream arguments" << std::endl << std::endl;
                    }
                }

                /**
                 * @brief Reset the USRP clock to be zero 
                 * 
                 */
                void reset_usrp_clock(void){
                    if(debug){
                        std::cout << "USRPHandler::reset_usrp_clock: setting device timestamp to 0" << 
                                    std::endl << std::endl;
                    }
                    usrp -> set_time_now(uhd::time_spec_t(0.0));
                    return;
                }


        //streaming functions
                
                /**
                 * @brief streams a series of rx frames
                 * 
                 * @param frame_start_times a vector of uhd::time_spec_t with the start time of each frame
                 * @param rx_buffer a pointer to a Buffer_2D data type
                 */
                void stream_rx_frames(std::vector<uhd::time_spec_t> frame_start_times,
                                    Buffer_2D<std::complex<data_type>> * rx_buffer){
                    //determine the number of samples to be streamed in the frame
                    size_t num_samps_per_buff = rx_buffer -> num_cols;
                    size_t num_rows = rx_buffer -> num_rows;
                    size_t total_samps = num_samps_per_buff * num_rows;
                    size_t num_samps_received;

                    //determine the number of frames to be streamed
                    size_t num_frames = frame_start_times.size();

                    //reset the overflow message
                    overflow_detected = false;
                    rx_first_buffer = true;

                    //initialize the stream command
                    uhd::stream_cmd_t rx_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
                    rx_stream_cmd.num_samps = total_samps;
                    rx_stream_cmd.stream_now = false;

                    for (size_t i = 0; i < num_frames; i++)
                    {
                        //set the time spec for the frame start
                        rx_stream_cmd.time_spec = frame_start_times[i] + rx_stream_start_offset;

                        //send the stream command
                        rx_stream -> issue_stream_cmd(rx_stream_cmd);

                        for (size_t j = 0; j < num_rows; j++)
                        {
                            //receive the data
                            num_samps_received = rx_stream -> recv(
                                            &(rx_buffer->buffer[j].front()),
                                            num_samps_per_buff,rx_md,1.0,true);
                            
                            //check the metadata to confirm good receive
                            if (num_samps_received != num_samps_per_buff){
                                std::cerr << "USRPHandler::stream_rx_frame: Tried receiving " << num_samps_per_buff <<
                                            " samples, but only received " << num_samps_received << std::endl;
                            }
                            check_rx_metadata(rx_md);

                            //if an overflow was detected, the frame is bad, save what we had and start a new frame
                            if (overflow_detected){
                                std::cout << "USRPHandler::stream_rx_frames: Overflow detected on frame " <<
                                            i + 1 << " cancelling frame and starting again" <<std::endl;
                                //reset the overflow tag
                                overflow_detected = false;
                                break;
                            }
                        }
                        rx_buffer -> save_to_file();   
                    }
                    return;
                }

                /**
                 * @brief 
                 * 
                 * @param rx_md uhd::rx_metadata_t object containing the metadata from a recent Tx stream
                 */
                void check_rx_metadata(uhd::rx_metadata_t & rx_md){
                    //create a unique lock for managing outputs using std::cout
                    std::unique_lock<std::mutex> cout_unique_lock(cout_mutex, std::defer_lock);

                    if(rx_first_buffer){
                        cout_unique_lock.lock();
                        if(debug){
                            std::cout << "USRPHandler::check_rx_metadata: start of burst metadata: " << std::endl <<
                                        rx_md.to_pp_string(simplified_metadata) << std::endl <<std::endl;
                        }
                        rx_first_buffer = false;
                        cout_unique_lock.unlock();
                    }
                    if(rx_md.end_of_burst && rx_md.has_time_spec && not simplified_metadata && debug) {
                        cout_unique_lock.lock();
                        std::cout << "USRPHandler::check_rx_metadata: end of burst occurred at " <<
                                    rx_md.time_spec.get_real_secs() << " s" <<std::endl;
                        cout_unique_lock.unlock();
                    }
                    if (rx_md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                        std::cout << "USRPHandler::check_rx_metadata: Timeout while streaming" << std::endl;
                    }
                    if (rx_md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                        if (not overflow_detected) {
                            overflow_detected = true;
                            if (! simplified_metadata && debug)
                            {
                                std::cerr <<    
                                    "Got an overflow indication. Please consider the following:\n" <<
                                    "  Your write medium must sustain a rate of " << 
                                    (usrp->get_rx_rate(rx_channel) * sizeof(std::complex<float>) / 1e6) <<"MB/s.\n" <<
                                    "  Dropped samples will not be written to the file.\n" <<
                                    "  Please modify this example for your purposes.\n" <<
                                    "  This message will not appear again.\n"; 
                            }
                            else
                            {
                                std::cerr << "USRPHandler::check_rx_metadata: Overflow Detected";
                            }
                            
                            
                        }
                    }
                    if (rx_md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                        std::stringstream error;
                        error << "USRPHandler::check_rx_metadata: Receiver error: " << rx_md.strerror();
                        std::cerr << error.str() << std::endl;
                    }
                    return;
                }

                /**
                 * @brief stream a series of tx frames
                 * 
                 * @param frame_start_times a vector of uhd::time_spec_t with the start time for each framne
                 * @param tx_buffer a Buffer_2D that will be used to stream the chirps for each frame
                 */
                void stream_tx_frames(std::vector<uhd::time_spec_t> frame_start_times,
                                        Buffer_2D<std::complex<data_type>> * tx_buffer){
                    //create a unique lock for managing outputs using std::cout
                    std::unique_lock<std::mutex> cout_unique_lock(cout_mutex, std::defer_lock);
                    
                    //determine the number of samples to be streamed in the frame
                    size_t num_samps_per_buff = tx_buffer -> num_cols;
                    size_t num_rows = tx_buffer -> num_rows;
                    size_t total_samps = num_samps_per_buff * num_rows;
                    size_t num_samps_sent;

                    //determine the number of frames to stream
                    size_t num_frames = frame_start_times.size();

                    cout_unique_lock.lock();
                    if(debug){
                        std::cout << "USRPHandler::stream_tx_frame: streaming frame starting at : " <<
                                    frame_start_times[0].get_real_secs() << " s" << std::endl;
                    }
                    cout_unique_lock.unlock();

                    //stream desired number of frames
                    for (size_t i = 0; i < num_frames; i++)
                    {
                        //initialize the metadata
                        tx_md.has_time_spec = true;
                        tx_md.time_spec = frame_start_times[i];
                        tx_md.start_of_burst = false;
                        tx_md.end_of_burst = false;

                        if (debug && not simplified_metadata && i > 0)
                        {
                            cout_unique_lock.lock();
                            std::cout << "USRPHandler::stream_tx_frame: streaming frame starting at : " <<
                                        tx_md.time_spec.get_real_secs() << " s" << std::endl;
                            cout_unique_lock.unlock();
                        }
                        
                        //stream the desired number of chirps
                        for (size_t j = 0; j < num_rows; j++)
                        {
                            num_samps_sent = tx_stream -> send(
                                        &(tx_buffer -> buffer[j].front()),
                                        num_samps_per_buff,
                                        tx_md,0.5);
                            
                            tx_md.start_of_burst = false;
                            tx_md.has_time_spec = false;

                            //confirm that sent correct amount of samples
                            if (num_samps_sent != num_samps_per_buff){
                                std::cerr << "USRPHandler::stream_tx_frame: Tried sending " << num_samps_per_buff <<
                                            " samples, but only sent " << num_samps_sent << std::endl;
                            }
                        }

                        // send a mini EOB packet
                        tx_md.end_of_burst = true;
                        tx_stream->send("", 0, tx_md);

                    }
                    tx_stream_complete = true;
                }

                /**
                 * @brief Tx Metadata messages are delivered asynchronously 
                 * and as such have to be processed separately
                 * 
                 */
                void check_tx_async_messages(void){
                    //create a unique lock for managing outputs using std::cout
                    std::unique_lock<std::mutex> cout_unique_lock(cout_mutex, std::defer_lock);
                    
                    bool exit_flag = false;
                    while (true) {
                        if (tx_stream_complete) {
                            cout_unique_lock.lock();
                                if(debug){
                                    std::cout << "USRPHandler::check_tx_async_messages: exiting async handler due to exit flag" <<std::endl;
                                }
                                cout_unique_lock.unlock();
                            exit_flag = true;
                            return;
                        }

                        if (not tx_stream->recv_async_msg(tx_async_md,0.01)) {
                            if (exit_flag == true){
                                cout_unique_lock.lock();
                                if(debug){
                                    std::cout << "USRPHandler::check_tx_async_messages: exiting async handler due to exit flag and tx metadata timeout" <<std::endl;
                                }
                                cout_unique_lock.unlock();
                                return;
                            }
                            continue;
                        }

                        // handle the error codes
                        switch (tx_async_md.event_code) {
                            case uhd::async_metadata_t::EVENT_CODE_BURST_ACK:
                                //std::cout << "USRPHandler::check_tx_async_messages: exiting async handler due to end of burst" <<std::endl;
                                if (tx_async_md.has_time_spec && not simplified_metadata && debug){
                                    cout_unique_lock.lock();
                                    std::cout << "USRPHandler::check_tx_async_messages: end of burst occurred at " <<
                                            tx_async_md.time_spec.get_real_secs() << " s" << std::endl;
                                    cout_unique_lock.unlock();
                                }
                                continue;
                                //return;

                            case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW:
                            case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET:
                                std::cerr << "USRPHandler::check_tx_async_messages: Underflow Detected" << std::endl;
                                break;

                            case uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR:
                            case uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST:
                                std::cerr << "USRPHandler::check_tx_async_messages: Packet Loss Detected" << std::endl;
                                break;
                            case uhd::async_metadata_t::EVENT_CODE_TIME_ERROR:
                                std::cerr <<  "USRPHandler::check_tx_async_messages: Packet had time that was late" << std::endl;
                                break;
                            default:
                                std::cerr <<  "USRPHandler::check_tx_async_messages: Event code: " << tx_async_md.event_code
                                        << std::endl;
                                std::cerr << "Unexpected event on async recv, continuing..." << std::endl;
                                break;
                        }
                    }
                }

                /**
                 * @brief Stream a series of rx and tx frames depending on the config
                 * 
                 * @param frame_start_times a vector of start times for each frame
                 * @param tx_buffer a pointer to a buffer holding the chirps to be transmitted for each frame
                 * @param rx_buffer a pointer to a buffer to save the received signal for each frame and to save to a file
                 */
                void stream_frames(std::vector<uhd::time_spec_t> frame_start_times,
                                    Buffer_2D<std::complex<data_type>> * tx_buffer,
                                    Buffer_2D<std::complex<data_type>> * rx_buffer){
                    //set the start time
                    reset_usrp_clock();

                    if (tx_enabled && rx_enabled)
                    {
                        //create transmit thread
                        std::thread transmit_thread([&]() {
                            stream_frames_tx_only(frame_start_times,tx_buffer,false);
                        });

                        //stream rx_frames
                        stream_frames_rx_only(frame_start_times,rx_buffer,false);

                        //wait for transmit thread to finish
                        transmit_thread.join();
                    }
                    else if (tx_enabled)
                    {
                        stream_frames_tx_only(frame_start_times,tx_buffer,false);
                    }
                    else
                    {
                        //stream rx_frames
                        stream_frames_rx_only(frame_start_times,rx_buffer,false);
                    }
                    if(debug){
                        std::cout << "USRPHandler::stream_frame: Complete" << std::endl << std::endl;
                    }
                }

                /**
                 * @brief only run a transmit stream
                 * 
                 * @param frame_start_times a vector of uhd::time_spec_t's with the start time for each frame
                 * @param tx_buffer a pointer to a buffer with the tx signal
                 * @param reset_clock (on true) reset the USRP clock (defaults to false)
                 */
                void stream_frames_tx_only(std::vector<uhd::time_spec_t> frame_start_times,
                                            Buffer_2D<std::complex<data_type>> * tx_buffer,
                                            bool reset_clock = false){
                    //set the start time
                    if (reset_clock)
                    {
                        reset_usrp_clock();
                    }
                   
                    //create transmit thread
                    tx_stream_complete = false;
                    std::thread transmit_thread([&]() {
                        stream_tx_frames(frame_start_times,tx_buffer);
                    });

                    //create transmit async handler
                    check_tx_async_messages();
                    //wait for transmit thread to finish
                    transmit_thread.join();
                }

                /**
                 * @brief only run a transmit stream
                 * 
                 * @param frame_start_times a vector of uhd::time_spec_t's with the start time for each frame
                 * @param rx_buffer a pointer to a buffer where the rx signal will be stored
                 * @param reset_clock (on true) reset the USRP clock (defaults to false)
                 */
                void stream_frames_rx_only(std::vector<uhd::time_spec_t> frame_start_times,
                                            Buffer_2D<std::complex<data_type>> * rx_buffer,
                                            bool reset_clock = false){
                    //set the start time
                    if (reset_clock)
                    {
                        reset_usrp_clock();
                    }

                    //stream rx_frames
                    stream_rx_frames(frame_start_times,rx_buffer);
                }

                /**
                 * @brief Saves a continuous stream of rx data to a given 1D rx_buffer
                 * 
                 * @param rx_buffer the 1D buffer to load rx samples into and save to a file
                 * @param stream_time_s the length of time to stream samples for
                 */
                void rx_stream_to_file(Buffer_1D<std::complex<data_type>> * rx_buffer,
                                    double stream_time_s){
                    
                    
                    //determine the number of samples per buffer
                    size_t num_samps_per_buff = rx_buffer -> num_samples;
                    
                    //compute the number of samples to stream
                    double sample_rate = usrp -> get_rx_rate(rx_channel);
                    size_t total_samps = static_cast<size_t>(ceil(
                                            sample_rate * stream_time_s));
                    

                    //reset the overflow message
                    overflow_detected = false;
                    rx_first_buffer = true;

                    //initialize the stream command
                    uhd::stream_cmd_t rx_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
                    rx_stream_cmd.num_samps = total_samps;
                    rx_stream_cmd.stream_now = true;

                    //initialize tracking for when done streaming
                    size_t num_samps_received;
                    size_t expected_samps_to_receive;
                    size_t total_samps_received = 0;
                    bool streaming_complete = false;
                    
                    //send the stream command
                    rx_stream -> issue_stream_cmd(rx_stream_cmd);

                    while(not streaming_complete)
                    {                        
                        //determine the number of samples we are expecting to receive
                        if((total_samps - total_samps_received)>= num_samps_per_buff){
                            expected_samps_to_receive = num_samps_per_buff;
                        }
                        else{
                            expected_samps_to_receive = total_samps - total_samps_received;
                        }
                        
                        //receive the data
                        num_samps_received = rx_stream -> recv(
                                        &(rx_buffer->buffer.front()),
                                        num_samps_per_buff,rx_md,0.5,true);
                        
                        //check the metadata to confirm good receive
                        if (num_samps_received != expected_samps_to_receive){
                            std::cerr << "USRPHandler::rx_stream_to_file: Tried receiving " << expected_samps_to_receive <<
                                        " samples, but only received " << num_samps_received << std::endl;
                        }
                        check_rx_metadata(rx_md);

                        //if an overflow was detected, the frame is bad, save what we had and start a new frame
                        if (overflow_detected){
                            std::cout << "USRPHandler::rx_stream_to_file: Overflow detected " << std::endl;
                            //reset the overflow tag
                            overflow_detected = false;
                            break;
                        }
                        //save the buffer to the file
                        rx_buffer -> save_to_file();

                        //update the tracking for the number of samples sent
                        total_samps_received += num_samps_received;
                        
                        if(total_samps_received >= total_samps){
                            streaming_complete = true;
                        }
                    } //end of while loop
                    return;
                }

                /**
                 * @brief Saves a continuous stream of samples until a given 2D buffer has been filled
                 * 
                 * @param rx_buffer the 2D buffer to load rx samples into and save to a file
                 * @param print_num_samples_error (default true), on false, will not print an error pertaining to the number
                 * of requested samples not matching (set to false when flushing the rx buffer)
                 */
                void rx_stream_to_buffer(Buffer_2D<std::complex<data_type>> * rx_buffer, bool print_num_samples_error = true){
                    
                    
                    //determine the number of samples per buffer
                    size_t num_samps_per_buff = rx_buffer -> num_cols;
                    
                    //compute the number of samples to stream
                    size_t total_samps = num_samps_per_buff * rx_buffer -> num_rows;
                    

                    //reset the overflow message
                    overflow_detected = false;
                    rx_first_buffer = true;

                    //initialize the stream command
                    uhd::stream_cmd_t rx_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
                    rx_stream_cmd.num_samps = total_samps;
                    rx_stream_cmd.stream_now = true;

                    //initialize tracking for when done streaming
                    size_t num_samps_received;
                    size_t expected_samps_to_receive = num_samps_per_buff;
                    
                    //send the stream command
                    rx_stream -> issue_stream_cmd(rx_stream_cmd);

                    for (size_t i = 0; i < rx_buffer -> num_rows; i++)
                    {
                        //receive the data
                        num_samps_received = rx_stream -> recv(
                                        &(rx_buffer->buffer[i].front()),
                                        num_samps_per_buff,rx_md,0.5,true);
                        
                        //check the metadata to confirm good receive
                        if (num_samps_received != expected_samps_to_receive && print_num_samples_error){
                            std::cerr << "USRPHandler::rx_stream_to_buffer: Tried receiving " << expected_samps_to_receive <<
                                        " samples, but only received " << num_samps_received << std::endl;
                        }
                        check_rx_metadata(rx_md);

                        //if an overflow was detected, the frame is bad, save what we had and start a new frame
                        if (overflow_detected){
                            std::cout << "USRPHandler::rx_stream_to_buffer: Overflow detected " << std::endl;
                            //reset the overflow tag
                            overflow_detected = false;
                            break;
                        }
                    }
                    return;
                }

                /**
                 * @brief Saves a continuous stream of samples until a given 2D buffer has been filled
                 * 
                 * @param sensing_subsystem the sensing subsystem object
                 */
                void rx_record_next_frame(SpectrogramHandler<data_type> * spectrogram_handler,
                                            EnergyDetector<data_type> * energy_detector,
                                            double stream_start_time,
                                            double max_waiting_time = 1){
                    
                    
                    //determine the number of samples per buffer
                    size_t num_samps_per_buff = spectrogram_handler -> rx_buffer.num_cols;

                    //determine number of rows in the rx buffer
                    size_t num_rows = spectrogram_handler -> rx_buffer.num_rows;

                    //set the number of rows for energy detection (i.e: the number of rows prior to a chirp detection)
                    size_t num_energy_detection_rows = energy_detector -> num_rows_chirp_detector;
                    
                    //compute the number of samples to stream after a chirp is detected
                    size_t total_samps = num_samps_per_buff * (num_rows - num_energy_detection_rows);

                    //reset the overflow message
                    overflow_detected = false;
                    rx_first_buffer = true;

                    //initialize the stream command
                    double current_time = usrp -> get_time_now().get_real_secs();
                    //uhd::stream_cmd_t rx_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
                    uhd::stream_cmd_t rx_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
                    rx_stream_cmd.num_samps = num_samps_per_buff;

                    if ((stream_start_time - current_time) >= 1e-3)
                    {
                        rx_stream_cmd.time_spec = uhd::time_spec_t(stream_start_time);
                        rx_stream_cmd.stream_now = false;
                    }
                    else{
                        rx_stream_cmd.stream_now = true;
                    }
                    
                    //detemine the maximum waiting time for deteting a chirp
                    double max_end_time = current_time + max_waiting_time;

                    //initialize tracking for detecting overflows
                    size_t num_samps_received;
                    size_t num_total_samps_received = 0;
                    size_t expected_samps_to_receive = num_samps_per_buff;

                    //initialize chirp tracking
                    bool chirp_detected = false;
                    size_t current_idx;
                    energy_detector -> reset_chirp_detector();
                    
                    //send the stream command
                    rx_stream -> issue_stream_cmd(rx_stream_cmd);

                    while (! chirp_detected)
                    {
                        //receive the data
                        current_idx = energy_detector -> get_current_chirp_detector_index();
                        num_samps_received = rx_stream -> recv(
                                        &(energy_detector->chirp_detector_signal.buffer[current_idx].front()),
                                        num_samps_per_buff,rx_md,0.5,true);
                        
                        //check the metadata to confirm good receive
                        if ((num_samps_received != expected_samps_to_receive) &&
                            (rx_md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)){
                            std::cerr << "USRPHandler::rx_record_next_frame: (overflowed) Tried receiving " << expected_samps_to_receive <<
                                        " samples when waiting for chirp, but only received " << num_samps_received << std::endl;
                        }
                        check_rx_metadata(rx_md);
                        
                        if(rx_md.time_spec.get_real_secs() <= stream_start_time){
                            continue;
                        }
                        else if (rx_md.time_spec.get_real_secs() >= max_end_time)
                        {
                            std::cout << "USRPHandler::rx_record_next_frame: timed out while waiting for new chirp" << std::endl;
                            break;
                        }
                        
                        
                        chirp_detected = energy_detector -> check_for_chirp(rx_md.time_spec.get_real_secs());
                    }
                    
                    //send a new stream command
                    if(debug)
                    {
                        std::cout << "USRPHandler::rx_record_next_frame: detected chirp" << std::endl;
                    }
                    rx_stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
                    rx_stream_cmd.num_samps = total_samps;
                    rx_stream_cmd.stream_now = true;
                    rx_stream -> issue_stream_cmd(rx_stream_cmd);

                    for (size_t i = num_energy_detection_rows; i < num_rows; i++)
                    {
                        //receive the data
                        num_samps_received = rx_stream -> recv(
                                        &(spectrogram_handler->rx_buffer.buffer[i].front()),
                                        num_samps_per_buff,rx_md,0.5,true);
                        
                        num_total_samps_received += num_samps_received;
                        
                        //check the metadata to confirm good receive
                        if (num_samps_received != expected_samps_to_receive){
                            std::cout << "USRPHandler::rx_record_next_frame: Tried receiving " << expected_samps_to_receive <<
                                        " samples when spectrogram sensing, but only received " << num_samps_received << std::endl;
                        }
                        check_rx_metadata(rx_md);
                    }
                    return;
                }
        };
    }
#endif