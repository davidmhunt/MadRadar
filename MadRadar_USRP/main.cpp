
//C standard libraries
#include <iostream>
#include <cstdlib>
#include <string>
#include <chrono>
#include <complex>
#include <csignal>
#include <thread>

//uhd specific libraries
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

//JSON class
#include <nlohmann/json.hpp>

//source libraries
#include "src/JSONHandler.hpp"
#include "src/USRPHandler.hpp"
#include "src/BufferHandler.hpp"
#include "src/RADAR.hpp"
#include "src/FMCWHandler.hpp"

//set namespaces
using json = nlohmann::json;
using USRPHandler_namespace::USRPHandler;
using Buffers::Buffer;
using Buffers::Buffer_1D;
using RADAR_namespace::RADAR;
using FMCWHandler_namespace::FMCWHandler;

int UHD_SAFE_MAIN(int argc, char* argv[]) {

    //radar configuration
    std::string radar_config_file = "/home/david/Documents/BlackBoxRadarAttacks/FMCW_radar_uhd/Config_uhd_victim_400us_to_500us.json";
    //std::string radar_config_file = "/home/davidh/Documents/BlackBoxRadarAttacks/FMCW_radar_uhd/Config_uhd_100MHz.json";
    
    //attacker configuration
    std::string attack_config_file = "/home/david/Documents/BlackBoxRadarAttacks/FMCW_radar_uhd/Config_uhd_attack_400us_to_500us.json";
    //std::string attack_config_file = "/home/davidh/Documents/BlackBoxRadarAttacks/FMCW_radar_uhd/Config_uhd_attack_100MHz.json";


    //fmcw config files
    std::string fmcw_config_file = "/home/david/Documents/BlackBoxRadarAttacks/FMCW_radar_uhd/Config_FMCW.json";
    //std::string fmcw_config_file = "/home/davidh/Documents/BlackBoxRadarAttacks/FMCW_radar_uhd/Config_FMCW.json";


    //read the config file
    std::cout << "\nMAIN: Parsing JSON\n";
    json radar_config = JSONHandler::parse_JSON(radar_config_file,false);
    json attack_config = JSONHandler::parse_JSON(attack_config_file,false);
    json fmcw_config = JSONHandler::parse_JSON(fmcw_config_file,false);


    //check to make sure that the radar and attacker have a valid config format:
    if(radar_config["USRPSettings"]["Multi-USRP"]["type"].is_null() ||
        radar_config["USRPSettings"]["Multi-USRP"]["cpufmt"].is_null() ||
        attack_config["USRPSettings"]["Multi-USRP"]["type"].is_null() ||
        attack_config["USRPSettings"]["Multi-USRP"]["cpufmt"].is_null()){
            std::cerr << "MAIN: type or cpu format is not specified in radar or attack config file" <<std::endl;
            return EXIT_FAILURE;
    }


    //confirm that the radar and attacker have the same type and cpu fmt
    if(radar_config["USRPSettings"]["Multi-USRP"]["type"].get<std::string>() !=
        attack_config["USRPSettings"]["Multi-USRP"]["type"].get<std::string>()||
        radar_config["USRPSettings"]["Multi-USRP"]["cpufmt"].get<std::string>() !=
        radar_config["USRPSettings"]["Multi-USRP"]["cpufmt"].get<std::string>()){
            std::cerr << "MAIN: Attack and Victim cpu or type not the same" <<std::endl;
            return EXIT_FAILURE;
    }

    //determine that there is a valid cpu format and type
    
    std::string type = radar_config["USRPSettings"]["Multi-USRP"]["type"].get<std::string>();
    std::string cpufmt = radar_config["USRPSettings"]["Multi-USRP"]["cpufmt"].get<std::string>();

    
    if (type == "double" && cpufmt == "fc64"){
        FMCWHandler<double> fmcw_handler(fmcw_config,radar_config,attack_config,true);
    }
    else if (type == "float" && cpufmt == "fc32")
    {
        FMCWHandler<float> fmcw_handler(fmcw_config,radar_config,attack_config,true);
    }
    /*
    else if (type == "int16_t" && cpufmt == "sc16")
    {
        FMCWHandler<int16_t> fmcw_handler(radar_config,attack_config,false);
    }
    else if (type == "int8_t" && cpufmt == "sc8")
    {
        FMCWHandler<int8_t> fmcw_handler(radar_config,attack_config,false);
    }
    */
    else{
        std::cerr << "MAIN: type and cpufmt don't match valid combination (must use float or double)" << std::endl;
        return EXIT_FAILURE;
    }


/*
    USRPHandler<std::complex<float>> victim(radar_config);
    USRPHandler<std::complex<float>> attack(attack_config);
 */   
 
/* 
    usrp_handler.load_BufferHandler( & buffer_handler);
    
    //stream the frame
    std::cout << "\nMAIN: Streaming Frames\n";
    usrp_handler.stream_frames();

    //EXTRA CODE FOR DEBUGGING PURPOSES

    //std::cout << "Rx Buffer Preview" <<std::endl;
    //buffer_handler.print_2d_buffer_preview(buffer_handler.rx_buffer);

    //buffer_handler.rx_buffer = buffer_handler.tx_buffer;
    //buffer_handler.save_rx_buffer_to_file();
*/
    return EXIT_SUCCESS;
}
