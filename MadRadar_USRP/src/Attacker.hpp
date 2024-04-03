#ifndef ATTACKERCLASS
#define ATTACKERCLASS

    //c standard library
    #include <iostream>
    #include <cstdlib>
    #include <string>
    #include <complex>
    #include <csignal>
    #include <thread>

    //JSON class
    #include <nlohmann/json.hpp>

    //source libraries
    #include "JSONHandler.hpp"
    #include "USRPHandler.hpp"
    #include "BufferHandler.hpp"
    #include "sensing_subsystem/SensingSubsystem.hpp"
    #include "attacking_subsystem/AttackingSubsystem.hpp"

    using json = nlohmann::json;
    using USRPHandler_namespace::USRPHandler;
    using Buffers::Buffer_2D;
    using Buffers::Buffer_1D;
    using SensingSubsystem_namespace::SensingSubsystem;
    using AttackingSubsystem_namespace::AttackingSubsystem;

    namespace ATTACKER_namespace{
        
        /**
         * @brief Class for the attacker
         * 
         * @tparam data_type the data_type of the data that the attacker uses
         */
        template<typename data_type>
        class ATTACKER{
            private:

                //configuration information
                json config;
                USRPHandler<data_type> usrp_handler;

                //key subsystems
                AttackingSubsystem<data_type> attacking_subsystem;
                SensingSubsystem<data_type> sensing_subsystem;


            public:

                /**
                 * @brief Construct a new ATTACKER object - DEFAULT CONSTRUCTOR, DOES NOT INITIALIZE ATTACKER
                 * 
                 */
                ATTACKER(){}
                
                /**
                 * @brief Construct a new ATTACKER object,
                 * loads the configuration, and initializes the usrp_handler 
                 * 
                 * @param config_data a json config object
                 * @param run on false (default) does not run attacker implementation,
                 * on true runs the attacker implementation
                 */
                ATTACKER(json config_data, bool run = false){
                    init(config_data,run);
                }

                /**
                 * @brief Copy Constructor
                 * 
                 * @param rhs pointer to an existing ATTACKER object
                 */
                ATTACKER(const ATTACKER & rhs) : config(rhs.config),
                                                usrp_handler(rhs.usrp_handler),
                                                attacking_subsystem(rhs.attacking_subsystem),
                                                sensing_subsystem(rhs.sensing_subsystem)
                                                {}
                
                /**
                 * @brief Assignment Operator
                 * 
                 * @param rhs pointer to existing ATTACKER object
                 * @return ATTACKER& 
                 */
                ATTACKER & operator=(const ATTACKER & rhs){
                    if (this != &rhs)
                    {
                        config = rhs.config;
                        usrp_handler = rhs.usrp_handler;
                        attacking_subsystem = rhs.attacking_subsystem;
                        sensing_subsystem = rhs.sensing_subsystem;
                    }

                    return *this;
                }
                
                /**
                 * @brief Destroy the ATTACKER object
                 * 
                 */
                ~ATTACKER(){}

                /**
                 * @brief Initialize the Attacker
                 * 
                 * @param config_data json object with configuration data
                 * @param run on false (default) does not run attacker implementation,
                 * on true runs the attacker implementation
                 */
                void init(json & config_data, bool run = false){
                    config = config_data;
                    usrp_handler.init(config_data);
                    attacking_subsystem.init(config_data, & usrp_handler);
                    sensing_subsystem.init(config_data, & usrp_handler, & attacking_subsystem);

                    //run if specified
                    if (run){
                        //run the sensing subsystem
                        sensing_subsystem.run();
                    }
                }

                /**
                 * @brief Run the attacker. If the attacker is being run as part of a 
                 * series of experiments, the multiple_runs and run_number parameters can be used to
                 * specify the run number so that each parameter estimation has a unique result file.
                 * 
                 * @param multiple_runs set to true if the attacker is being run multiple times
                 * @param run_number the number of the run (experiment) being performed
                 */
                void run_attacker(bool multiple_runs = false,
                    size_t run_number = 0){
                    usrp_handler.reset_usrp_clock();

                    std::vector<std::thread> threads;

                    if (attacking_subsystem.enabled)
                    {
                        threads.emplace_back([&]() {
                            //TODO: Appears that an exception is occuring here. Appears to be occuring at the end of the function (right before the else statement)
                            std::cout << "Attacker::run_attacker: running attacking_subsystem" <<std::endl;
                            attacking_subsystem.run();
                        });

                        std::cout << "Attacker::run_attacker: running sensing_subsystem" <<std::endl;
                        sensing_subsystem.run(multiple_runs,run_number);
                        std::cout << "Attacker::run_attacker: sensing_subsystem_run_complete" <<std::endl;
                    }
                    else{
                        sensing_subsystem.run(multiple_runs,run_number);
                    }

                    //if any threads were spawned, join with them here
                    for (auto& thread: threads)
                    {
                        thread.join();
                    }
                    return;
                }

                /**
                 * @brief Resets the Attacker to a fresh state (useful if performing multiple experiments).
                 * Resets the sensing subsystem and attacking subsystem
                 * 
                 * @param multiple_runs On True, implements certain behaviors to support multiple runs. Defaults to false
                 * @param run_number. When multiple runs are enabled, specifies the current run number
                 */
                void reset(bool multiple_runs = false, size_t run_number = 0){
                    sensing_subsystem.reset();
                    attacking_subsystem.reset(multiple_runs,run_number);
                }
        };
    }

#endif