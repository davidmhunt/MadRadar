#ifndef FMCWHANDLER
#define FMCWHANDLER


//c standard library
    #include <iostream>
    #include <cstdlib>
    #include <string>
    #include <complex>
    #include <csignal>
    #include <thread>

//Radar Class
    #include "RADAR.hpp"
    #include "Attacker.hpp"
//JSON class
    #include <nlohmann/json.hpp>

using RADAR_namespace::RADAR;
using ATTACKER_namespace::ATTACKER;
using json = nlohmann::json;

namespace FMCWHandler_namespace {
    
    template<typename data_type>
    class FMCWHandler {
        private:
        
            bool attack_enabled;
            bool victim_enabled;

            json victim_config;
            json attack_config;
            json fmcw_config;

            RADAR<data_type> Victim;
            ATTACKER<data_type> Attacker;

            //to support multiple runs
            bool multiple_runs;
            bool evaluate_spoofing_enabled;
            bool evaluate_parameter_estimation_enabled;
            std::string victim_tx_files_folder_path;
            size_t num_runs;

        public:
            /**
             * @brief Construct a new FMCWHandler object
             * 
             * @param fmcw_config_obj a JSON config object for the FMCW simulation
             * @param victim_config_obj a JSON config object for the victim
             * @param attack_config_obj a JSON config object for the attacker
             * @param run (default false) on true, runs the experiment
             */
            FMCWHandler(json fmcw_config_obj, json victim_config_obj,json attack_config_obj, bool run = false)
                :fmcw_config(fmcw_config_obj),
                victim_config(victim_config_obj),
                attack_config(attack_config_obj),
                Victim(), //use the default constructor
                Attacker() //use the default constructor and only initialize FMCW devices that need to be initialized
                {
                    
                    if(check_config())
                    {
                        get_enabled_status();
                        get_multiple_runs_configuration();
                        init_FMCW_devices();
                        if (run)
                        {
                            run_FMCW();
                        } 
                    }
            }
            
            /**
             * @brief Check the json config files to make sure all necessary parameters are included
             * 
             * @return true - JSON is all good and has required elements
             * @return false - JSON is missing certain fields
             */
            bool check_config(){
                bool config_good = true;
                //attacker enabled
                if(fmcw_config["Attacker_enabled"].is_null()){
                    std::cerr << "FMCWHandler::check_config: Attacker_enabled not in JSON" <<std::endl;
                    config_good = false;
                }

                //Radar enabled
                if(fmcw_config["Radar_enabled"].is_null()){
                    std::cerr << "FMCWHandler::check_config: Radar_enabled not in JSON" <<std::endl;
                    config_good = false;
                }

                //performing multiple runs
                if(fmcw_config["multiple_runs"]["perform_multiple_runs"].is_null()){
                    std::cerr << "FMCWHandler::check_config: perform_multiple_runs not in JSON" <<std::endl;
                    config_good = false;
                }else if (fmcw_config["multiple_runs"]["perform_multiple_runs"].get<bool>())
                {
                    //the number of runs to perform
                    if(fmcw_config["multiple_runs"]["num_runs"].is_null()){
                        std::cerr << "FMCWHandler::check_config: num_runs not in JSON" <<std::endl;
                        config_good = false;
                    }
                    //the number of runs to perform
                    if(fmcw_config["multiple_runs"]["evaluate_spoofing"].is_null()){
                        std::cerr << "FMCWHandler::check_config: evaluate_spoofing not in JSON" <<std::endl;
                        config_good = false;
                    }
                    //the number of runs to perform
                    if(fmcw_config["multiple_runs"]["evaluate_parameter_estimation"].is_null()){
                        std::cerr << "FMCWHandler::check_config: evaluate_parameter_estimation not in JSON" <<std::endl;
                        config_good = false;
                    }
                }
                return config_good;
            }

            /**
             * @brief Get the enabled status from the JSON
             * 
             */
            void get_enabled_status(void){
                attack_enabled = fmcw_config["Attacker_enabled"].get<bool>();
                victim_enabled = fmcw_config["Radar_enabled"].get<bool>();
            }

            /**
             * @brief Setup whether or not to perform multiple runs. If multiple runs are requested,
             * also get the number of runs
             * 
             */
            void get_multiple_runs_configuration(void){
                multiple_runs = fmcw_config["multiple_runs"]["perform_multiple_runs"].get<bool>();
                if(multiple_runs)
                {
                    num_runs = fmcw_config["multiple_runs"]["num_runs"].get<size_t>();
                    evaluate_spoofing_enabled = fmcw_config["multiple_runs"]["evaluate_spoofing"].get<bool>();
                    evaluate_parameter_estimation_enabled = fmcw_config["multiple_runs"]["evaluate_parameter_estimation"].get<bool>();
                }
            }

            /**
             * @brief Initialize the victim and attacker based on their enable status
             * 
             */
            void init_FMCW_devices(void){
                if (attack_enabled && victim_enabled)
                    {
                        //initialize victim device
                        Victim.init(victim_config);

                        //run the attacker
                        Attacker.init(attack_config,false);
                    }
                else if (attack_enabled)
                {
                    //initialize the attacker
                    Attacker.init(attack_config,false);
                }
                else if (victim_enabled)
                {
                    //initialize the victim device
                    Victim.init(victim_config);
                }
            }
            
            /**
             * @brief Run the FMCW simulation with the vicitm and attacker in separate threads
             * 
             * @param run_number (defaults to 0) the run number if multiple runs are being performed
             */
            void run_FMCW_trial(size_t run_number = 0){

                if(multiple_runs)
                {
                    std::cout << std::endl << std::endl << "FMCWHandler::run_FMCW_trial: running trial: " << run_number << std::endl;
                }
                else
                {
                    std::cout << "FMCWHandler::run_FMCW_trial: running trial" << std::endl;
                }

                if (attack_enabled && victim_enabled)
                    {
                        //initialize victim and attacker
                        Victim.initialize_radar(multiple_runs,run_number,evaluate_spoofing_enabled,evaluate_parameter_estimation_enabled);
                        Attacker.reset(multiple_runs ? evaluate_spoofing_enabled : false,run_number);

                        //create victim thread
                        std::thread victim_thread([&]() {
                            Victim.run_RADAR();
                        });

                        //run the attacker
                        Attacker.run_attacker(multiple_runs ? evaluate_parameter_estimation_enabled : false,run_number);
                        
                        //wait for victim thread to finish
                        victim_thread.join();
                    }
                else if (attack_enabled)
                {
                    //initialize attacker
                    Attacker.reset(multiple_runs ? evaluate_spoofing_enabled : false,run_number);


                    //run the attacker
                    Attacker.run_attacker(multiple_runs ? evaluate_parameter_estimation_enabled : false,run_number);
                }
                else if (victim_enabled)
                {
                    //initialize victim
                    Victim.initialize_radar(multiple_runs,run_number,evaluate_spoofing_enabled,evaluate_parameter_estimation_enabled);

                    //run the victim
                    Victim.run_RADAR();
                }
                else
                {
                    std::cout << "FMCWHandler::run_FMCW_trial: neither attacker nor victim enabled, no run performed" << std::endl;
                }

                if(multiple_runs)
                {
                    std::cout << "FMCWHandler::run_FMCW_trial: trial " << run_number << " complete" << std::endl;
                }
                else
                {
                    std::cout << "FMCWHandler::run_FMCW_trial: trial complete" << std::endl;
                }
            }

            /**
             * @brief Runs the FMCW implementation (runs multiple trials if multiple_trials is true)
             * 
             */
            void run_FMCW(void){
                
                if (multiple_runs)
                {
                    for (size_t i = 1; i <= num_runs; i++)
                    {
                        run_FMCW_trial(i);
                    }
                    
                }
                else //only a single run requested
                {
                    run_FMCW_trial();
                }
                
                
            }
    };
}

#endif