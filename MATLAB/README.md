# MadRadar MATLAB Code Repository

The following instructions are provided for using the MadRadar MATLAB code base. 

**TBDs**
* document EVAL functions
* document DIAG_USRP functions
* document multiple_graph_plotting and performance_characterization functions

## Codebase Overview

The MATLAB code base can be broken down into two key parts:

### 1. MATLAB classes:

I developed the backbone of MadRadar using several MATLAB classes. A brief description of each class is provided below:

* **Attacker_revB**: The overarching attacker MATLAB class used for simulations. It utilizes two sub classes to perform attacks
    * **Subsystem_attacking**: This class is used to generate the attack signals based on the desired attack type and the victim radar's esimated parameters
    * **Subsystem_spectrum_sensing**: This class is used to estimate a victim radar's actual parameters
* **Simulator_revB**: This is the overall simulator class that is used to perform all of the simulations
* **Radar_revB**: This class emulates a radar including the signal generation, waveform Tx/Rx, and signal processing. This class uses another sub class to perform the actual processing
    * **Radar_Signal_Processor_revA**: This is the class that performs all of the radar signal processing for the radars
* **Target_revA**: This class emulates a target with a specified speed, range, and radar cross section

## 2.MATLAB Live Scripts
Next a series of MATLAB have been created to perform specific functions. They can be broadly broken up into the following categories.

* **SIM**: these scripts allow users to simulate a single attack and to see how the attack propagates through the radar's signal processing pipeline. Here, there are two main scripts, one for visualizing the effect on the attacking radar and one for evaluating how the attacker learns a victim radar's parameters

* **DIAG_USRP**: these scripts are used for evaluating the real-time attack performance on the physical prototype that we implemented on the USRPB210. They integrate with the MadRadar_USRP directory to perform the tests.

* **EVAL**: These scripts allow users to conduct large scale evaluations (in simulation or real-world tests) to evaluate MadRadar's sensing and spoofing accuracy

* **SETUP**: Finally, the SETUP scripts are helpful utilities for generating new radar configurations

## Tutorials
We now present several tutorials that can help to get started with our code base.

### Helpful Notes:
1. All victim configurations must be stored as .json files in the config_files directory. Here, several pre-defined radar configurations have already been defined, but you are welcome to define your own should you wish. If you want to generate your own configuration, you can use the SETUP_Compute_radar_settings.mlx live script to aid in computing some of the key parameters. 

2. The victim radar configurations simulated in this code base have a large range of specifications (ex: chirp bandwidth from 4 GHz to 25 MHz) and performance (ex: range resolutions from 5m - 4cm). While we do our best to adapt the pipeline to each configuration, some modifications must be made to the MATLAB classes depending on the victim's configuration. We describe the modifications below:

1. **subsystem_spectrum_sensing.m**: Two functions must be modified in this class depending on the configuration:
    * **initialize_spectrum_sensing_parameters()**:
        * For simulated configurations (4GHz, 1GHz, 100MHz, and 20 MHz), the **min_processing_delay_ms** should be set to 20 ms.
        * For USRP configurations (configurations with USRP in the name), the frame duration may be quite long due to the limited number of configurations we could use on the USRP's. Thus, the **min_processing_delay_ms** may vary per configuration. As a general rule, USRP configurations with a 500us chirp duration should set **min_processing_delay_ms** to 200ms, and most others should set it to **min_processing_delay_ms** 90 ms.
    * **initialize_spectrogram_params**
        * For simulated configurations, set the **USRP** flag to false
        * For USRP configurations, set the **USRP** flag to true
2. **radar_signal_processor_revA.m**: The radar signal processing pipeline's CFAR detector may need to be tuned depending on radar configurations
    * **configure_CFAR_detector()**: 

For the 4GHz and 1GHz configurations:
```
range_training_size = min(8,ceil(obj.Radar.ADC_Samples * 0.05)); 
velocity_training_size = max(3,ceil(obj.Radar.NumChirps * 0.05)); 
```
For the 100 MHz and 20 Mhz configurations
```
range_training_size = 2;
velocity_training_size = 3;
```
also note that the guard region may also have to be calibrated in some cases as the resolution of these configurations is quite low.

### 1. Simulating black box radar attacks
Perform the following steps to simulate a black-box radar attack using the MadRadar MATLAB simulation:

1. Open the SIM_attack_radar_simulation.mlx MATLAB live script
2. In the first code block, specify the configuration that you would like to test against. Here, we make a few notes:
    * Depending on the configuration that you select, you may need to modify certain classes in the MATLAB codebase. See the helpful hints above for more information
3. In the "Initialize the Target" section, specify the range and velocity of the simulated target in the environment
4. In the "Initialize the Victim and Simulation Parameters" Perform the following:
    1. specify the number of victim radar frames that you want to simulate using the frames_to_compute variable
    2. if you want to generate a .gif movie of the victim's perception for each frame, set record_movie to true
    3. Next, you can modify the range_lims and vel_lims so that all generated plots and movies are zoomed around the target/attack of interest.
    4. The vel_exclusion_region will automatically filter out detections (ex: static objects for USRP data) with these velocities which helps visualize only the object of interest. 
5. In the "Initialize the Attacker", specify the attack type that you wish to launch. If you specify the FP (false positive), you can specify the desired position and velocity you wish the attack to launch at. Note that negative velocities will register as positive velocities from the victim's point of view (TBD: check this)
6. Finally, run the simulation to simulate the black box attacks and generate various plots to understand the attack's effect. 
