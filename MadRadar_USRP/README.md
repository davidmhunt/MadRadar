# MadRadar USRP (SDR Implementation) Code

The following repository was created to implement a simple FMCW radar on the USRP with offline processing using a MATLAB script

## Install Required Dependencies

To run the code, the following dependencies must also be installed:
1. nlohmann_json (at least 3.10.5)
2. uhd (at least 4.1.0)
3. c++ (at least C++ 11)

Instructions to install each package are available here:

### 1. Install nlohmann_json
1. clone the nlohmann_json repository
```
git clone https://github.com/nlohmann/json.git
```

2. create a build directory and navigate to it
```
mkdir build
cd build
```

3. run cmake with the path to the source directory
```
cmake ../json
```

4. run make to build the library
```
make
```

5. install the library
```
sudo make install
```

### 2. Install uhd from source

I installed uhd from source. This can be accomplished by the instructions on the [UHD Install Guide](https://files.ettus.com/manual/page_build_guide.html)


## Building C++ Code
To build the c++ code used to interact with the USRP radar, perform the following steps:

1. Clone the MadRadar git repository and its submodules (if not done before)
```
git clone --recurse-submodules https://github.com/davidmhunt/CPSL_USRP_FMCW_Radar.git
```

If you forget to perform the --recurse-submodules when cloning the repository, you can use the following command to load the necessary submodules
```
git submodule update --init --recursive
```

2. Navigate to the MadRadar_USRP folder
```
cd MadRadar/MadRadar_USRP
```

3. make the build folder
```
cd build
```

4. build the project
```
cmake ..
make
```

## Running Experiments on the USRP

Running experiments can be accomplished using the following steps:
1. Start experiment in MATLAB
2. Run experiment on USRP
    1. (optional) update config.json file for radar
    2. Rebuild c++ code for updated experiment
    3. Run experiment
3. Process streamed data from USRP in MATLAB

## [UPDATING EVERYTHING BELOW]
Note: this readme file is currently being updated. The below instructions shouldn't change much but may not result in perfect operation at this time.

### 1. Start Experiment in MATLAB

1. Open up the [DIAG_USRP_Attack_system.mlx](MATLAB/DIAG_USRP_Attack_system.mlx) matlab live script file.
2. In the first section ("Initialize the FMCW Configuration), update the config_folder_path variable with the path to the CPSL_USRP_FMCW_Radar/MATLAB/config_files folder on your machine.The [config_files](MATLAB/config_files/) folder contains several configurations that I have used in the past, however you can also create your own configuration as well. To do so, complete the following steps:
    1. Create a new config.json file in the [config_files](MATLAB/config_files/) folder.
    2. See the other config.json files for the template that should be followed. When specifying your own configuration, you will need to fill out the following fields
        * StartFrequency_GHz: the start frequency (in GHz) that the radar will transmit FMCW chirps at
        * FrequencySlope_MHz_us: the frequency slope of the chirps
        * TxStartTime_US: the start time of the FMCW chirp (I generally leave this at zero). This is left-over from replicating the settings of the TI-IWR radars
        * ADC_Samples: the number of samples recorded per chirp
        * ADC_SampleRate_MSps: the ADC sampling rate of a would-be ADC sampling the IF frequency of the FMCW chirps
        * ChirpCycleTime_us: the time between consecutive chirps (in us)
        * NumChirps: The number of chirps per radar frame
        * FramePeriodicity_ms: the time between frames in ms
    3. If you are new to radar, I've created a helpful matlab live script to assist in coming up with these values for a given set of performance values. To use this script, perform the following steps:
        1. Open the [SETUP_Compute_radar_settings.mlx](MATLAB/SETUP_Compute_radar_settings.mlx) MATLAB live script file
        2. In the first section, specify B (bandwidth in Hz), ADC_samples (per chirp, usually set to 256), f (start frequency in Hz), v_max (maximum velocity in m/s), n_chirps (number of chirps, usually set to 256). Then run the first section. 
        3. In the second section, specify the exact sampling rate that the USRP will operate at (I have previously done 25 MHz) in MHz. Then, run the second section.
        4. At the bottom of the script, it will provide the slope, chirp period (Tc_us), and ADC sample rate that you should use to meet your performace specs. 
        5. NOTE: This script doesn't guarantee a valid configuration, in later sections, I will show how to verify that your configuration is valid. If you configuration is not valid, you may hve to try using different settings
    4. Once your configuration has been updated, go back to the [DIAG_USRP_Attack_system.mlx](MATLAB/DIAG_USRP_Attack_system.mlx), and run the first section. At the end of this section, it will print out key radar parameters and performance specifications. Here, check the following:
        * Ensure that none of the values in the Chirp Parameters or Frame Parameters are negative. If so, the configuration is not valid
        * Ensure that the frame periodicity is not shorter than the active frame time. If so, you may have to use less chirps or shorten your frame duration
        * Ensure that the FMCW sampling rate is exactly the frequency that the USRP will operate at. This is because the MATLAB script will save the transmitted chirp at this sampling rate.
    5. Run the next 3 sections. This will accomplish the following:
        1. Precompute victim chirps - this will precompute the victim chirps so that they can then be transmitted by the USRP
        2. Save the victim chirp to a file: this will save the victim chirp to a file so that it can be streamed by the USRP
            * Note: Please update the path (including a file name) to the location that you want the Tx file to be streamed from
            * The file name should always be MATLAB_chirp_full.bin
        3. Plot Tx Chirp to confirm correctness: this will plot the raw ADC samples and a spectrogram of a single chirp.

### 2. Run experiment on the USRP

#### Update .json config file
Depending on the configuratin that you used in the previous steps, you will either need to select one of the currently available .json configurations located in [FMCW_radar_uhd](FFMCW_radar_uhd/) or create your own. For creating a new .json configuration file for the USRP, use the [Config_uhd_victim_400us_to_500us.json](FMCW_radar_uhd/Config_uhd_victim_400us_to_500us.json) as a reference.

To updating/modifying the .json config file, take note of the following fields:
* USRPSettings
    * Multi-USRP
        * use_serial: set to true if using the USRPB210
        * serial: the serial address to the USRPB210
        * use_addr: set to true if using a USRP that connects over Ethernet
            * NOTE: While this capability should be implemented, I haven't tested my code on a USRP X310 at this point. Thus some modifications may be required
        * addrs: the addres of the X310 on ethernet
        * sampling_rate: the sampling rate of the USRP in Hz
            * make sure this is the same as the FMCW_sampling_rate specified in the earlier steps
        * center_freq: the transmit frequency for the radar in Hz
            * make ure that this is the same as the chirp start freq specified earlier
        * IF_filter_bw: I set this to be the same as the sampling rate
        * stream_start_time: This is a delay in seconds between once we start streaming samples on the USRP and the radar actually starts operating
    * Tx:
        * enabled: set to True so that the transmitter on the USRP is used
        * amt" use to specify the antenna being used on the USRP
        * subdev: use to specify the subdevice being used for transmit on the USRP
        * channel: use to specify the channel being used for transmit on the USRP
    * Rx:
        * enabled: set to True so that the receiver on the USRP is used
        * amt" use to specify the antenna being used on the USRP
        * subdev: use to specify the subdevice being used for receive on the USRP
        * channel: use to specify the channel being used for receive on the USRP
        * offset_us: I empirically found that there is a time difference between when the Tx starts transmitting and when the Rx starts receiving. This specifies this delay in us. See "Calibrating for USRP Tx and Rx timing offset" section for how this should be set
    * AdditionalSettings:
        * debug: when this is set to true, the c++ program prints additional information that may be useful for debugging
        * simplified_streamer_metadata: if debug is enabled, this will print simplified streamer metadata instead of the full thing
* RadarSettings:
    * tx_file_folder_path: this should be set to the same path that you used in step 2.5 (where you saved the precomputed Tx chirp). However, you should only specify location of the folder, not the file name. The c++ code assumes that the file is called MATLAB_chirp_full.bin
    * rx_file_folder_path: this is the path to a folder that the C++ code will save the streamed receive signal to. The saved file will be called cpp_rx_data.bin and will be used later
    * num_chirps: the number of chirps per frame.
        * should be the same as the MATLAB configuration used
        * num_frames: the number of frames to stream
        * frame_periodicity_ms: this should be set to the same frame periodicity that was used in the MATLAB code
        * parameter_randomization: set to false as this is not required for nominal radar operation
        * debug: on True will print additional information that may be helpful for debugging purposes

#### Rebuild the c++ code with the updated configruation:
1. Open the [main.cpp](FMCW_radar_uhd/main.cpp) file, and update the following paths:
    1. radar_config_file (line 42): set this to the full path of the .json file that you are using for your experiments
    2. fmcw_config_file (line 47): set this to the full path of the [Config_FMCW.json](FMCW_radar_uhd/Config_FMCW.json) configuration file. This is an additional configuration file, but should not require modification for running the radar only
2. Open a terminal window, and navigate to the build folder
```
cd CPSL_USRP_FMCW_Radar/FMCW_radar_uhd/build
```
3. remake the project
```
cmake ..
make
```

#### Run the c++ code with the updated configuration
1. From the build folder (should already be in it), run the code to stream the FMCW chirps and receive the reflected signals using the USRP
```
./FMCWImplementation
```

### 3. Processe the received signals in MATLAB
1. Go back to the [DIAG_USRP_Run_Radar.mlx](MATLAB/DIAG_USRP_Run_Radar.mlx) matlab file. You should now be on the section titled "RUN Experiment on USRP". 
2. Update the path to be the path specified by rx_file_folder_path in the c++'s .json config file. Then, run the section
3. In the section titled: "Configure the movie to focus in on specific areas of the range-doppler and CFAR plots, update the following parameters as follows: 
    * record_movie: on true, will generate a .gif of the range doppler, clustering, and detections over all frames and save them in the generated_plots folder
    * enable_simplified_plots: leave to true. This makes the plots look better for use in a paper
    * range_lims: the min and max range that detections should be made in
    * vel_lims: the min and max velocity for detections. All others will be filtered out
    * vel_exclusion_region: the min,max velocity to exclude. For example to exclude detections from static objects (velocity of zero), you could set this to be [-2,2] to filter out all objects with velocities of less than 2 m/s
    * frame_period_ms: the frame period that you used on the USRP
4. You should now be able to run all of the remaining sections. Note that all generated plots will be saved in the [generated_plots](MATLAB/generated_plots) folder.

### [Optional] Calibrating for USRP Tx and Rx timing Offset

I emperically found that there is an offset between the timing of the Tx and Rx on the USRP B210. To compensate for this, the last section in the [DIAG_USRP_Run_Radar.mlx](MATLAB/DIAG_USRP_Run_Radar.mlx) matlab live script file can be used to align the tx and rx chains in time. This will output a delay_us that should then be used to adjust the value of the "offset_us" term in the .json config file for the USRP.
