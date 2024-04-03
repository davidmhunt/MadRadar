# MADRadar Code Repository

The following repository was created to implement a simple FMCW radar on the USRP with offline processing using a MATLAB script

## Cloning the repository

To access the MadRadar code base, perform the following steps

1. Clone the MadRadar git repository and its submodules
```
git clone --recurse-submodules https://github.com/davidmhunt/CPSL_USRP_FMCW_Radar.git
```

If you forget to perform the --recurse-submodules when cloning the repository, you can use the following command to load the necessary submodules
```
git submodule update --init --recursive
```

## Next Steps

1. If you intend to run tests using the SDR interface for USRPB210's, you'll need to build the MadRadar_USRP module. In the [MadRadar_USRP](https://github.com/davidmhunt/MadRadar/tree/main/MadRadar_USRP) directory
2. If you just want to perform simulations, you can use the code in the [Matlab](https://github.com/davidmhunt/MadRadar/tree/main/MATLAB) directory.