%% Used for SIMULINK Model


classdef Target_revA < handle
    %TARGET class used to support target generation in the Simulink
    %simulation

    properties (Access = public)
        position_m
        velocity_meters_per_s
        rcs_sq_meters
        operating_frequency_Hz

        %key FMCW parameters
        radar_target            %phased.RadarTarget object
        platform                %phased.Platform object
    end
    
    methods
        function obj = Target_revA(position_m,velocity_meters_per_s,rcs_sq_meters,operating_frequency_Hz)
            %{
                Purpose: creates an instance of the Target Class 
                Inputs:
                    position_m: the position of the target specified as
                        [x,y,z] in meters
                    velocity_meters_per_s: the velocity of the target
                        specified as [x,y,z] in meters
                    rcs_sq_meters: the radar cross section
                    operating_frequency_HZ: the operating frequency in Hz
            %}
            obj.position_m = position_m;
            obj.rcs_sq_meters = rcs_sq_meters;
            obj.velocity_meters_per_s = velocity_meters_per_s;
            obj.operating_frequency_Hz = operating_frequency_Hz;

            obj.configure_target_FMCW_params();
        end

        function  configure_target_FMCW_params(obj)
            %{
                Purpose: compute the relevant parameters needed for the
                    FMCW simulation
            %}

            obj.radar_target = phased.RadarTarget( ...
                'MeanRCS', obj.rcs_sq_meters, ...
                'PropagationSpeed',physconst('LightSpeed'), ...
                'OperatingFrequency', obj.operating_frequency_Hz);
            obj.platform = phased.Platform( ...
                'InitialPosition', obj.position_m, ...
                'Velocity',obj.velocity_meters_per_s);
        end
    end
end