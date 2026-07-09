use super::user_interface_main;

/*------------------------------------ Struct ------------------------------------*/
// Define how the IMU can be configured by keeping a HIGH level of abstraction
pub struct UiGyroscope {
    pub enable: user_interface_main::BoolField,                                         /*Enable or disable the Gyroscope from the Inertial measurement unit (true=enable, false=disable)*/
    pub sampling_freq: user_interface_main::IntegerField<u8>,                           /*Define the sampling frequency in Hz of the gyroscope*/
    pub range: user_interface_main::IntegerField<u8>,                                   /*Define the gyroscope range (check bno055 datasheet for more informations)*/
    pub low_pass_filter: user_interface_main::IntegerField<u8>,                         /*Define how the gyroscope output if filtered (check bno055 datasheet for more informations)*/
    pub high_g_detection_enabled: user_interface_main::BoolField,                       /*Enable or disable the HIGH_G detection feature (true=enabled, false=disabled)*/
    pub high_g_detection_threshold: user_interface_main::IntegerField<u8>,              /*Configure the threshold of the HIGH-G detection feature*/
    pub high_g_detection_time_ms: user_interface_main::IntegerField<u16>,               /*Configure how many sample should be greater than the threshold before triggering HIGH-G*/
}

pub struct UiAccelerometer {
    pub enable: user_interface_main::BoolField,                                         /*Enable or disable the Accelerometer from the Inertial measurement unit (true=enable, false=disable)*/
    pub sampling_freq: user_interface_main::IntegerField<u8>,                           /*Define the sampling frequency in Hz of the accelerometer*/
    pub range: user_interface_main::IntegerField<u8>,                                   /*Define the accelerometer range (check bno055 datasheet for more informations)*/
    pub low_pass_filter: user_interface_main::IntegerField<u8>,                         /*Define how the accelerometer output if filtered (check bno055 datasheet for more informations)*/
    pub high_g_detection_enabled: user_interface_main::BoolField,                       /*Enable or disable the HIGH_G detection feature (true=enabled, false=disabled)*/
    pub high_g_detection_threshold: user_interface_main::IntegerField<u8>,              /*Configure the threshold of the HIGH-G detection feature*/
    pub high_g_detection_time_ms: user_interface_main::IntegerField<u16>,               /*Configure how many sample should be greater than the threshold before triggering HIGH-G*/
}

pub struct UiMagnetometer {
    pub enable: user_interface_main::BoolField,                                         /*Enable or disable the Magnetometer from the Inertial measurement unit (true=enable, false=disable)*/
    pub sampling_freq: user_interface_main::IntegerField<u8>,                           /*Define the sampling frequency in Hz of the magnetometer*/
    pub range: user_interface_main::IntegerField<u8>,                                   /*Define the magnetometer range (check bno055 datasheet for more informations)*/
}

pub struct UiInertialMeasurementUnit {
    pub enable: user_interface_main::BoolField,                                         /*Enable or disable fully the IMU (true=enabled, false=disabled)*/
    pub accelerometer: UiAccelerometer,                                                 
    pub magnetometer: UiMagnetometer,
    pub gyroscope: UiGyroscope,
    pub axis_remap: user_interface_main::IntegerField<u8>,
}

pub struct UiBarometer {
    pub enable: user_interface_main::BoolField,                                         /*Enable or disable fully the barometer (true=enabled, false=disabled)*/
    pub temperature_range: user_interface_main::IntegerField<u8>,                       /*Configure temperature range captured by the sensor*/
    pub pressure_range: user_interface_main::IntegerField<u8>,                         /*Configure pressure range captured by the sensor*/
}

pub struct UiGps {
    pub enable: user_interface_main::BoolField,
    pub baud_rate_bps: user_interface_main::IntegerField<u32>,                          /*Uart baud rate configured between sensors and device*/
}

pub struct UiThermalSensor {
    pub sampling_frequency: user_interface_main::IntegerField<u16>,                     /*Define at which frequency the thermal sensor will be sampled*/
    pub enable: user_interface_main::BoolField,                                         /*Define if the thermal sensor is read or not (true=enabled, false=disabled)*/
}

pub struct UiVoltageSensing {
    pub enable: user_interface_main::BoolField,                                         /*Define if the voltage sensing should be enable or not (true=enabled, false=disabled)*/
    pub sampling_frequency: user_interface_main::IntegerField<u16>,                     /*Define at which frequency the voltage sensor will be sampled*/
    pub ovp_threshold_mv: user_interface_main::IntegerField<u16>,                       /*Define the threshold for over voltage protection in mV. A flag will be raised if pass*/
}

pub struct UiCurrentSensing {
    pub enable: user_interface_main::BoolField,                                         /*Define if the current sensing should be enable or not (true=enabled, false=disabled)*/
    pub sampling_frequency: user_interface_main::IntegerField<u16>,                     /*Define at which frequency the current sensor will be sampled*/
    pub ocp_threshold_ma: user_interface_main::IntegerField<u16>,                       /*Define the threshold for over current protection in mA. A flag will be raised if pass*/
}

pub struct UiCurrentVoltageSensing {
    pub engine_current: UiCurrentSensing,                                               /*All current sensing connected to engine only*/
    pub avionics_current: UiCurrentSensing,                                             /*All current sensing connected to electronics and avionics only*/
    pub engine_voltage: UiVoltageSensing,
    pub avionics_voltage: UiVoltageSensing,                                 
}

/* Define every register available in the sensor control section*/
pub struct UiSensorsControl {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,    /*Define if the section can or can't be access from user point of view*/
    pub imu: UiInertialMeasurementUnit,
    pub barometer: UiBarometer,
    pub gps: UiGps,
    pub thermal_sensor: UiThermalSensor,
    pub current_voltage_sensing: UiCurrentVoltageSensing,
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiSensorsControl {
        fn default () -> UiSensorsControl{
            UiSensorsControl {
                section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},
                imu: UiInertialMeasurementUnit {
                    enable: user_interface_main::BoolField {value: false},
                    axis_remap: user_interface_main::IntegerField {value: 0x00},
                    
                    accelerometer: UiAccelerometer {
                        enable: user_interface_main::BoolField {value: false},
                        sampling_freq: user_interface_main::IntegerField {value: 0x64},
                        range: user_interface_main::IntegerField {value: 0x00},
                        low_pass_filter: user_interface_main::IntegerField {value: 0x00},
                        high_g_detection_enabled: user_interface_main::BoolField {value: false},
                        high_g_detection_threshold: user_interface_main::IntegerField {value: 0x03},
                        high_g_detection_time_ms: user_interface_main::IntegerField {value: 0x1F4},
                    },
                    magnetometer: UiMagnetometer {
                        enable: user_interface_main::BoolField {value: false},
                        sampling_freq: user_interface_main::IntegerField{value: 0x0F},                     
                        range: user_interface_main::IntegerField{value: 0x00},     
                    },
                    gyroscope: UiGyroscope {
                        enable: user_interface_main::BoolField {value: false},
                        sampling_freq: user_interface_main::IntegerField {value: 0x64},
                        range: user_interface_main::IntegerField {value: 0x00},
                        low_pass_filter: user_interface_main::IntegerField {value: 0x00},
                        high_g_detection_enabled: user_interface_main::BoolField {value: false},
                        high_g_detection_threshold: user_interface_main::IntegerField {value: 0x03},
                        high_g_detection_time_ms: user_interface_main::IntegerField {value: 0x1F4},
                    },                                                             
                },

                barometer: UiBarometer {
                    enable: user_interface_main::BoolField {value: false},
                    temperature_range: user_interface_main::IntegerField {value: 0x00},
                    pressure_range: user_interface_main::IntegerField {value: 0x00},
                },

                gps: UiGps {
                    enable: user_interface_main::BoolField {value: false},
                    baud_rate_bps: user_interface_main::IntegerField {value: 0x2580},
                },

                thermal_sensor: UiThermalSensor{
                    enable: user_interface_main::BoolField {value: false},      
                    sampling_frequency: user_interface_main::IntegerField {value: 0x64},  
                },

                current_voltage_sensing: UiCurrentVoltageSensing{
                    engine_current: UiCurrentSensing {
                        enable: user_interface_main::BoolField {value: true},
                        sampling_frequency: user_interface_main::IntegerField {value: 0x64},  
                        ocp_threshold_ma: user_interface_main::IntegerField {value: 0x4E20},  // 20A default OCP for engine --> To adjust based on board       
                    },
                    avionics_current: UiCurrentSensing {
                        enable: user_interface_main::BoolField {value: true},
                        sampling_frequency: user_interface_main::IntegerField {value: 0x64},
                        ocp_threshold_ma:  user_interface_main::IntegerField {value: 0x1F4},  // 500mA default OCP for avionics --> To adjust based on board        
                    },
                    engine_voltage: UiVoltageSensing{
                        enable: user_interface_main::BoolField {value: true},
                        sampling_frequency: user_interface_main::IntegerField {value: 0x64},
                        ovp_threshold_mv:  user_interface_main::IntegerField {value: 0x1B58}, // 7V default for OVP 
                    },   
                    avionics_voltage: UiVoltageSensing{
                        enable: user_interface_main::BoolField {value: true},
                        sampling_frequency: user_interface_main::IntegerField {value: 0x64},
                        ovp_threshold_mv:  user_interface_main::IntegerField {value: 0x1B58}, // 7V default for OVP 
                    },          
                },

            }
        }

}


/*------------------------------------ Function ------------------------------------*/

pub fn ui_sensors_control_init() -> UiSensorsControl{
    /*
    @brief: Init the sensor control section for the user interface
    @input: None
    @output: (UiSensorsControl) - The section fully initialised
    */
    println!("UI_Init: SENSOR_CONTROL");
    let mut ui_sensor_control_init = UiSensorsControl::default();
    // Configure this section in READ_ONLY
    ui_sensor_control_init.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadWrite};
    return ui_sensor_control_init;
}