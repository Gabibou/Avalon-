use super::user_interface_main;

/*------------------------------------ Struct ------------------------------------*/
// Define how the IMU can be configured by keeping a HIGH level of abstraction
pub struct UiGyroscope {
    pub enable: user_interface_main::BoolField,                                         /*Enable or disable the Gyroscope from the Inertial measurement unit (true=enable, false=disable)*/
    pub sampling_freq: user_interface_main::IntegerField<u8>                            /*Define the sampling frequency in Hz of the gyroscope*/
    pub range: user_interface_main::IntegerField<u8>                                    /*Define the gyroscope range (check bno055 datasheet for more informations)*/
    pub low_pass_filter: user_interface_main::IntegerField<u8>                          /*Define how the gyroscope output if filtered (check bno055 datasheet for more informations)*/
    pub high_g_detection_enabled: user_interface_main::BoolField,                       /*Enable or disable the HIGH_G detection feature (true=enabled, false=disabled)*/
    pub high_g_detection_threshold: user_interface_main::IntegerField<u8>,              /*Configure the threshold of the HIGH-G detection feature*/
    pub high_g_detection_time_ms: user_interface_main::IntegerField<u8>,                /*Configure how many sample should be greater than the threshold before triggering HIGH-G*/
}

pub struct UiAccelerometer {
    pub enable: user_interface_main::BoolField,                                         /*Enable or disable the Accelerometer from the Inertial measurement unit (true=enable, false=disable)*/
    pub sampling_freq: user_interface_main::IntegerField<u8>                            /*Define the sampling frequency in Hz of the accelerometer*/
    pub range: user_interface_main::IntegerField<u8>                                    /*Define the accelerometer range (check bno055 datasheet for more informations)*/
    pub low_pass_filter: user_interface_main::IntegerField<u8>                          /*Define how the accelerometer output if filtered (check bno055 datasheet for more informations)*/
    pub high_g_detection_enabled: user_interface_main::BoolField,                       /*Enable or disable the HIGH_G detection feature (true=enabled, false=disabled)*/
    pub high_g_detection_threshold: user_interface_main::IntegerField<u8>,              /*Configure the threshold of the HIGH-G detection feature*/
    pub high_g_detection_time_ms: user_interface_main::IntegerField<u8>,                /*Configure how many sample should be greater than the threshold before triggering HIGH-G*/
}

pub struct UiMagnetometer {
    pub enable: user_interface_main::BoolField,                                         /*Enable or disable the Magnetometer from the Inertial measurement unit (true=enable, false=disable)*/
    pub sampling_freq: user_interface_main::IntegerField<u8>                            /*Define the sampling frequency in Hz of the magnetometer*/
    pub range: user_interface_main::IntegerField<u8>                                    /*Define the magnetometer range (check bno055 datasheet for more informations)*/
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
    pub power_mode: user_interface_main::IntegerField<u8>,                              /*Select the power mode used by the sensor (check bmp390 for more informations)*/    
    pub temperature_range: user_interface_main::IntegerField<u8>,                       /*Configure temperature range captured by the sensor*/
        pub pressure_range: user_interface_main::IntegerField<u8>,                      /*Configure pressure range captured by the sensor*/
}

pub struct UiGpsRawData {
    pub longitude_integer: user_interface_main::IntegerField<u8>,                       /*Integer part of the longitude 4 for 4.3959847894 for ex*/
    pub longitude_float: user_interface_main::IntegerField<u32>,                        /*Float part of the longitude 3959847894 for 4.3959847894 for ex*/
    pub latitude_integer: user_interface_main::IntegerField<u8>,                        /*Integer part of the latitude 4 for 4.3959847894 for ex*/
    pub latitude_float: user_interface_main::IntegerField<u32>,                         /*Float part of the latitude 3959847894 for 4.3959847894 for ex*/
    pub altitude_integer: user_interface_main::IntegerField<u8>,                        /*Integer part of the altitude 4 for 4.3959847894 for ex*/
    pub altitude_float: user_interface_main::IntegerField<u32>,                         /*Float part of the altitude 3959847894 for 4.3959847894 for ex*/
}

pub struct UiGps {
    pub enable: user_interface_main::BoolField,
    pub baud_rate_bps: user_interface_main::IntegerField<u32>,                          /*Uart baud rate configured between sensors and device*/
    pub raw_data: UiGpsRawData,                                                         /*Store GPS raw data*/
}


/* Define every register available in the sensor control section*/
pub struct UiSensorsControl {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,    /*Define if the section can or can't be access from user point of view*/
    pub imu: UiInertialMeasurementUnit,
    pub barometer: UiBarometer,
    pub gps: UiGps,
    pub thermal_sensor:
    pub current_sensing:
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiSensorsControl {
        fn default () -> UiSensorsControl{
            UiSensorsControl {section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},
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