
use super::user_interface_main;

/*------------------------------------ Enum ------------------------------------*/

#[derive(Clone, Copy)]
pub enum SystemFsm {
    COLD_BOOT = 0x00,                                                       /*Used while hardware is initialising*/
    WARM_BOOT = 0x01,                                                       /*Used while firmware is initialising*/
    IDLE = 0x02,                                                            /*Used as default state. From here the device can jump to ARMED or CONFIGURATION only*/
    ARMED = 0x03,                                                           /*Used when flight computer enabled and ready to run. From here no safety are applied anymore*/
    FLIGHT = 0x04,                                                          /*Used when flight computer is fully running. See FlightFSM for more informations*/
    FAIL_SAFE = 0x05,                                                       /*Used when flight computer detect a CRASH or */
    CONFIGURATION = 0x06,                                                   /*Used to configure any settings of the device like PID, actuator range or engine throttle range*/
}

#[derive(Clone, Copy)]
pub enum FlightMode {
    MANUAL = 0x00,                                                          /*embedded flight computer will wait for user input to actuate every actuator, fully manual, all sensors are disabled*/
    STABILIZED = 0x01,                                                      /*embedded flight computer will wait for user input to change direction. 3 axis stabilisation is enabled */
    AUTO_PILOT = 0x02,                                                      /*embedded flight computer will follow the next GPS coordinate. All sensors enabled*/
}

#[derive(Clone, Copy)]
pub enum SystemErrors {
    NO_ERROR = 0x00,                                                       /*Default where no error are detected*/
}

#[derive(Clone, Copy)]
pub enum SystemWarnings {
    NO_WARNING = 0x00,                                                      /*Default where no error are detected*/           
}

/*------------------------------------ Struct ------------------------------------*/

/* Define every register available in the sytem status section*/
pub struct UiSystemStatus {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,        /*Define if the section can or can't be access from user point of view*/
    pub system_fsm: user_interface_main::EnumField<SystemFsm>,                                                  /*Define the current firmware state*/
    pub system_error: user_interface_main::EnumField<SystemErrors>,                                             /*Define the last error state that occurs, this might block the transition to ARMED state*/        
    pub system_warning: user_interface_main::EnumField<SystemWarnings>,                                         /*Define the last warning state that occurs*/
    pub flight_mode: user_interface_main::EnumField<FlightMode>,                                                /*Define the method use to drive the vehicle (see FlightMode for more informations)*/
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiSystemStatus {
        fn default () -> UiSystemStatus{
            UiSystemStatus {section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},
                            system_fsm: user_interface_main::EnumField {value: SystemFsm::COLD_BOOT},
                            system_error: user_interface_main::EnumField {value: SystemErrors::NO_ERROR},
                            system_warning: user_interface_main::EnumField {value: SystemWarnings::NO_WARNING},
                            flight_mode: user_interface_main::EnumField {value: FlightMode::MANUAL},          
            }
        }
}

/*------------------------------------ Function ------------------------------------*/

pub fn ui_system_status_init() -> UiSystemStatus{
    /*
    @brief: Init the system status section for the user interface
    @input: None
    @output: (UiSystemStatus) - The section fully initialised
    */
    println!("UI_Init: SYSTEM_STATUS");
    let mut ui_system_status_struct = UiSystemStatus::default();
    // Configure this section in READ_ONLY
    ui_system_status_struct.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadOnly};
    return ui_system_status_struct;
}