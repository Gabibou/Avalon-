use super::user_interface_main;

/*------------------------------------ Struct ------------------------------------*/

pub struct PidAxis {
    pub kp: user_interface_main::IntegerField<u32>,                                                         /*Proportional coeff for PID stored as signed fixe point 8.24*/    
    pub ki: user_interface_main::IntegerField<u32>,                                                         /*Integrale coeff for PID stored as signed fixe point 8.24*/
    pub kd: user_interface_main::IntegerField<u32>,                                                         /*Derivative coeff for PID stored as signed fixe point 8.24*/
}


/* Define every register available in the alerts section*/
pub struct UiPidControl {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,    /*Define if the section can or can't be access from user point of view*/
    pub x_axis: PidAxis,
    pub y_axis: PidAxis,
    pub z_axis: PidAxis,
}   

/*------------------------------------ Impl default ------------------------------------*/
//TODO: Twick PID here as default settings
/* Add default value for this section of the User interface */
impl Default for UiPidControl {
        fn default () -> UiPidControl{
            UiPidControl {section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},
                          x_axis: PidAxis {kp: user_interface_main::IntegerField {value: 0x00},
                                   ki: user_interface_main::IntegerField {value: 0x00},
                                   kd: user_interface_main::IntegerField {value: 0x00},
                                    },
                          y_axis: PidAxis {kp: user_interface_main::IntegerField {value: 0x00},
                                   ki: user_interface_main::IntegerField {value: 0x00},
                                   kd: user_interface_main::IntegerField {value: 0x00},
                                    },
                          z_axis: PidAxis {kp: user_interface_main::IntegerField {value: 0x00},
                                   ki: user_interface_main::IntegerField {value: 0x00},
                                   kd: user_interface_main::IntegerField {value: 0x00},
                                    },                
            }
        }
}

/*------------------------------------ Function ------------------------------------*/

pub fn ui_pid_control_init() -> UiPidControl{
    /*
    @brief: Init the PID control section for the user interface
    @input: None
    @output: (UiPidControl) - The section fully initialised
    */
    println!("UI_Init: PID Control");
    let mut ui_pid_ctrl_struct = UiPidControl::default();
    // Configure this section in READ_ONLY
    ui_pid_ctrl_struct.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadWrite};
    return ui_pid_ctrl_struct;
}

