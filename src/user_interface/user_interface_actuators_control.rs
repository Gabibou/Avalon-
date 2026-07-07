use super::user_interface_main;

/*------------------------------------ Struct ------------------------------------*/

/* Define every register available in the actuator control section*/
pub struct UiActuatorsControl {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,    /*Define if the section can or can't be access from user point of view*/
    pub command: user_interface_main::EnumField<CommandAvailable>,                                          /*Define which command is currently processed by the firmware*/
    pub command_assertion: user_interface_main::BoolField,                                                  /*Define whenever the command set has been process by the firmware, must be cleared by user*/ 
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiActuatorsControl {
        fn default () -> UiActuatorsControl{
            UiActuatorsControl {section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},
                       command: user_interface_main::EnumField {value: CommandAvailable::UNKNOWN_CMD},
                       command_assertion: user_interface_main::BoolField {value: false},
            }
        }
}

/*------------------------------------ Function ------------------------------------*/

pub fn ui_actuators_control_init() -> UiActuatorsControl{
    /*
    @brief: Init the actuator control section for the user interface
    @input: None
    @output: (UiActuatorsControl) - The section fully initialised
    */
    println!("UI_Init: ACTUATOR_CONTROL");
    let mut ui_actuator_control_init = UiActuatorsControl::default();
    // Configure this section in READ_ONLY
    ui_actuator_control_init.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadWrite};
    return ui_actuator_control_init;
}