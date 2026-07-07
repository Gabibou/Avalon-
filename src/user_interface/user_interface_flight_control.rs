use super::user_interface_main;

/*------------------------------------ Struct ------------------------------------*/

/* Define every register available in the flight control section*/
pub struct UiFlightControl {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,    /*Define if the section can or can't be access from user point of view*/
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiFlightControl {
        fn default () -> UiFlightControl{
            UiFlightControl {section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},

            }
        }
}

/*------------------------------------ Function ------------------------------------*/

pub fn ui_flight_control_init() -> UiFlightControl{
    /*
    @brief: Init the flight control section for the user interface
    @input: None
    @output: (UiFlightControl) - The section fully initialised
    */
    println!("UI_Init: FLIGHT_CONTROL");
    let mut ui_flight_control_init = UiFlightControl::default();
    // Configure this section in READ_ONLY
    ui_flight_control_init.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadWrite};
    return ui_flight_control_init;
}