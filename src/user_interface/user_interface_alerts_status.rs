use super::user_interface_main;

/*------------------------------------ Struct ------------------------------------*/

/* Define every register available in the alerts section*/
pub struct UiAlertsStatus {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,    /*Define if the section can or can't be access from user point of view*/
    pub over_current_protection_flag: user_interface_main::IntegerField<u8>,                                /*Flag raised when OCP flags raised by the power supply*/
    pub over_voltage_protection_flag: user_interface_main::IntegerField<u8>,                                /*Flag raised when OVP flag raised by power supply*/
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiAlertsStatus {
        fn default () -> UiAlertsStatus{
            UiAlertsStatus {section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},
                            over_current_protection_flag: user_interface_main::IntegerField {value: 0x00},
                            over_voltage_protection_flag: user_interface_main::IntegerField {value: 0x00},        
            }
        }
}

/*------------------------------------ Function ------------------------------------*/

pub fn ui_alerts_status_init() -> UiAlertsStatus{
    /*
    @brief: Init the alerts status section for the user interface
    @input: None
    @output: (UiAlertsStatus) - The section fully initialised
    */
    println!("UI_Init: ALERTS_STATUS");
    let mut ui_alerts_status_struct = UiAlertsStatus::default();
    // Configure this section in READ_ONLY
    ui_alerts_status_struct.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadOnly};
    return ui_alerts_status_struct;
}