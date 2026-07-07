use super::user_interface_main;

/*------------------------------------ Enum ------------------------------------*/

#[derive(Clone, Copy)]
pub enum CommandAvailable {
    UNKNOWN_CMD = 0x00,
}

/*------------------------------------ Struct ------------------------------------*/

/* Define every register available in the alerts section*/
pub struct UiCommand {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,    /*Define if the section can or can't be access from user point of view*/
    pub command: user_interface_main::EnumField<CommandAvailable>,                                          /*Define which command is currently processed by the firmware*/
    pub command_assertion: user_interface_main::BoolField,                                                  /*Define whenever the command set has been process by the firmware, must be cleared by user*/ 
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiCommand {
        fn default () -> UiCommand{
            UiCommand {section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},
                       command: user_interface_main::EnumField {value: CommandAvailable::UNKNOWN_CMD},
                       command_assertion: user_interface_main::BoolField {value: false},
            }
        }
}

/*------------------------------------ Function ------------------------------------*/

pub fn ui_command_init() -> UiCommand{
    /*
    @brief: Init the external memory section for the user interface
    @input: None
    @output: (UiCommand) - The section fully initialised
    */
    println!("UI_Init: COMMAND");
    let mut ui_command_init = UiCommand::default();
    // Configure this section in READ_ONLY
    ui_command_init.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadWrite};
    return ui_command_init;
}