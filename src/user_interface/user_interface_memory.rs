use super::user_interface_main;

/*------------------------------------ Enum ------------------------------------*/

#[derive(Clone, Copy)]
pub enum ExternalMemProtocol {
    SPI = 0x00,
    I2C = 0x01,
    QSPI = 0x02,
}

/*------------------------------------ Struct ------------------------------------*/

/* Define every register available in the alerts section*/
pub struct UiMemory {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,    /*Define if the section can or can't be access from user point of view*/
    pub external_memory_enable: user_interface_main::BoolField,                                             /*Define if the external memory should be enabled or not*/
    pub memory_size_Kbytes: user_interface_main::IntegerField<u32>,                                         /*Define the size of the external memory applied in KBytes*/
    pub memory_access_protocol: user_interface_main::EnumField<ExternalMemProtocol>,                        /*Define the protocol used to speak to this external memory*/
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiMemory {
        fn default () -> UiMemory{
            UiMemory {section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite},
                      external_memory_enable: user_interface_main::BoolField {value: false},
                      memory_size_Kbytes: user_interface_main::IntegerField {value: 125000},
                      memory_access_protocol: user_interface_main::EnumField {value: ExternalMemProtocol::SPI},             
            }
        }
}

/*------------------------------------ Function ------------------------------------*/

pub fn ui_memory_init() -> UiMemory{
    /*
    @brief: Init the external memory section for the user interface
    @input: None
    @output: (UiMemory) - The section fully initialised
    */
    println!("UI_Init: EXTERNAL_MEMORY");
    let mut ui_memory_struct = UiMemory::default();
    // Configure this section in READ_ONLY
    ui_memory_struct.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadWrite};
    return ui_memory_struct;
}