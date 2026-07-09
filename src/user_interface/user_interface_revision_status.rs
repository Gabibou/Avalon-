
use super::user_interface_main;

/*------------------------------------ Const ------------------------------------*/

const UI_REVISION: UiRevisionStatusRegRevision = UiRevisionStatusRegRevision {major: user_interface_main::IntegerField {value: 0x00},
                                                                              minor: user_interface_main::IntegerField {value: 0x00},
                                                                              bugfix: user_interface_main::IntegerField {value: 0x00}};
const FW_REVISION: UiRevisionStatusRegRevision = UiRevisionStatusRegRevision {major: user_interface_main::IntegerField {value: 0x00},
                                                                              minor: user_interface_main::IntegerField {value: 0x00},
                                                                              bugfix: user_interface_main::IntegerField {value: 0x00}};

/*------------------------------------ Struct ------------------------------------*/

/* Define every register available in the revision status section*/
pub struct UiRevisionStatus {
    pub section_access: user_interface_main::EnumField<user_interface_main::UserInterfaceAccessControl>,   /*Define if the section can or can't be access from user point of view*/
    pub mcu: UiRevisionStatusRegMcu,                                                                       /*Define on which mcu the firmware is running and it's unique identifier*/
    pub board: UiRevisionStatusRegBoard,                                                                   /*Define on which board the firmware is running*/
    pub ui_revision: UiRevisionStatusRegRevision,                                                          /*Define the user interface revision status*/
    pub fw_revision: UiRevisionStatusRegRevision,                                                          /*Define the firmware revision status*/
}

/*Define every identification used in the mcu*/
pub struct UiRevisionStatusRegMcu {
    pub unique_identifier: user_interface_main::IntegerField<u32>,                                           /*Single ID generated based on chip wafer position X,Y + Lot ID*/
    pub build_model_identifier: user_interface_main::IntegerField<u8>,                                       /*Model identification based on compilation target*/ 
}

/*Define every identification used in the board*/
pub struct UiRevisionStatusRegBoard {
    pub unique_identifier: user_interface_main::IntegerField<u32>,                                          /*Unique identifier read from embedded FLASH or EEPROM*/
    pub build_model_identifier:  user_interface_main::IntegerField<u8>,                                     /*Selected build board model identification*/        
    pub revision: user_interface_main::IntegerField<u8>,                                                    /*Board revision read from embedded FLASH or EEPROM*/
}

/*Define how version are considered in the UI*/
pub struct UiRevisionStatusRegRevision {
    pub major: user_interface_main::IntegerField<u8>,                                                      /*Major version update*/
    pub minor: user_interface_main::IntegerField<u8>,                                                      /*Minor version update*/
    pub bugfix: user_interface_main::IntegerField<u8>,                                                     /*Bufgix version update*/    
}

/*------------------------------------ Impl default ------------------------------------*/

/* Add default value for this section of the User interface */
impl Default for UiRevisionStatus {
        fn default () -> UiRevisionStatus{
            UiRevisionStatus{section_access: user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::NoReadNoWrite}, 
                             mcu: UiRevisionStatusRegMcu {unique_identifier: user_interface_main::IntegerField {value: 0x00000000},
                                                          build_model_identifier: user_interface_main::IntegerField {value: 0x00}},
                             board: UiRevisionStatusRegBoard {unique_identifier: user_interface_main::IntegerField {value: 0x00000000},
                                                              build_model_identifier: user_interface_main::IntegerField {value: 0x00},
                                                              revision: user_interface_main::IntegerField {value: 0x00}},
                             ui_revision: UiRevisionStatusRegRevision {major: user_interface_main::IntegerField {value: 0x00},
                                                                       minor: user_interface_main::IntegerField {value: 0x00},
                                                                       bugfix: user_interface_main::IntegerField {value: 0x00}},
                             fw_revision: UiRevisionStatusRegRevision {major: user_interface_main::IntegerField {value: 0x00},
                                                                       minor: user_interface_main::IntegerField {value: 0x00},
                                                                       bugfix: user_interface_main::IntegerField {value: 0x00}},
            }
        }
}

/*------------------------------------ Function ------------------------------------*/

pub fn ui_revision_status_init() -> UiRevisionStatus{
    /*
    @brief: Init the revision status section for the user interface
    @input: None
    @output: (UiRevisionStatus) - The section fully initialised
    */
    println!("UI_Init: REVISION_STATUS");
    let mut ui_revision_status_struct = UiRevisionStatus::default();

    // Configure this section in READ_ONLY
    ui_revision_status_struct.section_access = user_interface_main::EnumField {value: user_interface_main::UserInterfaceAccessControl::ReadOnly};
    // Configure firmware and ui revision configured at the top of this file
    ui_revision_status_struct.ui_revision = UI_REVISION;
    ui_revision_status_struct.fw_revision = FW_REVISION;
    return ui_revision_status_struct;
}