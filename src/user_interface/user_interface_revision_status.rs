
use super::user_interface_main;

/*------------------------------------ Const ------------------------------------*/

const UI_REVISION: UiRevisionStatusRegRevision = UiRevisionStatusRegRevision {major: user_interface_main::IntegerField {value: 0x00},
                                                                              minor: user_interface_main::IntegerField {value: 0x00},
                                                                              bugfix: user_interface_main::IntegerField {value: 0x00}};
const FW_REVISION: UiRevisionStatusRegRevision = UiRevisionStatusRegRevision {major: user_interface_main::IntegerField {value: 0x00},
                                                                              minor: user_interface_main::IntegerField {value: 0x00},
                                                                              bugfix: user_interface_main::IntegerField {value: 0x00}};

/*------------------------------------ Enum ------------------------------------*/

/* Define every MCU available in this project */
#[derive(Clone, Copy)]
pub enum McuModelIdentifier {
    DEFAULT = 0x0000,
    STM32G474 = 0x7474,                                                      
}

/* Define every board available in this project */
#[derive(Clone, Copy)]
pub enum BoardModelIdentifier {
    AVALON_STM32G4_V1 = 0x01,
    AVALON_STM32G4_V2 = 0x02,
}

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
    pub unique_identifier: user_interface_main::IntegerField<u32>,                                         /*Single ID generated based on chip wafer position X,Y + Lot ID*/
    pub build_model_identifier: user_interface_main::EnumField<McuModelIdentifier>,                        /*Model identification based on compilation target*/ 
}

/*Define every identification used in the board*/
pub struct UiRevisionStatusRegBoard {
    pub unique_identifier: user_interface_main::IntegerField<u32>,                                         /*Unique identifier read from embedded FLASH or EEPROM*/
    pub build_model_identifier: user_interface_main::EnumField<BoardModelIdentifier>,                      /*Selected build board model identification*/        
    pub revision: user_interface_main::IntegerField<u8>,                                                   /*Board revision read from embedded FLASH or EEPROM*/
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
                                                          build_model_identifier: user_interface_main::EnumField {value: McuModelIdentifier::DEFAULT}},
                             board: UiRevisionStatusRegBoard {unique_identifier: user_interface_main::IntegerField {value: 0x00000000},
                                                              build_model_identifier: user_interface_main::EnumField {value: BoardModelIdentifier::AVALON_STM32G4_V2},
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
    // Configure the mcu revision
    ui_revision_status_struct.mcu = get_mcu_identifier();
    // Configure the board revision
    ui_revision_status_struct.board = get_board_identifier();
    return ui_revision_status_struct;
}

/*------------------------------------ Conditional compilation ------------------------------------*/

#[cfg(feature="mcu_stm32g474")]
fn get_mcu_identifier() -> UiRevisionStatusRegMcu{
    /*
    @brief: Compute unique identifier for STM32G474 and add build model identifier
    @input: None
    @output: (UiRevisionStatusRegMcu) - mcu revision and unique identifier
    */
    // DBGMCU_IDCODE address pointer
    let dbgmcu_idcode_pointer = 0xE0042000 as *const u32;
    // Read the DBGMCU_IDCODE register
    let value = unsafe { dbgmcu_idcode_pointer.read_volatile() };
    // Apply to the struct the value
    // TODO: Add correct uuid computation below --> set to 0x11 for now
    let mcu_identifier_struct = UiRevisionStatusRegMcu{build_model_identifier: user_interface_main::EnumField {value: McuModelIdentifier::STM32G474},
                                                       unique_identifier: user_interface_main::IntegerField {value: 0x11}};
    return mcu_identifier_struct;
}

#[cfg(feature="board_avalon_g474_rev_b")]
fn get_board_identifier() -> UiRevisionStatusRegBoard{
    /*
    @brief: Read the embedded eeprom to retrieve board identifier and revision
    @input: None
    @output: (UiRevisionStatusRegMcu) - board revision and identifier
    */

    //TODO: Add board EEPROM readings 

    let board_identifier_struct = UiRevisionStatusRegBoard{build_model_identifier: user_interface_main::EnumField {value: BoardModelIdentifier::AVALON_STM32G4_V2},
                                                           unique_identifier: user_interface_main::IntegerField {value: 0x11},
                                                           revision: user_interface_main::IntegerField {value: 0x00}};
    return board_identifier_struct;
}

#[cfg(feature="x86_x64_debug_target")]
fn get_mcu_identifier() -> UiRevisionStatusRegMcu{
    /*
    @brief: Generate default identifier for MCU when running on debug mode on x86 target
    @input: None
    @output: (UiRevisionStatusRegMcu) - mcu revision and unique identifier
    */
    let mcu_identifier_struct = UiRevisionStatusRegMcu{build_model_identifier: user_interface_main::EnumField {value: McuModelIdentifier::DEFAULT},
                                                       unique_identifier: user_interface_main::IntegerField {value: 0xCAFECAFE}};
    return mcu_identifier_struct;
}

#[cfg(feature="x86_x64_debug_target")]
fn get_board_identifier() -> UiRevisionStatusRegBoard{
    /*
    @brief: Read the embedded eeprom to retrieve board identifier and revision
    @input: None
    @output: (UiRevisionStatusRegMcu) - board revision and identifier
    */

    let board_identifier_struct = UiRevisionStatusRegBoard{build_model_identifier: user_interface_main::EnumField {value: BoardModelIdentifier::AVALON_STM32G4_V1},
                                                           unique_identifier: user_interface_main::IntegerField {value: 0xCAFECAFE},
                                                           revision: user_interface_main::IntegerField {value: 0x00}};
    return board_identifier_struct;
}