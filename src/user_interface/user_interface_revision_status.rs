
use super::user_interface_main;

/* Define every register available in the revision status section*/
pub struct UiRevisionStatus {
    section_access: user_interface_main::UserInterfaceAccessControl,        /*Define if the section can or can't be access from user point of view*/
    mcu: UiRevisionStatusRegMcu,                                            /*Define on which mcu the firmware is running and it's unique identifier*/
    board: UiRevisionStatusRegBoard,                                        /*Define on which board the firmware is running*/
    ui_revision: UiRevisionStatusRegRevision,                               /*Define the user interface revision status*/
    fw_revision: UiRevisionStatusRegRevision,        /*Define the firmware revision status*/
}

/*Define every identification used in the mcu*/
struct UiRevisionStatusRegMcu {
    unique_identifier: u64,                         /*Single ID generated based on chip wafer position X,Y + Lot ID*/
    model_identifier: u32,                          /*Model identification based on compilation target*/ 
}

/*Define every identification used in the board*/
struct UiRevisionStatusRegBoard {
    unique_identifier: u64,
    identifier: u32,
    revision: u8,
}

/*Define how version are considered in the UI*/
struct UiRevisionStatusRegRevision {
    major: u8,
    minor: u8,
    bugfix: u8,
}

/* Define every MCU available in this project */
enum McuModelIdentifier {
    STM32G474 = 0x7474
}




pub fn ui_revision_status_init() {
    /*
    @brief: Init the revision status section for the user interface
    @input: None
    @output: (UiRevisionStatus) - The section fully initialised
    */
    println!("UI_Init: REVISION_STATUS");

    let toto = 12;
    let ui_revision_status_struct = UiRevisionStatus;
    return ui_revision_status_struct;
}

#[cfg(mcu_stm32g474 = "linux")]
fn get_mcu_identification(){
    /*
    @brief: Retrieve the mcu identification code and compute the UUID 
    @input: None
    @output: (UiRevisionStatusRegMcu) - MCU identification structure
    @note: Function only compiled when target is STM32G474
    */
    let computed_identifier = 0x12345678;
    let mcu_identification_t = UiRevisionStatusRegMcu {unique_identifier: computed_identifier, model_identifier: McuModelIdentifier::STM32G474};
    return mcu_identification_t;
}