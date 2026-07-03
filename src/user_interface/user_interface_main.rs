
use super::user_interface_revision_status;
use super::user_interface_revision_status::UiRevisionStatus;

/*Define how each section can be access from the user point of view*/
pub enum UserInterfaceAccessControl {
    ReadOnly,
    WriteOnly,
    ReadWrite,
    NoReadNoWrite,
}

pub struct UserInterface {

    revision_status: UiRevisionStatus,
}



pub fn ui_init() {
    /*
    @brief: Init the user interface
    @input: None
    @output: None
    */

    /*Initialise REVISION_STATUS section*/
    println!("UI_Init: Starting initialisation");
    let revision_status = user_interface_revision_status::ui_revision_status_init();

    /* Output the user interface*/
    //let user_interface = UserInterface {revision_status: revision_status};
    //return user_interface;
}
