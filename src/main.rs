mod user_interface;
mod bsp;

use crate::user_interface::user_interface_main::Field;
use bsp::Board;
use bsp::bsp_main::GenericBoard


fn main() {

    /*Init the user interface*/
    let mut user_interface = user_interface::user_interface_main::ui_init();
    let mut board = Board::new();
    board.init();

    // let test = user_interface.revision_status.section_access.get();
    // user_interface.revision_status.section_access.set(user_interface::user_interface_main::UserInterfaceAccessControl::WriteOnly);
    // let tes1 = user_interface.revision_status.section_access.get();

    // let test1 = user_interface.revision_status.board.unique_identifier.get();
    // let test2 = user_interface.system_status.flight_mode.get();
    // let test3 = user_interface.system_status.flight_mode.get();


/*    while true {
        println!("Loop")
    }*/
}