
use super::user_interface_revision_status;
use super::user_interface_system_status;
use super::user_interface_revision_status::UiRevisionStatus;
use super::user_interface_system_status::UiSystemStatus;

/*------------------------------------ Enum ------------------------------------*/

/*Define how each section can be access from the user point of view*/
#[derive(Clone, Copy)]
pub enum UserInterfaceAccessControl {
    ReadOnly,
    WriteOnly,
    ReadWrite,
    NoReadNoWrite,
}

/*------------------------------------ Trait ------------------------------------*/

pub trait Field {
    type Value;

    fn get(&self) -> Self::Value;
    fn set(&mut self, value: Self::Value);
}

/*------------------------------------ Struct ------------------------------------*/

pub struct UserInterface {

    pub revision_status: UiRevisionStatus,
    pub system_status: UiSystemStatus,
}

pub struct BoolField {
    pub value: bool,
}

pub struct IntegerField<T> {
    pub value: T,
}

pub struct EnumField<E> {
    pub value: E,
}

/*------------------------------------ Function ------------------------------------*/

/* 
Add a Field in every type used in the UI to make it simpler when accessing the UI
Added also a get and set in every field so each can be access with these methods
*/
impl Field for BoolField {
    type Value = bool;

    fn get(&self) -> bool{
        return self.value
    }

    fn set(&mut self, value: bool){
        self.value = value
    }
}

impl <T: Copy> Field for IntegerField<T> {
    type Value = T;

    fn get(&self) -> T{
        return self.value
    }

    fn set(&mut self, value: T){
        self.value = value;
    }
}

impl <E: Copy> Field for EnumField<E> {
    type Value = E;

    fn get(&self) -> E{
        return self.value
    }

    fn set(&mut self, value: E){
        self.value = value;
    }
}

/*------------------------------------ Implement ------------------------------------*/

pub fn ui_init() -> UserInterface{
    /*
    @brief: Init the user interface
    @input: None
    @output: The user interface structure fully configured with default settings
    */

    /*Initialise REVISION_STATUS section*/
    println!("UI_Init: Starting initialisation");
    let revision_status = user_interface_revision_status::ui_revision_status_init();
    let system_status = user_interface_system_status::ui_system_status_init();

    /* Output the user interface*/
    let user_interface = UserInterface {revision_status: revision_status,
                                        system_status: system_status, 
                                        };
    return user_interface;
}

// pub fn ui_write_field<F: Field>(field: &mut F, value: F::Value){
//     /*
//     @brief: Write a single field in the User interface
//     @input: (Field) - Any Field of the User interface to write
//     @input: (Field::Value) - The value to write. It should be the same type as the Field type
//     @output: None, modify the User interface
//     */
//     println!("UI_Init: Starting initialisation");
//     //field.set(value);
// }

// pub fn ui_read_field<F: Field>(&field: F) -> F::Value{
//     /*
//     @brief: Read a single field from the user interface
//     @input: (Field) - The field to read 
//     @output: (F::Value) - The data read from the user interface. The type of this depend on the type in Field
//     */
//     println!("UI_Init: Starting initialisation");
//     //return field.get();
// }