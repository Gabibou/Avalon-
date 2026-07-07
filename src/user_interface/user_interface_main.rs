
use super::user_interface_revision_status;
use super::user_interface_system_status;
use super::user_interface_alerts_status;
use super::user_interface_memory;
use super::user_interface_pid_control;
use super::user_interface_command;
use super::user_interface_sensors_control;
use super::user_interface_actuators_control;
use super::user_interface_flight_control;
use super::user_interface_revision_status::UiRevisionStatus;
use super::user_interface_system_status::UiSystemStatus;
use super::user_interface_alerts_status::UiAlertsStatus;
use super::user_interface_memory::UiMemory;
use super::user_interface_pid_control::UiPidControl;
use super::user_interface_command::UiCommand;
use super::user_interface_sensors_control::UiSensorsControl;
use super::user_interface_actuators_control::UiActuatorsControl;
use super::user_interface_flight_control::UiFlightControl;

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
    pub alerts_status: UiAlertsStatus,
    pub external_memory: UiMemory,
    pub pid_control: UiPidControl,
    pub command: UiCommand,
    pub sensor_control: UiSensorsControl,
    pub actuator_control: UiActuatorsControl,
    pub flight_control: UiFlightControl,
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
    let alerts_status = user_interface_alerts_status::ui_alerts_status_init();
    let external_memory = user_interface_memory::ui_memory_init();
    let pid_control = user_interface_pid_control::ui_pid_control_init();
    let command = user_interface_command::ui_command_init();
    let sensors_control = user_interface_sensors_control::ui_sensors_control_init();
    let actuator_control = user_interface_actuators_control::ui_actuators_control_init();
    let flight_control = user_interface_flight_control::ui_flight_control_init();

    /* Output the user interface*/
    let user_interface = UserInterface {revision_status: revision_status,
                                        system_status: system_status, 
                                        alerts_status: alerts_status,
                                        external_memory: external_memory,
                                        pid_control: pid_control,
                                        command: command,
                                        sensors_control: sensors_control,
                                        actuator_control: actuator_control,
                                        flight_control: flight_control,
                                        };
    return user_interface;
}
