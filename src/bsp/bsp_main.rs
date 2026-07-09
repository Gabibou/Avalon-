
/*------------------------------------ Enum ------------------------------------*/

pub enum BoardModelID {
    AVALON_REV_A = 0x00,
    AVALON_REV_B = 0x01,
}


/*------------------------------------ Trait ------------------------------------*/

/* LED control trait */
pub trait GenericBoardLed {
    
    fn user_led_on(&self);
    fn user_led_off(&self);
}

/* Barometer control trait*/
pub trait GenericBoardBarometer {

    fn read_barometer_temp(&self) -> f32;
    fn read_barometer_pressure(&self) -> f32;

}
/* Board control super trait */
pub trait GenericBoard{

    type Led: GenericBoardLed;
    type Barometer: GenericBoardBarometer;
    
    fn init(&mut self);
    
    //Add pointer from the super trait to below class
    fn led(&mut self) -> &mut Self::Led;
    fn barometer(&mut self) -> &mut Self::Barometer;

}