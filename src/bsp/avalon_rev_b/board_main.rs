
/*-------------------- Import --------------------*/

use bsp::avalon_rev_b::led::AvalonRevBLed;
use bsp::avalon_rev_b::barometer::AvalonRevBBarometer;
use bsp::bsp_main::GenericBoard;


/*-------------------- Struct --------------------*/

pub struct AvalonRevB {
    led: AvalonRevBLed,
    barometer: AvalonRevBBarometer,
}

/*-------------------- Implementation --------------------*/

impl GenericBoard for AvalonRevB{

    type Led = AvalonRevBLed;
    type Barometer = AvalonRevBBarometer;

    fn init(&mut self){
        println!("BOARD_INIT");
    }

    fn led(&mut self) -> &mut Self::Led {
        &mut self.led
    }

    fn barometer(&mut self) -> &mut Self::Barometer {
        &mut self.barometer
    }

}