/*
@Description: File use as abstraction layer for the barometer sensor (BMP390) embedded on AVALON_REV_B (PCB_000001)
*/

/*-------------------- Import --------------------*/

use bsp::bsp_main::GenericBoardBarometer;

/*-------------------- Struct --------------------*/

pub struct AvalonRevBBarometer;

/*-------------------- Implementation --------------------*/

impl GenericBoardBarometer for AvalonRevBBarometer{

    fn read_barometer_temp(&self) -> f32{
        /*
        @brief: Power ON the user LED 
        */


        println!("READ_BARO_TEMP");
        return 0.00;
    }

    fn read_barometer_pressure(&self) -> f32{
        /*
        @brief: Power OFF the user LED 
        */


        println!("READ_BARO_PRESS");
        return 0.00;
    }

}