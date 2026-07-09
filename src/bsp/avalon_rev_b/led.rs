/*
@File description: File used as abstraction layer to control LED on AVALON rev B board
*/

/*-------------------- Import --------------------*/

use bsp::bsp_main::GenericBoardLed;

/*-------------------- Struct --------------------*/

pub struct AvalonRevBLed;

/*-------------------- Implementation --------------------*/

impl GenericBoardLed for AvalonRevBLed{

    fn user_led_on(&self){
        /*
        @brief: Power ON the user LED 
        */


        println!("USER_LED_POWERED_ON");
    }

    fn user_led_off(&self){
        /*
        @brief: Power OFF the user LED 
        */


        println!("USER_LED_POWERED_OFF");
    }

}

