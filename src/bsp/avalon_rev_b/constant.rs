/*
@Description: Store all constant linked to the Avalon Rev B board (PCB_000002)
*/

/*-------------------- Import --------------------*/

use bsp::bsp_main::BoardModelID;

/*-------------------- Constant --------------------*/


const BOARD_MODEL_ID: BoardModelID = BoardModelID::AVALON_REV_B;


/* ----------- GPIO definition ---------- */
const LED_GPIO_PIN: u8 = 0x00;
const LED_GPIO_PORT: u8 = 0x00;

