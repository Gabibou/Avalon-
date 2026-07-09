pub mod bsp_main;

// --------------------------------- AVALON_REV_A compilation ---------------------------------
#[cfg(feature = "board_avalon_g474_rev_a")]
mod avalon_rev_a;
#[cfg(feature = "board_avalon_g474_rev_a")]
pub use bsp::avalon_rev_a::board_main::AvalonRevA as Board;

// --------------------------------- AVALON_REV_B compilation ---------------------------------
#[cfg(feature = "board_avalon_g474_rev_b")]
mod avalon_rev_b;
#[cfg(feature = "board_avalon_g474_rev_b")]
pub use bsp::avalon_rev_b::board_main::AvalonRevB as Board;