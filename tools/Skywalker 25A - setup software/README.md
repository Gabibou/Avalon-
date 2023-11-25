# Avalon - ESC configuration software

## Sofware description :
This project is part of the AVALON project, and work with the AVA board V2.0. The purpose of this software is to test and program the **hobbywing skywalker 25A** esc. 
It is made of two main files, the first one is the main.c file where all the script is currently running from. The second one is the main.h file, where you should set up your configuration.

## Sofware setup :
In order to use this software correctly, please follow a few simple steps.
1. Open with STM32Cube IDE the .project 
1. Go to main.h file located in Core/Inc/main.h
1. Choose the needed mode between TEST_MODE and PROGRAMING_MODE. Make sure only one mode is selected at the same time to avoid error.
1. Select the esc settings in the programming section 
1. Build and run the project
1. Connect your ESC properly on the PWM signal 
1. Reset your board and the ESC power at the same time. I did it by switching power supply from the board and esc on/off at the same time.
1. If the programming mode is selected then wait for the blue led to light up.

## Authors

- [@gabibou](https://www.github.com/gabibou)