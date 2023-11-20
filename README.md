
![Logo](https://i.goopics.net/1q71jm.png)

# Avalon

Hi, and welcome to the Avalon github page ! 

	The avalon project is a project started in January 2023 by two student in electrical engineering.
	As we were passionate about rocket science and basicaly every thing that could go against gravity , 
	we first thought about building a rocket. But too many people already dit it ! 
	We then choose to build a flying wing ! 
	To spice the project a little we decided to make it self flying. 
	
# Prototype "Must have" 
	So the avalon project is a fully open source self flying software and hardware ! 
		The UAV should be able to fly during at least 1 hours straight with 2 lipo battery 
		The UAV should be able to follow a path with a maximum acceptable deviation of 2m
		The UAV should be able to communicate with a ground station (also called GS) 
		The UAV should be able to store at least 1k coordinate onboard 
		The UAV should have some redondancy
		The UAV should be low cost and easy to repair 
	Then some other idea come's up :
		The UAV should have an obstacle avoidance system
		The UAV should be able to fall down into a low power mode
		The UAV should be able to deploy a parachute in emergency	
Well the project is quit a complex one but we have some time to learn an make it happend.

# First prototype overview

	The first prototype is a STM32G474RET6 custom board that embed a BNO055 IMU / a BMP390 barometric pressure sensor / 
	a L80 gps module / a wio-e5 for telemetry / Some current and battery voltage monitoring and much more.
	As we wanted to test first only hardware and software , we choosed to buy a 3D printed aicraft model. We choosed 
	the EBW-160 from Eclipson compagny. And yes 160 mean 160cm or (5.2 ft) so it took a long time to build it. 
	We build it using Pla for the first version. As the first version will probably be destroy in a few month by tesing it,
	we doesen't want to pay to much extra. For the second version, the aircraft will be printed using LW-PLA which is basicaly a pla light weight. 
	
	The software devellopent is done using STM32 Cube IDe and the full project is right now availible into the software directory.
	Later on, only .C and .h files will be availible and a doc will be created to help user to import and use this .C and .H files. 
![PCB_V1_Front](https://github.com/Gabibou/Avalon-/assets/100377842/f3d56f3c-d216-4641-97ab-885c58d7cbd2)
![PCB_V1_Back](https://github.com/Gabibou/Avalon-/assets/100377842/ae4605fd-2f81-4c70-bdc7-ab92b161ce24)
![PCB_V1_Front](https://github.com/Gabibou/Avalon-/assets/100377842/4c123e75-89c8-42f4-b1b9-96b7fef8e4b2)

# Second prototype overview
![3](https://github.com/Gabibou/Avalon-/assets/100377842/3b982855-6625-4e5e-af99-b0ac4b951086)
![2](https://github.com/Gabibou/Avalon-/assets/100377842/15719c67-abf8-411c-8773-0dc2c518725d)
![4](https://github.com/Gabibou/Avalon-/assets/100377842/bdb9e4fc-5298-45ac-9387-bad0bebaa782)
![5](https://github.com/Gabibou/Avalon-/assets/100377842/781caa51-0c11-4fb7-86e4-799b0d28d572)
![6](https://github.com/Gabibou/Avalon-/assets/100377842/2cb2b9c9-7d9d-47dc-99a5-98edd1a6b5de)



## License

[MIT](https://choosealicense.com/licenses/mit/)


## Acknowledgements

 - [Partnership from SeedStudio](https://www.seeedstudio.com/)

