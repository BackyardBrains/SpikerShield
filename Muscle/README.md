# Muscle SpikerShield

You are in control... literally! Now you can control machines and electronics of your Arduino projects with the electrical activity of your muscles! What will you create?

[Muscle SpikerShield](https://backyardbrains.com/products/muscleSpikerShield) can be used for numerous experiments. In this repository you will find Arduino code for six example experiments: 
 - [Human-Human interface](https://backyardbrains.com/products/HHI) ([MuscleSpikerShieldWithHHI_V1_0.ino](Arduino%20Code/HHI/MuscleSpikerShieldWithHHI_V1_0.ino))
 - EMG controlled gripper (robotic claw) ( [MuscleSpikerShieldWithGripper_V1_0.ino](Arduino%20Code/Gripper/MuscleSpikerShieldWithGripper_V1_0.ino) )
 - Simple loop for testing gripper ([MuscleSpikerShield_GripperLoop.ino](Arduino%20Code/Gripper/MuscleSpikerShield_GripperLoop.ino))
 - EMG controlled auxiliary output ([MuscleSpikerShieldForPowerCordV1_0.ino](Arduino%20Code/PowerCord/MuscleSpikerShieldForPowerCordV1_0.ino))
 - EMG controlled on board relay ([MuscleSpikerShieldWithRelay_V1_0.ino](Arduino%20Code/OnBoardRelay/MuscleSpikerShieldWithRelay_V1_0.ino))
 - EMG controlled digital output on header ([MuscleSpikerShieldWithHeaderOutput_V1_0.ino](Arduino%20Code/HeaderOutput/MuscleSpikerShieldWithHeaderOutput_V1_0.ino))
 - EMG remote control of TENS device ([MuscleSpikerShieldWithRF_V1_2.ino](Arduino%20Code/RF/MuscleSpikerShieldWithRF_V1_2.ino))
 - HHI + gripper control (both functionalities in one code)([MuscleSpikerShieldWithHHIAndGripper_V1_0.ino](Arduino%20Code/GripperAndHHI/MuscleSpikerShieldWithHHIAndGripper_V1_0.ino))
 - HHI + gripper control + auxiliary output combined in one code ([HHI_gripper_light_combined.ino](Arduino%20Code/PowerCord/HHI_gripper_light_combined.ino))
 - Six channels EMG shield with VU meters and digital outputs ([EMGWithSixChAndVUMeters.ino](Arduino%20Code/SixChannels/EMGWithSixChAndVUMeters.ino))
 - Communication with SpikeRecorder desktop aplication ([SpikeRecorderSpikerShield_V1_1.ino](Arduino%20Code/SpikeRecorder/SpikeRecorderSpikerShield_V1_1.ino))
 - All in one code! HHI interface with gripper and communication with desktop application in one code. 
       For arduino UNO: [SpikeRecorderComplete.ino](Arduino%20Code/Complete/MuscleSpikerShiledComplete.ino)
       For Arduino Leonardo: [MuscleLeonardo.ino](Arduino%20Code/MuscleLeonardo/MuscleLeonardo.ino)
 - 
 one of the last two codes [SpikeRecorderSpikerShield_V1_1.ino](Arduino%20Code/SpikeRecorder/SpikeRecorderSpikerShield_V1_1.ino) needs to be loaded in Arduino connected with Muscle SpikerShield in order to establish communication between Muscle SpikerShield and our SpikeRecorder desktop application. If you want to connect Muscle SpikerShield to Matlab you can check our simple Matlab example [readSR.m](Documentation/Matlab/readSR.m) that can read 10 seconds of EMG signal from one SpikerShield's channel. 
 

 
 
 
