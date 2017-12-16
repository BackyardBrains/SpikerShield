# Heart and Brain SpikerShield 

With our [Heart and Brain (EEG/EKG) SpikerShield](https://backyardbrains.com/products/heartAndBrainSpikerShieldBundle), you can view and record the action potentials of your heart and the slow rhythms of your brain using a Computer running our free [Spike Recorder Software](https://backyardbrains.com/products/spikerecorder). In order to connect Arduino to our software you will have to upload this sketch to Arduino board:

 - Communication with SpikeRecorder desktop application with Arduino UNO board([SpikeRecorderSpikerShield_V1_1.ino](Arduino%20Code/SpikeRecorder/SpikeRecorderSpikerShield_V1_1.ino))
 - Communication with SpikeRecorder desktop application with Arduino Leonardo board (this code controls LEDs on header for eye movement experiment)[HeartAndBrainLeonardoWithEyeLEDs.ino](Arduino%20Code/HeartAndBrainLeonardoWithEyeLEDs/HeartAndBrainLeonardoWithEyeLEDs.ino)
 
If you want to connect Heart and Brain SpikerShield to Matlab you can check our simple Matlab example [readSR.m](Documentation/Matlab/readSR.m) that can read 10 seconds of EEG or EKG signal from one SpikerShield's channel.
 

 
 
 
