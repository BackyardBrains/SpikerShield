# Matlab code for analysis of recordings for P300 experiment

This code is made for analysis of data for [P300 experiment](https://backyardbrains.com/experiments/p300)
This code expects that experiment was recorded with Heart & Brain SpikerBox (board based on Arduino Leonardo chip) and that board is programmed with custom code for P300 experiment:  [HeartAndBrainLeonardoForP300.ino](Arduino%20Code/HeartAndBrainLeonardoForP300/HeartAndBrainLeonardoForP300.ino). 

How o use this code:
- Put recording (wav file) along with it's event file (text file with "-events.txt" and the end) to the directory with this code.
- Run Matlab code by running runP300.m file.
- Matlab will ask: "What is the name of this file?"
- Type in name of the file with .wav extension and press enter. Example:

  "What is the name of this file? BYB_Recording_2017-12-31_1855.11.wav"
  
- Matlab should produce all relevant graphs for [P300 experiment](https://backyardbrains.com/experiments/p300)
 
 
