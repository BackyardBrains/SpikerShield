clear;
NamePrompt = 'What is the name of this file? ';
FileNameStr = input(NamePrompt,'s');
[filepath,name,ext] = fileparts(FileNameStr);
eventsFileName = strcat(filepath,name,'-events.txt');
[rundata, fs] = audioread(FileNameStr);
eventsData = csvread(eventsFileName,2,0);
fileName = regexprep(FileNameStr,'_',' ');
TitleString = sprintf('Neural responses to Oddball Task: %s', fileName);
%Gets the name of the file being analyzed from the user and inputs this
%into a string that will be used as the plot title.


[meanOddball, meanStandard, fs2] = FindP300( rundata, fs, eventsData );
[mcMeanArray, mcMean, mcStd] = MonteCarloSimulationP300( rundata, fs, eventsData );
%rundata is created so that no changes are made to the actual data
%file during analysis. Passing it this way allows for the data file to be untouched and
%manipulations to be made to the data outside of this script. FindP300 is 
%called to create an array of epochs centered around standard and oddball 
%tone onset and averages these to create a mean line of tone response. 
%The MonteCarloSimulation function is called to simulate chance within our 
%data and create boundaries through which we can judge the statistical 
%significance of our results.

plottime = [-0.7 0.7];
P300AnalInterval = [250 600];
stdValue = 2;
%plottime sets the intervals around which the data is gathered and plotted.
%P300AnalInterval sets the latencies that we will look for the P300 in.

t = linspace( plottime(1), plottime(2), length(meanOddball) );
%t can be said to be time. The number of data points in each epoch centered
%at tone onset is found and these data points are fitted to a the plottime
%scale spaced equidistantly. Therefore, one second of eeg data surrounding
%flash onset is fitted to a one second window, where t houses the time
%of each data point relative to 0, tone onset. 

for i=1:length(meanOddball)
    latency(i) = i*2-length(meanOddball);
end
%Allows for the latency to be quickly found when given the x value for the
%data point. 

if(max(meanOddball) > mcMean+stdValue*mcStd)
    maxLimit = max(meanOddball);
end
if(max(meanOddball) < mcMean+stdValue*mcStd)
    maxLimit = mcMean+stdValue*mcStd;
end

if(min(meanOddball) < mcMean-stdValue*mcStd)
    minLimit = min(meanOddball);
end
if(min(meanOddball) > mcMean-stdValue*mcStd)
    minLimit = mcMean-stdValue*mcStd;
end
%The block of code above finds the minimum and maximum values that will be
%plotted on the figure generated below. This allows for the plot to be
%scaled to the data analyzed. 

figure;
axis([plottime(1) plottime(2) (minLimit-.0001) (maxLimit+.0001)]);
%Above line is included because otherwise the plot window is not scaled to
%the data being plotted.
hold on;
plot(t,meanOddball, 'g');
plot(t,meanStandard, 'r');
plot(t,mcMeanArray , 'b');
hline(mcMean+stdValue*mcStd,'b:','95% CI'); %Can name lines in third spot
hline(mcMean-stdValue*mcStd,'b:','95% CI');
vline((0),'k:','Tone onset');
title(TitleString);
xlabel('Time (s)');
ylabel('Response (Volts)');
legend('Oddball Tone','Standard Tone','Monte Carlo');
set(legend,'Location','NorthWest');
hold off;
%Creates a plot with meanStandard, meanOddball, and the mean of the Monte Carlo simulations.
%Applies axes and title to the plot. A vertical dotted line is added to the
%plot to show tone onset and confidence interval lines are plotted to show
%data significance on the same plot. 

meanP300Val = sum(meanOddball)/length(meanOddball);
%Finds the average value for the recording so that the change in amplitude
%can be found for peaks and valleys. 

[~, peak_location] = findpeaks(meanOddball);
%Finds peaks in the meanOddball waveform. 

meanOddball_interved = -meanOddball;
[~, minima_location] = findpeaks(meanOddball_interved);
%Inverts the mean wave and finds the peaks of that wave. Therefore, it
%finds the valleys in the wave.

potentialOddball = [];
diffPosAmplitude = [];
counter = 1;
for i=1:length(peak_location)
    if(peak_location(i) > ((length(meanOddball)/2)+ (length(meanOddball)/2)*(P300AnalInterval(1)/(plottime(2)*1000))) && (peak_location(i) < ((length(meanOddball)/2)+ (length(meanOddball)/2)*(P300AnalInterval(2)/(plottime(2)*1000)))))
        potentialOddball(counter) = peak_location(i);
        diffPosAmplitude(counter)= (meanOddball(peak_location(i)) - meanP300Val)/110; 
        counter = counter+1;
    end
end
%Checks to see if the peaks fall within the latency interval specified by
%P300AnalInterval. If it is, it is placed into potentialOddball as a
%candidate, as is the amplitude. This is done for each peak.

[maxPosPotential, maxAmpLoc] = max(diffPosAmplitude);
P300Loc = potentialOddball(maxAmpLoc);
%The largest change in amplitude over that interval is found. This is the
%P300! The value of the change in amplitude is found, as well as the x
%value for that data point. 

minima_reversed = [];
n = length(minima_location);
for i = 1:length(minima_location)
    minima_reversed(i) = minima_location(n);  
    n = n - 1;
end
%Reverses the valley array so that find() will find the first instance
%closest to that latency from behind. This will be used to simulate a reverse find method

prev_valley_loc = 0;
for i = 1:length(minima_reversed)
    if(minima_reversed(i) < P300Loc)
        prev_valley_loc = minima_reversed(i);
    end
end
%Finds the preceding potential

P300Amp = (meanOddball(P300Loc) - meanOddball(prev_valley_loc))/110;
%Amplitude of the P300 is defined by the literature as being measured from 
%the bottom of the preceding valley to the top of the P300 peak. 
%The change in amplitude is inserted into a string that displays the amplitude 
%and latency of the peak. A label is put on the plot at the peak that gives 
%a 'P' to indicate a positive potential and the latency of the P300 peak. 
