function [ mcMeanArray, mcMean, mcStd ] = MonteCarloSimulationP300( data, fs, eventsData )
%The MonteCarloSimulation function is
%called to simulate chance within our data and create boundaries through
%which we can judge the statistical significance of our results.

plotTime = [-0.7 0.7];
%plottime has two values that describe the upper and lower bound of the
%samples that will be plotted

eeg = decimate(data(:,1),10,10);
fs = fs/10;
%lower the sample rate of signal 10 times

[b, a] = butter(10,35/fs);
eeg = filter(b,a,eeg);
%The EEG data has been filtered through a 10th order low pass digital 
%Butterworth filter with a cutoff value of 35 Hz and is restored into eeg.

meanEEG = mean(eeg);
stdEEG = std(eeg);
%calculate mean and STD of eeg 

randomSampleBegin = 1+-fs*plotTime(1); 
%Sets the lower bound for the randomly generated data points. This is the
%lowest point at which 1 second of data can be collected around
randomSampleEnd = length(eeg)-fs*plotTime(2);
%Sets the upper bound for the randomly generated data points. This is the
%highest point at which 1 second of data can be collected around 

oddballToneOn = round(eventsData(eventsData(:,1)==2,2)*fs);
%extract odd ball tone timestamps from event file
%Odd ball tone is marked with event named "2"

%remove odd ball tone timestamps that will produce ROI that goes out of the
%EEG recording
for i = length(oddballToneOn):-1:1
    if((oddballToneOn(i) + round(plotTime(2)*fs-1)) > length(eeg))
        oddballToneOn(i) = [];
        continue;
    end
    if((oddballToneOn(i) + round(plotTime(1)*fs+1)) <1)
        oddballToneOn(i) = [];
        continue;
    end
end


%extract odd ball tones
for i = 1:length(oddballToneOn)
       oddballEpochs(i,:) = eeg(oddballToneOn(i)+floor(plotTime(1)*fs):oddballToneOn(i)+floor(plotTime(2)*fs-1));
end

%remove odd ball tone intervals in which absolute value of signal crosses 2
%STD. This should remove eye blink artefacts 
artefactDetectionSTDThreshold = 2.5;
oddballEpochs(find(min(oddballEpochs, [], 2) < meanEEG-artefactDetectionSTDThreshold*stdEEG),:)=[];
oddballEpochs(find(max(oddballEpochs, [], 2) > meanEEG+artefactDetectionSTDThreshold*stdEEG),:)=[];

%get number of odd ball tone intervals that are valid
numberOfOddball = size(oddballEpochs,1)
%Calculate number of random intervals we should create. We will initialy
%create more random interval than odd ball tone intervals since some of
%those will be eliminated bacause signal will cross 2*STD
numberOfRNDTrials = numberOfOddball *20; 

montecarlo = [];
randomP300 = [];
for iMonty = 1:100
    randomSampleTimes = randi(round(randomSampleEnd-randomSampleBegin),numberOfRNDTrials,1)+ round(randomSampleBegin);
%generates an array of random numbers that fall somewhere between the upper
%and lower bounds as they are defined above. The upper bound is
%ransdomSampleEnd-randomSampleBegin since the points will need to be
%shifted to avoid using points outside of our intentions and the lower
%bound is 1. The size of this array is the same size as lighton.
    goodIndex = 1;
    for i = 1:numberOfRNDTrials
           trialEEG = eeg([randomSampleTimes(i)+round(plotTime(1)*fs)+1:randomSampleTimes(i)+round(plotTime(2)*fs)]);
           if(max(trialEEG) > meanEEG+artefactDetectionSTDThreshold*stdEEG)
                continue;
           end
           if(max(trialEEG) < meanEEG-artefactDetectionSTDThreshold*stdEEG)
                continue;
           end
           randomP300(goodIndex,:) = trialEEG;
           if(goodIndex == numberOfOddball)
               break;
           end
           goodIndex = goodIndex+1;
    end 
    if(goodIndex<numberOfOddball)
        error('Error: can not find enough random intervals where max(abs(EEG))<2*STD for Monte Carlo Simulation');
    end
    montecarlo(iMonty,:) = mean(randomP300);
end
%The inner for loop grabs data points from plottime(1) to plottime(2)
%centered at randomSampleTimes(i), the random points that we are sampling
%around. The wave in this location is stored in randomP300(i). The mean of
%all of these randomly chosen patches of code is calculated and then placed
%into montecarlo(i). This is is done 100 times until 100 averages of ~50
%randomly chosen chunks of waves have been placed into one array.

mcMeanArray = mean(montecarlo);
mcStdArray = std(montecarlo);
%Taking the mean and standard deviation of montecarlo creates a 1x500
%array. Taking the mean of this further returns another 1x500 array. For
%that reason, the values in these arrays are summed below and divided by
%the number of values present to get the average.

mcMean = sum(mcMeanArray)/length(montecarlo);
mcStd = sum(mcStdArray)/length(montecarlo);
%The mean and standard deviation are calculated for the array of averages
%of averages.

t = linspace( plotTime(1), plotTime(2), size(randomP300,2) );
%t can be said to be time. The number of data points in each epoch centered
%at tone onset is found and these data points are fitted to a the plottime
%scale spaced equidistantly. Therefore, one second of eeg data surrounding
%tone onset is fitted to a one second window, where t houses the time
%of each data point relative to 0, tone onset.

randomP300(find( min(randomP300, [], 2) < meanEEG-2*stdEEG),:)=[];
%Eyeblink down.  Outlier data is removed from the recording here to avoid
%influence in the mean line.

randomP300(find( max(randomP300, [], 2) > meanEEG+2*stdEEG),:)=[];
%Eyeblink up.  Outlier data is removed from the recording here to avoid
%influence in the mean line.   

figure;
axis([plotTime(1) plotTime(2) min(min(montecarlo)) max(max(montecarlo))]);
%Above line is included because otherwise the plot window is not scaled to
%the data being plotted.

xflip = [t(1 : end - 1) fliplr(t)];

for i = 1:100
    mcArray = montecarlo(i,:);
    yflip = [mcArray(1 : end - 1) fliplr(mcArray)];
    patch(xflip, yflip, 'r', 'EdgeAlpha', 0.1, 'FaceColor', 'none');
    hold on;
end
%The code above creates a figure and plots all of the individual Monte
%Carlo lines as patches so that the overlapped waves can be seen darker.
%Data-dense latencies will be visible on the plot.

mline = plot(t, mean(montecarlo),'b','LineWidth',3);
%Plots the mean line of all individual Monte Carlo lines

title('Monte Carlo Points Average');
xlabel('Time (s)');
ylabel('Response (Volts)');
%Labels the plot the axes and title.
end

