function [meanOddball, meanStandard, fs] = FindP300( data, fs, eventsData )
%FindVEP is creates an array of VEP epochs centered around flash onset and
%averages these to create a mean line of VEP response.

% Fs is the frequency of sampling. In other words, fs describes how many
% times data is collected per second. 

plotTime = [-0.7 0.7];
%plottime sets the intervals around which the data is gathered and plotted
eeg = decimate(data(:,1),10,10);
fs = fs/10;
%The code above separates the three columns in data and writes that
%information into an array corresponding to what the data represents after
%it has been downsampled or decimated (1/10th fewer data points in array).
%The sampling frequency is changed to reflect this as well.


standardToneOn = round(eventsData(eventsData(:,1)==1,2)*fs);
       
%Fills standardToneOn. If there is a change between the two consecutive that
%is larger than a set threshold, here 0.002, the x value for the data point
%is put into standardToneOn and the loop begins to look for another change
%of this sort 200 ms after this point (tone duration is 100 ms). If a
%change is not found, the loop moves to the next data point in the array. 

oddballToneOn = round(eventsData(eventsData(:,1)==2,2)*fs);
        
%Fills oddballToneOn. If there is a change between the two consecutive that
%is larger than a set threshold, here 0.002, the x value for the data point
%is put into oddballToneOn and the loop begins to look for another change
%of this sort 200 ms after this point (tone duration is 100 ms). If a
%change is not found, the loop moves to the next data point in the array. 

[b, a] = butter(10,35/fs);
eeg = filter(b,a,eeg);
%The EEG data has been filtered through a 10th order low pass digital 
%Butterworth filter with a cutoff value of 35 Hz and is restored into eeg.

meanEEG = mean(eeg);
stdEEG = std(eeg);

for i = length(standardToneOn):-1:1
    if((standardToneOn(i) + round(plotTime(2)*fs-1)) > length(eeg))
        standardToneOn(i) = [];
        continue;
    end
    if((standardToneOn(i) + round(plotTime(1)*fs+1)) < 1)
        standardToneOn(i) = [];
        continue;
    end
end

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
%Checks to make sure that the points in standardToneOn and oddballToneOn
%can have the amount of data collected around it as is specified by
%plottime. If this collection exceeds array boundaries, it is removed from
%analysis.

for i = 1:length(standardToneOn)
       standardEpochs(i,:) = eeg(standardToneOn(i)+floor(plotTime(1)*fs):standardToneOn(i)+floor(plotTime(2)*fs-1));
end

for i = 1:length(oddballToneOn)
       oddballEpochs(i,:) = eeg(oddballToneOn(i)+floor(plotTime(1)*fs):oddballToneOn(i)+floor(plotTime(2)*fs-1));
end
%For each value in standardToneOn or oddballToneOn, the code looks
%into the array of wave values and assigns the data found within the
%plottime boundaries centered at ToneOn to oddballEpochs or standardEpochs.
%Now we have arrays of all responses to standard or oddball tones.
artefactDetectionSTDThreshold = 2.5;
standardEpochs(find(min(standardEpochs, [], 2) < meanEEG-artefactDetectionSTDThreshold*stdEEG),:)=[];
oddballEpochs(find(min(oddballEpochs, [], 2) < meanEEG-artefactDetectionSTDThreshold*stdEEG),:)=[];
%Eyeblink down.  Outlier data is removed from the recording here to avoid
%influence in the mean line. 

standardEpochs(find(max(standardEpochs, [], 2) > meanEEG+artefactDetectionSTDThreshold*stdEEG),:)=[];
oddballEpochs(find(max(oddballEpochs, [], 2) > meanEEG+artefactDetectionSTDThreshold*stdEEG),:)=[];
%Eyeblink up.  Outlier data is removed from the recording here to avoid
%influence in the mean line. 

t = linspace(plotTime(1), plotTime(2), (plotTime(2)-plotTime(1))*fs);
%t can be said to be time. The number of data points in each epoch centered
%at tone onset is found and these data points are fitted to a the plottime
%scale spaced equidistantly. Therefore, one second of eeg data surrounding
%tone onset is fitted to a one second window, where t houses the time
%of each data point relative to 0, tone onset. 

figure;
xflip = [t(1 : end - 1) fliplr(t)];
axis([plotTime(1) plotTime(2) min(min(standardEpochs)) max(max(standardEpochs))]);
%Above line is included because otherwise the plot window is not scaled to
%the data being plotted.
for i = 1:size(standardEpochs,1)
    standardToneArray = standardEpochs(i,:);
    yflip = [standardToneArray(1 : end - 1) fliplr(standardToneArray)];
    patch(xflip, yflip, 'r', 'EdgeAlpha', 0.1, 'FaceColor', 'none');
    hold on;
end
%The code above creates a figure and plots all of the individual standard 
%tone responses as patches so that the overlapped waves can be seen darker.
%Data-dense latencies will be visible on the plot.

meanStandard = mean(standardEpochs);
mlineStandard = plot (t, meanStandard, 'r', 'LineWidth',3);
%Plots the mean waveform of all responses to the standard tone. meanStandard
%is created as its own variable so that it can be returned through the script

title('Neural response to standard tone');
xlabel('Time (s)');
ylabel('Response (Volts)');
%Labels the plot the axes and title.

figure;
xflip = [t(1 : end - 1) fliplr(t)];
axis([plotTime(1) plotTime(2) min(min(oddballEpochs)) max(max(oddballEpochs))]);
%Above line is included because otherwise the plot window is not scaled to
%the data being plotted.
for i = 1:size(oddballEpochs, 1)
    oddballArray = oddballEpochs(i,:);
    yflip = [oddballArray(1 : end - 1) fliplr(oddballArray)];
    patch(xflip, yflip, 'r', 'EdgeAlpha', 0.1, 'FaceColor', 'none');
    hold on;
end
%The code above creates a figure and plots all of the individual oddball 
%tone responses as patches so that the overlapped waves can be seen darker.
%Data-dense latencies will be visible on the plot.

meanOddball = mean(oddballEpochs);
mlineOddball = plot (t, meanOddball, 'g', 'LineWidth',3);
%Plots the mean waveform of all responses to the oddball tone. meanOddball
%is created as its own variable so that it can be returned through the script

title('Neural response to oddball tone');
xlabel('Time (s)');
ylabel('Response (Volts)');
%Labels the plot the axes and title.
end


