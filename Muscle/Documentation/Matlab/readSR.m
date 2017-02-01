%Script reads about 10sec of data from BackyardBrain's Arduino SpikerShield
%https://backyardbrains.com/products/heartAndBrainSpikerShieldBundle
%script produces data in "result" variable
s = serial('COM6');%change this to your com port
set(s,'BaudRate',230400);
s.InputBufferSize = 20000;
s.Terminator ='';
fopen(s);
s.Status
s.ReadAsyncMode = 'continuous';
fprintf(s,'conf s:10000;c:1;\n');%this will initiate sampling on Arduino
%pause(0.5);
data = [];
for i=0:10
    data = [data fread(s)'];%the result stream will be in data variable
    disp('.');
end
data = uint8(data);
if(length(findstr(data, 'StartUp!')) == 1)
    data = data(findstr(data, 'StartUp!')+10:end);%eliminate 'StartUp!' string with new line characters (8+2)
end



%unpacking data from frames
foundBeginingOfFrame = 0;
result = [];
for i=1:2:length(data)-1
    if(foundBeginingOfFrame==0)
        %found begining of the frame
        %frame begins with MSB set to 1
        if(uint8(data(i))>127)
            foundBeginingOfFrame = 1;
            %extract one sample from 2 bytes
            intout = uint16(uint16(bitand(uint8(data(i)),127)).*128);
            i = i+1;
            intout = intout + uint16(uint8(data(i)));
            result = [result intout];
        end
    else
        %extract one sample from 2 bytes
        intout = uint16(uint16(bitand(uint8(data(i)),127)).*128);
        i = i+1;
        intout = intout +uint16( uint8(data(i)));
        result = [result intout];
    end
end
fclose(s);
delete(s);
clear s
plot(result);