%   clfall;
format shortg
%if ~exist('device')
   clear device;
  pause(1);
  device = serialport("COM18",  115200 ,'timeout',10);
%end  

 
 device.ByteOrder = 'little-endian';
 device.Timeout = 4;    
 
 
 Fs = 44100;          % Частота дискретизации (Гц)
nBits = 16;         % Разрядность
nChannels = 1;      % Количество каналов (1 - моно, 2 - стерео)

% Создание объекта для записи
recObj = audiorecorder(Fs, nBits, nChannels);

% Начало записи
disp('Начинаю запись... Говорите в микрофон');
 
 
write(device,['E' 0 0 'Z'],"uint8"); 
record(recObj);
pause(2);

 
ICS_BUFFER_RAW       = GetDataRawFmt(device,['A' 0 0 'Z'], "int16");
ICS_BUFFER           = GetDataRawFmt(device,['B' 0 0 'Z'], "int16");
ICS_BUFFER_FFT       = GetDataRawFmt(device,['C' 0 0 'Z'], "single");
ADXL345DATA_2BUF     = GetDataRawFmt(device,['D' 0 0 'Z'], "single");
ADXL345DATA_1BUF     = GetDataRawFmt(device,['G' 0 0 'Z'], "int16");

write(device,['F' 0 0 'Z'],"uint8"); 



stop(recObj);
disp('Запись завершена');

figure(1);
plot(ICS_BUFFER);
title('ICS BUFFER');
figure(2);
ICS_BUFFER_FFT_X = linspace(0,8000,length(ICS_BUFFER_FFT));
plot(ICS_BUFFER_FFT_X,   ICS_BUFFER_FFT);
title('ICS BUFFER FFT');

[f,Pout,phase,Y]=myFFT(0,ICS_BUFFER,48000);
figure(3);
plot(f,Pout); xlim([0 8000])
title('ICS BUFFER MyFFT MATLAB');

% Получение аудиоданных
audioData = getaudiodata(recObj);
figure(4);
plot(audioData);
title('USB Mic RAW');
[f,Pout,phase,Y]=myFFT(0,audioData,44100);
figure(5);
plot(f,Pout); xlim([0 8000])
title('USB MIC RAW MyFFT MATLAB');

figure(6);

[f,Pout,phase,Y]=myFFT(0,ADXL345DATA_2BUF,6400);
plot(f,Pout);
title('ADXL 2xBUF RAW MyFFT MATLAB');

figure(7);

[f,Pout,phase,Y]=myFFT(0,ADXL345DATA_1BUF,3200);
plot(f,Pout);
title('ADXL 1xBUF RAW MyFFT MATLAB');


 
