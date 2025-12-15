%   clfall;
format shortg
%if ~exist('device')
   clear device;
  pause(1);
  device = serialport("COM18",  115200 ,'timeout',10);
%end  


 
 device.ByteOrder = 'little-endian';
 device.Timeout = 4;    
 
 
 Fs_mic = 44100;          % Частота дискретизации (Гц)
nBits_mic = 16;         % Разрядность
nChannels_mic = 1;      % Количество каналов (1 - моно, 2 - стерео)


% Параметры
fs2_spk = 44100;          % Частота дискретизации (Гц)
duration2_spk = 3;        % Длительность воспроизведения (секунды)

fs_ics = 48000;

 
freqset = linspace(200,8000,200);
%freqset = [1000];

if length(freqset)>2
  videoFile = VideoWriter('C:\Users\SuperHero\git\STM32\MultipleADXL\MATLAB\real_data10.mp4', 'MPEG-4');
  videoFile.FrameRate = 10; % Частота кадров
  open(videoFile);
end

% Создание объекта для записи
recObj = audiorecorder(Fs_mic, nBits_mic, nChannels_mic);


i=1;
while i<=length(freqset)
     
  f_spk=freqset(i);
  disp([f_spk]);

  t_spk = 0:1/fs2_spk:duration2_spk-1/fs2_spk;
  y_spk = 0.15*sin(2 * pi * f_spk * t_spk);
  player = audioplayer(y_spk, fs2_spk);
  play(player);
  pause(1);
 
  %write(device,['E' 0 0 'Z'],"uint8"); 
  record(recObj);
  pause(2);

 
  ICS_BUFFER_RAW       = GetDataRawFmt(device,['A' 0 0 'Z'], "int16");
  ICS_BUFFER           = GetDataRawFmt(device,['B' 0 0 'Z'], "int16");
  ICS_BUFFER_FFT       = GetDataRawFmt(device,['C' 0 0 'Z'], "single");
  ADXL345DATA_2BUF     = GetDataRawFmt(device,['D' 0 0 'Z'], "single");
  ADXL345DATA_1BUF     = GetDataRawFmt(device,['G' 0 0 'Z'], "int16");

  %write(device,['F' 0 0 'Z'],"uint8"); 

  stop(recObj);
  audioData = getaudiodata(recObj);
  stop(player);
  

  fig1=figure(1);
  ICS_BUFFER_x = 1000*linspace(0,1/fs_ics*length(ICS_BUFFER),length(ICS_BUFFER));
  plot(ICS_BUFFER_x,ICS_BUFFER);
  title('ICS43434 BUFFER (Г:'+string(f_spk)+' Гц)');
  xlabel('Время, мсек');
  
  fig2=figure(2); clf; hold on;
  yyaxis left
  ICS_BUFFER_FFT_X = linspace(0,fs_ics/2,length(ICS_BUFFER_FFT));
  plot(ICS_BUFFER_FFT_X(3:end),   ICS_BUFFER_FFT(3:end));
  [f,Pout2,phase,Y]=myFFT(0,ICS_BUFFER,fs_ics);
  plot(f(3:end),Pout2(3:end),'.'); xlim([0 8000])
  line([f_spk f_spk],[0.5*max([ICS_BUFFER_FFT(3:end)' Pout2']) 0.35*max([ICS_BUFFER_FFT(3:end)' Pout2'])],'Color',[0.6350 0.0780 0.1840],'LineWidth',3);
  ylabel('ICS STM32-FFT,MATLAB-FFT');
  yyaxis right
  [f,Pout3,phase,Y]=myFFT(0,audioData,fs2_spk);
  ylabel('USBMic MATLAB-FFT');
  plot(f,Pout3); xlim([0 8000])
  title('ICS43434 FFT-STM32/FFT-MATLAB USBMic-FFT-MATLAB  (Г:'+string(f_spk)+' Гц)');
  xlabel('Частота, Гц');


  fig4=figure(4);
  audioData_x = 1000*linspace(0,1/fs2_spk*length(audioData),length(audioData));
  plot(audioData_x,audioData);
  xlabel('Время, мсек');
  title('USB Mic RAW');
  
  fig6=figure(6);clf; hold on;
  [f,Pout1,phase,Y]=myFFT(0,ADXL345DATA_2BUF,6400);
  plot(f,Pout1);
  [f,Pout2,phase,Y]=myFFT(0,ADXL345DATA_1BUF,3200);
  plot(f,Pout2 );
  line([f_spk f_spk],[0.35*max([Pout1' Pout2']) 0.5*max([Pout1' Pout2'])],'Color',[0.6350 0.0780 0.1840],'LineWidth',3);
  title('ADXL 2xBUF и 1xBUF FFT-MATLAB (Г:'+string(f_spk)+' Гц)');
  xlabel('Частота, Гц');
  
  fig7=figure(7);clf; hold on;
  adxl2x_t = 1000*linspace(0,length(ADXL345DATA_2BUF)*1/6400,length(ADXL345DATA_2BUF));
  plot(adxl2x_t(1:1:end),   ADXL345DATA_2BUF(1:1:end));
  title('BUFFER ADXL 2xBUF  (Г:'+string(f_spk)+' Гц)');
  xlabel('Время, мсек');
 
  if (length(freqset)>2)
    if sum(ICS_BUFFER_FFT(10:end))>3 
     compositeFrame = combineFigures([fig1, fig7, fig2, fig4, fig6], [3, 2]);  
     writeVideo(videoFile,  compositeFrame );
    end    
  end  
  
   i=i+1; 
 end 

if length(freqset)>2
  close(videoFile);
end  