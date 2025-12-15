
videoFile = VideoWriter('C:\Users\SuperHero\git\STM32\MultipleADXL\MATLAB\myAnimation4.mp4', 'MPEG-4');
videoFile.FrameRate = 10; % Частота кадров
 open(videoFile);

for freq=1:10:4000
 
w = freq/238*1500 ;  
y1=[];
y2=[];
x1=[];
x2=[];
y0=[];
for i=0:3199 
   % y1 это амплитуды с первого акселерометра 
   % амплитуда сигнала в время отсчета i берется как среднее между отсчетом
   % i и i+1, каждый отсчет происходит каждую 1/3200 секунду, w угловая скорость, 
   % y2 сдвинуты на момент времени 1/6400 
   
   
  y1=[y1 (sin(w*i*(1/3200))+sin(w*(i+1)*(1/3200)))/2];
  y2=[y2 (sin(w*i*(1/3200)+w*1/6400)+sin(w*(i+1)*(1/3200)+w*1/6400))/2];
  x1=[x1 i*(1/3200)];
  x2=[x2 i*(1/3200)+1/6400];
  y0=[y0 sin(w*i*(1/3200))];
  
end  
fig3=figure(3); clf; hold on;
plot(x1,y1);
plot(x2,y2);

ys=[];
for i=1:length(x1)
  ys=[ys y1(i) y2(i)]; 
end  
clf; hold on;
plot(ys);
 

xlim([0 100])
title('Вид сигнала синусоиды '+string(freq)+'Гц с двух ADXL')

fig1=figure(1);
myFFT(1,ys,6400);

plot([freq freq],[0.05 1],'r')
ylim([0 1]);

fig2=figure(2);
myFFT(2,y1,3200);
plot([freq freq],[0.05 1],'r')
ylim([0 1]);

figure(1);
title('Частота генератора='+string(freq)+'Гц, 2xADXL (2x3200) Гц');
figure(2);
title('Частота генератора='+string(freq)+'Гц, ADXL Fs:3200 Гц');
  
  
   compositeFrame = combineFigures([fig1, fig2, fig3], [3, 1]); % 3 строки, 1 столбец

writeVideo(videoFile,  compositeFrame);

end
 close(videoFile);
disp('Анимация сохранена как myAnimation.mp4');
