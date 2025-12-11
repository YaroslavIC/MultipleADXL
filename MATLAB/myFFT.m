function [f,Pout,phase,Y]=myFFT(fig,signal,Fs)
%signal = p_adc ;
%length_sec = timeX2(end);

%Fs = length(signal)/length_sec;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = length(signal);             % Length of signal
t = (0:L-1)*T;        % Time vector

Y = fft(signal);

P2 = abs(Y/L);
P1 = P2(1:round(L/2+1));
%P1(1:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

Pout = P1(1:end);

if (fig>0)
  figure(fig); clf; hold on;   
  plot(f,P1,'.') 
  title('Single-Sided Amplitude Spectrum of X(t) Fs:'+ string(Fs))
  xlabel('f (Hz)')
  ylabel('|P1(f)|')
end  

phase = angle(Y);

% figure(fig+1);   
% plot(f,phase(1:length(f))) ;
% title('Single-Sided Spectrum of y(t) Fs:'+ string(Fs))
% xlabel('f (Hz)')
% ylabel('Phase (rad)')

 
