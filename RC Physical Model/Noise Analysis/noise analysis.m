n1 = numel(D);        %Get number of log data
Fs = n1 / 31015                  %data frequency


Data = D(5000*Fs:25000*Fs);

n = numel(Data);        %Get number of log data

%t = n / Fs;

MV = mean(Data);        %Mean Value Calculation

SD = std(Data);         %Standard Deviation Calculation

%Fast Fourier Transform of Data
%FFT = fft(Data);

%prompt = 'Input Time Duration:';

%t = input(prompt);          %Time Duration of Data


% P2 = abs(FFT/t);
% P1 = P2(1:t/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% 
% f = Fs*(0:(t/2))/t;
% 
% figure(1)
% plot(f,P1) 
% title('PSD Plot')
% xlabel('f (Hz)')
% ylabel('Amplitude')

figure(2)
Hs = spectrum.welch;
psd(Hs,Data,'Fs',Fs)
axis([-inf,inf,-inf,inf])

figure(3)
t=linspace(0,20000,n);
plot(t,Data)
title('Time Domain Plot')
xlabel('Time(s)')
ylabel('Airspeed (m/s)')


figure(4)
histfit(Data,40)

Mean_Value = MV

Standard_Deviation = SD