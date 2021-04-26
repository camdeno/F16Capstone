n1 = numel(D);        %Get number of log data
Data = D(1000:n1-1000);

MV = mean(Data);        %Mean Value Calculation

SD = std(Data);         %Standard Deviation Calculation

%Fast Fourier Transform of Data
FFT = fft(Data);

prompt = 'Input Time Duration:';

%t = input(prompt);          %Time Duration of Data

n = numel(Data);        %Get number of log data

t=n;

Fs = n/t;                   %data frequency

P2 = abs(FFT/t);
P1 = P2(1:t/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(t/2))/t;

figure(1)
plot(f,P1) 
title('PSD Plot')
xlabel('f (Hz)')
ylabel('Amplitude')

figure(2)
plot(Data)
title('Time Zone Plot')
xlabel('Time?s)')
ylabel('Air Speed')


figure(4)
histfit(Data)

Mean_Value = MV

Standard_Deviation = SD