%drum.wav
%guitar.wav
%piano.wav

[y, Fs] = audioread('piano.wav');

y_firstsec = y(1:Fs);

y_norm = (y_firstsec - min(y_firstsec)) / (max(y_firstsec) - min(y_firstsec));
y_scaled = round(y_norm * 4095);

N = 1024;
lut = y_scaled(round(linspace(1, length(y_scaled), N)));

disp(lut)

sound(y_firstsec, Fs);

figure;
plot(lut, '-');       % plot with circles at each sample
title(sprintf('Lookup Table with %d Samples', N));
xlabel('Sample Index');
ylabel('Amplitude');
grid on;

figure;
t = (0:length(y_firstsec)-1) / Fs;   % time vector in seconds
plot(t, y_firstsec);
title('Original Audio — First Second');
xlabel('Time (seconds)');
ylabel('Amplitude');
grid on;