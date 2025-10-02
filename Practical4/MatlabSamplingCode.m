% -------- Settings --------
files = {'piano.wav','guitar.wav','drum.wav'};
names = {'Piano','Guitar','Drum'};
N = 256;    % <-- LUT resolution
Fs_target = 44100;   % input wav sample rate

% -------- Process each wav --------
for k = 1:numel(files)
    [y, Fs] = audioread(files{k});
    if size(y,2) == 2
        y = mean(y,2);  % convert to mono if stereo
    end

    % Take first second (or less if shorter)
    y_firstsec = y(1:min(Fs, numel(y)));

    % Normalize to [0..1]
    y_norm = (y_firstsec - min(y_firstsec)) / (max(y_firstsec) - min(y_firstsec));

    % Scale to 12-bit [0..4095]
    y_scaled = round(y_norm * 4095);

    % Downsample to N samples evenly spaced
    lut = y_scaled(round(linspace(1, length(y_scaled), N)));

    % Plot to verify
    figure;
    plot(lut, '-o');
    title(sprintf('%s LUT (%d samples)', names{k}, N));
    xlabel('Sample Index');
    ylabel('Amplitude (0–4095)');
    grid on;

    % Print as C array (copy/paste into main.c)
    fprintf('static const uint16_t %s_LUT[%d] = {', names{k}, N);
    fprintf('%d, ', lut(1:end-1));
    fprintf('%d};\n\n', lut(end));
end
