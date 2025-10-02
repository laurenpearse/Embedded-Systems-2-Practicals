% -------- Settings --------
files = {'piano.wav','guitar.wav','drum.wav'};
names = {'Piano','Guitar','Drum'};
N = 8000;    % target LUT length
NS=8000;
for k = 1:numel(files)
    [y, Fs] = audioread(files{k});
    if size(y,2) == 2
        y = mean(y,2);  % convert to mono
    end

    % Take first second
    y_firstsec = y(1:min(Fs, numel(y)));

    % Normalize
    y_norm = (y_firstsec - min(y_firstsec)) / (max(y_firstsec) - min(y_firstsec));

    % Scale to 12-bit [0..4095]
    y_scaled = round(y_norm * 4095);

    % Downsample to N samples evenly spaced
    lut = y_scaled(round(linspace(1, length(y_scaled), N)));

    % ---- Print to Command Window ----
    fprintf('static const uint16_t %s_LUT[%d] = {\n', names{k}, N);

    perLine = 4000; % values per line
    for i = 1:N
        fprintf('%d', lut(i));
        if i < N
            fprintf(', ');
        end
        if mod(i, perLine) == 0
            fprintf('\n'); % new line every 16 numbers
        end
    end

    fprintf('};\n\n');
end

amp = 4095/2;  % amplitude (half scale, so centered around 2048)



theta = linspace(0, 2*pi, NS+1);  % NS+1 points
theta(end) = [];                  % drop the duplicate 2π endpoint

sin_lut = round((sin(theta) + 1) * (4095/2));

% Just to be safe
sin_lut(sin_lut < 0) = 0;
sin_lut(sin_lut > 4095) = 4095;

% --- Sawtooth ---
saw_lut = round(linspace(0, 4095, NS));

% --- Triangle ---
half = NS/2;
tri_up = linspace(0, 4095, half);
tri_down = linspace(4095, 0, half);
tri_lut = round([tri_up tri_down]);

% --- Print in C array format ---
perLine = 4000;

fprintf('static const uint32_t Sin_LUT[%d] = {\n', NS);
for i = 1:NS
    fprintf('%d', sin_lut(i));
    if i < NS, fprintf(', '); end
    if mod(i,perLine)==0, fprintf('\n'); end
end
fprintf('};\n\n');

fprintf('static const uint32_t Saw_LUT[%d] = {\n', NS);
for i = 1:NS
    fprintf('%d', saw_lut(i));
    if i < NS, fprintf(', '); end
    if mod(i,perLine)==0, fprintf('\n'); end
end
fprintf('};\n\n');

fprintf('static const uint32_t Triangle_LUT[%d] = {\n', NS);
for i = 1:NS
    fprintf('%d', tri_lut(i));
    if i < NS, fprintf(', '); end
    if mod(i,perLine)==0, fprintf('\n'); end
end
fprintf('};\n\n');