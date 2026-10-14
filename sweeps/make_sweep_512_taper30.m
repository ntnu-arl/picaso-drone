%% make_sweep_512_taper30.m — erzeugt sweep_512_taper30.csv fuer die naechste Kampagne
% Design-Begruendung: SonoNerf-2/GaussianSplattingReal/NEXT_CAMPAIGN.md
%   - 512 statt 1024 Samples: Pulsende bei 0.20 m statt 0.39 m
%     -> Crosstalk-Abklingschwanz halbiert, Objektfenster (0.45-0.70 m) frei
%   - 30%-Hanning-Rampen: Range-Sidelobes -35 dB statt -22 dB
%     -> Mount-Echo maskiert schwache Glints nicht mehr
%   - gleicher hyperbolischer Downsweep 80 -> 20 kHz, gleiches DAC-Format
%   - Kosten: -4.8 dB Sendeenergie (bei 29 dB SNR unkritisch)
%
% Ausfuehren in diesem Ordner (nutzt ./fm_sweep.m):
%   >> make_sweep_512_taper30

fs        = 4.44444e6 / 10;          % DAC-Rate = dezimierte Rate (Konvention der Pipeline)
nSamples  = 512;
duration  = (nSamples - 1) / fs * 1e3;   % [ms] -> fm_sweep liefert genau nSamples
fStart    = 80e3;                    % Downsweep wie sweep_40cm.csv
fEnd      = 20e3;
winprct   = 30;                      % Hanning-Rampe je Seite [% der Laenge]

sig = fm_sweep(fStart, fEnd, fs, duration, 1, winprct);
assert(numel(sig) == nSamples, 'Sample-Anzahl %d != %d', numel(sig), nSamples);

dac = round(2048 + 2047 * sig);
dac = min(max(dac, 0), 4095);

writematrix(dac, 'sweep_512_taper30.csv');
fprintf('sweep_512_taper30.csv: %d Samples, %.2f ms, %g -> %g kHz, Rampen %d%%\n', ...
    numel(dac), duration, fStart/1e3, fEnd/1e3, winprct);
