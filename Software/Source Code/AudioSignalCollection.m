Fs = 10000;
Duration = 20;
n = Duration * Fs;
t = (1:n)/Fs;
samples = [1,n];
filename = "1.mp3";
[ft1,Fs1] = SignalSampling(filename);
figure(1);
subplot(4,2,1);
nft = length(ft1);
 
time  = (0:(nft-1))/Fs1;
 
% plot(time,ft1);
% title("原语音信号时域波形图");
% xlabel('时间/s');
% ylabel('幅度');
%  
% [Fejw2,Fs2] = SpectrumAnalysis(ft1,Fs1);
%  
% subplot(4,2,2);
% plot(Fs2,abs(Fejw2));
% title("原语音信号频域波形图");
% xlabel('频率/Hz');
% ylabel('幅度');
 
ftn1 = AddCosNoise(ft1,Fs1);
[Fejw3,Fs3] = SpectrumAnalysis(ftn1,Fs1);
% subplot(4,2,3);
% plot(time,ftn1);
% title("加余弦噪声的语音信号时域波形图");
% xlabel('时间/s');
% ylabel('幅度');
%  
% subplot(4,2,4);
% plot(Fs3,abs(Fejw3));
% title("加余弦噪声的语音信号频域波形图");
% xlabel('频率/Hz');
% ylabel('幅度');
 
[D,C] = IIR_LowPassFilter(2000,2500,Fs1);
y = filter(D,C,ftn1);
[Fejw4,Fs4] = SpectrumAnalysis(y,Fs1);
 
% subplot(4,2,5);
% plot(time,y);
% title("IIR滤波处理的语音信号时域波形图");
% xlabel('时间/s');
% ylabel('幅度');
%  
% subplot(4,2,6);
% plot(Fs4,abs(Fejw4));
% title("IIR滤波处理的语音信号频域波形图");
% xlabel('频率/Hz');
% ylabel('幅度');
 
H = FIR_LowPassFilter(2000,2500,Fs1);
y = filter(H,1,ftn1);
[Fejw5,Fs5] = SpectrumAnalysis(y,Fs1);
%  
% subplot(4,2,7);
% plot(time,y);
% title("FIR滤波处理的语音信号时域波形图");
% xlabel('时间/s');
% ylabel('幅度');
 
% subplot(4,2,8);
% plot(Fs5,abs(Fejw5));
% title("FIR滤波处理的语音信号频域波形图");
% xlabel('频率/Hz');
% ylabel('幅度');
%  
% figure(2);
% subplot(4,2,1);
% plot(time,ft1);
% title("原语音信号时域波形图");
% xlabel('时间/s');
% ylabel('幅度');
%  
% subplot(4,2,2);
% plot(Fs2,abs(Fejw2));
% title("原语音信号频域波形图");
% xlabel('频率/Hz');
% ylabel('幅度');
 
ftn2 = AddRandNoise(ft1);
[Fejw6,Fs6] = SpectrumAnalysis(ftn2,Fs1);
% subplot(4,2,3);
% plot(time,ftn2)
% title("加随机白噪声的语音信号时域波形图");
% xlabel('时间/s');
% ylabel('幅度');
%  
% subplot(4,2,4);
% plot(Fs6,abs(Fejw6));
% title("加随机白噪声的语音信号频域波形图");
% xlabel('频率/Hz');
% ylabel('幅度');
 
[D,C] = IIR_LowPassFilter(2000,2500,Fs1);
y = filter(D,C,ftn2);
[Fejw7,Fs7] = SpectrumAnalysis(y,Fs1);
% subplot(4,2,5);
% plot(time,y);
% title("IIR滤波后的语音信号时域波形图");
% xlabel('时间/s');
% ylabel('幅度');
%  
% subplot(4,2,6);
% plot(Fs7,abs(Fejw7));
% title("IIR滤波后的语音信号频域波形图");
% xlabel('频率/Hz');
% ylabel('幅度');
 
H = FIR_LowPassFilter(2000,2500,Fs1);
y = filter(H,1,ftn2);
[Fejw8,Fs8] = SpectrumAnalysis(y,Fs1);
% subplot(4,2,7);
% plot(time,y);
% title("FIR滤波后的语音信号时域波形图");
% xlabel('时间/s');
% ylabel('幅度');
%  
% subplot(4,2,8);
% plot(Fs8,abs(Fejw8));
% title("FIR滤波后的语音信号频域波形图");
% xlabel('频率/Hz');
% ylabel('幅度');
 
function [ft,Fs] = SignalSampling(filename)
    starttime = 5;
    endtime = 25;
    [Y,Fs] = audioread(filename);
    ft = Y((Fs * starttime + 1):Fs * endtime,1);
end
 
function [Fejw,f] = SpectrumAnalysis(ft,Fs)
    N = length(ft);
    f = (0:N - 1) * Fs/N;
    Fejw = fft(ft,N);
end
 
function Fejw = AddCosNoise(ft,Fs)
    f = 2500;
    A = 0.05;
    t = [0:0.0001:0.1];
 
    N = length(ft);
    T = (0:N-1) / Fs; 
    noise1 = A * cos(2 * pi * f * T);
    A = 0.05;
    noise2 = A * cos(2 * pi * f * T);
    noise1 = noise1';
    noise2 = noise2';
    noise1 = [noise1, noise2];
    Fejw = ft + noise1 ;  
end
 
function Fejw = AddRandNoise(ft)
    noise = 0.35 * randn(1,length(ft));
    noise = noise';
    noise = [noise,noise];
    Fejw = ft + noise;
end
 
function [D,C] = IIR_LowPassFilter(fp,fs,Fs)
    Wp = 2 * pi * fp / Fs;
    Ws = 2 * pi * fs/ Fs;
    Ap = 3;
    As = 20;
    Fs = Fs/Fs;
    wap = tan(Wp/2);
    was = tan(Ws/2);
    
    [N,Wn] = buttord(wap,was,Ap,As,'s');
    [B,A] = butter(N,Wn,'s');
    
    [D,C] = bilinear(B,A,Fs);
    [B,A] = freqz(D,C);
    figure(3);
    subplot(2,1,1);
    plot(A,abs(B));
    xlabel('Normalized Frequency/(×π rad/sample)');
    ylabel('Magnitude');  
    subplot(2,1,2);
    plot(A,angle(B));
    xlabel('Normalized Frequency/(×π rad/sample)');
    ylabel('Phase');  
    
end
 
function [H] = FIR_LowPassFilter(fp,fs,Fs)
    Wp =  2 * pi * fp;
    Ws =  2 * pi * fs;
%n = ceil(6.6 * pi * (Ws - Wp)); 
    n = 40; 
    Wn = 0.5 * (Wp + Ws)/(Fs * 2 * pi);
    H = fir1(n,Wn,'low',hann(n+1));
    %freqz(H,1,512);   
end