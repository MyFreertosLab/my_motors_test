//Frequency components of a signal
//----------------------------------
// build a noised signal sampled at 1000hz  containing  pure frequencies
// at 50 and 70 Hz
sample_rate=1000;
t = 0:1/sample_rate:2.500;
N=size(t,'*'); //number of samples
s=A(1:size(A,1),3)';
T=0:0.001:2.500;
//s=sin(T*8*%pi)+cos(T*4*%pi);

y=fft(s);

//s is real so the fft response is conjugate symmetric and we retain only the first N/2 points
f=sample_rate*(0:(N/2))/N; //associated frequency vector
n=size(f,'*')
clf()
plot(f,abs(y(1:n)))
