// https://help.scilab.org/doc/5.5.2/en_US/kalm.html
// Construction of the sinusoid
// utilizzato con samples-plus-reduced.sci

w=%pi/4; // angular frequency
T=0.001; // period
t=0:T:10.477;
signal=A(1:size(A,1),2)';
y=signal;

// Sinusoid with noise
// Plot the sinusoid with noise
subplot(2,1,1);
plot(t,y);
xtitle("sinusoid with noise","t");

// System
n=1; // system order
f=[1];
g=0.90;
h=[1];
p0=[1000];
R=100;
Q=0.25;
x0=11500*ones(n,1);
// Initialize for loop
x1=x0;
p1=p0;
// Kalman filter
for i=1:length(t)-1
    [x1(:,i+1),p1,x,p]=kalm(y(i),x1(:,i),p1,f,g,h,Q,R);
end
// Plot the results (in red) to compare with the sinusoid (in green)
subplot(2,1,2);
plot(t,signal,"color","green");
plot(t,x1(1,:),"color","red");
xtitle("Comparison between sinusoid (green) and extraction with Kalman filter (red)","t");
