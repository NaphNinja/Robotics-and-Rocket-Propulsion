clc
clear
cpa = 1005;
cpg = 1148;
gammaA = 1.4;
gammaG = 1.33;
Ca = 260;
pa = 0.4111;
Ta = 242.7;
a = 299.5;
ma = 15;
R = 287;
compratio = 8;
T03 = 1200;
Qr = 43*10^6
effint = 0.93;
effcomp = 0.87;
effburn = 0.97;
effturb = 0.90;
effnoz = 0.95;
effmech = 0.99;
effmech = 0.99;
M = 0.8;

[T01,p01,T02,p02,CPL,p03,f,T04,T041,p04,T5,p5,Density,Cj,A5,Thrust] = Zanpakto(cpa,cpg,gammaA,gammaG,Ca,pa,Ta,a,ma,R,compratio,T03,Qr,effint,effcomp,effburn,effturb,effnoz,effmech,M)