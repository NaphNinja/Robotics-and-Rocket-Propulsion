function [T01,p01,T02,p02,CPL,p03,f,T04,T041,p04,T5,p5,Density,Cj,A5,Thrust] = Zanpakto(cpa,cpg,gammaA,gammaG,Ca,pa,Ta,a,ma,R,compratio,T03,Qr,effint,effcomp,effburn,effturb,effnoz,effmech,M)
%Zanpakto TurboJet Cycle Analysis
% This program is for analyzing the TurboJet Cycle and to determine the
% Thrust in the engine
 'Turbo Jet Ananlysis';
 
T01 = Ta+((M*a)^2/(2*cpa));

p01 = pa*(1+(effint*((M*a)^2/(2*cpa*Ta)))^((gammaA)/(gammaA-1)));

p02 = compratio*p01;

T02 = T01+((T01/effcomp)*((p02/p01)^((gammaA-1)/gammaA)-1));

CPL = 0.04*p02;

p03 = p02-CPL;

f = (cpg*T03 - cpa*T02)/(effburn*Qr - cpg*T03);

T04 = T03-(cpa*(T02-T01)/(cpg*effmech));

T041 = T03-((T03-T04)/effturb);

p04 = p03*((T041/T03)^((gammaG)/(gammaG-1)));

T5 = (2/(gammaG+1))*T04;

p5 = p04*(1-((1/effnoz)*((gammaG-1)/(gammaG+1))))^(gammaG/(gammaG-1));

Density = (p5*100)/(0.287*T5);

Cj = (gammaG*0.287*1000*T5)^(0.5);

A5 = ma*(1+f)*R*T5/(effnoz*p5);

Thrust = ma*(1+f)*effnoz - Ca +(A5*(p5 - pa));

end

