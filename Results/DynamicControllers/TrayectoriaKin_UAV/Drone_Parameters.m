function Drone_Parameters(scale)

if nargin < 1
    scale = 1;
end 

global Drone

%% Parámetros físicos
%Vista Superior
Drone.x=[-9.3689 -9.3792 -10.5143 -11.9195 -12.5368 -11.9014 -10.4916 -4 -4 -10.2766 -11.7206 -12.6129 -12.2818 -10.9767 -9.6803 -9.3689 -1.4868 1.4868 9.25 9.2603 10.3954 11.8007 12.4180 11.7825 10.3727 4 4 10.2766 11.7206 12.6129 12.2818 10.9767 9.6803 9.3689 1.4868 -1.4868 -9.3689]*scale;
Drone.y=[10.3737 11.8177 12.7100 12.3789 11.0738 9.7774 9.4660 1.4868 -1.4868 -9.25 -9.2603 -10.3954 -11.8007 -12.4180 -11.7825 -10.3727 -4 -4 -10.2766 -11.7206 -12.6129 -12.2818 -10.9767 -9.6803 -9.3689 -1.4868 1.4868 9.25 9.2603 10.3954 11.8007 12.4180 11.7825 10.3727 4 4 10.3737]*scale;
Drone.z=[1.7875 1.7875 1.7875 1.7875 1.7875 1.7875 1.7875 2.7875 2.7875 1.7875 1.7875 1.7875 1.7875 1.7875 1.7875 1.7875 2.7875 2.7875 1.7875 1.7875 1.7875 1.7875 1.7875 1.7875 1.7875 2.7875 2.7875 1.7875 1.7875 1.7875 1.7875 1.7875 1.7875 1.7875 2.7875 2.7875 1.7875]*scale;
Drone.Estructura{1}=[Drone.x;Drone.y;Drone.z];

%Vista Inferior
Drone.x1=[-9.3689 -9.3792 -10.5143 -11.9195 -12.5368 -11.9014 -10.4916 -4 -4 -10.2766 -11.7206 -12.6129 -12.2818 -10.9767 -9.6803 -9.3689 -1.4868 1.4868 9.25 9.2603 10.3954 11.8007 12.4180 11.7825 10.3727 4 4 10.2766 11.7206 12.6129 12.2818 10.9767 9.6803 9.3689 1.4868 -1.4868 -9.3689]*scale;
Drone.y1=[10.3737 11.8177 12.7100 12.3789 11.0738 9.7774 9.4660 1.4868 -1.4868 -9.25 -9.2603 -10.3954 -11.8007 -12.4180 -11.7825 -10.3727 -4 -4 -10.2766 -11.7206 -12.6129 -12.2818 -10.9767 -9.6803 -9.3689 -1.4868 1.4868 9.25 9.2603 10.3954 11.8007 12.4180 11.7825 10.3727 4 4 10.3737]*scale;
Drone.z1=[1.4875 1.4875 1.4875 1.4875 1.4875 1.4875 1.4875 0.75 0.75 1.4875 1.4875 1.4875 1.4875 1.4875 1.4875 1.4875 0.75 0.75 1.4875 1.4875 1.4875 1.4875 1.4875 1.4875 1.4875 0.75 0.75 1.4875 1.4875 1.4875 1.4875 1.4875 1.4875 1.4875 0.75 0.75 1.4875]*scale;
Drone.Estructura{2}=[Drone.x1;Drone.y1;Drone.z1];

%Laterales bases hélices
Drone.x2=[9.25 9.2603 9.2603 9.25 9.25]*scale;
Drone.y2=[-10.2766 -11.7206 -11.7206 -10.2766 -10.2766]*scale;
Drone.z2=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{3}=[Drone.x2;Drone.y2;Drone.z2];

Drone.x3=[9.2603 10.3954 10.3954 9.2603 9.2603]*scale;
Drone.y3=[-11.7206 -12.6129 -12.6129 -11.7206 -11.7206]*scale;
Drone.z3=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{4}=[Drone.x3;Drone.y3;Drone.z3];

Drone.x4=[10.3954 11.8007 11.8007 10.3954 10.3954]*scale;
Drone.y4=[-12.6129 -12.2818 -12.2818 -12.6129 -12.6129]*scale;
Drone.z4=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{5}=[Drone.x4;Drone.y4;Drone.z4];

Drone.x5=[11.8007 12.4180 12.4180 11.8007 11.8007]*scale;
Drone.y5=[-12.2818 -10.9767 -10.9767 -12.2818 -12.2818]*scale;
Drone.z5=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{6}=[Drone.x5;Drone.y5;Drone.z5];

Drone.x6=[12.4180 11.7825 11.7825 12.4180 12.4180]*scale;
Drone.y6=[-10.9767 -9.6803 -9.6803 -10.9767 -10.9767]*scale;
Drone.z6=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{7}=[Drone.x6;Drone.y6;Drone.z6];

Drone.x7=[11.7825 10.3727 10.3727 11.7825 11.7825]*scale;
Drone.y7=[-9.6803 -9.3689 -9.3689 -9.6803 -9.6803]*scale;
Drone.z7=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{8}=[Drone.x7;Drone.y7;Drone.z7];

Drone.x8=[10.2766 11.7206 11.7206 10.2766 10.2766]*scale;
Drone.y8=[9.2500 9.2603 9.2603 9.2500 9.2500]*scale;
Drone.z8=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{9}=[Drone.x8;Drone.y8;Drone.z8];

Drone.x9=[11.7206 12.6129 12.6129 11.7206 11.7206]*scale;
Drone.y9=[9.2603 10.3954 10.3954 9.2603 9.2603]*scale;
Drone.z9=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{10}=[Drone.x9;Drone.y9;Drone.z9];

Drone.x10=[12.6129 12.2818 12.2818 12.6129 12.6129]*scale;
Drone.y10=[10.3954 11.8007 11.8007 10.3954 10.3954]*scale;
Drone.z10=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{11}=[Drone.x10;Drone.y10;Drone.z10];

Drone.x11=[12.2818 10.9767 10.9767 12.2818 12.2818]*scale;
Drone.y11=[11.8007 12.4180 12.4180 11.8007 11.8007]*scale;
Drone.z11=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{12}=[Drone.x11;Drone.y11;Drone.z11];

Drone.x12=[10.9767 9.6803 9.6803 10.9767 10.9767]*scale;
Drone.y12=[12.4180 11.7825 11.7825 12.4180 12.4180]*scale;
Drone.z12=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{13}=[Drone.x12;Drone.y12;Drone.z12];

Drone.x13=[9.6803 9.3689 9.3689 9.6803 9.6803]*scale;
Drone.y13=[11.7825 10.3727 10.3727 11.7825 11.7825]*scale;
Drone.z13=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{14}=[Drone.x13;Drone.y13;Drone.z13];

Drone.x14=[-9.3689 -9.3792 -9.3792 -9.3689 -9.3689]*scale;
Drone.y14=[10.3737 11.8177 11.8177 10.3737 10.3737]*scale;
Drone.z14=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{15}=[Drone.x14;Drone.y14;Drone.z14];

Drone.x15=[-9.3792 -10.514 -10.514 -9.3792 -9.3792]*scale;
Drone.y15=[11.8177 12.7100 12.7100 11.8177 11.8177]*scale;
Drone.z15=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{16}=[Drone.x15;Drone.y15;Drone.z15];

Drone.x16=[-10.514 -11.919 -11.919 -10.514 -10.514]*scale;
Drone.y16=[12.7100 12.3789 12.3789 12.7100 12.7100]*scale;
Drone.z16=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{17}=[Drone.x16;Drone.y16;Drone.z16];

Drone.x17=[-11.919 -12.536 -12.536 -11.919 -11.919]*scale;
Drone.y17=[12.3789 11.0738 11.0738 12.3789 12.3789]*scale;
Drone.z17=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{18}=[Drone.x17;Drone.y17;Drone.z17];

Drone.x18=[-12.536 -11.901 -11.901 -12.536 -12.536]*scale;
Drone.y18=[11.0738 9.7774 9.7774 11.0738 11.0738]*scale;
Drone.z18=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{19}=[Drone.x18;Drone.y18;Drone.z18];

Drone.x19=[-11.901 -10.491 -10.491 -11.901 -11.901]*scale;
Drone.y19=[9.7774 9.4660 9.4660 9.7774 9.7774]*scale;
Drone.z19=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{20}=[Drone.x19;Drone.y19;Drone.z19];

Drone.x20=[-10.276 -11.720 -11.720 -10.276 -10.276]*scale;
Drone.y20=[-9.2500 -9.2603 -9.2603 -9.2500 -9.2500]*scale;
Drone.z20=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{21}=[Drone.x20;Drone.y20;Drone.z20];

Drone.x21=[-11.720 -12.612 -12.612 -11.720 -11.720]*scale;
Drone.y21=[-9.2603 -10.3954 -10.3954 -9.2603 -9.2603]*scale;
Drone.z21=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{22}=[Drone.x21;Drone.y21;Drone.z21];

Drone.x22=[-12.612 -12.281 -12.281 -12.612 -12.612]*scale;
Drone.y22=[-10.3954 -11.8007 -11.8007 -10.3954 -10.3954]*scale;
Drone.z22=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{23}=[Drone.x22;Drone.y22;Drone.z22];

Drone.x23=[-12.281 -10.976 -10.976 -12.281 -12.281]*scale;
Drone.y23=[-11.8007 -12.4180 -12.4180 -11.8007 -11.8007]*scale;
Drone.z23=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{24}=[Drone.x23;Drone.y23;Drone.z23];

Drone.x24=[-10.976 -9.6803 -9.6803 -10.976 -10.976]*scale;
Drone.y24=[-12.4180 -11.7825 -11.7825 -12.4180 -12.4180]*scale;
Drone.z24=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{25}=[Drone.x24;Drone.y24;Drone.z24];

Drone.x25=[-9.6803 -9.3689 -9.3689 -9.6803 -9.6803]*scale;
Drone.y25=[-11.7825 -10.3727 -10.3727 -11.7825 -11.7825]*scale;
Drone.z25=[1.7875 1.7875 1.4875 1.4875 1.7875]*scale;
Drone.Estructura{26}=[Drone.x25;Drone.y25;Drone.z25];

%Tapas laterales
Drone.x26=[4.0000 10.3727 10.3727 4.0000 4.0000]*scale;
Drone.y26=[-1.4868 -9.3689 -9.3689 -1.4868 -1.4868]*scale;
Drone.z26=[2.7875 1.7875 1.4875 0.7500 2.7875]*scale;
Drone.Estructura{27}=[Drone.x26;Drone.y26;Drone.z26];

Drone.x27=[4.0000 4.0000 4.0000 4.0000 4.0000]*scale;
Drone.y27=[-1.4868 1.4868 1.4868 -1.4868 -1.4868]*scale;
Drone.z27=[2.7875 2.7875 0.7500 0.7500 2.7875]*scale;
Drone.Estructura{28}=[Drone.x27;Drone.y27;Drone.z27];

Drone.x28=[4.0000 10.2766 10.2766 4.0000 4.0000]*scale;
Drone.y28=[1.4868 9.2500 9.2500 1.4868 1.4868]*scale;
Drone.z28=[2.7875 1.7875 1.4875 0.7500 2.7875]*scale;
Drone.Estructura{29}=[Drone.x28;Drone.y28;Drone.z28];

Drone.x29=[1.4868 9.3689 9.3689 1.4868 1.4868]*scale;
Drone.y29=[4.0000 10.3727 10.3727 4.0000 4.0000]*scale;
Drone.z29=[2.7875 1.7875 1.4875 0.7500 2.7875]*scale;
Drone.Estructura{30}=[Drone.x29;Drone.y29;Drone.z29];

Drone.x30=[1.4868 -1.4868 -1.4868 1.4868 1.4868]*scale;
Drone.y30=[4.0000 4.0000 4.0000 4.0000 4.0000]*scale;
Drone.z30=[2.7875 2.7875 0.7500 0.7500 2.7875]*scale;
Drone.Estructura{31}=[Drone.x30;Drone.y30;Drone.z30];

Drone.x31=[-1.4868 -9.3689 -9.3689 -1.4868 -1.4868]*scale;
Drone.y31=[4.0000 10.3737 10.3737 4.0000 4.0000]*scale;
Drone.z31=[2.7875 1.7875 1.4875 0.7500 2.7875]*scale;
Drone.Estructura{32}=[Drone.x31;Drone.y31;Drone.z31];

Drone.x32=[-4.0000 -10.491 -10.491 -4.0000 -4.0000]*scale;
Drone.y32=[1.4868 9.4660 9.4660 1.4868 1.4868]*scale;
Drone.z32=[2.7875 1.7875 1.4875 0.7500 2.7875]*scale;
Drone.Estructura{33}=[Drone.x32;Drone.y32;Drone.z32];

Drone.x33=[-4.0000 -4.0000 -4.0000 -4.0000 -4.0000]*scale;
Drone.y33=[1.4868 -1.4868 -1.4868 1.4868 1.4868]*scale;
Drone.z33=[2.7875 2.7875 0.7500 0.7500 2.7875]*scale;
Drone.Estructura{34}=[Drone.x33;Drone.y33;Drone.z33];

Drone.x34=[-4.0000 -10.276 -10.276 -4.0000 -4.0000]*scale;
Drone.y34=[-1.4868 -9.2500 -9.2500 -1.4868 -1.4868]*scale;
Drone.z34=[2.7875 1.7875 1.4875 0.7500 2.7875]*scale;
Drone.Estructura{35}=[Drone.x34;Drone.y34;Drone.z34];

Drone.x35=[-1.4868 -9.3689 -9.3689 -1.4868 -1.4868]*scale;
Drone.y35=[-4.0000 -10.3727 -10.3727 -4.0000 -4.0000]*scale;
Drone.z35=[2.7875 1.7875 1.4875 0.7500 2.7875]*scale;
Drone.Estructura{36}=[Drone.x35;Drone.y35;Drone.z35];

Drone.x36=[-1.4868 1.4868 1.4868 -1.4868 -1.4868]*scale;
Drone.y36=[-4.0000 -4.0000 -4.0000 -4.0000 -4.0000]*scale;
Drone.z36=[2.7875 2.7875 0.7500 0.7500 2.7875]*scale;
Drone.Estructura{37}=[Drone.x36;Drone.y36;Drone.z36];

Drone.x37=[1.4868 9.2500 9.2500 1.4868 1.4868]*scale;
Drone.y37=[-4.0000 -10.2766 -10.2766 -4.0000 -4.0000]*scale;
Drone.z37=[2.7875 1.7875 1.4875 0.7500 2.7875]*scale;
Drone.Estructura{38}=[Drone.x37;Drone.y37;Drone.z37];


%Hélice 1
%Unión
Drone.x38=[10.8789 10.8789 10.8789 10.8789 10.8789]*scale;
Drone.y38=[-11.7129 -10.2640 -10.2640 -11.7129 -11.7129]*scale;
Drone.z38=[3.4256 3.4256 3.0376 3.0376 3.4256]*scale;
Drone.Helice1{1}=[Drone.x38;Drone.y38;Drone.z38];

Drone.x39=[10.6289 10.8789 10.8789 10.6289 10.6289]*scale;
Drone.y39=[-10.2640 -10.2641 -10.2640 -10.2640 -10.2640]*scale;
Drone.z39=[3.4254 3.4256 3.0374 3.0374 3.4254]*scale;
Drone.Helice1{2}=[Drone.x39;Drone.y39;Drone.z39];

Drone.x40=[10.6289 10.6289 10.6289 10.6289 10.6289]*scale;
Drone.y40=[-10.2640 -11.7129 -11.7129 -10.2640 -10.2640]*scale;
Drone.z40=[3.4254 3.4254 3.0374 3.0376 3.4254]*scale;
Drone.Helice1{3}=[Drone.x40;Drone.y40;Drone.z40];

Drone.x41=[10.6289 10.8789 10.8789 10.6289 10.6289]*scale;
Drone.y41=[-11.7129 -11.7129 -11.7129 -11.7129 -11.7129]*scale;
Drone.z41=[3.4256 3.4256 3.0374 3.0376 3.4256]*scale;
Drone.Helice1{4}=[Drone.x41;Drone.y41;Drone.z41];

%Aspas
Drone.x44=[10.8789 18.3789 18.3789 15.3789 10.8789 10.8789]*scale;
Drone.y44=[-11.7129 -11.7129 -11.23 -10.264 -10.2641 -11.7129]*scale;
Drone.z44=[3.0376 3.0374 3.1668 3.4256 3.4256 3.0376]*scale;
Drone.Helice1{5}=[Drone.x44;Drone.y44;Drone.z44];

Drone.x45=[10.6289 6.1289 3.1289 3.1289 10.6289 10.6289]*scale;
Drone.y45=[-11.7129 -11.7129 -10.747 -10.264 -10.264 -11.7129]*scale;
Drone.z45=[3.4254 3.4256 3.1668 3.0374 3.0374 3.4254]*scale;
Drone.Helice1{6}=[Drone.x45;Drone.y45;Drone.z45];

%Eje
Drone.x46=[10.8789 10.8789 10.8789 10.8789 10.8789]*scale;
Drone.y46=[-11.2385 -11.2385 -10.7385 -10.7385 -11.2385]*scale;
Drone.z46=[3.0376 1.7876 1.7876 3.0376 3.0376]*scale;
Drone.Helice1{7}=[Drone.x46;Drone.y46;Drone.z46];

Drone.x47=[10.8789 10.8789 10.6289 10.6289 10.8789]*scale;
Drone.y47=[-10.7385 -10.7385 -10.7385 -10.7385 -10.7385]*scale;
Drone.z47=[3.0376 1.7876 1.7875 3.0375 3.0376]*scale;
Drone.Helice1{8}=[Drone.x47;Drone.y47;Drone.z47];

Drone.x48=[10.6289 10.6289 10.6289 10.6289 10.6289]*scale;
Drone.y48=[-10.7385 -10.7385 -11.2385 -11.2385 -10.7385]*scale;
Drone.z48=[3.0375 1.7875 1.7875 3.0375 3.0375]*scale;
Drone.Helice1{9}=[Drone.x48;Drone.y48;Drone.z48];

Drone.x49=[10.6289 10.6289 10.8789 10.8789 10.6289]*scale;
Drone.y49=[-11.2385 -11.2385 -11.2385 -11.2385 -11.2385]*scale;
Drone.z49=[3.0375 1.7875 1.7876 3.0376 3.0375]*scale;
Drone.Helice1{10}=[Drone.x49;Drone.y49;Drone.z49];

%Hélice 2
%Unión
Drone.x382=[11.4473 11.6132 11.6133 11.4473 11.4473]*scale;
Drone.y382=[11.3283 11.1413 11.1413 11.3283 11.3283]*scale;
Drone.z382=[3.4254 3.4256 3.0374 3.0374 3.4254]*scale;
Drone.Helice2{1}=[Drone.x382;Drone.y382;Drone.z382];

Drone.x392=[11.4473 10.3637 10.3637 11.4473 11.4473]*scale;
Drone.y392=[11.3283 10.3664 10.3664 11.3283 11.3283]*scale;
Drone.z392=[3.4254 3.4254 3.0376 3.0374 3.4254]*scale;
Drone.Helice2{2}=[Drone.x392;Drone.y392;Drone.z392];

Drone.x402=[10.3637 10.5297 10.5297 10.3637 10.3637]*scale;
Drone.y402=[10.3664 10.1795 10.1795 10.3664 10.3664]*scale;
Drone.z402=[3.4256 3.4256 3.0374 3.0376 3.4256]*scale;
Drone.Helice2{3}=[Drone.x402;Drone.y402;Drone.z402];

Drone.x412=[10.5297 11.6133 11.6133 10.5297 10.5297]*scale;
Drone.y412=[10.1795 11.1413 11.1413 10.1795 10.1795]*scale;
Drone.z412=[3.4256 3.4256 3.0376 3.0376 3.4256]*scale;
Drone.Helice2{4}=[Drone.x412;Drone.y412;Drone.z412];

%Aspas
Drone.x442=[10.3637 7.3764 6.1073 6.4685 11.4473 10.3637]*scale;
Drone.y442=[10.3664 13.7319 16.6167 16.9373 11.3283 10.3664]*scale;
Drone.z442=[3.4254 3.4256 3.1668 3.0374 3.0374 3.4254]*scale;
Drone.Helice2{5}=[Drone.x442;Drone.y442;Drone.z442];

Drone.x452=[10.5297 15.5085 15.8697 14.6005 11.6133]*scale;
Drone.y452=[10.1795 4.5704 4.891 7.7759 11.1413]*scale;
Drone.z452=[3.0376 3.0374 3.1668 3.4256 3.4256]*scale;
Drone.Helice2{6}=[Drone.x452;Drone.y452;Drone.z452];

%Eje
Drone.x462=[11.2584 11.2584 11.0925 11.0925 11.2584]*scale;
Drone.y462=[10.8263 10.8263 11.0133 11.0133 10.8263]*scale;
Drone.z462=[3.0376 1.7876 1.7875 3.0375 3.0376]*scale;
Drone.Helice2{7}=[Drone.x462;Drone.y462;Drone.z462];

Drone.x472=[11.0925 11.0925 10.7185 10.7185 11.0925]*scale;
Drone.y472=[11.0133 11.0133 10.6814 10.6814 11.0133]*scale;
Drone.z472=[3.0375 1.7875 1.7875 3.0375 3.0375]*scale;
Drone.Helice2{8}=[Drone.x472;Drone.y472;Drone.z472];

Drone.x482=[10.7185 10.7185 10.8845 10.8845 10.7185]*scale;
Drone.y482=[10.6814 10.6814 10.4944 10.4944 10.6814]*scale;
Drone.z482=[3.0375 1.7875 1.7876 3.0376 3.0375]*scale;
Drone.Helice2{9}=[Drone.x482;Drone.y482;Drone.z482];

Drone.x492=[10.8845 10.8845 11.2584 11.2584 10.8845]*scale;
Drone.y492=[10.4944 10.4944 10.8263 10.8263 10.4944]*scale;
Drone.z492=[3.0376 1.7876 1.7876 3.0376 3.0376]*scale;
Drone.Helice2{10}=[Drone.x492;Drone.y492;Drone.z492];

%Hélice 3
%Unión
Drone.x383=[-10.240 -10.151 -10.151 -10.240 -10.240]*scale;
Drone.y383=[11.4602 11.2265 11.2265 11.4602 11.4602]*scale;
Drone.z383=[3.4254 3.4256 3.0374 3.0374 3.4254]*scale;
Drone.Helice3{1}=[Drone.x383;Drone.y383;Drone.z383];

Drone.x393=[-10.240 -11.594 -11.594 -10.240 -10.240]*scale;
Drone.y393=[11.4602 10.9447 10.9447 11.4602 11.4602]*scale;
Drone.z393=[3.4254 3.4254 3.0374 3.0376 3.4254]*scale;
Drone.Helice3{2}=[Drone.x393;Drone.y393;Drone.z393];

Drone.x403=[-11.594 -11.505 -11.505 -11.594 -11.594]*scale;
Drone.y403=[10.9447 10.7110 10.7110 10.9447 10.9447]*scale;
Drone.z403=[3.4256 3.4256 3.0374 3.0376 3.4256]*scale;
Drone.Helice3{3}=[Drone.x403;Drone.y403;Drone.z403];

Drone.x413=[-11.505 -10.151 -10.151 -11.505 -11.505]*scale;
Drone.y413=[10.7110 11.2265 11.2265 10.7110 10.7110]*scale;
Drone.z413=[3.4256 3.4256 3.0376 3.0376 3.4256]*scale;
Drone.Helice3{4}=[Drone.x413;Drone.y413;Drone.z413];

%Aspas
Drone.x443=[-11.5943 -13.1954 -13.36 -12.9087 -10.2402 -11.5943]*scale;
Drone.y443=[10.9447 15.1502 18.2975 18.4694 11.4602 10.9447]*scale;
Drone.z443=[3.4254 3.4256 3.1668 3.0374 3.0374 3.4254]*scale;
Drone.Helice3{5}=[Drone.x443;Drone.y443;Drone.z443];

Drone.x453=[-11.5053 -8.8368 -8.3855 -8.5501 -10.1513 -11.5053]*scale;
Drone.y453=[10.711 3.7018 3.8736 7.021 11.2265 10.711]*scale;
Drone.z453=[3.0374 3.0374 3.1668 3.4256 3.4256 3.0374]*scale;
Drone.Helice3{6}=[Drone.x453;Drone.y453;Drone.z453];

%Eje
Drone.x463=[-10.594 -10.594 -10.683 -10.683 -10.594]*scale;
Drone.y463=[11.0577 11.0577 11.2914 11.2914 11.0577]*scale;
Drone.z463=[3.0376 1.7876 1.7875 3.0375 3.0376]*scale;
Drone.Helice3{7}=[Drone.x463;Drone.y463;Drone.z463];

Drone.x473=[-10.683 -10.683 -11.150 -11.150 -10.683]*scale;
Drone.y473=[11.2914 11.2914 11.1135 11.1135 11.2914]*scale;
Drone.z473=[3.0375 1.7875 1.7875 3.0375 3.0375]*scale;
Drone.Helice3{8}=[Drone.x473;Drone.y473;Drone.z473];

Drone.x483=[-11.150 -11.150 -11.061 -11.061 -11.150]*scale;
Drone.y483=[11.1135 11.1135 10.8798 10.8798 11.1135]*scale;
Drone.z483=[3.0375 1.7875 1.7876 3.0376 3.0375]*scale;
Drone.Helice3{9}=[Drone.x483;Drone.y483;Drone.z483];

Drone.x493=[-11.061 -11.061 -10.594 -10.594 -11.061]*scale;
Drone.y493=[10.8798 10.8798 11.0577 11.0577 10.8798]*scale;
Drone.z493=[3.0376 1.7876 1.7876 3.0376 3.0376]*scale;
Drone.Helice3{10}=[Drone.x493;Drone.y493;Drone.z493];

%Hélice 4
%Unión
Drone.x384=[-10.284 -10.396 -10.396 -10.284 -10.284]*scale;
Drone.y384=[-10.9660 -11.1896 -11.189 -10.9660 -10.9660]*scale;
Drone.z384=[3.4253 3.4255 3.0373 3.0373 3.4253]*scale;
Drone.Helice4{1}=[Drone.x384;Drone.y384;Drone.z384];

Drone.x394=[-10.284 -11.580 -11.580 -10.284 -10.284]*scale;
Drone.y394=[-10.9660 -10.3181 -10.3181 -10.9660 -10.9660]*scale;
Drone.z394=[3.4253 3.4253 3.0375 3.0375 3.4253]*scale;
Drone.Helice4{2}=[Drone.x394;Drone.y394;Drone.z394];

Drone.x404=[-11.580 -11.692 -11.692 -11.580 -11.580]*scale;
Drone.y404=[-10.3181 -10.5417 -10.5417 -10.3181 -10.3181]*scale;
Drone.z404=[3.4255 3.4255 3.0375 3.0375 3.4255]*scale;
Drone.Helice4{3}=[Drone.x404;Drone.y404;Drone.z404];

Drone.x414=[-11.692 -10.396 -10.396 -11.692 -11.692]*scale;
Drone.y414=[-10.5417 -11.1897 -11.1897 -10.5417 -10.5417]*scale;
Drone.z414=[3.4255 3.4255 3.0375 3.0375 3.4255]*scale;
Drone.Helice4{4}=[Drone.x414;Drone.y414;Drone.z414];

%Aspas
Drone.x444=[-10.3965 -12.4089 -14.6145 -15.0464 -11.6923 -10.3965]*scale;
Drone.y444=[-11.1896 -15.2146 -17.4659 -17.2499 -10.5417 -11.1896]*scale;
Drone.z444=[3.4255 3.4255 3.1667 3.0373 3.0375 3.4255]*scale;
Drone.Helice4{5}=[Drone.x444;Drone.y444;Drone.z444];

Drone.x454=[-11.5805 -9.5681 -7.3625 -6.9305 -10.2846 -11.5805]*scale;
Drone.y454=[-10.3181 -6.2932 -4.0419 -4.2578 -10.966 -10.3181]*scale;
Drone.z454=[3.4253 3.4255 3.1667 3.0373 3.0373 3.4253]*scale;
Drone.Helice4{6}=[Drone.x454;Drone.y454;Drone.z454];

%Eje
Drone.x464=[-10.709 -10.709 -11.156 -11.156 -10.709]*scale;
Drone.y464=[-10.7539 -10.7539 -10.5303 -10.5303 -10.7539]*scale;
Drone.z464=[3.0374 1.7874 1.7874 3.0374 3.0374]*scale;
Drone.Helice4{7}=[Drone.x464;Drone.y464;Drone.z464];

Drone.x474=[-11.156 -11.156 -11.268 -11.268 -11.156]*scale;
Drone.y474=[-10.5303 -10.5303 -10.7539 -10.7539 -10.5303]*scale;
Drone.z474=[3.0374 1.7874 1.7875 3.0375 3.0374]*scale;
Drone.Helice4{8}=[Drone.x474;Drone.y474;Drone.z474];

Drone.x484=[-11.268 -11.268 -10.820 -10.820 -11.268]*scale;
Drone.y484=[-10.7539 -10.7539 -10.9775 -10.9775 -10.7539]*scale;
Drone.z484=[3.0375 1.7875 1.7875 3.0375 3.0375]*scale;
Drone.Helice4{9}=[Drone.x484;Drone.y484;Drone.z484];

Drone.x494=[-10.820 -10.820 -10.709 -10.709 -10.820]*scale;
Drone.y494=[-10.9775 -10.9775 -10.7539 -10.7539 -10.9775]*scale;
Drone.z494=[3.0375 1.7875 1.7874 3.0374 3.0375]*scale;
Drone.Helice4{10}=[Drone.x494;Drone.y494;Drone.z494];

%Soportes
%Soporte 1
Drone.x50=[4 3.8 3.8 -3.8 -3.8 -4 -4 4 4]*scale;
Drone.y50=[-3.8 -3.8 -3.8 -3.8 -3.8 -3.8 -3.8 -3.8 -3.8]*scale;
Drone.z50=[0.9 0.9 -1 -1 0.9 0.9 -1 -1 0.9]*scale;
Drone.Soportes{1}=[Drone.x50;Drone.y50;Drone.z50];

Drone.x51=[4 3.8 3.8 -3.8 -3.8 -4 -4 4 4]*scale;
Drone.y51=[-4 -4 -4 -4 -4 -4 -4 -4 -4]*scale;
Drone.z51=[0.9 0.9 -1 -1 0.9 0.9 -1 -1 0.9]*scale;
Drone.Soportes{2}=[Drone.x51;Drone.y51;Drone.z51];

Drone.x52=[4 4 4 4 4]*scale;
Drone.y52=[-4 -3.8 -3.8 -4 -4]*scale;
Drone.z52=[0.9 0.9 -1 -1 0.9]*scale;
Drone.Soportes{3}=[Drone.x52;Drone.y52;Drone.z52];

Drone.x53=[4 4 -4 -4 4]*scale;
Drone.y53=[-4 -3.8 -3.8 -4 -4]*scale;
Drone.z53=[-1 -1 -1 -1 -1]*scale;
Drone.Soportes{4}=[Drone.x53;Drone.y53;Drone.z53];

Drone.x54=[-4 -4 -4 -4 -4]*scale;
Drone.y54=[-3.8 -4 -4 -3.8 -3.8]*scale;
Drone.z54=[-1 -1 0.9 0.9 -1]*scale;
Drone.Soportes{5}=[Drone.x54;Drone.y54;Drone.z54];

% %Soporte 2
Drone.x501=[4 3.8 3.8 -3.8 -3.8 -4 -4 4 4]*scale;
Drone.y501=[4 4 4 4 4 4 4 4 4]*scale;
Drone.z501=[0.9 0.9 -1 -1 0.9 0.9 -1 -1 0.9]*scale;
Drone.Soportes{6}=[Drone.x501;Drone.y501;Drone.z501];

Drone.x511=[4 3.8 3.8 -3.8 -3.8 -4 -4 4 4]*scale;
Drone.y511=[3.8 3.8 3.8 3.8 3.8 3.8 3.8 3.8 3.8]*scale;
Drone.z511=[0.9 0.9 -1 -1 0.9 0.9 -1 -1 0.9]*scale;
Drone.Soportes{7}=[Drone.x511;Drone.y511;Drone.z511];

Drone.x521=[4 4 4 4 4]*scale;
Drone.y521=[3.8 4 4 3.8 3.8]*scale;
Drone.z521=[0.9 0.9 -1 -1 0.9]*scale;
Drone.Soportes{8}=[Drone.x521;Drone.y521;Drone.z521];

Drone.x531=[4 4 -4 -4 4]*scale;
Drone.y531=[3.8 4 4 3.8 3.8]*scale;
Drone.z531=[-1 -1 -1 -1 -1]*scale;
Drone.Soportes{9}=[Drone.x531;Drone.y531;Drone.z531];

Drone.x541=[-4 -4 -4 -4 -4]*scale;
Drone.y541=[3.8 4 4 3.8 3.8]*scale;
Drone.z541=[0.9 0.9 -1 -1 0.9]*scale;
Drone.Soportes{10}=[Drone.x541;Drone.y541;Drone.z541];


% soporte brazo
d=0.85; % altura de la base para el brazo
Drone.x542=[-4 4 4 -4  ]*scale;
Drone.y542=[4 4 -4 -4]*scale;
Drone.z542=[-1 -1 -1 -1 ]*scale;
Drone.Soportes{11}=[Drone.x542;Drone.y542;Drone.z542];

Drone.x543=[-4 4 4 -4]*scale;
Drone.y543=[4 4 -4 -4]*scale;
Drone.z543=[-d -d -d -d]*scale;
Drone.Soportes{12}=[Drone.x543;Drone.y543;Drone.z543];

Drone.x544=[-4 4 4 -4]*scale;
Drone.y544=[-4 -4 -4 -4]*scale;
Drone.z544=[-d -d -1 -1]*scale;
Drone.Soportes{13}=[Drone.x544;Drone.y544;Drone.z544];

Drone.x545=[-4 4 4 -4]*scale;
Drone.y545=[4 4 4 4]*scale;
Drone.z545=[-d -d -1 -1]*scale;
Drone.Soportes{14}=[Drone.x545;Drone.y545;Drone.z545];


Drone.x546=[4 4 4 4]*scale;
Drone.y546=[4 -4 -4 4]*scale;
Drone.z546=[-d -d -1 -1]*scale;
Drone.Soportes{15}=[Drone.x546;Drone.y546;Drone.z546];

Drone.x547=[-4 -4 -4 -4]*scale;
Drone.y547=[4 -4 -4 4]*scale;
Drone.z547=[-d -d -1 -1]*scale;
Drone.Soportes{16}=[Drone.x547;Drone.y547;Drone.z547];

% %Eje cámara
% Drone.x55=[0.1000 0.1000 0.1000 0.1000 0.1]*scale;
% Drone.y55=[-0.1000 -0.1000 0.1000 0.1000 -0.1]*scale;
% Drone.z55=[0.7500 -0.7500 -0.7500 0.7500 0.75]*scale;
% Drone.Camara{1}=[Drone.x55;Drone.y55;Drone.z55];
% 
% Drone.x56=[0.1000 0.1000 -0.1000 -0.1000 0.1]*scale;
% Drone.y56=[0.1000 0.1000 0.1000 0.1000 0.1]*scale;
% Drone.z56=[0.7500 -0.7500 -0.7500 0.7500 0.75]*scale;
% Drone.Camara{2}=[Drone.x56;Drone.y56;Drone.z56];
% 
% Drone.x57=[-0.1000 -0.1000 -0.1000 -0.1000 -0.1]*scale;
% Drone.y57=[0.1000 0.1000 -0.1000 -0.1000 0.1]*scale;
% Drone.z57=[0.7500 -0.7500 -0.7500 0.7500 0.75]*scale;
% Drone.Camara{3}=[Drone.x57;Drone.y57;Drone.z57];
% 
% Drone.x58=[-0.1000 -0.1000 0.1000 0.1000 -0.1]*scale;
% Drone.y58=[-0.1000 -0.1000 -0.1000 -0.1000 -0.1]*scale;
% Drone.z58=[0.7500 -0.7500 -0.7500 0.750 0.75]*scale;
% Drone.Camara{4}=[Drone.x58;Drone.y58;Drone.z58];

% %Cámara
% Drone.x59=[1.0500 1.0500 1.0500 1.0500 1.0500]*scale;
% Drone.y59=[1.0500 -1.0500 -1.0500 1.0500 1.0500]*scale;
% Drone.z59=[-0.7500 -0.7500 -2.1500 -2.1500 -0.7500]*scale;
% Drone.Camara{5}=[Drone.x59;Drone.y59;Drone.z59];
% 
% Drone.x60=[-1.0500 1.0500 1.0500 -1.0500 -1.0500]*scale;
% Drone.y60=[1.0500 1.0500 1.0500 1.0500 1.0500]*scale;
% Drone.z60=[-0.7500 -0.7500 -2.1500 -2.1500 -0.7500]*scale;
% Drone.Camara{6}=[Drone.x60;Drone.y60;Drone.z60];
% 
% Drone.x61=[-1.0500 -1.0500 -1.0500 -1.0500 -1.0500]*scale;
% Drone.y61=[-1.0500 1.0500 1.0500 -1.0500 -1.0500]*scale;
% Drone.z61=[-0.7500 -0.7500 -2.1500 -2.1500 -0.7500]*scale;
% Drone.Camara{7}=[Drone.x61;Drone.y61;Drone.z61];
% 
% Drone.x62=[-1.0500 1.0500 1.0500 -1.0500 -1.0500]*scale;
% Drone.y62=[-1.0500 -1.0500 -1.0500 -1.0500 -1.0500]*scale;
% Drone.z62=[-0.7500 -0.7500 -2.1500 -2.1500 -0.7500]*scale;
% Drone.Camara{8}=[Drone.x62;Drone.y62;Drone.z62];
% 
% Drone.x63=[1.05 1.05 -1.05 -1.05 1.05]*scale;
% Drone.y63=[-1.05 1.05 1.05 -1.05 -1.05]*scale;
% Drone.z63=[-0.75 -0.75 -0.75 -0.75 -0.75]*scale;
% Drone.Camara{9}=[Drone.x63;Drone.y63;Drone.z63];
% 
% Drone.x64=[1.05 1.05 -1.05 -1.05 1.05]*scale;
% Drone.y64=[-1.05 1.05 1.05 -1.05 -1.05]*scale;
% Drone.z64=[-2.15 -2.15 -2.15 -2.15 -2.15]*scale;
% Drone.Camara{10}=[Drone.x64;Drone.y64;Drone.z64];

% %Lente de la cámara
% Drone.x65=[1.3 1.3 1.3 1.3 1.3 1.3 1.3]*scale;
% Drone.y65=[0.311 0.6221 0.311 -0.311 -0.6221 -0.311 0.311]*scale;
% Drone.z65=[-0.9112 -1.45 -1.988 -1.988 -1.45 -0.9112 -0.9112]*scale;
% Drone.Camara{11}=[Drone.x65;Drone.y65;Drone.z65];
% 
% Drone.x66=[1.3 1.3 1.05 1.05 1.3]*scale;
% Drone.y66=[0.311 0.6221 0.6221 0.311 0.311]*scale;
% Drone.z66=[-0.9112 -1.45 -1.45 -0.9112 -0.9112]*scale;
% Drone.Camara{12}=[Drone.x66;Drone.y66;Drone.z66];
% 
% Drone.x67=[1.3 1.3 1.05 1.05 1.3]*scale;
% Drone.y67=[0.6221 0.311 0.311 0.6221 0.6221]*scale;
% Drone.z67=[-1.45 -1.988 -1.988 -1.45 -1.45]*scale;
% Drone.Camara{13}=[Drone.x67;Drone.y67;Drone.z67];
% 
% Drone.x68=[1.3 1.3 1.05 1.05 1.3]*scale;
% Drone.y68=[0.311 -0.311 -0.311 0.311 0.311]*scale;
% Drone.z68=[-1.988 -1.988 -1.988 -1.988 -1.988]*scale;
% Drone.Camara{14}=[Drone.x68;Drone.y68;Drone.z68];
% 
% Drone.x69=[1.3 1.3 1.05 1.05 1.3]*scale;
% Drone.y69=[-0.311 -0.6221 -0.6221 -0.311 -0.311]*scale;  
% Drone.z69=[-1.988 -1.45 -1.45 -1.988 -1.988]*scale;
% Drone.Camara{15}=[Drone.x69;Drone.y69;Drone.z69];
% 
% Drone.x70=[1.3 1.3 1.05 1.05 1.3]*scale;
% Drone.y70=[-0.6221 -0.311 -0.311 -0.6221 -0.6221]*scale;
% Drone.z70=[-1.45 -0.9112 -0.9112 -1.45 -1.45]*scale;
% Drone.Camara{16}=[Drone.x70;Drone.y70;Drone.z70];
% 
% Drone.x71=[1.3 1.3 1.05 1.05 1.3]*scale;
% Drone.y71=[-0.311 0.311 0.311 -0.311 -0.311]*scale;
% Drone.z71=[-0.9112 -0.9112 -0.9112 -0.9112 -0.9112]*scale;
% Drone.Camara{17}=[Drone.x71;Drone.y71;Drone.z71];

axis equal