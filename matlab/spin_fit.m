  clc
clear

% 英雄
if 0
 disp("芝士英雄");
 x = [33,40,45,50,60,75,100,105,120,150]; % 功率
 y = [6,7,8,10,12,15,15.5,16,17,18]; % wz
elseif 0
% 穷步
 disp("芝士穷步");
 x = [17,20  ,27,40,55,67,78,95,110,130,140,150]; % 功率
 y = [15,16.5,20,25,30,35,40,45,50 ,60 ,65 ,70 ]; % wz
elseif 1
% 全向轮
 disp("芝士全向");                    % | <-开始随便加的
 x = [9, 13,18,21,30,41,55,65,80,90,95 ,120,130,140,150]; % 功率
 y = [15,20,25,30,40,50,60,70,80,90,100,120,130,140,150]; % wz
end

param=polyfit(x,y,4)
x_t=1:0.1:150;
y_t=param(5)+param(4).*x_t+...
param(3).*x_t.^2+param(2).*x_t.^3+...
param(1).*x_t.^4;
plot(x_t,y_t);
y_t(x_t==70)
y_t(x_t==120)
fprintf("%.8ff*pow4of(P)+%.8ff*pow3of(P)+\n" + ...
    "%.8ff*pow2of(P)+%.8ff*P+\n" + ...
    "%.8ff\n",...
    param(1),param(2),param(3),param(4),param(5)); 