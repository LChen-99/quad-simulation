close all;
clear;

a = 1;
time = 0:0.01:10;
x = a*cos(time)./(1 + sin(time).^2);
y = zeros(size(time));
z = a*sin(time).*cos(time)./(1 + sin(time).^2);

plot3(x, y, z);
xlabel('x');
ylabel('y');
zlabel('z');