clc; clear; close all;

Qx = 1.9;
Sx = pi/(2*atan(Qx));
alpha = deg2rad(10);
mux_max = 2.5;
FZ = 1000;

f1 = @(fx,kappa) fx - mux_max*sin(Qx*atan(Sx*(kappa^2 + alpha^2)))*FZ*kappa/(kappa^2 + alpha^2);

N = 100;
FX = linspace(0,1000,N);

for ii = 1:N
    f2 = @(kappa) f1(FX(ii),kappa);
    kappa(ii) = fsolve(f2,0);
end

figure
plot(kappa,FX)
min(kappa)
max(kappa)