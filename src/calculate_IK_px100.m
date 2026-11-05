function [t1, t2, t3, t4, valid] = calculate_IK_px100(Links, OP)
% CALCULATE_IK_PX100  Cinematica inversa do PX100 (garra sempre vertical).
%   OP = [x y z] em metros (referencial da base do robo)
%   Devolve os 4 angulos [rad] e valid=false se o ponto estiver fora de alcance.

L1 = Links(1);   % base (ombro)
L2 = Links(2);   % braco
Lm = Links(3);   % desvio do ombro
L3 = Links(4);   % antebraco
L4 = Links(5);   % garra

% --- junta 1: rotacao da base ---
t1 = atan2(OP(2), OP(1));
r  = hypot(OP(1), OP(2));
z  = OP(3);

% altura efetiva do pulso (desconta a garra e a base)
h = z + L4 - L1;

% --- geometria no plano r-h ---
LA = hypot(Lm, L2);       % comprimento efetivo do braco
LB = hypot(r, h);         % distancia ombro -> pulso

valid = (LB <= LA + L3) && (LB >= abs(LA - L3));
if ~valid
    fprintf('FORA DE ALCANCE: r=%.3f, LB=%.3f, max=%.3f\n', r, LB, LA+L3);
    t1 = 0; t2 = 0; t3 = 0; t4 = 0;
    return;
end

% --- junta 2 ---
Beta  = atan2(Lm, L2);
alpha = acos(max(min((L3^2 - LA^2 - LB^2) / (-2*LA*LB), 1), -1));
gama  = atan2(h, r);
t2    = pi/2 - Beta - alpha - gama;

% --- junta 3 ---
Beta2 = pi/2 - Beta;
lamda = acos(max(min((LB^2 - LA^2 - L3^2) / (-2*LA*L3), 1), -1));
t3    = pi - Beta2 - lamda;

% --- junta 4: manter a garra vertical ---
t4 = pi/2 - t2 - t3;
end