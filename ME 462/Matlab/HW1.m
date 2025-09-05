%% Unit step responses (analytic via state eq. vs MATLAB)
clear; clc; close all;

% ---- parameters ----
m = 1; b = 2; k = 5;         % <--- 修改成你自己的数值

% ---- common matrices ----
A = [0 1; -k/m -b/m];
C1 = [1 0]; D1 = 0;
C2 = [1 0]; D2 = 0;
B1 = [0; 1/m];
B2 = [b/m; k/m - (b/m)^2];

% time grid
Tend = 8; t = linspace(0,Tend,4001).';
I = eye(2); Ai = inv(A);

% ----- Analytic via state-equation: y(t)=C*A^{-1}(e^{At}-I)B -----
y1_ana = zeros(size(t));
y2_ana = zeros(size(t));
for i = 1:numel(t)
    E = expm(A*t(i));
    y1_ana(i) = C1 * (Ai*(E - I) * B1) + D1;
    y2_ana(i) = C2 * (Ai*(E - I) * B2) + D2;
end

% ----- Numeric using MATLAB step() -----
sys1 = ss(A,B1,C1,D1);
sys2 = ss(A,B2,C2,D2);
y1_num = step(sys1, t);
y2_num = step(sys2, t);

% ----- Compare & plot -----
figure('Color','w'); hold on; grid on;
plot(t,y1_ana,'LineWidth',2);
plot(t,y1_num,'--','LineWidth',1.6);
title('System 1: unit step (analytic vs MATLAB)'); xlabel t; ylabel('y_1(t)');
legend('analytic','MATLAB step'); 
fprintf('System 1: max |diff| = %.3e\n', max(abs(y1_ana - y1_num)));

figure('Color','w'); hold on; grid on;
plot(t,y2_ana,'LineWidth',2);
plot(t,y2_num,'--','LineWidth',1.6);
title('System 2: unit step (analytic vs MATLAB)'); xlabel t; ylabel('y_2(t)');
legend('analytic','MATLAB step');
fprintf('System 2: max |diff| = %.3e\n', max(abs(y2_ana - y2_num)));


%% -------- System b (fill in your A,B, C, D from Q1) --------------
% A = [...];  B = [...];  C = [...];  D = ...;   % <-- 在此填入 Q1 的状态空间矩阵
% % analytic
% n = size(A,1); I = eye(n); Ai = inv(A);
% Yb_analytic = zeros(size(t));
% for i = 1:numel(t)
%     Yb_analytic(i) = C * (Ai*(expm(A*t(i)) - I) * B) + D;
% end
% % numeric
% sysB = ss(A,B,C,D);
% Yb_numeric = step(sysB, t);
% % compare
% figure('Color','w'); hold on; grid on;
% plot(t, Yb_analytic, 'LineWidth',2);
% plot(t, Yb_numeric, '--', 'LineWidth',1.6);
% xlabel('Time (s)'); ylabel('y_b(t)');
% title('System b: unit-step (state-equation analytic) vs MATLAB step');
% legend('analytic','numeric','Location','southeast');
% text(0.02,0.02,nameStr,'Units','normalized','FontWeight','bold');
% fprintf('System b: max |analytic - numeric| = %.3e\n', max(abs(Yb_analytic-Yb_numeric)));
