clc
clear 
close all

%% State Space Model
% A is state matrix
A = [0 1;0 0];B = [0; 1];C = [1 0];D = 0;

%% Observed State FeedBack Gain
K = acker(A, B, [-1 -1]);
L = acker(A', C', [-2 -2])';
h = ss([A -B*K;L*C A-B*K-L*C],[B;0;0],[C 0 0], 0);
t = 0:0.01:10;
ref=zeros(size(t));
[ys, ts, xs] = lsim(h, ref, t, [1;0.5;0;0]);

%% Output
subplot(2,1,1)
plot(ts, xs(:, 1),'-b',ts, xs(:,3),'--b','LineWidth',2);
axis([0 8 -0.5 1.5])
set(gca, 'GridLineStyle', '-', 'FontName', 'times', 'FontSize', 10)
xlabel('Time(sec)');
ylabel('State')
grid on

subplot(2, 1, 2)
plot(ts, xs(:,2),'-b',ts,xs(:,4), '--b', 'LineWidth', 2);
axis([0 8 -1. 1.])
set(gca, 'GridLineStyle', '-', 'FontName', 'times', 'FontSize', 10)
xlabel('Time(sec)');
ylabel('State')
grid on