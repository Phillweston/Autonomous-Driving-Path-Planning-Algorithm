% �Ƚ����ֹ켣�����㷨�ĺ������
% ����:Ally
% ����:2021/04/29
clc
clear
close all
load latError_PP.mat
load latError_Stanley.mat
load latError_LQR.mat
load latError_MPC.mat

%% ��ͼ
figure
plot(latError_PP(:,1),latError_PP(:,2),'b')
hold on
plot(latError_Stanley(:,1),latError_Stanley(:,2),'k')
plot(latError_LQR(:,1),latError_LQR(:,2),'g')
plot(latError_MPC(:,1),latError_MPC(:,2),'r')
legend('PP','Stanley','LQR','MPC')
xlabel('�ο��켣������')
ylabel('�������')

