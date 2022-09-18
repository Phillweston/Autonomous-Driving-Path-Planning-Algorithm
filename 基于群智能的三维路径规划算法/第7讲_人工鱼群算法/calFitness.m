function [fitness,path] = calFitness(startPos, goalPos,pos)
% ���������������ɢ��
x_seq=[startPos(1), pos.x, goalPos(1)];
y_seq=[startPos(2), pos.y, goalPos(2)];
z_seq=[startPos(3), pos.z, goalPos(3)];

k = length(x_seq);
i_seq = linspace(0,1,k);
I_seq = linspace(0,1,100);
X_seq = spline(i_seq,x_seq,I_seq);
Y_seq = spline(i_seq,y_seq,I_seq);
Z_seq = spline(i_seq,z_seq,I_seq);
path = [X_seq', Y_seq', Z_seq'];



%% �������������õ�����ɢ���·�����ȣ���Ӧ�ȣ�
dx = diff(X_seq);
dy = diff(Y_seq);
dz = diff(Z_seq);
fitness = sum(sqrt(dx.^2 + dy.^2 + dz.^2));