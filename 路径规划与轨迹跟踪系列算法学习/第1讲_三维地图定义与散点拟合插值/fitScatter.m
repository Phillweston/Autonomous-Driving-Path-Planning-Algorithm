% ���ɢ��
% ���ߣ� Ally
% ���ڣ� 2021/07/03
clc
clear
close all

%% ����ɢ�������������ά·��
x_seq = [0,1,2,3,4];
y_seq = [5,2,3,4,1];
z_seq = [3,4,5,2,0];

% ����spline����������ϲ�ֵ
k = length(x_seq);
t_seq = linspace(0,1,k);
T_seq = linspace(0,1,100);
X_seq = spline(t_seq,x_seq,T_seq);
Y_seq = spline(t_seq,y_seq,T_seq);
Z_seq = spline(t_seq,z_seq,T_seq);

% ���������ͼ
figure
hold on
scatter3(x_seq, y_seq, z_seq, 100, 'bs','MarkerFaceColor','g')
plot3(X_seq, Y_seq, Z_seq, 'r','LineWidth',2)
grid on
title('ɢ���������')

%% �������ߵ����ʡ�����
% �������׵���
f = [X_seq; Y_seq; Z_seq];          % ��ʾ����
delta = 1 / length(X_seq);
f1 = gradient(f)./delta;            % һ�׵�
f2 = gradient(f1)./delta;           % ���׵�
f3 = gradient(f2)./delta;           % ���׵�
f1 = f1';
f2 = f2';
f3 = f3';

% ���ʡ�����
v = cross(f1,f2,2);                % һ�׵�����׵������
e = dot(f3,v,2);                   %��r',r'',r'''����ϻ�
c = zeros(length(T_seq),1);        % �������c����һ�׵����׵����ģ����d����һ�׵�ģ��
d = c;
for i = 1:length(f)
    c(i) = norm(v(i,:));             % һ�׵����׵������ģ��
    d(i) = norm(f1(i,:));            % һ�׵�ģ��
end
k = c./(d.^3);                     % ����
torsion = e./c.^2;                 % ����

%% ��ͼ
% ����ͼ
figure
plot(k, 'r','LineWidth',2)
title('����ͼ')

% ����ͼ
figure
plot(torsion, 'r','LineWidth',2)
title('����ͼ')