% B�������߷�
% ���ߣ�Ally
% ���ڣ�2021/2/6
clc
clear
close all

%% ���ݶ���
k = 4;                                    % k�ס�k-1��B����
flag = 2;                                  %1,2�ֱ���ƾ���B�������ߡ�׼����B��������
d = 3.5;
P=[0, 10, 25, 25, 40, 50;
    -d/2,-d/2,-d/2+0.5,d/2-0.5,d/2,d/2 ];   %n=5, 6�����Ƶ㣬����������������
n = size(P,2)-1;                          % n�ǿ��Ƶ��������0��ʼ����

%% ����B��������

path=[];
Bik = zeros(n+1, 1);

if flag == 1     % ����B����
    NodeVector = linspace(0, 1, n+k+1); %�ڵ�ʸ��
    for u = (k-1)/(n+k+1) : 0.001 : (n+2)/(n+k+1)
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path = [path; [p_u(1,1),p_u(2,1)]];
    end
    
elseif flag == 2  % ׼����B����
    NodeVector = U_quasi_uniform(n, k-1); % ׼����B�����Ľڵ�ʸ��
    for u = 0 : 0.005 : 1-0.005
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path=[path; [p_u(1),p_u(2)]];
    end
else
    fprintf('error!\n');
end

%% ��ͼ
d = 3.5;               % ��·��׼���
W = 1.8;               % �������
L = 4.7;               % ����
figure
len_line = 50;
P0 = [0, -d/2];

% ����ɫ·��ͼ
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  

% ���ֽ���
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %�ֽ���
plot([-5,len_line],[d,d],'w','linewidth',2);     %��߽���
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %��߽���

% ������������ʾ��Χ
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

% ����·��
scatter(path(:,1),path(:,2),100, '.b');%·����
scatter(P(1,:),P(2,:),'g')
plot(P(1,:),P(2,:),'r');%·����