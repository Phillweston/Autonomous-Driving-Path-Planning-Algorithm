% �˹��Ƴ���
% ���ߣ�Ally
% ���ڣ�2021/1/24
clc
clear
close all

%% ��ʼ�����Ĳ���
d = 3.5;               % ��·��׼���
W = 1.8;               % �������
L = 4.7;               % ����

P0 = [0,-d/2,1,1];     % ���������Ϣ��1-2��λ�ã�3-4���ٶ�
Pg = [99,d/2,0,0];     % Ŀ��λ��
Pobs = [15,7/4,0,0;
    30,-3/2,0,0;
    45,3/2,0,0;
    60,-3/4,0,0;
    80,7/4,0,0];       % �ϰ���λ��
P = [Pobs;Pg];         % ��Ŀ��λ�ú��ϰ���λ�úϷ���һ��

Eta_att = 5;           % ��������������ϵ��
Eta_rep_ob = 15;       % �������������ϵ��
Eta_rep_edge = 50;     % ����߽����������ϵ��

d0 = 20;               % �ϰ�Ӱ�����
n = size(P,1);         % �ϰ���Ŀ���ܼƸ���
len_step = 0.5;          % ����
Num_iter = 200;        % ���ѭ����������

%% ***************��ʼ����������ʼ����ѭ��******************
Pi = P0;               %��������ʼ���긳��Xi
i = 0;
while sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) > 1
    i = i + 1;
    Path(i,:) = Pi;    % ���泵�߹���ÿ���������
    
    %���㳵����ǰλ�����ϰ���ĵ�λ�����������ٶ�����
    for j = 1:n-1    
        delta(j,:) = Pi(1,1:2) - P(j,1:2);                              % �ó�����-�ϰ��������
        dist(j,1) = norm(delta(j,:));                                   % ������ǰλ�����ϰ���ľ���
        unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % �����ĵ�λ��������
    end
    
    %���㳵����ǰλ����Ŀ��ĵ�λ�����������ٶ�����
    delta(n,:) = P(n,1:2)-Pi(1,1:2);                                    %��Ŀ���-������������   
    dist(n,1) = norm(delta(n,:)); 
    unitVector(n,:)=[delta(n,1)/dist(n,1),delta(n,2)/dist(n,1)];

   %% ������� 
    % ��ԭ�����Ƴ���������Ŀ��������ӣ���������Ŀ����룩����ʹ��������Ŀ�������ҲΪ0
    for j = 1:n-1
        if dist(j,1) >= d0
            F_rep_ob(j,:) = [0,0];
        else
            % �ϰ���ĳ���1���������ϰ���ָ����
            F_rep_ob1_abs = Eta_rep_ob * (1/dist(j,1)-1/d0) * dist(n,1) / dist(j,1)^2;         
            F_rep_ob1 = [F_rep_ob1_abs*unitVector(j,1), F_rep_ob1_abs*unitVector(j,2)];   
           
            % �ϰ���ĳ���2�������ɳ���ָ��Ŀ���
            F_rep_ob2_abs = 0.5 * Eta_rep_ob * (1/dist(j,1) - 1/d0)^2;                
            F_rep_ob2 = [F_rep_ob2_abs * unitVector(n,1), F_rep_ob2_abs * unitVector(n,2)];  
            
            % �Ľ�����ϰ���ϳ�������
            F_rep_ob(j,:) = F_rep_ob1+F_rep_ob2;                                   
        end
    end
    
    
    % ���ӱ߽�����Ƴ������ݳ�����ǰλ�ã�ѡ���Ӧ�ĳ�������
    if Pi(1,2) > -d+W/2 && Pi(1,2) <= -d/2             %�µ�·�߽���������������ָ��y������
        F_rep_edge = [0,Eta_rep_edge * norm(Pi(:,3:4))*(exp(-d/2-Pi(1,2)))];
    elseif Pi(1,2) > -d/2 && Pi(1,2) <= -W/2           %�µ�·�ֽ�����������������ָ��y�Ḻ��
        F_rep_edge = [0,1/3 * Eta_rep_edge * Pi(1,2).^2];
    elseif Pi(1,2) > W/2  && Pi(1,2) < d/2             %�ϵ�·�ֽ�����������������ָ��y������ 
        F_rep_edge = [0, -1/3 * Eta_rep_edge * Pi(1,2).^2];
    elseif Pi(1,2) > d/2 && Pi(1,2)<=d-W/2             %�ϵ�·�߽���������������ָ��y�Ḻ��
        F_rep_edge = [0, Eta_rep_edge * norm(Pi(:,3:4)) * (exp(Pi(1,2)-d/2))];
    end
    
    %% ��������ͷ���
    F_rep = [sum(F_rep_ob(:,1))  + F_rep_edge(1,1),...
           sum(F_rep_ob(:,2)) + F_rep_edge(1,2)];                                      % �����ϰ���ĺϳ���ʸ��
    F_att = [Eta_att*dist(n,1)*unitVector(n,1), Eta_att*dist(n,1)*unitVector(n,2)];    % ����ʸ��
    F_sum = [F_rep(1,1)+F_att(1,1),F_rep(1,2)+F_att(1,2)];                             % �ܺ���ʸ��
    UnitVec_Fsum(i,:) = 1/norm(F_sum) * F_sum;                                         % �ܺ����ĵ�λ����
    
    %���㳵����һ��λ��
    Pi(1,1:2)=Pi(1,1:2)+len_step*UnitVec_Fsum(i,:);                     

%     %�ж��Ƿ񵽴��յ�
%     if sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) < 0.2 
%         break
%     end
end
Path(i,:)=P(n,:);            %��·�����������һ���㸳ֵΪĿ��

%% ��ͼ
figure
len_line = 100;



% ����ɫ·��ͼ
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  %2�ų�

% ���ֽ���
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %�ֽ���
plot([-5,len_line],[d,d],'w','linewidth',2);     %��߽���
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %��߽���

% ������������ʾ��Χ
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 


% ����·��
plot(P(1:n-1,1),P(1:n-1,2),'ro');   %�ϰ���λ��
plot(P(n,1),P(n,2),'gv');       %Ŀ��λ��
plot(P0(1,1),P0(1,2),'bs');    %���λ��
plot(Path(:,1),Path(:,2),'.b');%·����

