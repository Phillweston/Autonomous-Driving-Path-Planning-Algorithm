% Stanley��
% ���ߣ�Ally
% ���ڣ�20210429
clc
clear
close all
load  path.mat

%% ��ز�������
% �ο��켣
RefPos = path;            

% ����켣�Ĳο������
diff_x = diff(RefPos(:,1)) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(RefPos(:,2)) ;
diff_y(end+1) = diff_y(end);
RefHeading = atan2(diff_y ,diff_x);

% ������������
targetSpeed = 10;           % Ŀ���ٶȣ���λ�� m /s
InitialState = [RefPos(1,:)-0.5,RefHeading(1)+0.02,0];  % ����λ�á�����λ�á�����ǡ��ٶ�
k = 5;                      % �������
Kp = 1;                     % �ٶ�P������ϵ��
dt = 0.1;                   % ʱ��������λ��s
L = 2;                      % ������࣬��λ��m

%% ������
% ������ʼ״̬����
state = InitialState;
state_actual = state;
idx = 1;
latError_Stanley = [];

% ѭ��
while idx < size(RefPos,1)-1
    % Ѱ��Ԥ����뷶Χ�����·����
    idx = findTargetIdx(state,RefPos);
       
    % ����ǰ��ת��
    [delta,latError] = stanley_control(idx,state,RefPos,RefHeading,k);
    
    % ����������˳�ѭ��
    if abs(latError) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ������ٶ�
    a = Kp* (targetSpeed-state(4));
    
    % ����״̬��
    state_new = UpdateState(a,state,delta,dt,L);
    state = state_new;
    
    % ����ÿһ����ʵ����
    state_actual(end+1,:) = state_new;
    latError_Stanley(end+1,:) =  [idx,latError];
end

% ��ͼ
figure
plot(RefPos(:,1), RefPos(:,2), 'r');
xlabel('�������� / m');
ylabel('�������� / m');
hold on
for i = 1:size(state_actual,1)
    scatter(state_actual(i,1), state_actual(i,2),150, 'b.');
    pause(0.01)
end
legend('�滮�����켣', 'ʵ����ʻ�켣')

%  ����
path_stanley = state_actual(:,1:2);
save path_stanley.mat path_stanley;
save latError_Stanley.mat latError_Stanley
%% �����ڲο��켣�������뵱ǰλ������ĵ�
function target_idx = findTargetIdx(state,RefPos)
for i = 1:size(RefPos,1)
    d(i,1) = norm(RefPos(i,:) - state(1:2));
end
[~,target_idx] = min(d);  % �ҵ����뵱ǰλ�������һ���ο��켣������
end

%% ��ÿ�����
function [delta,latError] = stanley_control(idx,state,RefPos,RefHeading,k)

% ���ݰٶ�Apolo������������
dx = state(1) - RefPos(idx,1);
dy = state(2) - RefPos(idx,2);
phi_r = RefHeading(idx);
latError = dy*cos(phi_r) - dx*sin(phi_r);

% �ֱ����ֻ���Ǻ�������theta��ֻ���Ǻ�������theta
theta_fai =  RefHeading(idx)- state(3);
theta_y = atan2(-k*latError,state(4));

% �������ǶȺϲ���Ϊǰ��ת��
delta = theta_fai + theta_y;
end

%% ����״̬��
function state_new = UpdateState(a,state_old,delta,dt,L)
state_new(1) =  state_old(1) + state_old(4)*cos(state_old(3))*dt; %��������
state_new(2) =  state_old(2) + state_old(4)*sin(state_old(3))*dt; %��������
state_new(3) =  state_old(3) + state_old(4)*dt*tan(delta)/L;      %�����
state_new(4) =  state_old(4) + a*dt;                              %�����ٶ�
end
