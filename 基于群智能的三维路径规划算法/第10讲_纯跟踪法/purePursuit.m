% �����٣�Pure Pursuit����
% ���ߣ�Ally
% ���ڣ�20210429
clc
clear
close all
load  path.mat

%% ��ز�������
RefPos = path;
targetSpeed = 10;      % m/s
Kv = 0.1;              % ǰ�Ӿ���ϵ��
Kp = 0.8;              % �ٶ�P������ϵ��
Ld0 = 2;               % Ld0��Ԥ����������ֵ
dt = 0.1;              % ʱ��������λ��s
L = 2.9;               % ������࣬��λ��m

% ����ο������
diff_x = diff(RefPos(:,1)) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(RefPos(:,2)) ;
diff_y(end+1) = diff_y(end);
refHeading = atan2(diff_y , diff_x);                   % �����

%% ������

% ������ʼ״̬����
pos = RefPos(1,:)+1;
v = 0;
heading = 0.02;
 
% ����ʼ״̬����ʵ��״̬������
pos_actual = pos;
heading_actual = heading;
v_actual = v;
idx = 1;
latError_PP = [];

% ѭ�������켣��
while idx < size(RefPos,1)-1
    % Ѱ��Ԥ����뷶Χ�����·����
    [lookaheadPoint,idx] = findLookaheadPoint(pos, v, RefPos,Kv, Ld0);
   
    % ���������
    [delta,latError]  = pure_pursuit_control(lookaheadPoint,idx,pos, heading, v, RefPos,refHeading, Kv, Ld0,L);
    
    % ����������˳�ѭ��
    if abs(latError) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ������ٶ�
    a = Kp* (targetSpeed-v)/dt;
    
    % ����״̬��
    [pos, heading, v] = updateState(a,pos, heading, v,delta,L, dt);
    
    % ����ÿһ����ʵ����
    pos_actual(end+1,:) = pos;
    heading_actual(end+1,:) = heading;
    v_actual(end+1,:) = v;
    latError_PP(end+1,:) = [idx,latError];
end

% ��ͼ
figure
plot(RefPos(:,1), RefPos(:,2), 'b');
xlabel('�������� / m');
ylabel('�������� / m');
hold on 
for i = 1:size(pos_actual,1)
    scatter(pos_actual(i,1), pos_actual(i,2),150, '.r');
    pause(0.01)
end
legend('�滮�����켣', 'ʵ����ʻ�켣')

% ����
path_PP = pos_actual;
save path_PP.mat path_PP
save latError_PP.mat latError_PP

%% �����ڲο��켣�������뵱ǰ����λ������ĵ�
function  [lookaheadPoint,idx_target] = findLookaheadPoint(pos, v, RefPos, Kv, Ld0)

% �ҵ����뵱ǰλ�������һ���ο��켣������
sizeOfRefPos = size(RefPos,1);
for i = 1:sizeOfRefPos
    dist(i,1) = norm(RefPos(i,:) - pos);   
end
[~,idx] = min(dist); 


% �Ӹõ㿪ʼ��켣ǰ���������ҵ���Ԥ������������һ���켣��
L_steps = 0;           % �ο��켣�ϼ������ڵ���ۼƾ���
Ld = Kv*v + Ld0;       % Ld0��Ԥ����������ֵ��
while L_steps < Ld && idx < sizeOfRefPos
    L_steps = L_steps + norm(RefPos(idx + 1,:) - RefPos(idx,:));
    idx = idx+1;
end
idx_target = idx;
lookaheadPoint = RefPos(idx,:);
end


%% ��ÿ�������ǰ��ת��
function [delta,latError] = pure_pursuit_control(lookaheadPoint,idx,pos, heading, v,RefPos,refHeading, Kv, Ld0, L)
sizeOfRefPos = size(RefPos,1);
if idx < sizeOfRefPos
    Point_temp = lookaheadPoint;
else
    Point_temp = RefPos(end,1:2);
end
alpha = atan2(Point_temp(1,2) - pos(2), Point_temp(1,1) - pos(1))  - heading;
Ld = Kv*v + Ld0;

% ��λ�á�����ǵ����
x_error  = pos(1) - RefPos(idx,1);
y_error = pos(2) - RefPos(idx,2);
heading_r = refHeading(idx);
% ���ݰٶ�Apolo������������
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% ǰ��ת��
delta = atan2(2*L*sin(alpha), Ld);

end

%% ����״̬��
function [pos_new, heading_new, v_new] = updateState(a,pos_old, heading_old, v_old,delta,wheelbase, dt)
pos_new(1) = pos_old(1) + v_old*cos(heading_old)*dt;
pos_new(2) =  pos_old(2) + v_old*sin(heading_old)*dt;
heading_new=  heading_old + v_old*dt*tan(delta)/wheelbase;
v_new =  v_old + a*dt;
end

