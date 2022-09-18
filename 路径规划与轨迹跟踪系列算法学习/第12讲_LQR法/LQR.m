% LQR��
% ���ߣ�Ally
% ���ڣ�20210429
clc
clear
close all
load  path.mat

%% ��ز�������
dt = 0.1;
L = 2.9 ;
Q = 100*eye(3);
R = eye(2)* 2;

%% �켣����
% ����ο��켣
refPos_x = path(:,1);
refPos_y = path(:,2);
refPos = [refPos_x, refPos_y];

% ���㺽��Ǻ�����
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y) ;
diff_y(end+1) = diff_y(end);
derivative1 = gradient(refPos_y) ./ abs(diff_x);              % һ�׵���
derivative2 = del2(refPos_y) ./ abs(diff_x);                  % ���׵���
refHeading = atan2(diff_y , diff_x);                   % �����
refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % ��������

% ���ݰ�����ת��ԭ������ο�ǰ��ת��
refPos_Delta = atan(L*refK);

% �ο��ٶ�
refSpeed = 40/3.6;

%% ������
% ����ֵ
x = refPos_x(1)+0.5; 
y = refPos_y(1)+0.5; 
yaw = refHeading(1)+0.02;
v = 10;
Delta = 0;
idx = 1;

% �켣����ʵ����
pos_actual = [x,y];
v_actual  = v;
Delta_actual = Delta;
idx_actual = 1;
latError_LQR = [];

% ѭ��
while idx < length(refPos_x)-1
    % Ѱ�Ҳο��켣���Ŀ���
    idx = calc_target_index(x,y,refPos_x,refPos_y);  
    
    % LQR������
    [v_delta,delta,delta_r,latError] =  LQR_control(idx,x,y,v,yaw,refPos_x,refPos_y,refHeading,refPos_Delta,refK,L,Q,R,dt);    
    
    % ����������˳�ѭ��
    if abs(latError) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ����״̬
    [x,y,yaw,v,Delta] = update(x,y,yaw,v, v_delta,delta, dt,L, refSpeed,delta_r);
    
    % ����ÿһ����ʵ����
    pos_actual(end+1,:) = [x,y];
    v_actual(end+1,:)  = v;
    Delta_actual(end+1)  = Delta;
    idx_actual(end+1) = idx;
    latError_LQR(end+1,:) =  [idx,latError];
end

% ��ͼ
figure
plot(refPos_x,refPos_y,'r')
hold on
for i = 1:size(pos_actual,1)
    scatter(pos_actual(i,1), pos_actual(i,2),150,'b.')
    pause(0.01);
end

% ����
path_LQR = pos_actual;
save path_LQR.mat path_LQR
save latError_LQR.mat latError_LQR

%% Ѱ�Ҳο��켣���Ŀ���
function target_idx = calc_target_index(pos_x,pos_y, refPos_x,refPos_y)
i = 1:length(refPos_x)-1;
dist = sqrt((refPos_x(i)-pos_x).^2 + (refPos_y(i)-pos_y).^2);
[~, target_idx] = min(dist);
end


%% LQR����
function [v_delta,Delta_delta,delta_r,latError] =  LQR_control(idx,x,y,v,yaw,refPos_x,refPos_y,refPos_yaw,refPos_Delta,refPos_k,L,Q,R,dt)
% ��λ�á�����ǲο���
x_r = refPos_x(idx);
y_r = refPos_y(idx);
heading_r = refPos_yaw(idx);
delta_r = refPos_Delta(idx);

% ��λ�á�����ǵ����
x_error  = x - x_r;
y_error = y - y_r;
yaw_error =  yaw - heading_r;

% ���ݰٶ�Apolo������������
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% �����ֵ��ֵ��״̬��
X(1,1) = x_error; 
X(2,1) = y_error;  
X(3,1) = yaw_error;

% ��״̬���̾���ϵ��������K
A = [1,  0,  -v*dt*sin(heading_r);
     0,  1,  v * dt * cos(heading_r);
     0,  0,  1];
B = [dt * cos(heading_r),    0;
     dt * sin(heading_r),    0;
     dt * tan(heading_r)/L,  v*dt/(L * cos(delta_r)^2)];


K = calcu_K(A,B,Q,R);

% ����ٶ��������ǰ��ת�����������������
u = -K * X;  % 2��1��
v_delta = u(1);      
Delta_delta = u(2);

end


%% ��������
function K = calcu_K (A,B,Q,R)

% ��ֹ��������
iter_max = 500;
epsilon = 0.01;

% ѭ��
P_old = Q;
for i = 1:iter_max
    P_new = A' * P_old * A - (A' * P_old * B) / (R + B' * P_old * B) *( B' * P_old * A) +Q;
    if abs(P_new - P_old) <= epsilon
        break
    else
        P_old = P_new; 
    end
end

P = P_new;
K = (B' * P * B + R) \ (B' * P * A);  % 2��3��
end

%% ����״̬
function [x, y, yaw, v, Delta] = update(x, y, yaw, v, v_delta,Delta_delta,dt,L,refSpeed,refDelta)
Delta = refDelta + Delta_delta;
x = x + v * cos(yaw) * dt;
y = y + v * sin(yaw) * dt;
yaw = yaw + v / L * tan(Delta) * dt;
v = refSpeed + v_delta;
end
