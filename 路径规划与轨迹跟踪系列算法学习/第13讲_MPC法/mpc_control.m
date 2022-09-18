function    [Delta_real,v_real,idx,latError,U ] = ...
    mpc_control(x,y,yaw,refPos_x,refPos_y,refPos_yaw,refDelta,dt,L,U,target_v)
%% MPCԤ�����
Nx = 3;         % ״̬���ĸ���
Nu = 2;         % �������ĸ���
Np = 60;        % Ԥ�ⲽ��
Nc = 30;        % ���Ʋ���
row = 10;       % �ɳ�����
Q = 100*eye(Np*Nx);      % (Np*Nx) �� (Np*Nx)
R = 1*eye(Nc*Nu);        % (Nc*Nu) �� (Nc*Nu)

% ������Լ������
umin = [-0.2; -0.54];
umax = [0.2; 0.332];
delta_umin = [-0.05; -0.64];
delta_umax = [0.05; 0.64];

%% ԭ�˶�ѧ���״̬�ռ䷽�̵���ؾ���
% ����ο�������
idx = calc_target_index(x,y,refPos_x,refPos_y); 
v_r = target_v;
Delta_r = refDelta(idx);
heading_r = refPos_yaw(idx);

% ʵ��״̬����ο�״̬��
X_real = [x,y,yaw];
Xr = [refPos_x(idx), refPos_y(idx), refPos_yaw(idx)];

% ��λ�á�����ǵ����
x_error  = x - refPos_x(idx);
y_error = y - refPos_y(idx);

% ���ݰٶ�Apolo������������
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% a,b��������
a = [1    0   -v_r*sin(heading_r)*dt;
     0    1   v_r*cos(heading_r)*dt;
     0    0   1];
b = [cos(heading_r)*dt     0;
     sin(heading_r)*dt     0;
     tan(heading_r)*dt/L   v_r*dt/(L * (cos(Delta_r)^2))];

%% �µ�״̬�ռ䷽�̵���ؾ���
% �µ�״̬��
kesi = zeros(Nx+Nu,1);              % (Nx+Nu) �� 1
kesi(1:Nx) = X_real - Xr;
kesi(Nx+1:end) = U;

% �µ�A����
A_cell = cell(2,2);
A_cell{1,1} = a;
A_cell{1,2} = b;
A_cell{2,1} = zeros(Nu,Nx);
A_cell{2,2} = eye(Nu);
A = cell2mat(A_cell);           % (Nx+Nu) �� (Nx+Nu)

% �µ�B����
B_cell = cell(2,1);
B_cell{1,1} = b;
B_cell{2,1} = eye(Nu);
B = cell2mat(B_cell);           % (Nx+Nu) �� Nu

% �µ�C����
C = [eye(Nx), zeros(Nx, Nu)];   % Nx �� (Nx+Nu)

% PHI����
PHI_cell = cell(Np,1);
for i = 1:Np
    PHI_cell{i,1}=C*A^i;  % Nx �� (Nx+Nu)
end
PHI = cell2mat(PHI_cell);   % (Nx * Np) �� (Nx + Nu)


% THETA����
THETA_cell = cell(Np,Nc);
for i = 1:Np
    for j = 1:Nc
        if j <= i
            THETA_cell{i,j} = C*A^(i-j)*B;    % Nx �� Nu
        else
            THETA_cell{i,j} = zeros(Nx,Nu);
        end
    end
end
THETA = cell2mat(THETA_cell);                 % (Nx * Np) �� (Nu * Nc)


%% ������Ŀ�꺯������ؾ���

% H����
H_cell = cell(2,2);
H_cell{1,1} = THETA'*Q*THETA + R;  % (Nu * Nc) �� (Nu * Nc)
H_cell{1,2} = zeros(Nu*Nc,1);
H_cell{2,1} = zeros(1,Nu*Nc);
H_cell{2,2} = row;
H = cell2mat(H_cell);            % (Nu * Nc + 1) �� (Nu * Nc + 1)

% E����
E = PHI*kesi;                    % (Nx * Np) �� 1

% g����
g_cell = cell(1,1);
g_cell{1,1} = E'*Q*THETA;          % (Nu * Nc ) �� 1������Ϊ�˺�H������ƥ�䣬�����һ��0
g_cell{1,2} = 0;
g = cell2mat(g_cell);              % (Nu * Nc + 1 ) �� 1

%% Լ����������ؾ���

% A_I����
A_t = zeros(Nc,Nc);     % �����Ƿ���
for i = 1:Nc
    A_t(i,1:i) = 1;
end
A_I = kron(A_t,eye(Nu));       % (Nu * Nc) �� (Nu * Nc)

% Ut����
Ut = kron(ones(Nc,1),U);       % (Nu * Nc) �� 1

% ��������������仯����Լ��
Umin = kron(ones(Nc,1),umin);
Umax = kron(ones(Nc,1),umax);
delta_Umin = kron(ones(Nc,1),delta_umin);
delta_Umax = kron(ones(Nc,1),delta_umax);

% ����quadprog��������ʽԼ��Ax <= b�ľ���A
A_cons_cell = {A_I, zeros(Nu*Nc,1);       % ����Ϊ�˺�H������ƥ�䣬�����һ��0��(Nu * Nc) �� (Nu * Nc), (Nu * Nc) ��1
    -A_I, zeros(Nu*Nc,1)}; 
A_cons = cell2mat(A_cons_cell);           % (Nu * Nc * 2) �� (Nu * Nc +1)

% ����quadprog��������ʽԼ��Ax <= b������b
b_cons_cell = {Umax-Ut;
    -Umin+Ut};
b_cons = cell2mat(b_cons_cell);

% ��U�����½�Լ��
lb = delta_Umin;
ub = delta_Umax;

%% ��ʼ������

options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
delta_U = quadprog(H,g,A_cons,b_cons,[],[],lb,ub,[],options);   %(Nu * Nc +1) �� 1

%% �������

% ֻѡȡ����delta_U�ĵ�һ���������ע�⣺������v_tilde�ı仯����Delta_tilde�ı仯��
delta_v_tilde = delta_U(1);
delta_Delta_tilde = delta_U(2);

% ������һʱ�̵Ŀ�������ע�⣬����ġ�����������v_tilde��Delta_tilde��������������v��Delta
U(1) = kesi(4) + delta_v_tilde; 
U(2) = kesi(5) + delta_Delta_tilde;  

% ��������Ŀ�����v_real��Delta_real
v_real = U(1) + v_r; 
Delta_real = U(2) + Delta_r;

end