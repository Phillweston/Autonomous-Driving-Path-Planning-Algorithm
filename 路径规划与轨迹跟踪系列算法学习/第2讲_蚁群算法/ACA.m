% Dijkstra�㷨
% ���ߣ�Ally
% ���ڣ�2020/12/26

clc
clear
close all


%% ��ʼ������
% ���ݽڵ���ڽ��ڵ����ĸ�ڵ�-���ֽڵ��Ӧ������ڵ�Ԫ������
nodes_data = cell(0);
nodes_data(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes_data(2,:) = {2, [1, 3, 6], [12, 10, 7]};
nodes_data(3,:) = {3, [2, 4, 5, 6], [10, 3, 5, 6]};
nodes_data(4,:) = {4, [3, 5], [3, 4]};
nodes_data(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes_data(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes_data(7,:) = {7, [1, 5, 6], [14, 8, 9]};

% ʼĩ�ڵ�
node_start = 4;                       % ��ʼԴ�ڵ�
node_end = 1;                         % �սڵ�

% ��Ⱥ��ض���
m = 50;                              % ��������
n = size(nodes_data,1);              % �ڵ�����
alpha = 1;                           % ��Ϣ����Ҫ�̶�����
beta = 5;                            % ����������Ҫ�̶�����
rho = 0.1;                           % ��Ϣ�ػӷ�����
Q = 1;                               % ����

% ���������У���ز�����ʼ������
iter = 1;                            % ����������ֵ
iter_max = 100;                      % ���������� 
Route_best = cell(iter_max,1);       % �������·��       
Length_best = zeros(iter_max,1);     % �������·���ĳ���  
Length_ave = zeros(iter_max,1);      % ����·����ƽ������

% ����Ϣ�ء��ӷ�����һ������nodes_data��
Delta_Tau_initial = nodes_data(:,1:2);
for i = 1:size(nodes_data,1)
    nodes_data{i,4} = ones(1, length(nodes_data{i,3}));   % ��Ϣ��
    nodes_data{i,5} = 1./nodes_data{i,3};                 % �ӷ�����
    Delta_Tau_initial{i,3} = zeros(1, length(nodes_data{i,3}));
end


%% ����Ѱ�����·��
while iter <= iter_max  
  
    route = cell(0);
    
    %%  �������·��ѡ��
    for i = 1:m
        % ����ڵ�·��ѡ��
        neighbor = cell(0);
        node_step = node_start;
        path = node_step;
        dist = 0;
        while ~ismember(node_end, path) %��·��������������սڵ�ʱ�����������·��Ѱ�ţ�����ѭ��
           
            % Ѱ���ڽ��ڵ�           
            neighbor = nodes_data{node_step,2};
            
            % ɾ���Ѿ����ʹ����ٽ��ڵ�
            idx = [];
            for k = 1:length(neighbor)
                if ismember(neighbor(k), path)
                    idx(end+1) =  k;
                end
            end
            neighbor(idx) = [];
            
            % �ж��Ƿ��������ͬ�� ���ǣ�ֱ�ӷ��ص���㣬����Ѱ·
            if isempty(neighbor)
                neighbor = cell(0);
                node_step = node_start;
                path = node_step;
                dist = 0;
                continue
            end
                
            
            %������һ���ڵ�ķ��ʸ���
            P = neighbor;
            for k=1:length(P)
                P(2,k) = nodes_data{node_step, 4}(k)^alpha * ...
                    nodes_data{node_step, 5}(k)^beta;
            end
            P(2,:) = P(2,:)/sum(P(2,:));
            
            % ���̶ķ�ѡ����һ�����ʽڵ�
            Pc = cumsum(P(2,:));
            Pc = [0, Pc];
            randnum = rand;
            for k = 1:length(Pc)-1
                if randnum > Pc(k) && randnum < Pc(k+1)
                    target_node = neighbor(k);
                end
            end
            
            
            % ���㵥������
            idx_temp = find(nodes_data{node_step, 2} == target_node);
            dist = dist + nodes_data{node_step, 3}(idx_temp);
            
            % ������һ����Ŀ��ڵ㼰·������            
            node_step = target_node;
            path(end+1) = node_step;         
                       
        end
        
        % ��ŵ�iֻ���ϵ��ۼƾ��뼰��Ӧ·��
        Length(i,1) = dist;
        route{i,1} = path;
    end
    
    %% ������һ����mֻ��������̾��뼰��Ӧ·��
    if iter == 1
        [min_Length,min_index] = min(Length);
        Length_best(iter) = min_Length;
        Length_ave(iter) = mean(Length);
        Route_best{iter,1} = route{min_index,1};
        
    else
        [min_Length,min_index] = min(Length);
        Length_best(iter) = min(Length_best(iter - 1),min_Length);
        Length_ave(iter) = mean(Length);
        if Length_best(iter) == min_Length
            Route_best{iter,1} = route{min_index,1};
        else
            Route_best{iter,1} = Route_best{iter-1,1};
        end
    end
    
    %% ������Ϣ��
    
    % ����ÿһ��·���ϵľ������������µ���Ϣ��
    Delta_Tau = Delta_Tau_initial;    
    
    % ������ϼ���
    for i = 1:m
       
        % ����ڵ�����
        for j = 1:length(route{i,1})-1
            node_start_temp = route{i,1}(j);
            node_end_temp = route{i,1}(j+1);
            idx =  find(Delta_Tau{node_start_temp, 2} == node_end_temp);
            Delta_Tau{node_start_temp,3}(idx) = Delta_Tau{node_start_temp,3}(idx) + Q/Length(i);
        end
        
    end
    
    % ���ǻӷ����ӣ�������Ϣ��
    for i = 1:size(nodes_data, 1)
        nodes_data{i, 4} = (1-rho) * nodes_data{i, 4} + Delta_Tau{i, 3};
    end
    
    % ���µ�������
    iter = iter + 1;
end


%% ��ͼ�����    

figure
plot(1:iter_max,Length_best,'b',1:iter_max,Length_ave,'r')
legend('��̾���','ƽ������')
xlabel('��������')
ylabel('����')
title('������̾�����ƽ������Ա�')

% ����·��
[dist_min, idx] = min(Length_best);
path_opt = Route_best{idx,1};
