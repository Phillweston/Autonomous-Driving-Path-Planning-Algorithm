% Dijkstra�㷨
% ���ߣ�Ally
% ���ڣ�2021/1/1
clc
clear
close all

%% �׶�-״̬����
stages = 5;
nodes_dist = cell(stages-1,3);

% ��1�׶�
nodes_dist{1,1} = 1;
nodes_dist{1,2} = [1,2,3];
nodes_dist{1,3} = [2,5,1];

% ��2�׶�
nodes_dist{2,1} = [1;2;3];
nodes_dist{2,2} = [1,2,3];
nodes_dist{2,3} = [12, 14, 10; 6, 10, 4; 13, 12, 11];

% ��3�׶�
nodes_dist{3,1} = [1;2;3];
nodes_dist{3,2} = [1,2];
nodes_dist{3,3} = [3, 9; 6, 5; 8, 10];

% ��4�׶�
nodes_dist{4,1} = [1;2];
nodes_dist{4,2} = 1;
nodes_dist{4,3} = [5; 2];

% ��4�׶�
nodes_dist{5,1} = 1;
nodes_dist{5,2} = 1;
nodes_dist{5,3} = 0;

% ����·���������ֵ����
path = cell(stages, 1);
dist = cell(stages, 1);
for i = 1:stages-1
    dist{i, 1} = nodes_dist{i,1};
    dist{i, 2} = inf(length(dist{i, 1}), 1);
    path{i, 1} = nodes_dist{i,1};
end
dist{stages, 1} = 1;  
dist{stages, 2} = 0;  
path{stages, 1} = 1;
path{stages, 2} = 1;
% �������һ���׶Σ�ֱ�ӳ�ʼ��

%% ����Ѱ��

% ��һ��ѭ�����������ÿһ���׶�
for i = stages-1:-1:1
    num_states_f = length(nodes_dist{i, 1});    

    % �ڶ���ѭ����������i�׶ε�ÿһ��״̬
    for j = 1:num_states_f
        num_states_r = length(nodes_dist{i+1, 1});        
        
        % ������ѭ����������i�׶εĵ�j��״̬����i+1�׶ε�ÿһ��·��
        for k = 1:num_states_r
            if  nodes_dist{i,3}(j,k) + dist{i+1,2}(k,1) < dist{i,2}(j,1)
                dist{i,2}(j,1) = nodes_dist{i,3}(j,k) + dist{i+1,2}(k,1);
                path{i, 2}(j,:) = [j, path{i+1, 2}(k,:)];
            end
        end
    end
end
            
%% �������
path_opt =  path(1,:);    
dist_opt =  dist{1,2};            
  