% Dijkstra�㷨
% ���ߣ�Ally
% ���ڣ�2020/12/18

clc
clear
close all

%% ͼ����
% ���ݽڵ���ڽ��ڵ����ĸ�ڵ�-���ֽڵ��Ӧ������ڵ�Ԫ������
nodes_dist = cell(0);
nodes_dist(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes_dist(2,:) = {2, [1, 3, 6], [12, 10, 7]};
nodes_dist(3,:) = {3, [2, 4, 5, 6], [10, 3, 5, 6]};
nodes_dist(4,:) = {4, [3, 5], [3, 4]};
nodes_dist(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes_dist(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes_dist(7,:) = {7, [1, 5, 6], [14, 8, 9]};

%% �㷨��ʼ��
% S/U�ĵ�һ�б�ʾ�ڵ���
% ����S���ڶ��б�ʾ��Դ�ڵ㵽���ڵ�����õ���С���룬���ٱ����
% ����U���ڶ��б�ʾ��Դ�ڵ㵽���ڵ���ʱ��õ���С���룬���ܻ���
S = [4, 0];
U(:,1) = [1, 2, 3, 5, 6, 7];
U(:,2) = [inf, inf, 3, 4, inf, inf];

% ����·������ʱ����·���ĳ�ʼ��
path_opt = cell(7,2);
path_opt(4,:) = {4, 4};

path_temp = cell(7,2);
path_temp(3,:) = {3, [4, 3]};
path_temp(4,:) = {4, 4};
path_temp(5,:) = {5, [4, 5]};

%% ѭ���������нڵ�
while ~isempty(U)
    
    % ��U�����ҳ���ǰ��С����ֵ����Ӧ�ڵ�,���Ƴ��ýڵ���S������
    [dist_min, idx] = min(U(:,2));
    node_min = U(idx, 1);
    S(end+1,:) = [node_min, dist_min];
    U(idx,:) = [];
    
    % ����С����ֵ�Ľڵ���ӵ�����·������
    path_opt(node_min,:) = path_temp(node_min,:);
    
    %% ���α�����С����ڵ���ڽڵ㣬�ж��Ƿ���U�����и����ڽڵ�ľ���ֵ
    for i = 1:length(nodes_dist{node_min, 2})
        
        % ��Ҫ�жϵĽڵ�
        node_temp = nodes_dist{node_min, 2}(i);
        
        % �ҳ�U�����нڵ�node_temp������ֵ
        idx_temp = find(node_temp == U(:,1));
        
        % �ж��Ƿ����
        if ~isempty(idx_temp)
            if dist_min + nodes_dist{node_min, 3}(i) < U(idx_temp, 2)
                U(idx_temp, 2) = dist_min + nodes_dist{node_min, 3}(i);
                
                % ������ʱ����·��
                path_temp{node_temp, 1} = node_temp;
                path_temp{node_temp, 2} = [path_opt{node_min, 2}, node_temp];                
            end
        end
    end
end
        
        
        
    
    





