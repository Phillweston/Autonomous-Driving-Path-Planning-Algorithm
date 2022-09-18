% A*�㷨
% ���ߣ�Ally
% ���ڣ�2021/1/9
clc
clear
close all

%% ����ͼ

% դ���ͼ����������������
m = 5;
n = 7;
start_node = [2, 3];
target_node = [6, 3];
obs = [4,2; 4,3; 4,4];

for i = 1:m
    plot([0,n], [i, i], 'k');
    hold on
end
    
for j = 1:n
     plot([j, j], [0, m], 'k');
end

axis equal
xlim([0, n]);
ylim([0, m]);   

% �����ϰ����ֹ����ɫ��
fill([start_node(1)-1, start_node(1), start_node(1), start_node(1)-1],...
    [start_node(2)-1, start_node(2)-1 , start_node(2), start_node(2)], 'g');

fill([target_node(1)-1, target_node(1), target_node(1), target_node(1)-1],...
    [target_node(2)-1, target_node(2)-1 , target_node(2), target_node(2)], 'r');

for i = 1:size(obs,1)
    temp = obs(i,:);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
end

%% Ԥ����

% ��ʼ��closeList
closeList = start_node;
closeList_path = {start_node,start_node};
closeList_cost = 0;
child_nodes = child_nodes_cal(start_node,  m, n, obs, closeList);  

% ��ʼ��openList
openList = child_nodes;
for i = 1:size(openList,1)
    openList_path{i,1} = openList(i,:);
    openList_path{i,2} = [start_node;openList(i,:)];
end

for i = 1:size(openList, 1)
    g = norm(start_node - openList(i,1:2));
    h = abs(target_node(1) - openList(i,1)) + abs(target_node(2) - openList(i,2));
    f = g + h;
    openList_cost(i,:) = [g, h, f];
end

%% ��ʼ����
% ��openList��ʼ�����ƶ�������С�Ľڵ�
[~, min_idx] = min(openList_cost(:,3));
parent_node = openList(min_idx,:);


%% ����ѭ��
flag = 1;
while flag   
    
    % �ҳ����ڵ�ĺ���closeList���ӽڵ�
    child_nodes = child_nodes_cal(parent_node,  m, n, obs, closeList); 
    
    % �ж���Щ�ӽڵ��Ƿ���openList�У����ڣ���Ƚϸ��£�û����׷�ӵ�openList��
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        [in_flag,openList_idx] = ismember(child_node, openList, 'rows');
        g = openList_cost(min_idx, 1) + norm(parent_node - child_node);
        h = abs(child_node(1) - target_node(1)) + abs(child_node(2) - target_node(2));
        f = g+h;
        
        if in_flag   % ���ڣ��Ƚϸ���g��f        
            if g < openList_cost(openList_idx,1)
                openList_cost(openList_idx, 1) = g;
                openList_cost(openList_idx, 3) = f;
                openList_path{openList_idx,2} = [openList_path{min_idx,2}; child_node];
            end
        else         % �����ڣ�׷�ӵ�openList
            openList(end+1,:) = child_node;
            openList_cost(end+1, :) = [g, h, f];
            openList_path{end+1, 1} = child_node;
            openList_path{end, 2} = [openList_path{min_idx,2}; child_node];
        end
    end
   
    
    % ��openList�Ƴ��ƶ�������С�Ľڵ㵽 closeList
    closeList(end+1,: ) =  openList(min_idx,:);
    closeList_cost(end+1,1) =   openList_cost(min_idx,3);
    closeList_path(end+1,:) = openList_path(min_idx,:);
    openList(min_idx,:) = [];
    openList_cost(min_idx,:) = [];
    openList_path(min_idx,:) = [];
 
    % ������������openList�����ƶ�������С�Ľڵ�
    [~, min_idx] = min(openList_cost(:,3));
    parent_node = openList(min_idx,:);
    
    % �ж��Ƿ��������յ�
    if parent_node == target_node
        closeList(end+1,: ) =  openList(min_idx,:);
        closeList_cost(end+1,1) =   openList_cost(min_idx,1);
        closeList_path(end+1,:) = openList_path(min_idx,:);
        flag = 0;
    end
end
    

%% ��·��
path_opt = closeList_path{end,2};
path_opt(:,1) = path_opt(:,1)-0.5;
path_opt(:,2) = path_opt(:,2)-0.5;
scatter(path_opt(:,1), path_opt(:,2), 'k');
plot(path_opt(:,1), path_opt(:,2), 'k');
  