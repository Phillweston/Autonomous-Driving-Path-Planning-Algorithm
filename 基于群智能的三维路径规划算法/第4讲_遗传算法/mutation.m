function childPop = mutation(childPop,p_mut,posBound)
% ��ȡ������Ⱥ����Ⱦɫ�峤��
m = size(childPop,1);
n = length(childPop(1).pos.x);
for i = 1:1:m
    if rand < p_mut
        idx = round(rand*n);
        
        % ����Խ��
        if idx <= 1
            idx = 2;
        end
        if idx == n
            idx = n-1;
        end
        
        % ���죺������滻
        childPop(i).pos.x(idx) = rand*(posBound(1,2)-posBound(1,1)) + posBound(1,1);
        childPop(i).pos.y(idx) = rand*(posBound(2,2)-posBound(2,1)) + posBound(2,1);
        childPop(i).pos.z(idx) = rand*(posBound(3,2)-posBound(3,1)) + posBound(3,1);
    end
end

% �����п��Ƶ㰴��x/y/z���������������
for i = 1:m
    childPop(i).pos.x = sort(childPop(i).pos.x);
    childPop(i).pos.y = sort(childPop(i).pos.y);
    childPop(i).pos.z = sort(childPop(i).pos.z);
end
