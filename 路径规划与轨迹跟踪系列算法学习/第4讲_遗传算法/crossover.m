function childPop = crossover(parentPop,p_crs)

% ��ȡ������Ⱥ����Ⱦɫ�峤��
m = size(parentPop,1);
n = length(parentPop(1).pos.x);

% ��parentPop��ֵ��childPop���Գ�ʼ���Ӵ���Ⱥ
childPop = parentPop;

% �������
for i = 1:2:m-1
    if rand < p_crs
        idx = round(rand*n);
        childPop(i).pos.x = [parentPop(i).pos.x(1:idx), parentPop(i+1).pos.x(idx+1:n)];
        childPop(i+1).pos.x = [parentPop(i+1).pos.x(1:idx), parentPop(i).pos.x(idx+1:n)];
        childPop(i).pos.y = [parentPop(i).pos.y(1:idx), parentPop(i+1).pos.y(idx+1:n)];
        childPop(i+1).pos.y = [parentPop(i+1).pos.y(1:idx), parentPop(i).pos.y(idx+1:n)];
        childPop(i).pos.z = [parentPop(i).pos.z(1:idx), parentPop(i+1).pos.z(idx+1:n)];
    end
end

% �����п��Ƶ㰴��x/y/z���������������
for i = 1:m
    childPop(i).pos.x = sort(childPop(i).pos.x);
    childPop(i).pos.y = sort(childPop(i).pos.y);
    childPop(i).pos.z = sort(childPop(i).pos.z);
end
