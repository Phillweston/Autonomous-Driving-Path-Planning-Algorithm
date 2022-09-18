function pop = initPop(popNum,chromLength,posBound)
pop = struct;

% ��һ���ĸ����ʼ��
for i = 1:popNum
    % ��ʼ��
    pop(i).pos= [];
    pop(i).fitness = [];
    pop(i).path = [];
    pop(i).Best.pos = [];
    pop(i).Best.fitness = inf;
    pop(i).Best.path = [];
    
    % ������ɳ�ʼ���Ƶ㣨Ⱦɫ�壩
    pop(i).pos.x = (posBound(1,2)-posBound(1,1)) * rand(1,chromLength) + posBound(1,1);
    pop(i).pos.y = (posBound(2,2)-posBound(2,1)) * rand(1,chromLength) + posBound(2,1);
    pop(i).pos.z = (posBound(3,2)-posBound(3,1)) * rand(1,chromLength) + posBound(3,1);
end

% �����п��Ƶ㰴��x/y/z���������������
for i = 1:popNum
    pop(i).pos.x = sort(pop(i).pos.x);
    pop(i).pos.y = sort(pop(i).pos.y);
    pop(i).pos.z = sort(pop(i).pos.z);
end