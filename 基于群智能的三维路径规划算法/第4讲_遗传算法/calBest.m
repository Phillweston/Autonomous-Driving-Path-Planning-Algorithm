function [pop,GlobalBest] = calBest(pop,GlobalBest)
for i = 1:size(pop,1)
    % ���¸��������
    if pop(i).fitness < pop(i).Best.fitness
        pop(i).Best.pos = pop(i).pos;
        pop(i).Best.fitness = pop(i).fitness;
        pop(i).Best.path = pop(i).path;
    end
    
    % ����ȫ������
    if pop(i).Best.fitness < GlobalBest.fitness
        GlobalBest = pop(i).Best;
    end
end
