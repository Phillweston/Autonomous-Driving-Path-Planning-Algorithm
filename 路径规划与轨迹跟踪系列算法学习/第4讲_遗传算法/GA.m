% ��4�����Ŵ��㷨
% ���ߣ� Ally
% ���ڣ� 2021/07/25
clc
clear
close all

%% ��ά·���滮ģ��
startPos = [1, 1, 1];
goalPos = [100, 100, 80];

% ����ɽ���ͼ
posBound = [0,100; 0,100; 0,100;];

% ��ͼ�������߷�Χ
[X,Y,Z] = defMap(posBound);

%% ���ó�����
chromLength = 5;     % Ⱦɫ�峤�ȣ�����·�ߵĿ��Ƶ�����δ����ĩ����
p_select = 0.5;      % ѡ�����
p_crs = 0.8;         % �������
p_mut = 0.2;         % �������
popNum = 50;         % ��Ⱥ��ģ
iterMax = 100;       % ��������

%% ��Ⱥ��ʼ��
% ������ʼ��Ⱥ   
pop = initPop(popNum,chromLength,posBound);

% ������Ⱥ��Ӧ��
pop = calFitness(startPos, goalPos, X,Y,Z,pop);

% ������Ⱥ����
GlobalBest.fitness = inf; % ��ʼ��ÿһ������������
[pop,GlobalBest] = calBest(pop,GlobalBest); 

%% ������
for i = 1:iterMax    
    % ѡ�����
    parentPop = select(pop, p_select);

    % �������
    childPop = crossover(parentPop,p_crs);
    
    % �������
    childPop = mutation(childPop,p_mut,posBound);
    
    % ���������Ӵ���ϵõ��µ���Ⱥ
    pop = [parentPop, childPop];
    
    % ������Ⱥ��Ӧ��
    pop = calFitness(startPos, goalPos, X,Y,Z,pop);

    % ������Ⱥ����
    [pop,GlobalBest] = calBest(pop,GlobalBest);
    
    % ��ÿһ�����������Ӹ�ֵ��fitness_beat_iters
    fitness_beat_iters(i) = GlobalBest.fitness;
    
    % �������д�����ʾÿһ������Ϣ
    disp(['��' num2str(i) '��:' '������Ӧ�� = ' num2str(fitness_beat_iters(i))]);
    
    % ��ͼ
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest);
    pause(0.001);
end

% ������С��Ӧ�ȣ�ֱ�߾���
fitness_best = norm(startPos - goalPos);
disp([ '����������Ӧ�� = ' num2str(fitness_best)]);

% ����Ӧ�ȵ���ͼ
figure
plot(fitness_beat_iters,'LineWidth',2);
xlabel('��������');
ylabel('������Ӧ��');
