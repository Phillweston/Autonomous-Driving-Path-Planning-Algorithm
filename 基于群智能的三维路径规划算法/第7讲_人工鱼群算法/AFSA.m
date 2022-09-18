% ��7�����˹���Ⱥ�㷨
% ���ߣ� Ally
% ���ڣ� 2021/10/16
clc
clear
close all

%% ��ά·���滮ģ�Ͷ���
startPos = [1, 1, 1];
goalPos = [100, 100, 80];

% �������ɽ���ͼ
mapRange = [100,100,100];              % ��ͼ�������߷�Χ
[X,Y,Z] = defMap(mapRange);

%% ��ʼ��������
N = 50;                % ��Ⱥ����
visual = 50;           % �˹���ĸ�֪����
step = 3;              % �˹�����ƶ���󲽳�
delta = 10;            % ӵ��������
try_number = 50;       % ��ֻ�������������
iterMax = 100;         % ��������
pointNum = 3;          % ���ά�� 

% λ�ý���
posBound = [[0,0,0]',mapRange'];

%% ��Ⱥ��ʼ��
% ��ʼ��һ���յ���Ⱥ�ṹ��
fishes = struct;
fishes.pos= [];
fishes.fitness = [];
fishes.path = [];
fishes = repmat(fishes,N,1);

% ��ʼ�������˹���
GlobalBest.fitness = inf;

% ��Ⱥ����ֲ�λ��
for i = 1:N
    % ��Ⱥ������̬�ֲ��������
    fishes(i).pos.x = unifrnd(posBound(1,1),posBound(1,2),1,pointNum);
    fishes(i).pos.y = unifrnd(posBound(2,1),posBound(2,2),1,pointNum);
    fishes(i).pos.z = unifrnd(posBound(3,1),posBound(3,2),1,pointNum);
    
    % ��Ӧ��
    [fitness,path] = calFitness(startPos, goalPos, fishes(i).pos);
    
    % ��ײ����ж�
    flag = judgeObs(path,X,Y,Z);
    if flag == 1
        % ��flag=1��������·�������ϰ����ཻ����������Ӧ��ֵ
        fishes(i).fitness = 1000*fitness;
        fishes(i).path = path;
    else
        % ���򣬱�������ѡ���·��
        fishes(i).fitness = fitness;
        fishes(i).path = path;
    end

    % ���¸����˹��������
    fishes(i).Best.pos = fishes(i).pos;
    fishes(i).Best.fitness = fishes(i).fitness;
    fishes(i).Best.path = fishes(i).path;
    
    % ����ȫ������
    if fishes(i).Best.fitness < GlobalBest.fitness
        GlobalBest = fishes(i).Best;
    end
end

% ��ʼ��ÿһ����������Ӧ�ȣ����ڻ���Ӧ�ȵ���ͼ
fitness_beat_iters = zeros(iterMax,1);


%% ѭ��
for iter = 1:iterMax
    for i = 1:N
        
        fish = fishes(i);
        % Ⱥ����Ϊ
        [pos_swarm,fitness_swarm,path_swarm] =  swarmBehavior(fish,fishes,...
            pointNum,visual,startPos,goalPos,try_number,delta,step,X,Y,Z);
        
        % ׷β��Ϊ
        [pos_follow,fitness_follow,path_follow] = followBehavior(fish,...
            fishes,pointNum,visual,startPos,goalPos,try_number,delta,step,X,Y,Z);

         % ������Ϊ������
        if fitness_swarm < fitness_follow
            fishes(i).pos = pos_swarm;
            fishes(i).path = path_swarm;
            fishes(i).fitness = fitness_swarm;
        else
            fishes(i).pos = pos_follow;
            fishes(i).path = path_follow;
            fishes(i).fitness = fitness_follow;
        end
        
        % ����ȫ�������˹���
        if fishes(i).fitness < GlobalBest.fitness
            GlobalBest = fishes(i);
        end
        
    end
    
    % ��ÿһ�����������Ӹ�ֵ��fitness_beat_iters
    fitness_beat_iters(iter) = GlobalBest.fitness;
    
    % �������д�����ʾÿһ������Ϣ
    disp(['��' num2str(iter) '��:' '������Ӧ�� = ' num2str(fitness_beat_iters(iter))]);
    
    % ��ͼ
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest);
    pause(0.001);
end
        
%% ���չʾ
% ������С��Ӧ�ȣ�ֱ�߾���
fitness_best = norm(startPos - goalPos);
disp([ '����������Ӧ�� = ' num2str(fitness_best)]);

% ����Ӧ�ȵ���ͼ
figure
plot(fitness_beat_iters,'LineWidth',2);
xlabel('��������');
ylabel('������Ӧ��');