% ���������߷�:ֱ�۶�ͼ���ܱ����������г̹���
% ���ߣ�Ally
% ���ڣ�2021/1/30
clc
clear
close all

%% ֱ�۶�ͼ���ܱ����������г̹���

% һ�α���������
P0 = [0, 0];
P1 = [1, 1];
P = [P0; P1];
figure;
plot(P(:,1), P(:,2), 'k');
MakeGif('һ�α���������.gif',1);
hold on
for t = 0:0.01:1
    P_t = (1-t)*P0 + t*P1;
    scatter(P_t(1), P_t(2), 200, '.r');
    stringName = "һ�α��������ߣ�t =" + num2str(t);
    title(stringName)
    MakeGif('һ�α���������.gif',t*100+1);
end

% ���α���������
P0 = [0, 0];
P1 = [1, 1];
P2 = [2, 1];
P = [P0; P1; P2];
figure;
plot(P(:,1), P(:,2), 'k');
MakeGif('���α���������.gif',1);
hold on
scatter(P(:,1), P(:,2), 200, '.b');
for t = 0:0.01:1
    P_t_1 = (1-t)*P0 + t*P1;
    P_t_2 = (1-t)*P1 + t*P2;
    P_t_3 = (1-t)*P_t_1 + t*P_t_2;
    h1 = scatter(P_t_1(1), P_t_1(2), 300, '.g');
    h2 = scatter(P_t_2(1), P_t_2(2), 300, '.g');
    h3 = plot([P_t_1(1), P_t_2(1)], [P_t_1(2), P_t_2(2)], 'g', 'linewidth',2);    
    scatter(P_t_3(1), P_t_3(2), 300, '.r');
    stringName = "���α��������ߣ�t =" + num2str(t);
    title(stringName)
    MakeGif('���α���������.gif',t*100+1);
    delete(h1);
    delete(h2);
    delete(h3);
end


% ���α���������
P0 = [0, 0];
P1 = [1, 1];
P2 = [2, 1];
P3 = [3, 0];
P = [P0; P1; P2; P3];
figure;
plot(P(:,1), P(:,2), 'k');
MakeGif('���α���������.gif',1);
hold on
scatter(P(:,1), P(:,2), 200, '.b');
for t = 0:0.01:1
    P_t_1_1 = (1-t)*P0 + t*P1;
    P_t_1_2 = (1-t)*P1 + t*P2;
    P_t_1_3 = (1-t)*P2 + t*P3;

    P_t_2_1 = (1-t)*P_t_1_1 + t*P_t_1_2;
    P_t_2_2 = (1-t)*P_t_1_2 + t*P_t_1_3;

    P_t_3 = (1-t)*P_t_2_1 + t*P_t_2_2;
    
    h1 = scatter(P_t_1_1(1), P_t_1_1(2), 300, '.b');
    h2 = scatter(P_t_1_2(1), P_t_1_2(2), 300, '.b');
    h3 = scatter(P_t_1_3(1), P_t_1_3(2), 300, '.b');
    h4 = plot([P_t_1_1(1); P_t_1_2(1)], [P_t_1_1(2); P_t_1_2(2)],  'b', 'linewidth',2);
    h5 = plot([P_t_1_2(1); P_t_1_3(1)], [P_t_1_2(2); P_t_1_3(2)],  'b', 'linewidth',2);
    
    h6 = scatter(P_t_2_1(1), P_t_2_1(2), 300, '.g');
    h7 = scatter(P_t_2_2(1), P_t_2_2(2), 300, '.g');
    h8 = plot([P_t_2_1(1); P_t_2_2(1)], [P_t_2_1(2); P_t_2_2(2)],  'g', 'linewidth',2);
    
    scatter(P_t_3(1), P_t_3(2), 300, '.r');
    stringName = "���α��������ߣ�t =" + num2str(t);
    title(stringName)
    MakeGif('���α���������.gif',t*100+1);
    delete(h1);
    delete(h2);
    delete(h3);
    delete(h4);
    delete(h5);
    delete(h6);
    delete(h7);
    delete(h8);
    
end

