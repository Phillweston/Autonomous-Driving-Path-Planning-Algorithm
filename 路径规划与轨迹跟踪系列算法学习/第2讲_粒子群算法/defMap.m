function [X,Y,Z] = defMap(mapRange)

% ��ʼ��������Ϣ
N = 10;                             % ɽ�����
peaksInfo = struct;                 % ��ʼ��ɽ��������Ϣ�ṹ��
peaksInfo.center = [];              % ɽ������
peaksInfo.range = [];               % ɽ������
peaksInfo.height = [];              % ɽ��߶�
peaksInfo = repmat(peaksInfo,N,1);

% �������N��ɽ�����������
for i = 1:N
    peaksInfo(i).center = [mapRange(1) * (rand*0.8+0.2), mapRange(2) * (rand*0.8+0.2)];
    peaksInfo(i).height = mapRange(3) * (rand*0.7+0.3);
    peaksInfo(i).range = mapRange*0.1*(rand*0.7+0.3);
end

% ����ɽ������ֵ
peakData = [];
for x = 1:mapRange(1)
    for y = 1:mapRange(2)
        sum=0;
        for k=1:N
            h_i = peaksInfo(k).height;
            x_i = peaksInfo(k).center(1);
            y_i = peaksInfo(k).center(2);
            x_si = peaksInfo(k).range(1);
            y_si = peaksInfo(k).range(2);
            sum = sum + h_i * exp(-((x-x_i)/x_si)^2 - ((y-y_i)/y_si)^2);
        end
        peakData(x,y)=sum;
    end
end

% ���������������ڲ�ֵ�ж�·���Ƿ���ɽ�彻��
x = [];
for i = 1:mapRange(1)
    x = [x; ones(mapRange(2),1) * i];
end
y = (1:mapRange(2))';
y = repmat(y,length(peakData(:))/length(y),1);
peakData = reshape(peakData,length(peakData(:)),1);
[X,Y,Z] = griddata(x,y,peakData,...
    linspace(min(x),max(x),100)',...
    linspace(min(y),max(y),100));
end

