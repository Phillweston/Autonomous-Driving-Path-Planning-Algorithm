function parentPop = select(pop, p_select)

% �������̶ķ�ִ��ѡ�����
fit_reverse = 1./[pop.fitness]'; 
totalFit = sum(fit_reverse);
accP = cumsum(fit_reverse/totalFit);        % �����ۼƺ�
selectNum = round(size(pop,2) * p_select);  % ѡ��ĸ�������

% ��pop�ĵ�һ�и�ֵ��parentPop����ʵ�ֳ�ʼ��
parentPop = pop(1);
for i=1:selectNum
    % �ҵ������������ۻ�����
    idx = find(accP>rand); 
    if isempty(idx)
        continue
    end  
    
    % ���׸������������ۻ����ʵ�λ�õĸ����Ŵ���ȥ
    parentPop(i) = pop(idx(1));
end

