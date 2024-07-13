function [cluster,breakLines] = vectorizebreakline(cluster)
%
% 从提取的标线点云中识别断裂线，并确定断裂线中元素的连接顺序
% INPUT：
% cluster - 点云聚类数据的结构体数组
%
% OUTPUT：
% cluster - 处理后的点云聚类数据的结构体数组，会多一些处理相关的字段信息
% breakLines - 断裂线在cluster中索引的元胞数组
%
if ~isfield(cluster,'center')
    cluster = getclusterinfo(cluster);
end
[cluster,breakLines] = getbreakline(cluster);

%%%%%%%%%%%%%%%%调试用代码%%%%%%%%%%%%%%%%%%%%%%%
% datasave = [];
% for i=1:size(breakLines,2)
%     idx = breakLines{i};
%     r = rand;
%     g = rand;
%     b = rand;
%     color = [r g b];
%     datatmp = [];
%     for m = 1:size(idx,2)
%         data = cluster(idx(m)).data;
%         datatmp = [datatmp;data];
%         if m==size(idx,2)
%             break;
%         end
% %         c1 = cluster(idx(m)).center;
% %         c2 = cluster(idx(m+1)).center;
% %         figure(3); plot([c1(1) c2(1)],[c1(2) c2(2)],'-','Color',color);hold on;axis equal
% %         rx = cluster(idx(m)).rectx;
% %         ry = cluster(idx(m)).recty;
% %         plot(rx,ry,'-','Color',color);
%     end
%     figure(3); plot(datatmp(:,1),datatmp(:,2),'.','Color',color);hold on;axis equal
%     datasave = [datasave;datatmp];
% end
% savepointcloud2file(datasave,'brokenline',0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

function [cluster,breakLines] = getbreakline(cluster)
%
% 提取断裂线
%
% INPUT：
% cluster - 附带属性信息的标线点云聚类
%
% OUTPUT - cluster、breakLines为提取的断裂线结果
%

[cluster,breakLines] = getbreaklinefragment(cluster);
[cluster,breakLines] = refinebreaklinefragment(cluster,breakLines);
breakindx = cell2mat(breakLines);
for i = 1:size(breakindx,2)
    cluster(breakindx(i)).label = 1;
end
end

function [cluster,linkedBreakLines] = refinebreaklinefragment(cluster,breakLines)
%
% 对初步提取的断裂线结果进行精细化处理，得到最终提取结果
%
% INPUT：
% cluster、breakLines
%
% OUTPUT：
% cluster - 
% linkedBreakLines - 将breakLines中较短的线段连接成较长的断裂线片段
%

% 每次只连接一个方向，连接过程中线段方向也会发生变化，所以多算几次使连接完全
[cluster,linkedBreakLines] = linkBbreaklinefragment(cluster,breakLines,30,5);
% return;
%滤掉小于3个元素的片段
nLinked = size(linkedBreakLines,2);
n = 0;
for iLinked = 1:nLinked
    linekedline = linkedBreakLines{iLinked};
    if size(linekedline,2)<3
        continue;
    end
    n = n+1;
    linkedTmp(n) = {linekedline};
end
linkedBreakLines = linkedTmp;
for i=1:5
    %需要迭代直到linkedBreakLines个数固定为止（一般需要5次计算）
    [cluster,linkedBreakLines] = linkBbreaklinefragment(cluster,linkedBreakLines,60,5);
end
end

function [cluster,linkedBreakLines,isLink] = linkBbreaklinefragment2(cluster,...
    breakLines,isLink,breakNum,breakFragmentinfo,terminalEnd,terminal,Mdl,direction)

% 将属于同一条断裂线的片段连接起来
% 暂时不进行预测

% breakFragmentinfo = getbreaklinefragmentinfo(cluster,breakLines);
nBreak = size(breakLines,2);
r = 60;
iBreak = breakNum;
    Y = terminalEnd(iBreak,1:2);%从endPoint开始增长
    pre_idx = iBreak;%breakLines中索引
    pre_idx_t = nBreak+iBreak;%点在terminal中对应索引
    breakLine = breakLines{pre_idx};
    pre_idx_c = breakLine(end);%在cluster中对应索引
    loopflag = true;
linkedBreakLines = [];
     while(loopflag)
        [Idx,D] = rangesearch(Mdl,Y,r);%对方圆20米内的聚类对象进行比对
        Idx = Idx{1,1};
        for iNearst = 2:size(Idx,2) %最近的是自己，所以从2开始
            now_idx_t = Idx(iNearst);
            now_idx = terminal(now_idx_t,3);
            breakLine = breakLines{now_idx};
            pre_breakLine = breakLines{pre_idx};
            if terminal(now_idx_t,4) == 1
                now_idx_c = breakLine(1);%在cluster中对应索引
            elseif terminal(now_idx_t,4) == 2
                now_idx_c = breakLine(end);%在cluster中对应索引
            end
            if  isLink(now_idx) % 如果线段已经标记则跳过
                if iNearst ==size(Idx,2)
                    %邻域检索完了，也没找到相似的，结束
                    loopflag = false;
                    break;
                end
                continue;
            end
            preX = terminal(pre_idx_t,1);
            preY = terminal(pre_idx_t,2);
            nowX = terminal(now_idx_t,1);
            nowY = terminal(now_idx_t,2);
            preA = cluster(pre_idx_c).angle;
            nowA =cluster(now_idx_c).angle;
            pre_now_angle = atand((nowY-preY)/(nowX-preX));%两个聚类连线的角度
            if pre_now_angle<0
                pre_now_angle = pre_now_angle+180;
            end
            searchAngle = 5;%搜寻角，即在在距当前矩形方向一定范围内搜索
            limitAngle1 = abs(preA-pre_now_angle);%连接线夹角
            limitAngle2 = abs(nowA-pre_now_angle);
            if limitAngle1>90
                limitAngle1 = abs(limitAngle1-180);
            end
            if limitAngle2>90
                limitAngle2 = abs(limitAngle2-180);
            end
            if ~(limitAngle1<searchAngle&&limitAngle2<searchAngle)
%             if ~(abs(preA-pre_now_angle)<searchAngle&&abs(nowA-pre_now_angle)<searchAngle)
                %两聚类不在同一条直线上，跳过
                if iNearst ==size(Idx,2)
                    %邻域检索完了，也没找到相似的，结束
                    loopflag = false;
                    break;
                end
                continue;
            end
            %检查连接方向是否一致
            if (size(linkedBreakLines,2)==0)&&(size(direction,2)==1)
                %如果pre只有一个元素，且该元素没有附加连接方向，则不分方向
                v2 = cluster(pre_idx_c).center - cluster(now_idx_c).center;
            else %size(linkedBreakLines,2)>1
                if size(direction,2)==2
                    v1 = -direction;
                   
                elseif size(linkedBreakLines,1)>0
                    ;
%                 else
%                     if terminal(pre_idx_t,4) == 1
%                         pre_idx_c2 = pre_breakLine(1,2);%与端点相邻的点在cluster中对应索引
%                     elseif terminal(pre_idx_t,4) == 2
%                         pre_idx_c2 = pre_breakLine(1,end-1);
%                     end
%                     v1 =  cluster(pre_idx_c2).center - cluster(pre_idx_c).center;
                    
                end
                v2 = cluster(pre_idx_c).center - cluster(now_idx_c).center;
                angleV12  = acosd(v1*v2'/(norm(v1)*norm(v2)));%向量夹角
                
                if size(breakLine,2)>1
                    if terminal(now_idx_t,4) == 1
                        now_idx_c2 = breakLine(1,2);%与端点相邻的点在cluster中对应索引
                    elseif terminal(now_idx_t,4) == 2
                        now_idx_c2 = breakLine(1,end-1);
                    end
                    v3 = cluster(now_idx_c).center - cluster(now_idx_c2).center;
                    angleV23  = acosd(v2*v3'/(norm(v2)*norm(v3)));%向量夹角
                else
                    angleV23=0;
                end
                if ~((abs(angleV12)<searchAngle)&&(abs(angleV23)<searchAngle))
                    if iNearst ==size(Idx,2)
                        %邻域检索完了，也没找到相似的，结束
                        loopflag = false;
                        break;
                    end
                    continue;
                end
            end             
            %下面就只需判断前后两聚类的形状近似程度
            preL = breakFragmentinfo(pre_idx).length;
            nowL = breakFragmentinfo(now_idx).length;
            preW = breakFragmentinfo(pre_idx).width;
            nowW = breakFragmentinfo(now_idx).width;
            permitDl = 0.5;%允许的长度差
            permitDw = 0.15;%允许的宽度差
            if abs(preL-nowL)<permitDl&&abs(preW-nowW)<permitDw
                % 这里简单以长度差作为相似程度评价量
                tmp = breakLines{now_idx};
                if terminal(now_idx_t,4) == 2
                    tmp = fliplr(tmp);
                end
                linkedBreakLines = [linkedBreakLines tmp];
                isLink(now_idx) = true;
                
                pre_idx = now_idx;
                if terminal(now_idx_t,4) == 1
                    pre_idx_c = breakLine(end);
                    pre_idx_t = nBreak+now_idx;
                elseif terminal(now_idx_t,4) == 2
                    pre_idx_c = breakLine(1);
                    pre_idx_t = now_idx;
                end
                Y = cluster(pre_idx_c).center;
                if exist('v2','var')
                    v1 = v2;
                end
                 direction=0;
                
%                 pre_idx = now_idx;
%                 pre_idx_t = now_idx_t;
%                 pre_idx_c = now_idx_c;
%                 Y = cluster(now_idx_c).center;
               break;% 中断邻域检索，更新增长点
            end  
        end
        if (isempty(iNearst))||(iNearst ==size(Idx,2))
            %邻域检索完了，也没找到相似的，结束
            loopflag = false;
        end
     end 
a = 0;
end

function [cluster,linkedBreakLines] = linkBbreaklinefragment(cluster,breakLines,r,searchAngle)
% 
% 将属于同一条断裂线的较短的线段连接成较长的片段，此算法只从线段的起点开始增长，
% 增长结束时起点变为终点，故可通过多次循环使用linkBbreaklinefragment使线段增长
% 完全
%
% INPUT：
% cluster,breakLines - 
% r - 线段增长时的搜索半径，在此范围内寻找线段连接点
% searchAngle - 搜索角度，在搜索方向一定角度范围内搜索
% 
% OUTPUT：
% cluster - 
% linkedBreakLines - 
%
%

breakFragmentinfo = getbreaklinefragmentinfo(cluster,breakLines);
nBreak = size(breakLines,2);
points = zeros(2*nBreak,2);
%端点矩阵，注意当断裂线片段只有一个元素，其首末端点是同一个点
for iBreak = 1:nBreak
   terminalStart(iBreak,1:4) =  [breakFragmentinfo(iBreak).startPoint iBreak 1];%末位是标记量，1表示start、2表示end
   terminalEnd(iBreak,1:4) =  [breakFragmentinfo(iBreak).endPoint iBreak 2];
end

%先从startPoint连接
terminal = [terminalStart;terminalEnd];
Mdl = KDTreeSearcher(terminal(:,1:2));
% r = 30;
isLink = zeros(nBreak,1);%标记量，标记线段是否已经连接过
nLinkedBreak = 0;
for iBreak = 1:nBreak
    if isLink(iBreak)
        continue;
    end
    nLinkedBreak = nLinkedBreak+1;
    Y = terminalStart(iBreak,1:2);
    isLink(iBreak) = true;
    pre_idx = iBreak;%breakLines中索引
%     pre_idx_t = 2*iBreak-1;%点在terminal中对应索引
    pre_idx_t = iBreak;
    breakLine = breakLines{pre_idx};
    pre_idx_c = breakLine(1);%在cluster中对应索引
    loopflag = true;
    linkedBreakLines(nLinkedBreak) = {fliplr(breakLines{iBreak})};%从start点连接顺序要反过来
%     [Idx,D]= rangesearch(Mdl,Y,r);
         while(loopflag)
        [Idx,D] = rangesearch(Mdl,Y,r);%对方圆20米内的聚类对象进行比对
        Idx = Idx{1,1};
        for iNearst = 2:size(Idx,2) %最近的是自己，所以从2开始
            now_idx_t = Idx(iNearst);
            now_idx = terminal(now_idx_t,3);
            breakLine = breakLines{now_idx};
            pre_breakLine = breakLines{pre_idx};
            if terminal(now_idx_t,4) == 1
                now_idx_c = breakLine(1);%在cluster中对应索引
            elseif terminal(now_idx_t,4) == 2
                now_idx_c = breakLine(end);%在cluster中对应索引
            end
            if (now_idx_c==42|| now_idx_c==109)&&nLinkedBreak==11
                a=0;
            end
            if  isLink(now_idx) % 如果线段已经标记则跳过
                if iNearst ==size(Idx,2)
                    %邻域检索完了，也没找到相似的，此时开始新的一个linkedLine
                    loopflag = false;
                    break;
                end
                continue;
            end
            preX = terminal(pre_idx_t,1);
            preY = terminal(pre_idx_t,2);
            nowX = terminal(now_idx_t,1);
            nowY = terminal(now_idx_t,2);
            preA = cluster(pre_idx_c).angle;
            nowA =cluster(now_idx_c).angle;
            pre_now_angle = atand((nowY-preY)/(nowX-preX));%两个聚类连线的角度
            if pre_now_angle<0
                pre_now_angle = pre_now_angle+180;
            end
%             searchAngle = 5;%搜寻角，即在在距当前矩形方向一定范围内搜索
            limitAngle1 = abs(preA-pre_now_angle);%连接线夹角
            limitAngle2 = abs(nowA-pre_now_angle);
            if limitAngle1>90
                limitAngle1 = abs(limitAngle1-180);
            end
            if limitAngle2>90
                limitAngle2 = abs(limitAngle2-180);
            end
            if ~(limitAngle1<searchAngle&&limitAngle2<searchAngle)
                %两聚类不在同一条直线上，跳过
                if iNearst ==size(Idx,2)
                    %邻域检索完了，也没找到相似的，此时开始新的一个breakLine
                    loopflag = false;
                    break;
                end
                continue;
            end
            %检查连接方向是否一致
            v2 = cluster(pre_idx_c).center - cluster(now_idx_c).center;
            if size(pre_breakLine,2)==1
                %如果只有一个元素，则不分方向
                v1 = v2;
                angleV12  = acosd(v1*v2'/(norm(v1)*norm(v2)));%向量夹角
            else
                if terminal(pre_idx_t,4) == 1
                    pre_idx_c2 = pre_breakLine(1,2);%与端点相邻的点在cluster中对应索引
                elseif terminal(pre_idx_t,4) == 2
                    pre_idx_c2 = pre_breakLine(1,end-1);
                end
                v1 =  cluster(pre_idx_c2).center - cluster(pre_idx_c).center;
                angleV12  = acosd(v1*v2'/(norm(v1)*norm(v2)));%向量夹角
            end
            if size(breakLine,2)>1
                if terminal(now_idx_t,4) == 1
                    now_idx_c2 = breakLine(1,2);%与端点相邻的点在cluster中对应索引
                elseif terminal(now_idx_t,4) == 2
                    now_idx_c2 = breakLine(1,end-1);
                end
                v3 = cluster(now_idx_c).center - cluster(now_idx_c2).center;
                angleV23  = acosd(v2*v3'/(norm(v2)*norm(v3)));%向量夹角
            else
                angleV23=0;
            end
            if ~((abs(angleV12)<searchAngle)&&(abs(angleV23)<searchAngle))
                if iNearst ==size(Idx,2)
                    %邻域检索完了，也没找到相似的，此时开始新的一个breakLine
                    loopflag = false;
                    break;
                end
                continue;
            end
            %下面就只需判断前后两聚类的形状近似程度
            preL = breakFragmentinfo(pre_idx).length;
            nowL = breakFragmentinfo(now_idx).length;
            preW = breakFragmentinfo(pre_idx).width;
            nowW = breakFragmentinfo(now_idx).width;
            permitDl = 0.5;%允许的长度差
            permitDw = 0.15;%允许的宽度差
            if abs(preL-nowL)<permitDl&&abs(preW-nowW)<permitDw
                % 这里简单以长度差作为相似程度评价量
                tmp = breakLines{now_idx};
                if terminal(now_idx_t,4) == 2
                    tmp = fliplr(tmp);
                end
                linkedBreakLines(nLinkedBreak) = {[linkedBreakLines{nLinkedBreak} tmp]};
                isLink(now_idx) = true;
                % 更新增长点，注意如果当前连接的是片段起点，则下一步的增长点应片段是终点
                pre_idx = now_idx;
                if terminal(now_idx_t,4) == 1
                    pre_idx_c = breakLine(end);
                    pre_idx_t = nBreak+now_idx;
                elseif terminal(now_idx_t,4) == 2
                    pre_idx_c = breakLine(1);
                    pre_idx_t = now_idx;
                end
                Y = cluster(pre_idx_c).center;
               break;% 中断邻域检索，更新增长点
            end  
        end
        if (isempty(iNearst))||(iNearst ==size(Idx,2))
%            % 邻域检索完了，也没找到相似的，此时开始新的一个breakLine
%            % 有个bug，向另一个方向增长时，端点按独立点处理的，没有考虑之前的增长方向
            if size(linkedBreakLines{nLinkedBreak},2)>1
                tmp = linkedBreakLines{nLinkedBreak};
                direction = cluster(tmp(1)).center - cluster(tmp(2)).center;
            else
                direction = 0;
            end
%             if nLinkedBreak==11
%                 a=0;
%             end
            %-----------------------------------------------
            % 每条线段有startpoint、endpoint两个端点，linkBbreaklinefragment算
            % 法先从startpoint进行增长，增长结束后进入linkBbreaklinefragment2
            % 进行endpoint的增长，但linkBbreaklinefragment2目前有2个bug，1是会出
            % 现锐角的连接，2是滤掉的无用聚类又会出现，需要完善。
            % 如果没有linkBbreaklinefragment2，则需要多次运行linkBbreaklinefragment使
            % 线段充分连接知道线段数目不在变化（一般需5次以上），而如果有linkBbreaklinefragment2，
            % linkBbreaklinefragment一般运行两次线段个数就会固定
%             [cluster,endlinkedBreakLine,isLink] = linkBbreaklinefragment2(cluster,...
%                 breakLines,isLink,iBreak,breakFragmentinfo,terminalEnd,terminal,Mdl,direction);
%             linkedBreakLines(nLinkedBreak) = {[fliplr(endlinkedBreakLine) linkedBreakLines{nLinkedBreak}]};
            %---------------------------------------------
            loopflag = false;
        end
     end 
end
end

function breaklinefragmentinfo = getbreaklinefragmentinfo(cluster,breakLines)
% 
% 计算断裂线片段的长度、首尾端点、元素的平均属性等信息，这些信息可用于后续处理中
%

nfrag  = size(breakLines,2);
for ifrag = 1:nfrag
    breakLine = breakLines{ifrag};
    startPoint = cluster(breakLine(1)).center;
    endPoint = cluster(breakLine(end)).center;
    %线段端点
    breaklinefragmentinfo(ifrag).startPoint = startPoint;
    breaklinefragmentinfo(ifrag).endPoint = endPoint;
    centers = zeros(size(breakLine,2),2);
    %求取平均形状
    nc = size(breakLine,2);
    for i = 1:nc
        centers(i,:) =  cluster(breakLine(i)).center;
        widths(i,:) = cluster(breakLine(i)).width;
        lengths(i,:) = cluster(breakLine(i)).length;
        areas(i,:) = cluster(breakLine(i)).area;
        perctsA(i,:) = cluster(breakLine(i)).perctA;
    end
    breaklinefragmentinfo(ifrag).width = mean(widths(1:nc));%理论上应该去掉奇异数据，这里暂时简单化处理
    breaklinefragmentinfo(ifrag).length = mean(lengths(1:nc));
    breaklinefragmentinfo(ifrag).area = mean(areas(1:nc));
    breaklinefragmentinfo(ifrag).perctA = mean(perctsA(1:nc));
    %求取线段长度
    if size(centers,1)>1
        pre_cx = centers(1:end-1,1);
        pre_cy = centers(1:end-1,2);
        next_cx = centers(2:end,1);
        next_cy = centers(2:end,2);
        breaklinefragmentinfo(ifrag).range = ...
            sum(sqrt((pre_cx-next_cx).^2+(pre_cy - next_cy).^2));
    else
        breaklinefragmentinfo(ifrag).range = 0;%只有一个聚类元素，认为长度为0
    end   
end
end

function [cluster,breakLines] = getbreaklinefragment(cluster)
%
% 对断裂线元素进行初步聚类形成断裂线线段片段
%
% INPUT：
% cluster - 
% 
% OUTPUT：
% cluster - 
% breakLines - 按顺序存储的断裂线在cluster中的索引
%
% 程序中的重要参数：r、searchAngle、permitDl、permitDw
%
% 提取断裂线片段（虚线）
% 这里是根据强筛选条件提取出断裂线片段，此时的片段是碎片化的，但其已经拥有更高
% 一级维度的信息，这些信心可用于断裂线的进一步精细化处理。

% 找出近似呈矩形分布的聚类
lengthLimit = 7;%断裂线的点云聚落最大允许长度

 nRect = 0;
for iCluster=1:size(cluster,2)
    if cluster(iCluster).label
        continue;
    end
    data = cluster(iCluster).data;
    center = cluster(iCluster).center;
    limit_perctA = 0.6;
    limit_area = 0.3;
    if (cluster(iCluster).length/cluster(iCluster).width)>7
        %若果长宽比大于7
        limit_perctA = 0.3;
        limit_area = 0.2;
    end
    if cluster(iCluster).perctA>limit_perctA&&cluster(iCluster).area>limit_area&&cluster(iCluster).length<lengthLimit
        % 以面积百分比以及大小作为筛选原则
        nRect = nRect+1;
        rectIndex(nRect,1:3) = [center,iCluster];
    end
    %figure(3); plot(data(:,1),data(:,2),'.');hold on;
end

% 对断裂线进行初步聚类，使用的是单线条生长算法
Mdl = KDTreeSearcher(rectIndex(:,1:2));
nBreakLines = 0;
breakLines = {};
r  = 20;
for iRect = 1:nRect
    idx = rectIndex(iRect,3);
    if  cluster(idx).isLabel % 如果已经标记则跳过
        continue;
    end
    nBreakLines = nBreakLines+1;
    Y = rectIndex(iRect,1:2); %矩形聚类的中心
    cluster(idx).isLabel = true;
    pre_idx = idx;
    pre_pre_idx = 0;% 0表示不存在pre_pre_idx
    loopflag = true;
    breakLines(nBreakLines) = {pre_idx};
%     if nBreakLines==28
%         a=0;
%     end
    while(loopflag)
        [Idx,D] = rangesearch(Mdl,Y,r);%对方圆一定距离内的聚类对象进行比对
        Idx = Idx{1,1};
        for iNearst = 2:size(Idx,2) %最近的是自己，所以从2开始
            now_idx = rectIndex(Idx(iNearst),3);
            if  cluster(now_idx).isLabel % 如果已经标记则跳过
                if iNearst ==size(Idx,2)
                    %邻域检索完了，也没找到相似的，此时开始新的一个breakLine
                    loopflag = false;
                    break;
                end
                continue;
            end
            preX = cluster(pre_idx).center(1);
            preY = cluster(pre_idx).center(2);
            nowX = cluster(now_idx).center(1);
            nowY = cluster(now_idx).center(2);
            preA = cluster(pre_idx).angle;
            nowA =cluster(now_idx).angle;
            pre_now_angle = atand((nowY-preY)/(nowX-preX));%两个聚类的中心连线的角度
            if pre_now_angle<0
                pre_now_angle = pre_now_angle+180;
            end
            if pre_pre_idx~=0
                v1 = cluster(pre_idx).center - cluster(pre_pre_idx).center;
                v2 = cluster(now_idx).center - cluster(pre_idx).center;
                angleV12  = acosd(v1*v2'/(norm(v1)*norm(v2)));%向量夹角,避免增长方向突变
            else
                angleV12 = 0;
            end
            searchAngle = 5;%搜寻角，即在在距当前矩形方向一定范围内搜索
            limitAngle1 = abs(preA-pre_now_angle);%连接线夹角
            limitAngle2 = abs(nowA-pre_now_angle);
            if limitAngle1>90
                limitAngle1 = abs(limitAngle1-180);
            end
            if limitAngle2>90
                limitAngle2 = abs(limitAngle2-180);
            end
            if ~(limitAngle1<searchAngle&&limitAngle2<searchAngle&&(angleV12<searchAngle))
                %两聚类不在同一条直线上，跳过
                if iNearst ==size(Idx,2)
                    %邻域检索完了，也没找到相似的，此时开始新的一个breakLine
                    loopflag = false;
                    break;
                end
                continue;
            end
            %下面就只需判断前后两聚类的形状近似程度
            preL = cluster(pre_idx).length;
            nowL = cluster(now_idx).length;
            preW = cluster(pre_idx).width;
            nowW = cluster(now_idx).width;
            permitDl = 1;%允许的长度差
            permitDw = 0.15;%允许的宽度差
            if abs(preL-nowL)<permitDl&&abs(preW-nowW)<permitDw
                % 这里简单以长度差作为相似程度评价量
                cluster(now_idx).isLabel = true;
                breakLineTmp = breakLines{nBreakLines};
                breakLines(nBreakLines) = {[breakLineTmp now_idx]};
                pre_idx = now_idx;
                Y = cluster(now_idx).center;
                if size(breakLineTmp,2)>0
                    pre_pre_idx = breakLineTmp(end);
                else
                    pre_pre_idx = 0;
                end
               break;% 中断邻域检索，更新增长点
            end  
        end
        if iNearst ==size(Idx,2)
            %邻域检索完了，也没找到相似的，此时开始新的一个breakLine
            loopflag = false;
        end
    end
end
end

% function cluster = getclusterinfo(cluster)
% %
% % 计算点云聚类的描述性信息，如外接矩形大小、面积、中心位置、朝向等，使用这些信
% % 息对可以对聚类进行更高层次的抽象以便做进一步的处理。
% %
% % INPUT：
% % cluster - 点云聚类结构体
% % 
% % OUTPUT：
% % cluster - 拥有相关描述性属性字段的点云聚类结构体
% 
% nc  = size(cluster,2);%聚类个数
% for i=1:nc
%     data = cluster(i).data;
%     cluster(i).isLabel = false;
%     cluster(i).label = -1;%-1表示类别还未知
%     x = data(:,1);
%     y = data(:,2);
%     [rectx,recty,rectArea,perimeter] = minboundrect(data(:,1),data(:,2));
%     cluster(i).rectx = rectx;
%     cluster(i).recty = recty;
%     cx = mean(rectx);
%     cy = mean(recty);
%     cluster(i).center = [cx cy];%外接矩形中心
%     cluster(i).rectArea = rectArea;%最小外接矩形面积
%     cluster(i).perimeter = perimeter;
%     d = sqrt((rectx(1:2) - rectx(2:3)).^2+(recty(1:2) - recty(2:3)).^2);%外接矩形边长
%     cluster(i).length = max(d);
%     cluster(i).width = min(d);
%     areaSeed = unique([roundn(x,-1) roundn(y,-1)],'rows');%坐标对应到0.1米的格网中
%     pointArea = 0.1*0.1*size(areaSeed,1)-0.5*perimeter*0.1;%以格网个数近似计算点云面积 
%     cluster(i).area = pointArea;
%     cluster(i).perctA = pointArea/rectArea;
%     
%     %矩形方向向量，表示了矩形的姿态（部分方向），模的大小表示了呈矩形的程度
%     if d(1)==d(2)
%         cluster(i).direction = [0,0];%正方形向量模为0
%     elseif d(1)>d(2)
%          cluster(i).direction = [(rectx(1) - rectx(2))/d(1),(recty(1) - recty(2))/d(1)].*(abs(d(1)/d(2)-1));   
%     elseif d(1)<d(2)
%         cluster(i).direction = [(rectx(3) - rectx(2))/d(2),(recty(3) - recty(2))/d(2)].*(abs(d(2)/d(1)-1));   
%     end
%     if cluster(i).direction(2)<0
%         cluster(i).direction = -cluster(i).direction;
%     end 
%     cluster(i).angle = atand(cluster(i).direction(2)/cluster(i).direction(1));
%     if cluster(i).angle<0
%         cluster(i).angle = cluster(i).angle+180;%角度换算到0~180
%     end
% %     plot(rectx,recty,'-');hold on;
% %     plot(x,y,'.');hold on;
% %     axis equal;
% end
% end
