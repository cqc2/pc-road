function tracePointData = searchtracepoint(pointCloudData)
% search the nearest point to scanner according to the point density of scanline 
%对轨迹点进行了分段拟合，暂未对端点进行平滑处理
    nPoint = size(pointCloudData,1);
    prePoint = pointCloudData(1:nPoint-1,1:4);
    nextPoint = pointCloudData(2:nPoint,1:4);    
    dx = (prePoint(1:nPoint-1,1)-nextPoint(1:nPoint-1,1));
    dy = (prePoint(1:nPoint-1,2)-nextPoint(1:nPoint-1,2));
    dh = (prePoint(1:nPoint-1,3)-nextPoint(1:nPoint-1,3));
    ds = sqrt(dx.^2+dy.^2+dh.^2);
%可以用此图像来理解本算法
%     plot(1:1000,ds(1:1000),'r.');
%     hold on
%     plot(10000:20000,ds(10000:20000));
    centerPointArray = zeros(fix(nPoint/100),4);
    iTracePoint = 0;
    num = 0;%每条扫描线中间隔小于0.5m的点的数量，轨迹点在这些点当中
    %intervalQuantity表示相邻扫描线之间间隔的点个数，一般取扫描线点个数的0.75
    %轨迹点提取结果对此参数不是很敏感，不要与实际情况差别太大即可
    %intervalQuantit过小会使轨迹点过度提取，过大会使一些扫描线的轨迹点被忽略
    intervalQuantity = 50;
    for i = 1:nPoint-1,
        if (ds(i))<0.5,
            num = num+1;
            linePointArray(num,1) = i;
            linePointArray(num,2) = round(ds(i),4);
        elseif (ds(i))>=0.5&&(num>=intervalQuantity),
            iTracePoint = iTracePoint+1;
            linePointArray=sortrows(linePointArray,2);
            minDs = linePointArray(1,2);%最小间隔
            nMinDs=0;%最小间隔数量
            while (minDs==linePointArray(nMinDs+1,2))&&((nMinDs+1)<=num),
                %有些最小间隔距离是一样的，所以不能只取第一个点
                nMinDs=nMinDs+1; 
            end
            a= median(linePointArray(1:nMinDs,1));
            %由检测的轨迹点拟合的轨迹与实测pos数据对比，发现向行车方向左侧存在大约
            %2cm左右的系统误差，在补偿了4个样本轨迹点距离后结果得到较大改善，推测
            %可能是测量车标定时产生的误差
            minDsPointOrder=ceil((linePointArray(1,1)+linePointArray(nMinDs,1))/2)-4;%最小间隔对应的左点序号   
            if minDsPointOrder<1
                minDsPointOrder = 1;
            end
            centerPointArray(iTracePoint,1:4) = pointCloudData(minDsPointOrder,1:4);
            num = 0;
            linePointArray = [];
        else
            num = 0;
        end     
    end
    nTracePoint = iTracePoint;
    tracePointData = centerPointArray(1:nTracePoint,:);
%     %对轨迹线分段拟合,这里以10米间隔二阶多项式拟合
% %     segmentArray = zeros(nCenterPoint/10,1);
%     segmentArray(1) = 1;
%     iSegmentArray = 1;
%     preX = centerPointArray(1,1);
%     preY = centerPointArray(1,2);
%     for i = 1:nCenterPoint,
%         x = centerPointArray(i,1);
%         y = centerPointArray(i,2);
%         %轨迹点之间的间隔距离
%         dist = sqrt((x-preX)^2+(y-preY)^2);
%         if dist>10,
%             iSegmentArray = iSegmentArray+1;
%             segmentArray(iSegmentArray) = i;
%             preX = x;
%             preY = y;
%         end      
%     end
%     segmentArray(iSegmentArray) = nCenterPoint;
%     traceData = zeros((iSegmentArray-1)*100,2);
%     for i=1:iSegmentArray-1,
%         nStart = segmentArray(i);
%         nEnd = segmentArray(i+1);
%         traceTemp = centerPointArray(nStart:nEnd,:);
%         xdata = traceTemp(:,1);
%         ydata = traceTemp(:,2);
%         hdata = traceTemp(:,3);
%         p=polyfit(xdata,ydata,1);
%         %等分成100个间隔
%         x1 = linspace(centerPointArray(nStart,1),centerPointArray(nEnd,1));
%         y1 = polyval(p,x1);
%         step = (nEnd-nStart)/100;
%         for m =1:100,
%             temp = ceil(step*m);      
%             h(m) = hdata(temp);
%         end
%         traceData((i-1)*100+1:i*100,1:3) = [x1' y1' h'];
% %         plot(xdata,ydata,'.')
% %         hold on
% %         plot(x1,y1,'r-')
% %         axis equal;
%     end
% %     traceData = 0;
end