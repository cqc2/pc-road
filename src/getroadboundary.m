function boundaryArray = getroadboundary(SliceArray)
%detect and fit boundary through road point slices.
addpath(genpath(pwd));
datetime('now','TimeZone','local','Format','HH:mm:ss Z')
% roadname ='ErqiBridge';
% roadname ='FazhanAvenue';

roadname ='MoshuiLake';
% roadname ='YingwuzhouBridge';

pospath = strcat('CASEDATA\',roadname,'\tracedata\road-all_GNSStrace.xyz');
posData = readposfile(pospath,3);
mileagedata(:,1) = posData.x;
mileagedata(:,2) = posData.y;
mileagedata(:,3) = posData.elevation;
% savepointcloud2file(mileagedata,'posXYData.xyz',true);
%     pointCloudFilePath = 'roadPoint_5.xyz';
%     pointCloudFilePath = 'CASEDATA\YingwuzhouBridge\roaddata\yingwuzhou-alldata.xyz';
%     tracePointFilePath = 'posXYData.xyz';
%     tracePointFilePath = 'boundary1_All_test.xyz';
%     pointCloudData = readpointcloudfile2(pointCloudFilePath);
%     traceData = readpointcloudfile2(tracePointFilePath);
    mileageArray = createmileage(mileagedata);
    
    
%     SliceArray = slice(pointCloudData);
%     boundaryArray = getroadboundarypoint(SliceArray); 
%     savepointcloud2file(boundaryArray{1},'CASEDATA\YingwuzhouBridge\boundary1_yingwuzhou',false);
%     savepointcloud2file(boundaryArray{2},'CASEDATA\YingwuzhouBridge\boundary2_yingwuzhou',false);
%     
%     a=boundaryArray{1};
%     b=boundaryArray{2};
% %     plot(a(:,1),a(:,2),'g-');hold on
% %     plot(b(:,1),b(:,2),'g-');hold on
%     plot(a(:,1),a(:,2),'b.','MarkerSize',2);hold on
%     plot(b(:,1),b(:,2),'b.','MarkerSize',2);hold on
    
%     plot(traceData(:,1),traceData(:,2),'y.','MarkerSize',5);hold on

    boundary1 = readpointcloudfile2(strcat('CASEDATA\',roadname,'\boundary1.xyz'));
    boundary2 = readpointcloudfile2(strcat('CASEDATA\',roadname,'\boundary2.xyz'));
%     plot(boundary1(:,1),boundary1(:,2),'g.','MarkerSize',1);hold on
%     plot(boundary2(:,1),boundary2(:,2),'b.','MarkerSize',1);
%     cluster = clusterpoint(boundary2,0.5);
%     pointCloudData = readpointcloudfile2('dataspace\treeNoise.xyz');
%    [coefficients,percet] = ransac(boundary2(2000:10000,1:2),2,0.05);
   
%  hold on
% x=pointCloudData(1:5000,1);y=pointCloudData(1:5000,2);z=pointCloudData(1:5000,3);
% [xi,yi]=meshgrid(linspace(min(x),max(x),50),linspace(min(y),max(y),50));
% zi=griddata(x,y,z,xi,yi,'v4');
% surf(xi,yi,zi);axis equal;
%shading interp %去除网格
   
%     ransac(boundary1(:,1:2),2);
%     a=0;
%     type = 'b.';
%     nCLuster = size(cluster,2);
%     for i=1:nCLuster,
%         if type == 'b.';
%             type = 'g.';
%         else
%             type = 'b.';
%         end
%         points = cluster{i};hold on
%         drawpoint(points,type); hold on
%     end    
    boundaryArray(1:2) = [{boundary1} {boundary2}];
    isBoundaryInfoArray = refineroadboundarypoint(boundaryArray,mileageArray);       

datetime('now','TimeZone','local','Format','HH:mm:ss Z')
end

function isBoundaryInfoArray = refineroadboundarypoint(boundaryArray,mileageArray)
%refine road boundary points
    nBoundary = size(boundaryArray,2);
    clusterInfoArray = cell(1,nBoundary);
    clusterArray = cell(1,nBoundary);
    for iBoundary = 1:nBoundary
    %计算边界与轨迹线的关系信息
       boundary = boundaryArray{iBoundary};
        cluster = clusterpoint(boundary,0.5,3);
        cluster = cluster2straightline(cluster,mileageArray);
        
%         for qq = 1:size(cluster,2)
%             dd = cluster{qq};
%             plot(dd(:,1),dd(:,2),'Marker','.','Color',[rand rand rand]);hold on; axis equal;
%             text(mean(dd(:,1)),mean(dd(:,2)),num2str(qq));
%         end
%         figure();
        
        clusterArray(iBoundary) = {cluster};
        nCluster = size(cluster,2);
        %[起点里程 起点-轨迹距离 终点里程 终点-轨迹距离 起终点长度]
        clusterInfo = zeros(nCluster,5);
        
        for i=1:nCluster,
            points = cluster{i};
            nPoint = size(points,1);
            pointStart = points(1,1:3);
            pointEnd = points(nPoint,1:3);
           rangeInfoStart= distpoint2trace2(pointStart,mileageArray);
           rangeInfoEnd= distpoint2trace2(pointEnd,mileageArray);
           dist = norm(pointStart-pointEnd);
           clusterInfo(i,:) = [rangeInfoStart(1,4:5) rangeInfoEnd(1,4:5) dist];
        end
%         save -ascii 'huaide.txt' clusterInfo;
        clusterInfoArray(iBoundary) ={clusterInfo};
    end
    
    for iBoundary = 1:nBoundary,
        %长度较短，且比相邻的聚类线更靠近轨迹的线段认为是非边界聚类线
       clusterInfo = clusterInfoArray(1,iBoundary);
       clusterInfo = clusterInfo{1};
       cluster = clusterArray{iBoundary};       
       nCluster =  size(clusterInfo,1);
       isBoundaryArray = zeros(nCluster,1);%标记，如果判断为边界则置为true，否则为0
       if nCluster==1,
           %当只有一个聚类线时，此时只能根据自特征判断
           points = cluster{1};
           info = clusterInfo(1,:);
          [mileageLength range] = calculatesolefeatures(points,info);
          if mileageLength>30&&range>1,
              isBoundaryArray(1,1) = true;
          end
       elseif nCluster==2,
           %当有2个聚类线时,此时仍只能根据明显的边界自特征判断
           for i = 1:2,
               points = cluster{i};
               info = clusterInfo(i,:);
               [mileageLength range] = calculatesolefeatures(points,info);
               if mileageLength>30&&range>1,
                   isBoundaryArray(i,1) = true;
               end
           end
       elseif nCluster>=3,
           %对于第一条聚类线，其前面没有数据，把本身数据作为前一条数据，这样方便处理
           adjacentCluster(1:3) =[{[]} cluster(1:2)]; 
           adjacentClusterInfo = [clusterInfo(1,:); clusterInfo(1:2,:)];
           multiFeaturesArray = zeros(nCluster,4);
           multiFeaturesArray(1,:) = calculatesmultifeatures(adjacentCluster,adjacentClusterInfo);
           for iCluster = 2:nCluster-1,
               adjacentCluster(1:3) =[cluster(iCluster-1:iCluster+1)]; 
               adjacentClusterInfo = [clusterInfo(iCluster-1:iCluster+1,:)];
               multiFeaturesArray(iCluster,:) = calculatesmultifeatures(adjacentCluster,adjacentClusterInfo);
           end
           adjacentCluster(1:3) =[cluster(nCluster-1:nCluster) {[]}];
           adjacentClusterInfo = [clusterInfo(nCluster-1:nCluster,:);clusterInfo(nCluster,:)];
           multiFeaturesArray(nCluster,:) = calculatesmultifeatures(adjacentCluster,adjacentClusterInfo);
           isBoundaryArray = checkBoundary(multiFeaturesArray);   
       end
       isBoundaryInfoArray(iBoundary) = {isBoundaryArray};
       for i = 1:nCluster,
           points = cluster{i};
           x = points(:,1);
           y = points(:,2);
           if isBoundaryArray(i),
               plot(x,y,'g.');hold on
           else
               plot(x,y,'r.');
           end
       end
       
       a=0;
    end
end

function isBoundaryArray = checkBoundary(multiFeaturesArray)
%determine boundary  
    nFeatures = size(multiFeaturesArray,1);
    isBoundaryArray = zeros(nFeatures,1);
    for i=1:nFeatures,
        dRangeL = multiFeaturesArray(i,1);
        dRangeR = multiFeaturesArray(i,2);
        length = multiFeaturesArray(i,3);
        range= multiFeaturesArray(i,4);
        if length>60||dRangeL>=1||dRangeR>=1,
            %长度大于60或者是左右凸出的认为是边界
            isBoundaryArray(i,1) = true;
        elseif dRangeL<-1&&dRangeR<-1,
            continue;
        else      
        end   
    end
    isBoundaryArray = checkBoundary2(isBoundaryArray,multiFeaturesArray);
end

function isBoundaryArray = checkBoundary2(isBoundaryArray,multiFeaturesArray)
%determine boundary by relative type of adjacent clusters 
%在初步边界判断结果基础上进行进一步判断
    nBoundary = size(isBoundaryArray,1);
    bunch.active = false;
    bunch.start = -1;
    bunch.end = -1;
    bunch.length = 0;
    for i =1:nBoundary-1,
        dRangeR = abs(multiFeaturesArray(i,2));
        length = multiFeaturesArray(i,3);
        isBoundary = isBoundaryArray(i);
        isBoundaryR = isBoundaryArray(i+1);
        if bunch.active,
        %bunch处于激活状态，说明当前是非边界，且和上一个有关系，即他们都是非边界且相聚距离小于1    
            if dRangeR<1&&~isBoundaryR,
                %与下一个距离小于1,且下一个也不是边界，bunch更新后继续处于激活状态
                bunch.end = i;
                bunch.length = bunch.length+length;
            elseif dRangeR<1&&isBoundaryR,
                %与下一个距离小于1,且下一个是边界，将bunch中都置为边界，关闭bunch。
                isBoundaryArray(bunch.start:i) = isBoundaryR;
                bunch.active = false;
                bunch.start = -1;
                bunch.end = -1;
                bunch.length = 0;
            else dRangeR>1,
                %结束bunch累计，由bunch累计的数据判断类型，关闭bunch。
                if bunch.length>50,
                    isBoundaryArray(bunch.start:i) = isBoundaryR;
                end
                bunch.active = false;
                bunch.start = -1;
                bunch.end = -1;
                bunch.length = 0;
            end
        else
        %bunch未激活
            if ~isBoundary,
                if dRangeR<1&&isBoundaryR,
                    %当前不是边界，下一个是边界，当前与下一个距离小于1，则认为当前也是边界
                    isBoundaryArray(i) = isBoundaryR;
                elseif dRangeR<1&&~isBoundaryR,
                    %当前不是边界，下一个不是边界，当前与下一个距离小于1，将当前与下一个性质捆绑，后面一起进行判断
                    bunch.active = 1;%激活bunch变量
                    bunch.start = i;
                    bunch.end = i;
                    bunch.length = bunch.length + length;
                end
            end
        end
    end
end

function [mileageLength range] =  calculatesolefeatures(points,info)
%calculate self features of cluster
%计算聚类线自特征，包括里程长度、到轨迹线距离
%注意：后面也可以添加别的自特征，如聚类线中点的离散程度、直线斜率等
    mileageStart = info(1,1);
    mileageEnd = info(1,3);
    mileageLength = abs(mileageEnd-mileageStart);%用里程计算的每个聚类线的长度
    rangeStart  = info(1,2);
    rangeEnd = info(1,4);
    range = (rangeStart+rangeEnd)/2;
end

function multiFeatures =  calculatesmultifeatures(adjacentCluster,infoArray)
%calculate relative features of clusters
%clusterArray是三个相邻的聚类线，对中间的聚类线进行相关特征计算
%这里暂时只考虑相邻的三条聚类线相互关系，后面可添加更多全局关系
%这里兼容了calculatesolefeatures功能
    preCluster = adjacentCluster{1};
    midCluster = adjacentCluster{2};
    nextCluster = adjacentCluster{3};
    preInfo = infoArray(1,:);
    midInfo = infoArray(2,:);
    nextInfo = infoArray(3,:);
    dRangeL = midInfo(1,2) - preInfo(1,4);
    dRangeR = midInfo(1,4) - nextInfo(1,2);
    [midMileageLength midRange] = calculatesolefeatures(midCluster,midInfo);
    multiFeatures = [dRangeL dRangeR midMileageLength midRange];  
end

function [rangeInfo nFlag]= distpoint2trace(point,mileageArray,nFlag)
%calculate distance between the point and trace.
%因为边界是随着里程增长的，所以从第nFlag个点附近开始检索，减少不必要的运算
        xp =point(1,1);
        yp =point(1,2);
        hp =point(1,3);
    nMileage = size(mileageArray,1);
    backNum = 100;%往回多检索个数
    if nFlag<backNum+1,
        nFlag = backNum+1;
    end
    nFlag = backNum+1;
    %rangeArray内容为[mx my mh mileage range],mx、my、mz为里程点坐标,mileage为里程，range为点到轨迹点距离
    rangeArray = zeros(nMileage-nFlag+backNum+1,6);
    num = 0; 
    for i=nFlag-backNum:nMileage,
        num = num+1;
        xMileage = mileageArray(i,1);
        yMileage = mileageArray(i,2);
        hMileage = mileageArray(i,3);
        range = norm([xMileage-xp yMileage-yp hMileage-hp]);   
        rangeArray(num,:) = [mileageArray(num,1:4) range i];
    end
    rangeArray = sortrows(rangeArray,5);
    nFlag = rangeArray(1,6);
    rangeInfo = rangeArray(1,1:5);
end

function rangeInfo= distpoint2trace2(point,mileageArray)
%calculate distance between the point and trace.
%代替distpoint2trace,
    %rangeInfo内容为[mx my mh mileage range],mx、my、mz为里程点坐标,mileage为里程，range为点到轨迹点距离
    [IDX, range]= knnsearch(mileageArray(:,1:2),point(1:2));%反复建立搜索树，效率较低，懒得改了
    rangeInfo = [mileageArray(IDX,1:4) range];
end

function mileageArray = createmileage(traceData)
%create mileage
%mileage存储里程点坐标与里程数
%more都是三维点
    intervalPoint = 10;%每10个点计算一次里程点
    nTracePoint = size(traceData,1);
    nMileage = floor(nTracePoint/intervalPoint);
    mileage = 0;
    mileageArray = zeros(nMileage,5);
    for i = 1:nMileage,
        xStart = traceData((i-1)*intervalPoint+1,1);
        yStart = traceData((i-1)*intervalPoint+1,2);
        hStart = traceData((i-1)*intervalPoint+1,3);
        xEnd = traceData(i*intervalPoint,1);
        yEnd = traceData(i*intervalPoint,2);
        hEnd = traceData(i*intervalPoint,3);
        dist = norm([xStart-xEnd yStart-yEnd hStart-hEnd]);
        mileage = mileage+dist;
        mileageArray(i,:) = [xEnd yEnd hEnd mileage atan2(yStart-yEnd,xStart-xEnd)];
    end
end

% function clusterArray = clusterpoint(pointData,breakDist)
% %cluster ordered points by distance between two adjacent points
%     clusterArray = {};
%     nCluster = 0;
%     [nPoint dim] = size(pointData);
%     if nPoint==0,
%         return;
%     elseif nPoint==1,
%         clusterArray(1) = {pointData};
%         return;
%     end
%     breakStart = 1;
%     breakEnd = 1;
%     if dim==2,
%         for iPoint = 2:nPoint,
%             breakEnd = iPoint;
%             xPre = pointData(iPoint-1,1);
%             yPre = pointData(iPoint-1,2);
%             x = pointData(iPoint,1);
%             y = pointData(iPoint,2);
%             dist = norm([xPre-x yPre-y]);
%             if dist>=breakDist,
%                 nCluster = nCluster+1;
%                 clusterArray(nCluster) = {pointData(breakStart:breakEnd-1,:)};
%                 breakStart = breakEnd;
%             end
%         end
%     elseif dim==3,
%         for iPoint = 2:nPoint,
%             breakEnd = iPoint;
%             xPre = pointData(iPoint-1,1);
%             yPre = pointData(iPoint-1,2);
%             hPre = pointData(iPoint-1,3);
%             x = pointData(iPoint,1);
%             y = pointData(iPoint,2);
%             h = pointData(iPoint,3);
%             dist = norm([xPre-x yPre-y hPre-h]);
%             if dist>=breakDist,
%                 nCluster = nCluster+1;
%                 clusterData = pointData(breakStart:breakEnd-1,:);
%                 clusterArray(nCluster) = {clusterData};
%                 breakStart = breakEnd;
%             end
%         end
%     else
%         return;
%     end
%     clusterArray(nCluster+1) = {pointData(breakStart:nPoint,:)};
% end

function drawpoint(points,type)
    x = points(:,1);
    y = points(:,2);
    plot(x(1:end),y(1:end),type);
    axis equal;
end

function boundaryArray = getroadboundarypoint(SliceArray)
%get filtered boundary points
    nFiled = size(fieldnames(SliceArray),1);
    nSliceArray = size(SliceArray,2);
    boundaryPointArray = zeros([nSliceArray nFiled*2]);
    for i=1:nSliceArray,
        x = SliceArray(i).x;
        y = SliceArray(i).y;
        h = SliceArray(i).h;
        ins = SliceArray(i).ins;
        nPoint = size(x,1);
        x1 = x(1);
        y1 = y(1);
        h1 = h(1);
        ins1 = ins(1);
        xn = x(nPoint);
        yn = y(nPoint);
        hn = h(nPoint);
        insn = ins(nPoint);      
        boundaryPointArray(i,1:nFiled*2) = [x1 y1 h1 ins1 xn yn hn insn];%扫描线的首尾点
    end
    x1 = boundaryPointArray(:,1);
    y1 = boundaryPointArray(:,2);
    h1 = boundaryPointArray(:,3);
    x2 = boundaryPointArray(:,5);
    y2 = boundaryPointArray(:,6);
    h2 = boundaryPointArray(:,7);   
    BoundaryPointArray1 = filterpoint([x1 y1 h1],0.5,10);
    data = BoundaryPointArray1.data;
    info = BoundaryPointArray1.info;
    [nBoundaryPoint,dim]  = size(data);
    num = 0;
    boundary1 = zeros(nBoundaryPoint,dim);
    for i=1:nBoundaryPoint,
        if info(i),
            %提取符合要求边界点
            num = num+1;
            boundary1(num,:) = data(i,:);
        end
    end
    boundaryArray(1) = {boundary1(1:num,:)};
    BoundaryPointArray2 = filterpoint([x2 y2 h2],0.5,10);
    data = BoundaryPointArray2.data;
    info = BoundaryPointArray2.info;
    [nBoundaryPoint,dim]  = size(data);
    num = 0;
    boundary2 = zeros(nBoundaryPoint,dim);
    for i=1:nBoundaryPoint,
        if info(i),
            num = num+1;
            boundary2(num,:) = data(i,:);
        end
    end
    boundaryArray(2) = {boundary2(1:num,:)};
%     plot(x(1:end),y(1:end),'g-');
%     plot(x(1:end),y(1:end),'g.');
%     hold on
%     axis equal;
end


function FilteredPoint = filterpoint(pointData,distanceLimit,nIsolate)
% filter point by setting flag value as 0 if the distance of previous point
% and next point to current point are both greater than the limit parameter.
% 这里的pointData应该是有序线状点云
%为了保存边界点是成对性质，滤掉的点不删除而是info属性值置为0,有时一次滤波不完
%全需要二次迭代滤波
    filteredPoint = pointData;
    [nPoint col] = size(pointData);
    info = ones([nPoint 1]);
    %滤除掉距离其他点群距离大于distanceLimit且个数不超过nIsolate个的孤立点群
%     distanceLimit = 0.5;
%     nIsolate = 1;
    x1 = pointData(1:end-1,1);
    y1 = pointData(1:end-1,2);
    h1 = pointData(1:end-1,3);
    x2 = pointData(2:end,1);
    y2 = pointData(2:end,2);
    h2 = pointData(2:end,3);
    ds = sqrt((x1-x2).^2+(y1-y2).^2+(h1-h2).^2);  
    nDs = nPoint-1;
    nVertex = 0;
    for i = 1:nDs,
        if ds(i)>distanceLimit,
            nVertex = nVertex+1;
            vertexInfoArray(nVertex,1) = i;
            vertexInfoArray(nVertex,2) = ds(i);
        end
    end
    for i = 1:nVertex-1,
        startVertex = vertexInfoArray(i,1);
        endVertex = vertexInfoArray(i+1,1);
        if  (endVertex -startVertex)<=nIsolate,
            info(startVertex+1:endVertex) = 0;
        end
    end 
    if vertexInfoArray(1,2)>distanceLimit,
        info(1) = 0;
    end
    if vertexInfoArray(nVertex,2)>distanceLimit,
        info(nPoint) = 0;
    end
    FilteredPoint.data = filteredPoint;
    FilteredPoint.info = info;
end



