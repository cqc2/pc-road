function getroadpoint
%getroadpoint - get road point from sliced cloud point.
%注意：大量计算时一定要把画图语句注释掉，否则会很慢！
datetime('now','TimeZone','local','Format','HH:mm:ss Z')
%     pointCloudFilePath = 'RawPointCloudData\#2Final_Point_Cloud_Data.xyz';
    pointCloudFilePath = 'dataspace_wuhan\data-20170418-085955.las';
    lasdata = LASreadAll(pointCloudFilePath);
    data = [lasdata.x lasdata.y lasdata.z lasdata.intensity];
    saveRoadFileName = 'data-20170418-085955_road';
    %注意：当使用切片法时，分次读入数据可能会一个扫描仪数据读完才会都下一个，使只对一条扫描仪数据切片，而不是同时对所有数据切片
    nPointEachTime = 1000000;%一次处理点的个数
    rows = size(data,1); 
    nData = ceil(rows/nPointEachTime);
    PointSet= struct('x',0,'y',0,'h',0,'ins',0);
    Road=repmat(PointSet,[1 nData]);
    pData = Road;
    for iData = 1:nData
        if iData == nData
            startindex = (iData-1)*nPointEachTime+1;
            pData(iData).x = data(startindex:end,1);
            pData(iData).y = data(startindex:end,2);
            pData(iData).h = data(startindex:end,3);
            pData(iData).ins = data(startindex:end,4);
        else
            startindex = (iData-1)*nPointEachTime+1;
            endindex = iData*nPointEachTime;
            pData(iData).x = data(startindex:endindex,1);
            pData(iData).y = data(startindex:endindex,2);
            pData(iData).h = data(startindex:endindex,3);
            pData(iData).ins = data(startindex:endindex,4);
        end
    end    
    PointSet= struct('x',0,'y',0,'h',0,'ins',0);
    Road=repmat(PointSet,[1 nData]);
    %---------------------------
    %创建并行池
    if isempty(gcp('nocreate'))
        parpool('local',4);
    end
    %--------------------------
    datetime('now','TimeZone','local','Format','HH:mm:ss Z')
    parfor iData = 1:nData
        pointCloudData = pData(iData);
        pointCloudData = [pointCloudData.x pointCloudData.y pointCloudData.h pointCloudData.ins];
        ScanLineArray = slice(pointCloudData);
        roadPointData = getroadbyscanline(ScanLineArray);
        Road(iData).x = roadPointData(:,1);
        Road(iData).y = roadPointData(:,2);
        Road(iData).h = roadPointData(:,3);
        Road(iData).ins = roadPointData(:,4);
    end
    datetime('now','TimeZone','local','Format','HH:mm:ss Z')
    points =  getpointfromstructArray(Road);
     savepointcloud2file(points,saveRoadFileName,false);
    datetime('now','TimeZone','local','Format','HH:mm:ss Z')
end
