function vectorizemarking()
%
%
addpath(genpath(pwd));%将当前目录文件夹及其子目录添加进文件夹
% clusterDir = 'CASEDATA\YingwuzhouBridge\segMarking2\';
% clusterDir = 'CASEDATA\FazhanAvenue\segMarking2\';
% clusterDir = 'CASEDATA\ErqiBridge\segMarking\';
% clusterDir = 'CASEDATA\MoshuiLake\segMarking2\';
% clusterDir = 'CASEDATA\YingwuzhouBridge\segMarking2\';
clusterDir = 'CASEDATA\test\'; %不分区去掉虚线
% clusterDir = 'CASEDATA\all-withut-solidline-5m重聚类\'; %不分区去掉虚线 重聚类
% clusterDir = 'CASEDATA\FazhanAvenue\test_zhiyoushixian\'; %分区分割的实线，噪声较多
% clusterDir = 'CASEDATA\ErqiBridge\test_zhiyoushixian\'; %分区分割的实线，噪声较多
% clusterDir = 'CASEDATA\cluster4guideline\';
% clusterDir = 'CASEDATA\ErqiBridge\erqi_bigpaper\';
posFilePath1 ='CASEDATA\ErqiBridge\tracedata\road-all_GNSStrace.xyz';
posFilePath2 ='CASEDATA\FazhanAvenue\tracedata\road-all_GNSStrace.xyz';
posFilePath3 ='CASEDATA\MoshuiLake\tracedata\road-all_GNSStrace.xyz';
tracedata1 = importdata(posFilePath1);
tracedata2 = importdata(posFilePath2);
tracedata3 = importdata(posFilePath3);
tracedata = [tracedata1;tracedata2;tracedata3];

dirInfo = dir(clusterDir);
nfile = size(dirInfo,1);
% cluster =  struct('index',[],'data',[]);
cluster = [];
r = rand;
g = rand;
b = rand;
color = [r g b ];
for i=3:nfile
    fileame = strcat(clusterDir,dirInfo(i).name);
%      cluster = readcluster('test_marking_seg.txt');
    newdata = readcluster(fileame);
    ndata = size(newdata,2);
    nCluster = size(cluster,2);
    newdata = getclusterinfo(newdata);
    for m = 1:ndata
%         r = rand;
        g = rand;
        b = rand;
        color = [r g b];
        cluster(nCluster+m).data = newdata(m).data;
        data = newdata(m).data;
        figure(3); 
        plot(data(:,1),data(:,2),'.','Color',color);
        hold on;axis equal
%         rx = newdata(m).rectx;
%         ry = newdata(m).recty;
%         plot(rx,ry,'-','Color',[0 0 0]);
    end    
end
cluster = getclusterinfo(cluster,tracedata);
% [cluster,breakLines] = vectorizebreakline(cluster);
% cluster4solid = removelines(cluster,breakLines);
% savecluster2file([],cluster,'erqi_without_solidline');
[cluster,breakLines,~] = vectorizesolidline(cluster,tracedata);
% for i=1:size(cluster4guidline,2 )
%     data = cluster4guidline(i).data;
%     color = [rand rand rand];
%      plot(data(:,1),data(:,2),'.','Color',color); hold on;axis equal
% end
% savecluster2file([],cluster4guidline,'MoshuiLake_cluster4guidline');
% cluster = removelines(cluster,breakLines);
%  cluster = vectorizeguidline(cluster,tracedata);
% [cluster,arrows] = vectorizearrow(cluster);
 
for i=1:size(breakLines,1)
    idx = breakLines(i);
    r = rand;
    g = rand;
    b = rand;
    color = [r g b];
    data = cluster(idx).data;
    rx = data(:,1);
    ry = data(:,2);
    plot(rx,ry,'-','Color',color);hold on;axis equal;
    continue;
    for m = 1:size(idx,2)
%         data = cluster(idx(m)).data;
%         figure(3); plot(data(:,1),data(:,2),'.','Color',color);hold on;axis equal
        if m==size(idx,2)
            break;
        end
            c1 = cluster(idx(m)).center;
        c2 = cluster(idx(m+1)).center;
        figure(3); plot([c1(1) c2(1)],[c1(2) c2(2)],'-','Color',color);hold on;axis equal
        rx = cluster(idx(m)).rectx;
        ry = cluster(idx(m)).recty;
        plot(rx,ry,'-','Color',color);
        
    end
end

end


















