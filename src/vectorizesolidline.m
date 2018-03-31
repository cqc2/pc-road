function [cluster1,breakLines1,remainCluster] = vectorizesolidline(cluster,tracedata)
%
%对实线进行矢量化
%注意cluster是否已经重聚类

%计算聚类信息
if ~isfield(cluster,'center')
    cluster = getclusterinfo(cluster);
end
% [cluster,removedCluster1] = recluster(cluster);
% savecluster2file([],cluster,'erqi_bigpaper_4solidline_5m');
[newCluster,removedCluster2]= removevertical(cluster,tracedata,45);
[cluster1,breakLines1] = vectorizebreakline_for_solid(newCluster);%提取主要实线
[removedCluster3]= removelines(cluster1,breakLines1);
remainCluster = mergecluster(mergecluster(removedCluster1,removedCluster2),removedCluster3);

% cluster2 = removelines(cluster1,breakLines1);
% [cluster2,breakLines2] = vectorizebreakline_for_solid2(cluster2);%提取模糊的实线
% for i=1:size(newCluster,2 )
%     data = newCluster(i).data;
%     
% end
% a=0;
end


function [newCluster,uselesscluster] = recluster(cluster,len,r)
%对聚类点云进行重聚类
% uselesscluster - 没有参与重聚类的聚落

if ~exist('len','var')||isempty(len), len=5;end
if ~exist('r','var')||isempty(r), r = 0.15;end

nCluster = size(cluster,2);
nNewCluster = 0;
nUselessNewCluster = 0;
for iCluster = 1:nCluster
    data = cluster(iCluster).data;
%     plot(data(:,1),data(:,2),'.','Color',[rand rand rand]);hold on;
%     continue;
    np = size(data,1);
    len = cluster(iCluster).length;
    if len>5
        maxp = ceil(np/(len/5));
        r = 0.15;
        index = clustereuclid(data(:,1:2),r,maxp);
        C = unique(index);
        for i = 1:size(C,1)
            X = data(index==C(i),:);
            nNewCluster = nNewCluster+1;
            newCluster(nNewCluster).data = X;
%             plot(X(:,1),X(:,2),'.','Color',[rand rand rand]);hold on;axis equal;
        end   
    else
        nUselessNewCluster = nUselessNewCluster+1;
        uselesscluster(nUselessNewCluster) = cluster(iCluster);
%         nNewCluster = nNewCluster+1;
%         newCluster(nNewCluster).data = data;
%         figure(1);  plot(data(:,1),data(:,2),'.','Color',[rand rand rand]);hold on;axis equal;
    end   
end
end
