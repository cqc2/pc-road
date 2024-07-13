function [newCluster,removedCluster] = removevertical(cluster,tracedata,limit_A)
%去掉矩形方向与轨迹线不近似平行呃的聚类
if ~isfield(cluster,'center')
    cluster = getclusterinfo(cluster);
end 
Mdl = KDTreeSearcher(tracedata(:,1:2));
nNewCluster = 0;
nRemoved = 0;
for i=1:size(cluster,2)
   c =  cluster(i).center;
   [Idx,D] = rangesearch(Mdl,c,30);
   Idx = Idx{1};
   angle1 = cluster(i).angle;
   isremoved = true;%标记量
   if size(Idx,2)>=2%计算轨迹方向至少需要2个点
       dx =  tracedata(Idx(1),1)-tracedata(Idx(2),1);
       dy = tracedata(Idx(1),2)-tracedata(Idx(2),2);
       angle2 = atand(dy/dx);%轨迹角度
       if angle2<0
          angle2 = angle2+180;%角度换算到0~180
       end
       if abs(angle1-angle2)<=limit_A
           nNewCluster = nNewCluster+1;
           newCluster(nNewCluster) =  cluster(i);
           isremoved = false;
       end
   end
   if isremoved
       nRemoved = nRemoved+1;
       removedCluster(nRemoved) =  cluster(i);
   end
end
if isempty(newCluster)
    newCluster = [];
end
end