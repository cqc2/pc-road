function cluster = vectorizeguidline(cluster,tracedata)
%
% 从点云聚类中识别导流线
% 目前算法只使用面积、矩形度等属性即取得较好效果，后期如有需要可进一步分析详细的形状

if ~isfield(cluster,'center')
    cluster = getclusterinfo(cluster,tracedata);
end
nc = size(cluster,2);

%由特征属性进行初步筛选
sVLimit = 99999;%矩形度阈值
sLimit = 0;%面积阈值
lwLimit = 20;%长宽比阈值
index_p = [];
center = [];
tmpdata = [];
for i = 1:nc
    sV = cluster(i).squareValue;
    s = cluster(i).rectArea;
    lwRate =  cluster(i).lenWidthRate;
    a2trace = cluster(i).angle2trace;
    w = cluster(i).width;
    pA = cluster(i).perctA;
    
    if (sV > sVLimit)||s<sLimit||lwRate>lwLimit||cluster(i).width<0.1
        continue;
    end
    if a2trace>70&&lwRate>10
            continue;
    end
    if a2trace>80&&cluster(i).width<0.45
        continue;
    end
    if cluster(i).perctA<0.118
        continue;
    end
    index_p = [index_p i];
%     center = [center;cluster(i).center];
%     tmpdata = [tmpdata;cluster(i).data];
%     pcd = cluster(i).data;
%     x = pcd(:,1);
%     y = pcd(:,2);
%     figure(12); plot(x,y,'.','Color',[rand rand rand]);hold on;axis equal;
%     plot(x,y,'g.');hold on;axis equal;
%         shp = alphaShape(pcd(:,1),pcd(:,2),10);
%     plot(shp);
%     j = boundary(x,y,0.1);
%     hold on;
%     plot(x(j),y(j),'r-','LineWidth',1);

end

%对筛选结果按中心点进行欧式聚类并去除元素个数较少的聚类结果
index_c = cluster_2(cluster,index_p,10,1.5);
[C,~,~] = unique(index_c);%统计一共聚集成几类
ic = 0;
for i=1:size(C,1)
    idx =  find(index_c==C(i));
% %     if size(idx,1)<4  %导流线一般由多个折线构成，所以个数少于3个的认为不是导流线
% %         continue;
% %     end
    ic = ic+1;
    cidx(ic) = {index_p(idx)};
    r = rand;g = rand;b = rand;
    m = cidx{ic};
    data_tmp = [];
    
    %去除文字
    loop = 1;
    for k = 1:size(idx)
        w = cluster(m(k)).width;
        len = cluster(m(k)).length;
        li = 0.2;
        if (w>(2-li))&&(w<2+li)&&(len>6-li)&&(len<6+li)
            loop = 0;
            continue;
        end
    end
    if ~loop
        continue;
    end    
    
    for k = 1:size(idx)
        cluster(m(k)).label = 2;
        data = cluster(m(k)).data;
%          plot(data(:,1),data(:,2),'.','Color',[r g b]);hold on;axis equal;
        data_tmp = [data_tmp;data];
    end
%    figure(1);   plot(data_tmp(:,1),data_tmp(:,2),'.','Color',[r g b]);hold on;axis equal;
    cluster_tmp.data = data_tmp;
    cluster_tmp = getclusterinfo(cluster_tmp);
    if cluster_tmp(1).length>10&&cluster_tmp(1).area>4&&cluster_tmp(1).perctA>0.05&&cluster_tmp(1).width<5.5
      figure(1);    plot(data_tmp(:,1),data_tmp(:,2),'.','Color',[r g b]);hold on;axis equal;
    end
end

%这个处理效果还是跟图像分割有很大关系，应该想办法在图像分割时就提高正确率
end

function out_idx = cluster_2(cluser,index_p,step,dist)
% 对cluster重新聚类，matlab直接聚类太慢，这里先抽稀在聚类
% cluser - 
% index_p - 要被重聚类的聚落在cluser中的索引号
%
% 需要注意的抽稀后聚类之间间距会变大
nc = size(index_p,2);
data2 = [];
for i=1:nc
    data = cluser(index_p(i)).data;
    data_tmp = data(1:step:end,:);%10倍抽稀
    np = size(data_tmp,1);
    data2 = [data2;[data_tmp i.*ones([np 1])]];
end
idx_c = clustereuclid(data2(:,1:2),dist);
[C,~,~] = unique(idx_c);%统计一共聚集成几类
for i=1:size(C,1)
    idx =  find(idx_c==C(i));
    idx_2 = data2(idx,end);
    [C2,~,~] = unique(idx_2);%C2存储的是cluser的编号，这些编号是一大类
    out_idx(C2,1) = i;
end
end

