function cluster = getclusterinfo(cluster,tracedata)
%
% 计算点云聚类的描述性信息，使用这些信息对可以对聚类进行更高层次的抽象以便做进
% 一步的处理。
%
% INPUT：
% cluster - 点云聚类结构体
% 
% OUTPUT：
% cluster - 拥有相关描述性属性字段的点云聚类结构体
%
% STRUCT：
% data - 点云数据
% isLabel - 是否已经标记过，false or true
% label - 点云类别，默认0
% rectx - 外接矩形的横坐标，其中首尾是同一个点，所以有5个数据
% recty - 外界矩形的纵坐标
% center - 外接矩形中心
% rectArea - 外接矩形面积
% perimeter - 外接矩形周长
% length - 外接矩形较长边
% width - 外接矩形较短边
% pointArea - 以格网个数近似计算的点云面积 
% perctA - 点云面积占其外接矩形面积比例
% density - 点密度
% direction - 矩形方向向量，表示了矩形的姿态（不分方向），模的大小表示了呈矩形的程度
% squareValue - 自定义矩形度，不是长宽比，也不是常规的矩形度
% angle - 与x轴夹角（0~180）
% lenWidthRate - 长宽比
% n - 点个数
% angle2trace - 聚落朝向与轨迹夹角
% len2trace - 聚落中心到轨迹距离

isCaltraceinfo = false;
if exist('tracedata','var')&&~isempty(tracedata)
    Mdl = KDTreeSearcher(tracedata(:,1:2));
   isCaltraceinfo = true;
end
    nc  = size(cluster,2);%聚类个数
for i=1:nc
    data = cluster(i).data;
    cluster(i).isLabel = false;
    cluster(i).label = 0;%-1表示类别还未知
    x = data(:,1);
    y = data(:,2);
    n = size(x,1);
    try
    [rectx,recty,rectArea,perimeter] = minboundrect(data(:,1),data(:,2));
    catch ME
        rectx =[0 0 0 0 0]';
        recty =[0 0 0 0 0]';
        rectArea = 0;
        perimeter =0;
    end
    if n<3
        % 当点个数不到3个时minboundrect会出现一些bug
        perimeter=perimeter(1,1);
    end
    cluster(i).rectx = rectx;
    cluster(i).recty = recty;
    cx = mean(rectx);
    cy = mean(recty);
    cluster(i).center = [cx cy];%外接矩形中心
    cluster(i).rectArea = rectArea;%最小外接矩形面积
    cluster(i).perimeter = perimeter;
    d = sqrt((rectx(1:2) - rectx(2:3)).^2+(recty(1:2) - recty(2:3)).^2);%外接矩形边长
    cluster(i).length = max(d);
    cluster(i).width = min(d);
    areaSeed = unique([roundn(x,-1) roundn(y,-1)],'rows');%坐标对应到0.1米的格网中
    pointArea = 0.1*0.1*size(areaSeed,1)-0.5*perimeter*0.1;%以格网个数近似计算点云面积 
    cluster(i).area = pointArea;
    cluster(i).n = n;%点个数
    cluster(i).perctA = pointArea/rectArea;%矩形度
    cluster(i).density = n/rectArea;%点密度
    
    %矩形方向向量，表示了矩形的姿态（部分方向），模的大小表示了呈矩形的程度
    if d(1)==d(2)
        cluster(i).direction = [0,0];%正方形向量模为0
    elseif d(1)>d(2)
         cluster(i).direction = [(rectx(1) - rectx(2))/d(1),(recty(1) - recty(2))/d(1)].*(abs(d(1)/d(2)-1));   
    elseif d(1)<d(2)
        cluster(i).direction = [(rectx(3) - rectx(2))/d(2),(recty(3) - recty(2))/d(2)].*(abs(d(2)/d(1)-1));   
    end
    if cluster(i).direction(2)<0
        cluster(i).direction = -cluster(i).direction;
    end 
    cluster(i).squareValue = cluster(i).direction*(cluster(i).direction)';%矩形度
    cluster(i).angle = atand(cluster(i).direction(2)/cluster(i).direction(1));
    if cluster(i).angle<0
        cluster(i).angle = cluster(i).angle+180;%角度换算到0~180
    end
    cluster(i).lenWidthRate = cluster(i).length/cluster(i).width;
    
    if isCaltraceinfo
        c =  cluster(i).center;
        [Idx,D] = rangesearch(Mdl,c,30);
        Idx = Idx{1};
        D =  D{1};
        angle1 = cluster(i).angle;
        if size(Idx,2)>=2%计算轨迹方向至少需要2个点
            dx =  tracedata(Idx(1),1)-tracedata(Idx(2),1);
            dy = tracedata(Idx(1),2)-tracedata(Idx(2),2);
            angle2 = atand(dy/dx);%轨迹角度
            if angle2<0
                angle2 = angle2+180;%角度换算到0~180
            end
            angle12 = abs(angle1-angle2);
            if angle12>90
                angle12 = 180-angle12;%夹角换算道0~90
            end
           cluster(i).angle2trace = angle12;
           cluster(i).len2trace = (D(1)+D(2))/2;
        end
        
    else
        cluster(i).angle2trace = [];
        cluster(i).len2trace = [];
    end
%     plot(rectx,recty,'-');hold on;
%     plot(x,y,'.');hold on;
%     axis equal;
end
end