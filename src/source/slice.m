  function  SliceArray = slice(pointCloudData,TYPE,posData,sliceInfo)
%slice - Sort in ascending or descending order.
% datetime('now','TimeZone','local','Format','HH:mm:ss Z')
%     pointCloudFilePath = 'thinnedpointdata.xyz';
%      posFilePath = 'POS_#2_20141114.txt';
%      pointCloudData = readpointcloudfile(pointCloudFilePath);

 if ~exist('TYPE','var') || isempty(TYPE), TYPE = 'scanlines'; end%默认使用扫描线法切割
 if ~exist('width','var'), width =[]; end
 if strcmp(TYPE,'scanlines')
     if ~exist('sliceInfo','var')
         sliceInfo = []; 
     end
     ScanLineArray = slice2scanlines(pointCloudData,sliceInfo);
     SliceArray = ScanLineArray;
 elseif strcmp(TYPE,'bypos')
     separation = sliceInfo(1);
     thickness = sliceInfo(2);
     if size(sliceInfo)==3
         width = sliceInfo(3);
     else
         width = [];
     end
     SliceArray = slicebypos(pointCloudData,posData,separation,thickness,width);
 elseif strcmp(TYPE,'direction')
      sliceX = sliceInfo(1);
      sliceY = sliceInfo(2);
      sliceK = sliceInfo(3);
      cutLenght = sliceInfo(4);
      SliceArray = slicebydirection(pointCloudData,sliceX,sliceY,sliceK,cutLenght);
  elseif strcmp(TYPE,'subarea')
      sliceX = sliceInfo(1);
      sliceY = sliceInfo(2);
      sliceK = sliceInfo(3);
      cutLocation = sliceInfo(4:end);
      SliceArray = slice2subarea(pointCloudData,sliceX,sliceY,sliceK,cutLocation);
 else
     error('invalid TYPE!');   
 end   
     
%      pointCloudFilePath = 'dataspace\#2Final_Point_Cloud_Data.xyz';
%     pointData = readpointcloudfile2(pointCloudFilePath);
%      tracePointData = searchtracepoint(pointData);
%      savepointcloud2file(tracePointData,'dataspace\traceData_notfit',false);
%      outputslices2file(ScanLineArray(1:200));
%     traceData = computetraceline(pointCloudData);
%     save traceData.xyz traceData -ascii;
% A = importdata('traceData - Copy.xyz');
% B = importdata('traceData.xyz');
%      posData = readposfile(posFilePath);
%      posXYData = [posData.x posData.y posData.elevation];
%      a=0;
%      save RawPointCloudData/posXYData.xyz posXYData -ascii;
%       thinnedPointData = thinpointdata(pointCloudData,10);
%      savedata = pointCloudData(1:3000,1:end);
%      save thinnedlittledata.xyz thinnedPointData -ascii;
%     separation=0;
%     thickness=0.5; 
%     ThinnedSliceArray = sliceonpos(thinnedPointData,posData,separation,thickness);
%     SliceArray = slicebythinned(pointCloudData,ThinnedSliceArray);
%     SliceArray = slicebypos(pointCloudData,posData,separation,thickness);
%     outputslices2file(SliceArray);
%     SliceArray = slicebythinned(pointCloudData,ThinnedSliceArray,10);
%     outputslices2file(SliceArray);
% datetime('now','TimeZone','local','Format','HH:mm:ss Z')
  end

function SliceArray = slice2subarea(pointData,sliceX,sliceY,sliceK,cutLocation)
% slice pointdata along given direction into several regions
% -cutInfo:切割点信息，包括切割点坐标、切割线方向
% -cutLenght:切割厚度
PointSet= struct('data',[],'range',[]);
SliceArray=repmat(PointSet,[1 50]);
x = pointData(:,1);
y = pointData(:,2);
%对点云进行旋转
A = pi/2-sliceK;%旋转角

%将坐标转换到以切点为原点，切线为纵坐标点的坐标系中
x0= (x - sliceX).*cos(A) - (y - sliceY).*sin(A) ;%逆时针旋转A
y0= (x - sliceX).*sin(A) + (y - sliceY).*cos(A) ;
data = [pointData x0 y0];

%切片
nSlice = size(cutLocation,2)/2;
for iSlice=1:nSlice
    locL = cutLocation(2*iSlice-1);
    locR = cutLocation(2*iSlice);
    tempData = data(data(:,end-1)<=locR&data(:,end-1)>locL,1:end-2);
    SliceArray(iSlice).data = tempData;
    SliceArray(iSlice).range = [locL,locR];
end
SliceArray = SliceArray(1:nSlice);
end
  
function SliceArray = slicebydirection(pointData,sliceX,sliceY,sliceK,cutLenght)
% slice pointdata along given direction
% -cutInfo:切割点信息，包括切割点坐标、切割线方向
% -cutLenght:切割厚度
PointSet= struct('data',[],'range',[]);
SliceArray=repmat(PointSet,[1 50]);
x = pointData(:,1);
y = pointData(:,2);
%对点云进行旋转
if sliceK<0;
    sliceK = sliceK+pi;
end
A = pi/2-sliceK;%旋转角
%将坐标转换到以切点为原点，切线为纵坐标点的坐标系中
x0= (x - sliceX).*cos(A) - (y - sliceY).*sin(A) ;%逆时针旋转A
y0= (x - sliceX).*sin(A) + (y - sliceY).*cos(A) ;
data = [pointData x0 y0];
%切片
tempData = data(data(:,end-1)<=(cutLenght/2)&data(:,end-1)>-(cutLenght/2),1:end-2);
if isempty(tempData),
    nArray = 0;
else
    nArray = 1;
    SliceArray(nArray).data = tempData;
    SliceArray(nArray).range = [-cutLenght/2,cutLenght/2];
end
isLeft = true;
isRight = true;
nSlice = 1;
while isLeft||isRight,
    nSlice = nSlice+1;
    if isLeft,
        L = -(nSlice*cutLenght-cutLenght/2);
        tempData = data(data(:,end-1)>=L&data(:,end-1)<(L+cutLenght),1:end-2);
        if ~isempty(tempData),
            nArray = nArray+1;
            SliceArray(nArray).data = tempData;
            SliceArray(nArray).range = [L,L+cutLenght];        
        elseif isempty(pointData)||nSlice>15||nArray>=1,
            isLeft = false;
        end
    end
    if isRight,
        R = (nSlice*cutLenght-cutLenght/2);
        tempData = data(data(:,end-1)>=(R-cutLenght)&data(:,end-1)<R,1:end-2);
        if ~isempty(tempData),
            nArray = nArray+1;
            SliceArray(nArray).data = tempData;
            SliceArray(nArray).range = [R-cutLenght,R];
        elseif isempty(pointData)||nSlice>15||nArray>=1,
            isRight = false;
        end
    end 
end
SliceArray = SliceArray(1:nArray);
end

function  ScanLineArray = slice2scanlines(pointCloudData,sliceInfo)
% slice point cloud data according to the maximal distance of two contiguous point
% the result of this process will abtain each scanline
%检测扫描线原则：一，间隔大于5米；二，在间隔图像中是顶点；三，顶点之间的点个数大于一定数量
    nPoint = size(pointCloudData,1);
    prePoint = pointCloudData(1:nPoint-1,1:4);
    nextPoint = pointCloudData(2:nPoint,1:4);    
    dx = (prePoint(1:nPoint-1,1)-nextPoint(1:nPoint-1,1));
    dy = (prePoint(1:nPoint-1,2)-nextPoint(1:nPoint-1,2));
    dh = (prePoint(1:nPoint-1,3)-nextPoint(1:nPoint-1,3));
    ds = sqrt(dx.^2+dy.^2+dh.^2);
    %间隔图像,调试用
%     plot(1:10000,ds(1:10000),'r.');
%     hold on
%     plot(1:10000,ds(1:10000));
    vertexInfoArray = zeros(fix(nPoint/100),2);
    nVertex = 1;
    vertexInfoArray(1,1) = 1;
    vertexInfoArray(1,2) = ds(1);   
    intervalQuantity = 5;%每条扫描线最少点个数，初始默认是5个
    
    %间隔默认是5米，这适用大多数道路提取；当从隧道数据中提取道路时，间隔应设置的很小，比如0.5米。
    %如果扫描线还要用于其他路面附属物的分析（如路灯、树木等），间隔应设置的大一些，但此时则不能用于提取隧道路面。
    if ~exist('sliceInfo','var') || isempty(sliceInfo)
        intervalDist = 5;%扫描线分界点之间的最小距离，一般比道路宽度略小 ,默认5米
    else
        intervalDist = sliceInfo(1);
    end
           
    nVertexTemp = 1;
    vertexInfoArrayTemp = zeros(20,2);
    vertexInfoArrayTemp(1,1) = 1;
    vertexInfoArrayTemp(1,2) = ds(1);
    for i = 2:(nPoint-2),
        %计算间隔参数intervalQuantity
        %当开始部分点云比较杂乱使，切片可能不精确，此时计算的intervalQuantity可能不合适
        %算法却要改进
        preDs = ds(i-1);
        currentDs = ds(i);
        nextDs = ds(i+1);
        if (preDs<currentDs)&&(nextDs<currentDs)&&currentDs>intervalDist,
            %currentDs是顶点
            preVertexOrder = vertexInfoArrayTemp(nVertexTemp,1);
            preVertexDs = vertexInfoArrayTemp(nVertexTemp,2);
            if (i - preVertexOrder)>=intervalQuantity,
                nVertexTemp = nVertexTemp+1;
                vertexInfoArrayTemp(nVertexTemp,1) = i;
                vertexInfoArrayTemp(nVertexTemp,2) = ds(i);
            elseif ((i - preVertexOrder)<5)&&(preVertexDs<ds(i)),
                vertexInfoArrayTemp(nVertexTemp,1) = i;
                vertexInfoArrayTemp(nVertexTemp,2) = ds(i);
            end
            if nVertexTemp>10,
                break;
            end
        end
    end  
    sumIntervalTemp = 0;
    for i = 1:nVertexTemp-1,
        intervalTemp = vertexInfoArrayTemp(i+1,1) - vertexInfoArrayTemp(i,1);
        sumIntervalTemp = sumIntervalTemp+intervalTemp;
    end
    intervalQuantity = fix((sumIntervalTemp/nVertexTemp)*0.75);%此参数可能不准
%     intervalQuantity = 90;
    for i = 2:(nPoint-2),
        preDs = ds(i-1);
        currentDs = ds(i);
        nextDs = ds(i+1);
        if (preDs<currentDs)&&(nextDs<currentDs)&&(currentDs>intervalDist),
            %currentDs是顶点
            preVertexOrder = vertexInfoArray(nVertex,1);
            preVertexDs = vertexInfoArray(nVertex,2);
            if (i - preVertexOrder)>=intervalQuantity,
                nVertex = nVertex+1;
                vertexInfoArray(nVertex,1) = i;
                vertexInfoArray(nVertex,2) = ds(i);
            elseif ((i - preVertexOrder)<intervalQuantity)&&(preVertexDs<ds(i)),
                vertexInfoArray(nVertex,1) = i;
                vertexInfoArray(nVertex,2) = ds(i);
            end
        end     
    end
    nScanLine = nVertex;
    PointSet= struct('x',0,'y',0,'h',0,'ins',0);
    ScanLineArray=repmat(PointSet,[1 nScanLine]);  
    
    for i = 1:nVertex-1,
        nStart = vertexInfoArray(i)+1;%比如有5多个点，但只有4个间距
        nEnd = vertexInfoArray(i+1);
        ScanLineArray(i).x=pointCloudData(nStart:nEnd,1);
        ScanLineArray(i).y=pointCloudData(nStart:nEnd,2);
        ScanLineArray(i).h=pointCloudData(nStart:nEnd,3);
        ScanLineArray(i).ins=pointCloudData(nStart:nEnd,4);
    end
    if ~exist('nEnd','var')
        return;
    end
    if nPoint>nEnd
        %当还有剩余点没有被切割成扫描线时，将剩余的点归为一条扫描线
        ScanLineArray(nVertex).x=pointCloudData(nEnd+1:nPoint,1);
        ScanLineArray(nVertex).y=pointCloudData(nEnd+1:nPoint,2);
        ScanLineArray(nVertex).h=pointCloudData(nEnd+1:nPoint,3);
        ScanLineArray(nVertex).ins=pointCloudData(nEnd+1:nPoint,4);
    else
        ScanLineArray = ScanLineArray(1:nVertex-1);
    end
end

function SliceArray = slicebythinned(rawPointData,ThinnedSliceArray,thinRate)
% slicebythinned - slice raw point data through slices of thinned point data
%
% Input Arguments
% rawPointData : 待切片原始点云数据矩阵（[x y h ints]）
% ThinnedSliceArray : 由rawPointData抽稀后的点云数据计算的切片结果数组
%
% Output Arguments
% SliceArray ： 原始点云切片结果数组
%
% Discription
% 
    nSlice = size(ThinnedSliceArray,2);
    PointSet= struct('x',0,'y',0,'h',0,'ins',0);
    SliceArray=repmat(PointSet,[1 nSlice]);
    SliceRangeArray = zeros(nSlice,2);%存储结果切片（不是稀疏切片）起止点在原始点云中的顺序号
    nRawPoint = size(rawPointData,1);
    iRawPoint = 0;
    iRawPoint2nd = 1;%第2个起始点在原始点云中的顺序号
    iRawPoint3rd = 2;
    for i = 1:nSlice,
        %用开头，结尾各三个点来判断，这样可以去减少飞点的影响
        StartPoint.x = ThinnedSliceArray(i).x(1:3);
        StartPoint.y = ThinnedSliceArray(i).y(1:3);
        StartPoint.h = ThinnedSliceArray(i).h(1:3);
        nThinnedSlicePoint = size(ThinnedSliceArray(i).x,1);
        EndPoint.x = ThinnedSliceArray(i).x(nThinnedSlicePoint-2:nThinnedSlicePoint);
        EndPoint.y = ThinnedSliceArray(i).y(nThinnedSlicePoint-2:nThinnedSlicePoint);
        EndPoint.h = ThinnedSliceArray(i).h(nThinnedSlicePoint-2:nThinnedSlicePoint);
        iPoint = 0;
        while iRawPoint<nRawPoint-2,
            iRawPoint = iRawPoint+1;
            x = rawPointData(iRawPoint,1);
            y = rawPointData(iRawPoint,2);
            h = rawPointData(iRawPoint,3);
            ints = rawPointData(iRawPoint,4);
            iRawPoint2ndBack = iRawPoint;
            iRawPoint3rdBack = iRawPoint-1;
            if (StartPoint.x(1) == x)&&(StartPoint.y(1) == y)&&(StartPoint.h(1) == h),
                iRawPoint2nd = iRawPoint2nd + 1;
                iRawPoint3rd = iRawPoint3rd + 1;
                x2nd = rawPointData(iRawPoint2nd,1);
                y2nd = rawPointData(iRawPoint2nd,2);
                h2nd = rawPointData(iRawPoint2nd,3);
                x3rd = rawPointData(iRawPoint3rd,1);
                y3rd = rawPointData(iRawPoint3rd,2);
                h3rd = rawPointData(iRawPoint3rd,3);
                iRawPoint2nd = iRawPoint;
                while (StartPoint.x(2) ~= x2nd)||(StartPoint.y(2) ~= y2nd)||(StartPoint.h(2) ~= h2nd),
                    iRawPoint2nd = iRawPoint2nd+1;%第二个起始点在原始点云中的顺序号
                    x2nd = rawPointData(iRawPoint2nd,1);
                    y2nd = rawPointData(iRawPoint2nd,2);
                    h2nd = rawPointData(iRawPoint2nd,3);
                end
                iRawPoint3rd = iRawPoint2nd;
                while (StartPoint.x(3) ~= x3rd)||(StartPoint.y(3) ~= y3rd)||(StartPoint.h(3) ~= h3rd),
                    iRawPoint3rd = iRawPoint3rd+1;
                    x3rd = rawPointData(iRawPoint3rd,1);
                    y3rd = rawPointData(iRawPoint3rd,2);
                    h3rd = rawPointData(iRawPoint3rd,3);
                end
                temp = 3;
                while (temp<nThinnedSlicePoint)&&(iRawPoint3rd<nRawPoint),
                    temp = temp+1;
                    if ((iRawPoint2nd-iRawPoint)~=thinRate)||((iRawPoint3rd-iRawPoint2nd)~=thinRate),
                        iRawPoint = iRawPoint2nd;
                        iRawPoint2nd = iRawPoint3rd;
                        StartPoint.x(1:2) = StartPoint.x(2:3);
                        StartPoint.y(1:2) = StartPoint.y(2:3);
                        StartPoint.h(1:2) = StartPoint.h(2:3);
                        StartPoint.x(3) = ThinnedSliceArray(i).x(temp);
                        StartPoint.y(3) = ThinnedSliceArray(i).y(temp);
                        StartPoint.h(3) = ThinnedSliceArray(i).h(temp);
                        while (StartPoint.x(3) ~= x3rd)||(StartPoint.y(3) ~= y3rd)||(StartPoint.h(3) ~= h3rd),
                            iRawPoint3rd = iRawPoint3rd+1;
                            x3rd = rawPointData(iRawPoint3rd,1);
                            y3rd = rawPointData(iRawPoint3rd,2);
                            h3rd = rawPointData(iRawPoint3rd,3);
                        end
                    else
                        %至此找到了三个顺序间隔相等的点
                        x = rawPointData(iRawPoint,1);
                        y = rawPointData(iRawPoint,2);
                        h = rawPointData(iRawPoint,3);
                        ints = rawPointData(iRawPoint,4);
                        iStart = iRawPoint;
                        SliceRangeArray(i,1) = iStart;
                        break;
                    end
                end
            elseif (EndPoint.x(3) == x)&&(EndPoint.y(3) == y)&&(EndPoint.h(3) == h),
                %注意计数方向，EndPoint（3）才是最后一个结束点，与x2nd，y3rd是反的
                iRawPoint2ndBack = iRawPoint2ndBack - 1;
                x2nd = rawPointData(iRawPoint2ndBack,1);
                y2nd = rawPointData(iRawPoint2ndBack,2);
                while (EndPoint.x(2) ~= x2nd)||(EndPoint.y(2) ~= y2nd),
                    %找到EndPoint.x(2)在原始点云中的顺序号
                    iRawPoint2ndBack = iRawPoint2ndBack-1;
                    if iRawPoint2ndBack <1,
                        break;
                    end
                    x2nd = rawPointData(iRawPoint2ndBack,1);
                    y2nd = rawPointData(iRawPoint2ndBack,2);
                end
                iRawPoint3rdBack = iRawPoint2ndBack;
                if iRawPoint2ndBack <1,break;end;
                x3rd = rawPointData(iRawPoint3rdBack,1);
                y3rd = rawPointData(iRawPoint3rdBack,2);
                while (EndPoint.x(1) ~= x3rd)||(EndPoint.y(1) ~= y3rd),
                    iRawPoint3rdBack = iRawPoint3rdBack-1;
                    if iRawPoint2ndBack <1,break;end;
                    if iRawPoint3rdBack <1,break;end;
                    x3rd = rawPointData(iRawPoint3rdBack,1);
                    y3rd = rawPointData(iRawPoint3rdBack,2);
                end
                %至此算出了末尾三个点在原始点云中的的存储顺序位置
                temp = 0;
                while (temp<nThinnedSlicePoint)&&(iRawPoint3rdBack>0),                  
                    if ((iRawPoint-iRawPoint2ndBack)~=thinRate)||((iRawPoint2ndBack-iRawPoint3rdBack)~=thinRate),
                        temp = temp+1;%EndPoint往回倒得次数
                        iRawPoint = iRawPoint2ndBack;
                        iRawPoint2ndBack = iRawPoint3rdBack;
                        EndPoint.x(2:3) = EndPoint.x(1:2);
                        EndPoint.y(2:3) = EndPoint.y(1:2);
                        EndPoint.x(1) = ThinnedSliceArray(i).x(nThinnedSlicePoint-temp-2);
                        EndPoint.y(1) = ThinnedSliceArray(i).y(nThinnedSlicePoint-temp-2);
                        while (EndPoint.x(1) ~= x3rd)||(EndPoint.y(1) ~= y3rd),
                            %找EndPoint.x(1)在原始点云中的顺序号
                            iRawPoint3rdBack = iRawPoint3rdBack-1;
                            x3rd = rawPointData(iRawPoint3rdBack,1);
                            y3rd = rawPointData(iRawPoint3rdBack,2);
                        end
                    else
                        %至此找到了三个顺序间隔相等的点
                        iEnd = iRawPoint;
                        SliceRangeArray(i,2) = iEnd;
                        x = rawPointData(iRawPoint,1);
                        y = rawPointData(iRawPoint,2);
                        h = rawPointData(iRawPoint,3);
                        ints = rawPointData(iRawPoint,4);
                        break;
                    end
                end
                break;
            end
        end
            if (i>1)&&(iStart == SliceRangeArray(i-1,1)),
                %即当没有找到起始点时，这时的iStart与上一次iStart相等
                iStart=SliceRangeArray(i-1,2)+1;
                SliceRangeArray(i,1) = iStart;
            end
            if (i>1)&&(iEnd == SliceRangeArray(i-1,2)),
                %没有找到终点，认为此时点云切割完毕
                SliceArray=SliceArray(1:i-1);
                return;
            end
            SliceArray(i).x=rawPointData(iStart:iEnd,1);
            SliceArray(i).y=rawPointData(iStart:iEnd,2);
            SliceArray(i).h=rawPointData(iStart:iEnd,3);
            SliceArray(i).ins=rawPointData(iStart:iEnd,4);  
    end
end

function SliceArray = slicebypos(pointCloudData,posData,separation,thickness,width)
% slicebypos - slcie point cloud data through pos data.
% the center point of slices is on the track curve fitted through pos data. 
% - separation:相邻切片间隔
% - thickness:切片厚度
% 对离散的Pos数据进行曲线拟合，从而可以确定轨迹线上任一点切向方向，进而可以
% 在任一点进行切片。
%   注意：当切片间隔较小时，若A，B为相邻切片，p为A、B相接边界上的点，则p有可能即被
% 归类为A切片，又被归类为B切片，导致此现象的原因是D(A,B)与（separation+thickness）
% 不严格相等；另外，如果pos数据重复的话点云也会被重复切片。
    %累计距离以直线近似确定
    %切向量本应通过拟合的曲线确定，这里通过前后两点直线斜率近似代替
    distTotal = 0;
    nSlice = 0;
    posX = posData.x;
    posY = posData.y;
    nPos = size(posData.x,1);
    PointSet= struct('x',0,'y',0,'h',0,'ins',0);
    SliceArray=repmat(PointSet,[1 1000]);
    nPoint = 0;
    k = zeros(nPos,1); %pos点到下一点直线的方位角（-180,180）
    for i = 1:nPos-1
        distTotal=distTotal+norm([posX(i+1)-posX(i) posY(i+1)-posY(i)]);
        k(i,1) = atan2(posY(i+1)-posY(i),posX(i+1)-posX(i));%四象限方向角
    end
    k(nPos,1) = k(nPos-1,1);
    nSlice = ceil(distTotal/(separation+thickness));%理论切片个数
    %初始值
    dist = norm([posX(2)-posX(1) posY(2)-posY(1)]);%相邻pos点之间的距离
    distTotal = dist;
    dist2 = dist;%切片点到下一个pos点的距离
    iPos = 1;
    iSlice = 1;
    sliceX = zeros(1,nSlice);
    sliceY = zeros(1,nSlice);
    sliceK = zeros(1,nSlice);
    sliceX(1) = posX(1);
    sliceY(1) = posY(1);
    sliceK(1) = k(1);
    while iSlice<nSlice,
        %计算切片点位置（有问题需要修改）
        iSlice = iSlice+1;
        if  dist2>(separation+thickness),
            sliceX(iSlice) = sliceX(iSlice-1)+(separation+thickness)*cosd(k(iPos));
            sliceY(iSlice) = sliceY(iSlice-1)+(separation+thickness)*sind(k(iPos));
            sliceK(iSlice) = k(iPos);
        elseif dist2<(separation+thickness),
            while dist2<(separation+thickness),
                iPos = iPos+1;
                dist2Pre = dist2;
                dist2 = dist2+norm([posX(iPos)-posX(iPos+1) posY(iPos)-posY(iPos+1)]);
            end
            dL = separation+thickness-dist2Pre;%增量长度
            sliceX(iSlice) = posX(iPos)+dL*cosd(k(iPos));
            sliceY(iSlice) = posY(iPos)+dL*sind(k(iPos));
            sliceK(iSlice) = k(iPos);
        elseif dist2==(separation+thickness),
            sliceX(iSlice) = posX(iPos+1);
            sliceY(iSlice) = posY(iPos+1);
            sliceK(iSlice) = k(iPos);
            iPos = iPos+1;
        end
        dist2 = norm([sliceX(iSlice)-posX(iPos+1) sliceY(iSlice)-posY(iPos+1)]);  
    end        
    PointSet= struct('x',0,'y',0,'h',0,'ins',0);
    SliceArray=repmat(PointSet,[1 nSlice]);
    nSlice2 = 0;%去掉空数据后的实际切片个数
    nZreos = 0;
    for i = 1:nSlice,    
        %点云到切割断面的垂直距离
        deltad=abs((pointCloudData(:,1)-sliceX(i))*cos(sliceK(i))+...
            (pointCloudData(:,2)-sliceY(i))*sin(sliceK(i)));
        if exist('width','var')&&~isempty(width),
            %点云到切割点的直线距离(斜边)
            deltad3_2 = (pointCloudData(:,1)-sliceX(i)).^2+(pointCloudData(:,2)-sliceY(i)).^2;
            %点云到切割点的连线在切割面上的投影距离
            deltad2 = sqrt(deltad3_2-deltad.^2);
            tempptcl=[pointCloudData deltad deltad2];         
            tempptcl=tempptcl(tempptcl(:,6)<=width,:); %剔除远离切割点的点
        else
             tempptcl=[pointCloudData deltad];
        end           
        tempptcl=tempptcl(tempptcl(:,5)<=(thickness/2),:); %剔除大于切片厚度的点
        %有时轨迹数据很长，而切片的点云只是一小段，这时会有大量空切片
        %为了防止大量空切片浪费计算时间，这里定义连续出现10次空切片就认为切片已完成
        %这种定义是不严谨的，后面应该对pos数据识别，只利用计算点云路段对应的pos数
        if size(tempptcl,1),
            nZreos = 0;
        elseif ~size(tempptcl,1)&&nSlice2>1,
            nZreos = nZreos+1;
            if nZreos>10&nSlice2>10,
                break;
            end
        end
        if size(tempptcl,1)>(thickness*100)
        %去除点数过少的切片
%         plot3(tempptcl(:,1),tempptcl(:,2),tempptcl(:,3),'r.');axis equal;hold on;
%          plot3(posX,posY, tempptcl(1,3)*ones(size(posX,1),1),'go');
%           plot3(sliceX(i),sliceY(i), tempptcl(1,3),'ro');
            nSlice2 = nSlice2+1;
            SliceArray(nSlice2).x=tempptcl(:,1);
            SliceArray(nSlice2).y=tempptcl(:,2);
            SliceArray(nSlice2).h=tempptcl(:,3);
            SliceArray(nSlice2).ins=tempptcl(:,4);
            SliceArray(nSlice2).info = [sliceX(i) sliceY(i) sliceK(i)];%注意k为角度，不是斜率
        end
    end
    SliceArray = SliceArray(1:nSlice2);    
end

function SliceArray = sliceonpos(pointCloudData,posData,separation,thickness)
% slice point cloud data by pos data.
% the center point of slices is the pos point.
% this algorithm was copied from Wangkai's program.
%
    dist=0;
    numslice=0;
    azimuth = posData.azimuth;
    xpos = posData.x;
    ypos = posData.y;
    numpos = size(posData.x,1);
    ptcl = pointCloudData;
    PointSet= struct('x',0,'y',0,'h',0,'ins',0);
    SliceArray=repmat(PointSet,[1 1000]);
    for i = 1:numpos-1,
        dist=dist+sqrt((xpos(i+1)-xpos(i))^2+(ypos(i+1)-ypos(i))^2);
        if dist >= separation
            deltad=abs((ptcl(:,1)-xpos(i))*cos(azimuth(i)/180*pi)+...
                (ptcl(:,2)-ypos(i))*sin(azimuth(i)/180*pi));
            tempptcl=[ptcl deltad];
            tempptcl=tempptcl(tempptcl(:,5)<=(thickness/2),:); %剔除大于切片厚度的点
            medianz=median(tempptcl(:,3));
            if size(tempptcl,1)<(thickness*100)
                %去除点数过少的切片
                dist=0.0;
                deltad=[];
                tempptcl=[];
                continue;
            end
            numslice=numslice+1;
            SliceArray(numslice).x=tempptcl(:,1);
            SliceArray(numslice).y=tempptcl(:,2);
            SliceArray(numslice).h=tempptcl(:,3);
            SliceArray(numslice).ins=tempptcl(:,4);
            dist=0.0;
            deltad=[];
            tempptcl=[];
        end
    end
    SliceArray = SliceArray(1:numslice);
end 

function outputslices2file(sliceArray)
% output point cloud slices data into file
    writetime=datestr(now,'yymmddHHMMSS');
    nSlices = size(sliceArray,2);
    for i=1:nSlices
        temp=[sliceArray(i).x sliceArray(i).y sliceArray(i).h sliceArray(i).ins];
        temp1=strcat('slice',writetime,'\');
        if ~exist(temp1,'dir')
            mkdir(temp1);
        end        
        fid1=fopen(strcat('slice',writetime,'\',num2str(i),'.xyz'),'wt');
        fprintf(fid1,'%f %f %f %d\n',temp');
        fclose(fid1);
    end
end

function pointCloudData = readpointcloudfile(filePath)
% read point cloud file
    fid=fopen(filePath,'r');
    tline=fgetl(fid);   
    lineData = regexp(tline, '\s+', 'split');
    col =  size(lineData,2);
    if col==4,
        %xyzi
        data = fscanf(fid,'%f %f %f %d',[4,inf])';
        pointCloudData = data;
    elseif col==7,
        %xyzirgb
        data = fscanf(fid,'%f %f %f %d %d %d %d',[7,inf])';
        pointCloudData = data(:,1:4);
    end
    fclose(fid);
end

function posData = readposfile(filePath)
% read pose file 
%读取pos数据时注意数据排列格式
    fid=fopen(filePath,'r');
    data = fscanf(fid,'%d %d %d %d %d %f %d %f %f %f %f %f %f %f %f %f',[16,inf])';
    posData.year = data(1:end,1);
    posData.month = data(1:end,2);
    posData.day = data(1:end,3);
    posData.hour = data(1:end,4);
    posData.minute = data(1:end,5);
    posData.second = data(1:end,6);
    posData.gpsweek = data(1:end,7);
    posData.gpstime = data(1:end,8);
    posData.latitude = data(1:end,9);
    posData.longitude = data(1:end,10);
    posData.elevation = data(1:end,11);
    posData.x = data(1:end,12);
    posData.y = data(1:end,13);
    posData.azimuth = data(1:end,14);
    posData.roll = data(1:end,15);
    posData.pitch = data(1:end,16);
    fclose(fid);
end

function thinnedPointData =  thinpointdata(pointdata,skip)
% thin point data
    [nPoint col] = size(pointdata);
    nThinnedPoints = ceil(nPoint/skip);
    thinnedPointData = zeros(nThinnedPoints,col);
    iThinnedPoint = 0;
    while iThinnedPoint<nThinnedPoints,
        iThinnedPoint = iThinnedPoint+1;
        iPoint = (iThinnedPoint-1)*skip+1;
        thinnedPointData(iThinnedPoint,1:col) = pointdata(iPoint,1:end);
    end
end
