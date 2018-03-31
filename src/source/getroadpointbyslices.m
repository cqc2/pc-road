  function roadPoint = getroadpointbyslices(pointCloudData,posData,roadThickness,marginW,minRoadWidth,windowW)
%
%通过点云切片识别道路
% datetime('now','TimeZone','local','Format','HH:mm:ss Z')
if ~exist('roadThickness','var') || isempty(roadThickness),roadThickness = []; end
if ~exist('marginW','var') || isempty(marginW), marginW = []; end
if ~exist('minRoadWidth','var') || isempty(minRoadWidth), minRoadWidth = []; end
if ~exist('windowW','var') || isempty(windowW), windowW = []; end

% pointCloudData =  readpointcloudfile2('dataspace_slice\test#123.xyz');
% posData = readposfile('dataspace_slice\POS_#2_20150703.txt',14);
SliceArray = slice(pointCloudData(:,:),'bypos',posData,0,0.5,20);%切片越细，计算越慢
projectArray = project2section(SliceArray);
roadPoint = getroadfromproject(projectArray,roadThickness,marginW,minRoadWidth,windowW);
% savepointcloud2file(roadPoint,'road#123_byslices8888',false);
% datetime('now','TimeZone','local','Format','HH:mm:ss Z')
end

function projectArray = project2section(SliceArray)
%
% -X: 在二维投影面中点的横坐标
%将切片投影到断面
nSlice = size(SliceArray,2);
projectArray = SliceArray;
for iSlice=1:nSlice,
    x1 = SliceArray(iSlice).x;
    y1 = SliceArray(iSlice).y;
    h1 = SliceArray(iSlice).h;
    info = SliceArray(iSlice).info;
    x0 = info(1);
    y0 = info(2);
    k0 = tan(info(3)+pi/2);
    k1 = -1/k0;
    %将切片点云压缩到三维空间的平面
%     pX = (y0-y1+k1.*x1-k0.*x0)/(k1-k0);
    pX = ((y0-y1)*k0+(-1).*x1-k0*k0.*x0)/(-1-k0*k0);%防止k0趋于0时，导致k1无穷大计算出错，所以分子分母同乘k0
    if k0==0,
        pY = y1;
    else
        pY = k1.*(pX-x1)+y1;
    end
    if abs(k0)>1000,
        a=0;
    end
    
    %将三维空间的平面转换为二维断面
    minx = min(pX);
    miny = min(pY);
    X = sqrt((pX-minx).^2 + (pY-miny).^2); %在二维投影面中点的横坐标
    projectArray(iSlice).X = X;
end
end

function roadPoint= getroadfromproject(projectArray,roadThickness,marginW,minRoadWidth,windowW)
%
%
nProject = size(projectArray,2);
roadPoint = [];
for iProject = 1:nProject,
    X = projectArray(iProject).X;%投影到断面时的横坐标
    H = projectArray(iProject).h;
    x = projectArray(iProject).x;
    y = projectArray(iProject).y;
    h = projectArray(iProject).h; 
    ins = projectArray(iProject).ins; 
    nPoint = size(X,1);
    pseudoWidth = 0.05;
    [pseudoScanline,pseudoIndex] = getpseudoscanline([X H],pseudoWidth,'min');
    %构造伪扫描线结构体
    pseudoX = pseudoScanline(:,1);
    pseudoY = (1:size(pseudoScanline,1))';%记录伪扫描线的顺序索引号，
    pseudoH = pseudoScanline(:,2);  
%     plot(pseudoX,pseudoH,'r.');axis equal
    PseudoScanlineArray.x = pseudoX;
    PseudoScanlineArray.h = pseudoH;
    PseudoScanlineArray.y = pseudoY; 
    PseudoScanlineArray.ins = pseudoScanline(:,1)*0;%只是让数据结构统一，不起实质作用
    %从伪扫描线中初步提取出伪道路断面点
     fluctuateH = 0.3;
     pseudoRoadPointData = getroadbyscanline(PseudoScanlineArray,[],[],fluctuateH);
     if isempty(pseudoRoadPointData),continue;end
     %对伪道路断面直线部分进行拟合
     pseudoRoadX = pseudoRoadPointData(:,1);
     pseudoRoadH = pseudoRoadPointData(:,3); 
     [coefficients,percet] = ransac([pseudoRoadX pseudoRoadH],1,0.1,10);
     if percet<0.6,continue;end
     %由拟合的直线参数提取直线缓冲区内的点进行进一步分析，
     %这里(1:nPoint)'为索引标记量，用于投影坐标与真实坐标对应
     bufferpoint = getbufferpoint([X H (1:nPoint)'],coefficients,1);  %投影坐标形式 
%      index = bufferpoint1(1:end,3);
% if iProject==70,
%     a=1;
% end
     raodPointXH = refineroadpoint(bufferpoint,roadThickness,marginW,minRoadWidth,windowW);
 if isempty(raodPointXH),continue;end
    roadIndex = raodPointXH(:,3);  
     roadPoint = [roadPoint;x(roadIndex) y(roadIndex) h(roadIndex) ins(roadIndex)];    
%      plot(bufferpoint1(pseudoRoadIndex,1),bufferpoint1(pseudoRoadIndex,2),'g.');hold off
%      plot(X(:,1),H(:,1),'g.' );axis equal;hold on;
%      plot(bufferpoint1(:,1),bufferpoint1(:,2),'r.' );axis equal;hold on;
%      plot(pseudoScanline2(:,1),pseudoScanline2(:,2),'b.');axis equal;hold off
end
end

function  raodPointXH = refineroadpoint(bufferpoint,roadThickness,marginW,minRoadWidth,windowW)
%
%移动窗口法滤波对路面进行精细提取
%假设路面点云不存在大的间隔
%-------默认参数----------------------
%     -roadThickness ：0.2，断面投影后的路面厚度，与路面粗糙程度、点的测量精度以及断面投影误差大小有关;
%     -marginW ： 2，道路边缘与栏杆的距离，与道路边缘小于此距离的突出物认为是栏杆，并以此确定要提取的路面范围
%     -minRoadWidth ：4，,最小路宽，小于次宽度的点云会忽略                
%     -windowW：0.4，计算过程中分割窗口的宽度，路面复杂、点云密度大可适当缩小，路面平摊、点云密度稀可适当放大，不能小于路面点间距
if ~exist('roadThickness','var') || isempty(roadThickness),roadThickness = 0.2; end
if ~exist('marginW','var') || isempty(marginW), marginW = 2; end
if ~exist('minRoadWidth','var') || isempty(minRoadWidth), minRoadWidth = 4; end
if ~exist('windowW','var') || isempty(windowW), windowW = 0.4; end
% windowW = 0.3;%窗口宽度
% roadThickness = 0.2;%道路投影厚度
% marginW = 2;%围栏与路边缘距离
% minRoadWidth = 4;%最小路宽

raodPointXH =[];
minX = min(bufferpoint(:,1));
maxX = max(bufferpoint(:,1));
nWindow = ceil((maxX-minX)/windowW);
windowInfo = Inf(nWindow,4);%分别为窗口的高差、最高点、最低点、点个数
windowArray = cell(1,nWindow);

%计算每个窗口的相关属性信息
for i=1:nWindow,
    window = bufferpoint((bufferpoint(:,1)<(i*windowW+minX))&(bufferpoint(:,1)>((i-1)*windowW+minX)),:);
    if isempty(window),windowInfo(i,4) = 0;continue;end
    windowArray(i) ={window};
    H = window(:,2);
    maxH = max(H);
    minH = min(H);
    dH = maxH-minH;
    np = size(H,1);
    windowInfo(i,1:4) = [dH,maxH,minH,np];
end

dminH = windowInfo(2:end,3) - windowInfo(1:end-1,3);
nCluster = 0;
isStart = false;
%识别道路候选片段
roadClusterIndex = [];
for  i=1:nWindow-1, 
    %认为道路主体的最低点是连续且高程近似的
    if abs(dminH(i))<=0.2&&~isStart,
        nCluster = nCluster+1;
        roadClusterIndex(nCluster,1) = i;
        startIndex = i;
        isStart = true;
    elseif abs(dminH(i))>0.2&&isStart,
        roadClusterIndex(nCluster,2:3) = [i,i-startIndex+1];%第二项为道路候选片段中包含的窗口数
        isStart = false;
    end 
end
if isStart,
    %一般是以高程突变窗口作为片段边界
    %当检索至最后一个窗口仍未发生突变时，则以其作为结尾边界
    roadClusterIndex(nCluster,2:3) = [nWindow,nWindow-startIndex+1];
end

%认为最长的候选片段是道路主体部分
if isempty(roadClusterIndex),
    return;
end;
roadClusterIndex = sortrows(roadClusterIndex,3);
roadStartIndex = roadClusterIndex(end,1);
roadEndIndex = roadClusterIndex(end,2);
if roadClusterIndex(end,3)<ceil(minRoadWidth/windowW),
    %如果候选片段长度小于2米，则忽略
    return;
end

%与道路主体部分边界相接的窗口也有可能包含部分道路，所以应向两侧各扩充一个窗口并检测出其中属于道路主体部分
if roadStartIndex~=1&&windowInfo(roadStartIndex-1,4)~=0,
    minH = windowInfo(roadStartIndex,3);
    pdata = windowArray{roadStartIndex-1};
    pdata = sortrows(pdata,1);
    npdata = size(pdata,1);
    for i = npdata:1,
        H = pdata(i,2);
        if abs(H-minH)>roadThickness,
            break;
        end
    end
    if i<npdata,
        %将扩充窗口中的道路点添加到道路窗口中
        windowData = windowArray{roadStartIndex};
        windowData = [windowData;pdata(i+1:end,:)];
        windowArray(roadStartIndex) = {windowData};
    end  
end
if roadEndIndex~=nWindow&&windowInfo(roadEndIndex+1,4)~=0,
   minH = windowInfo(roadEndIndex,3);
    pdata = windowArray{roadEndIndex+1};
    pdata = sortrows(pdata,1);
    npdata = size(pdata,1);
    for i = 1:npdata,
        H = pdata(i,2);
        if abs(H-minH)>roadThickness,
            break;
        end
    end
    if i>1,
        %将扩充窗口中的道路点添加到道路窗口中
        windowData = windowArray{roadEndIndex};
        windowData = [windowData;pdata(1:i-1,:)];
        windowArray(roadEndIndex) = {windowData};
    end     
end

%提取道路主体部分的关键窗口
%认为厚度超过0.2米为边界或者地面附作物。
critcleWinClusterIndex = [];%关键窗口索引聚类
numCluster = 0;
isAddCluster = true;
for i = roadStartIndex:roadEndIndex,
    dH = windowInfo(i,1);
    if dH>roadThickness,
        if isAddCluster,
            numCluster = numCluster+1;
            critcleWinClusterIndex(numCluster,1:2) = [i,i];
        end    
        critcleWinClusterIndex(numCluster,2) = i;
        isAddCluster = false;
    else
        isAddCluster = true;
    end   
end

%
if numCluster==0,
    %全部是路面点
    for m = roadStartIndex:roadEndIndex,
        winPoint =  windowArray{m};
        raodPointXH = [raodPointXH;winPoint];
    end
else
    %简单起见，这里只检测前两个和后两个关键窗口到边界的距离
    if numCluster>1&&(critcleWinClusterIndex(2,1)-roadStartIndex)<=ceil(marginW/windowW),
        cutIndexL = [critcleWinClusterIndex(2,2),2];
    elseif (critcleWinClusterIndex(1,1)-roadStartIndex)<=ceil(marginW/windowW),
        %认为是左边界切割点
        cutIndexL = [critcleWinClusterIndex(1,2),1];
    else
        cutIndexL = [roadStartIndex-1,0];
    end
    if numCluster>1&&(roadEndIndex-critcleWinClusterIndex(end-1,2))<=ceil(marginW/windowW),
        cutIndexR = [critcleWinClusterIndex(end-1,1),2];
    elseif (roadEndIndex-critcleWinClusterIndex(end,2))<=ceil(marginW/windowW),
        %认为是右侧边界切割点
        cutIndexR = [critcleWinClusterIndex(end,1),1];
    else
        cutIndexR = [roadEndIndex+1,0];
    end
    if cutIndexL>cutIndexR,return;end
    raodPointXH =  removeRoadNoise(windowArray,windowInfo,critcleWinClusterIndex,cutIndexL,cutIndexR,roadStartIndex,roadEndIndex);
end

% X = bufferpoint(:,1);
% H = bufferpoint(:,2);
% plot(X,H,'r.');axis equal;hold on;
% X = raodPointXH(:,1);
% H = raodPointXH(:,2);
% plot(X,H,'g.');axis equal;hold off;
% raodPointXH = [];
end

function raodPointXH = removeRoadNoise(wArray,wInfo,criWinCluIndex,cutL,cutR,rStart,rEnd)
%
%
raodPointXH = [];
flagL = 0;%0表示没有对边界窗口进行切割，1表示从左侧第一个关键窗口进行了切割，2表示从左侧第二个关键窗口进行了切割
flagR = 0;
cutLnum = cutL(1,2);
cutRnum = cutR(1,2);
cutL = cutL(1,1);
cutR = cutR(1,1);
%端点窗口处理
if cutL>=rStart&&cutL<rEnd,
    maxH = wInfo(cutL+1,2);%右邻窗口的最高点
   winPoint =  wArray{cutL};
   winPoint =  sortrows(winPoint,1);
   np = size(winPoint,1);
   for i=np:-1:1,
       H = winPoint(i,2);
       if H>maxH,
           break;
       end
   end
   raodPointXH =[raodPointXH;winPoint(i+1:np,:)];  
   flagL = cutLnum;
   rStart = cutL+1;
end
if cutR<=rEnd&&cutR>rStart,
    maxH = wInfo(cutR-1,2);%左邻窗口的最高点
   winPoint =  wArray{cutR};
   winPoint =  sortrows(winPoint,1);
   np = size(winPoint,1);
   for i=1:np,
       H = winPoint(i,2);
       if H>maxH,
           break;
       end
   end
   raodPointXH =[raodPointXH;winPoint(1:i-1,:)];  
   flagR = cutRnum;
   rEnd = cutR-1;
end

%突出路面的关键窗口处理，即滤除突出地面的点
num = size(criWinCluIndex,1);
for i = 1+flagL:num-flagR,
   indexL =  criWinCluIndex(i,1);
   indexR = criWinCluIndex(i,2);
   if size(wInfo,1)<indexR+1||1>indexL-1,
       a=0;%有时数组溢出，暂时未找到原因
       return;
   end
   maxHL = wInfo(indexL-1,2);
   maxHR = wInfo(indexR+1,2);
   maxH = max([maxHL;maxHR]);
   for m = indexL:indexR,
       winPoint =  wArray{m};
       winPoint = winPoint(winPoint(:,2)<maxH,:);
       raodPointXH = [raodPointXH;winPoint];      
   end
end

%其他处理，即未突出路面的点直接提取
if flagL~=0,flagL = flagL-1;end
if flagR~=0,flagR = flagR-1;end
for i = 1+flagL:num-flagR-1,
   indexL =  criWinCluIndex(i,2)+1;
   indexR = criWinCluIndex(i+1,1)-1;
   for m = indexL:indexR,
       winPoint =  wArray{m};
       raodPointXH = [raodPointXH;winPoint];      
   end
end

%边缘处理
indexL =  criWinCluIndex(1,1);
indexR =  criWinCluIndex(end,2);
for m = rStart:indexL-1,
    winPoint =  wArray{m};
    raodPointXH = [raodPointXH;winPoint];
end
for m = indexR+1:rEnd,
    winPoint =  wArray{m};
    raodPointXH = [raodPointXH;winPoint];
end

end

function bufferPoint = getbufferpoint(pointData,coefficients,length)
%
%
x = pointData(:,1);
y = pointData(:,2);
A = coefficients(1);
B = -1;
C = coefficients(2);
d = abs(A.*x+B.*y+C)./sqrt(A^2+1);
d(:,2) =1:size(d,1); 
bufferPoint = pointData(d(d(:,1)<length,2),:);
end

function pointData = getpointfrompseudo(projectArray,pseudoIndex,index)
%提取出伪扫描点对应的点
%   -projectArray:原始投影点数据的结构体
%   -pseudoIndex:伪扫描点与实际点的对应索引，如第一个伪扫描点对应哪些实际点
%   -index:需要提取的伪扫描点索引，如提取第三个伪扫描点
    nPseudoRoadPoint = size(index,1);
     pointData = [];
     for i=1:nPseudoRoadPoint,
          pIndex =   pseudoIndex{index(i)};
         px = projectArray.x(pIndex);
         py = projectArray.y(pIndex);
         ph = projectArray.h(pIndex);
         pins = projectArray.ins(pIndex);
         pointData = [pointData;px py ph pins];         
     end
end

function [pseudoScanline,pseudoIndex] = getpseudoscanline(pointData,width,type)
%
% -pseudoIndex:伪扫描线中的点所对应的的实际点集的索引
%从断面投影中生成伪扫描线
    nPoint = size(pointData,1);
    X = pointData(:,1);
    H = pointData(:,2);
    [X_sorted,index] = sortrows(X);
    XX = [X_sorted,index];
    nPseudoGrid = ceil((X_sorted(nPoint)-X_sorted(1))/width);
    pseudoX = X_sorted(1);
    nPseudoScanline = 0;
    for i=1:nPseudoGrid,
        pseudoX = pseudoX+width;
          XXtemp =  XX((XX(:,1)<pseudoX)&(XX(:,1)>=(pseudoX-width)),:);%按pseudoX从左到右生成
          if ~isempty(XXtemp),
              nPseudoScanline = nPseudoScanline+1;
              if strcmp(type,'min'),
                  pseudoH= min(H(XXtemp(:,2)));
              elseif strcmp(type,'max'),
                  pseudoH= max(H(XXtemp(:,2)));
              end  
             pseudoScanline(nPseudoScanline,:) = [pseudoX,pseudoH];
             pseudoIndex(nPseudoScanline,:) = {XXtemp(:,2)};
          end      
    end
%    if nPseudoScanline==0,
%        a=0;
%    end
end