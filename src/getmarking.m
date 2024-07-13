function [markingPoint ,markingPoint2, markingPoint3, markingPointBW] = getmarking()
datetime('now','TimeZone','local','Format','HH:mm:ss Z')
addpath(genpath(pwd));
% pointCloudFilePath = 'dataspace_wuhan/roaddata/data-20170418-085955_road.xyz';
pointCloudFilePath = 'CASEDATA\YingwuzhouBridge\roaddata\yingwuzhou-alldata.xyz';
fid=fopen(pointCloudFilePath,'r');
pointCloudData = readpointcloudfile2(fid);
%  roughTraceData = searchtracepoint(pointCloudData);
%  traceData = curvefit(roughTraceData);
%  plot(pointCloudData(:,1),pointCloudData(:,2),'y.');hold on
%  plot(roughTraceData(:,1),roughTraceData(:,2),'g.');hold on
%  plot(traceData(:,1),traceData(:,2),'r.');
% axis equal;
% posFilePath = 'RawPointCloudData\POS_#2_20150703.txt';
% posData = readposfile(posFilePath,14);

% 使用精密轨迹文件（推荐）
posFilePath ='YingwuzhouBridge\tracedata\road-all_GNSStrace.xyz';
% posData = readposfile(posFilePath,14);
traceData = importdata(posFilePath);

posData.x = traceData(:,1);
posData.y = traceData(:,2);
posData.h = traceData(:,3);
posData = cutuselesspos(pointCloudData,posData,2);
% savepointcloud2file([posData.x posData.y posData.h zeros(size(posData.h,1),1)],'tracedata',1);
%切片不宜过厚，否则对拐弯处纵向切割时切割线会与路不平行
%切割用的pos数据要至少比点云长出一个切割长度，这样才能避免切割时道路端点附近被忽略
SliceArray = slice(pointCloudData,'bypos',posData,[0,25,10]);
nSlice = size(SliceArray,2);
PointSet= struct('x',0,'y',0,'h',0,'ins',0);
markingPoint=repmat(PointSet,[1 nSlice]);
%---------------------------
% 创建并行池
% if isempty(gcp('nocreate'))
%     parpool('local',5); 
% end
%--------------------------
for iSlice = 1:nSlice
% parfor iSlice = 1:nSlice
    x = SliceArray(iSlice).x;
    y = SliceArray(iSlice).y;
    h = SliceArray(iSlice).h;
    ins = SliceArray(iSlice).ins;
    info = SliceArray(iSlice).info;
    slicePoint = [x y h ins];
    cutLenght = 5;
    blockArray = slice(slicePoint,'direction',[],[info cutLenght]);
    markingpiece = [];
    markingpiece2 = [];
    markingpiece3 = [];
    markingpieceBW = [];
    for iBlock = 1:size(blockArray,2)
        data = blockArray(iBlock).data;
        if size(data,1)<10
            continue;
        end
        [imageData ,gridArray] = convertPD2img(data,0.05,0.2);
        % 计算背景灰度值。自适应分割阈值是由积分图确定，非路面区域的灰度值为0，这会对结
        % 果路面边缘标线的提取产生影响用路面区域的非标线低灰度值像素均值确定
        [imrow,imcol] = size(imageData);
        num = 0;
        for i = 1:imrow
            sampleGray = imageData(i,ceil(imcol/2));
            if sampleGray~=0
                num = num+1;
                grays(num) = sampleGray;
            end
        end
        grays = sort(grays);
        backGray = mean(grays(1:ceil(num))) ;
        imageData(imageData==0) = backGray;
        SensityValue = 0;
        BW = imbinarize(imageData,'adaptive','Sensitivity',SensityValue);
        thresh = mygraythresh(imageData);
        imageData2 = im2bw(imageData,thresh);
        seg_I  = mymultithresh(imageData,2);
        imageData3 = imageData;
        imageData3(isnan(imageData3)) = 0;
        imageData3(imageData3<seg_I(2)) = 0;
        imageData3(imageData3>=seg_I(2)) = 1;
        se = strel('line',3,3);
        imageData2 = imopen(imageData2,se);%ustu单阈值分割
        imageData3 = imopen(imageData3,se);%ustu多阈值分割灰度值大的结果
        BW = imopen(BW,se);%自适应阈值结果
        point2 = getpointfromgrid(gridArray,imageData2,1,12);
        point3 = getpointfromgrid(gridArray,imageData3,1,12);
        pointBW = getpointfromgrid(gridArray,BW,1,12);

%        figure(1);imshow(imageData);
%        figure(2);imshow(imageData2);
%         figure(3);imshow(imageData3);       
%         figure(4);imshow(BW);
        if iBlock~=1
            markingpiece = [markingpiece;point3];
%             if ~isempty(pointBW)
%            figure(5);     plot(pointBW(:,1),pointBW(:,2),'.');hold on
%             end
        else
            markingpiece = [markingpiece;pointBW];
%             if ~isempty(pointBW)
%            figure(5);     plot(pointBW(:,1),pointBW(:,2),'.');hold on
%             end
        end
        markingpiece2 = [markingpiece2;point2];
        markingpiece3 = [markingpiece3;point3];
        markingpieceBW = [markingpieceBW;pointBW];
    end    
%     markingPoint = [markingPoint;markingpiece];
%parpool并行计算时似乎不能用元胞，这里使用结构体数组。
if ~isempty(markingpiece)
    markingPoint(iSlice).x = markingpiece(:,1);
    markingPoint(iSlice).y = markingpiece(:,2);
    markingPoint(iSlice).h = markingpiece(:,3);
    markingPoint(iSlice).ins = markingpiece(:,4);
end
% markingPoint2(iSlice).x = markingpiece2(:,1);
% markingPoint2(iSlice).y = markingpiece2(:,2);
% markingPoint2(iSlice).h = markingpiece2(:,3);
% markingPoint2(iSlice).ins = markingpiece2(:,4);
% 
% markingPoint3(iSlice).x = markingpiece3(:,1);
% markingPoint3(iSlice).y = markingpiece3(:,2);
% markingPoint3(iSlice).h = markingpiece3(:,3);
% markingPoint3(iSlice).ins = markingpiece3(:,4);
% 
% markingPointBW(iSlice).x = markingpieceBW(:,1);
% markingPointBW(iSlice).y = markingpieceBW(:,2);
% markingPointBW(iSlice).h = markingpieceBW(:,3);
% markingPointBW(iSlice).ins = markingpieceBW(:,4);
end
%------------------
%关闭并行池
% parpool close 
%-----------------
 markingPoint =  getpointfromstructArray(markingPoint);
 savepointcloud2file(markingPoint,'sdgdfg',false);
%  plot(markingPoint(:,1),markingPoint(:,2),'.');
% imageData = convertpointcloud2img(markingPoint,0.05,0.2);
% imshow(imageData);

% fclose(fid);

% imwrite(imageData,'road.png');%图像生成
% img = imread('roadimage\22.png');
% gimg = edge(img,'Canny',0.2);
datetime('now','TimeZone','local','Format','HH:mm:ss Z')
end

function point = getpointfromgrid(gridArray,seg_I,num,type)
%
% -seg_I:整数化后的图像矩阵
% -num;要提取的数字区域
% -type:横纵坐标所在位数，12代表在1、2位，56代表在5、6位。
[row,col] = size(seg_I);
point = zeros(10000,4);
np = 0;
for m = 1:row
    for n = 1:col
        if seg_I(m,n)~=num
            continue;
        end
        p = gridArray{m,n};
        if (type==56)&&(~isempty(p))
            x = p(:,5);
            y = p(:,6);
            h = p(:,3);
            ins = p(:,4);
            p = [x y h ins]; 
        elseif type==12&&(~isempty(p))
            x = p(:,1);
            y = p(:,2);
            h = p(:,3);
            ins = p(:,4);
            p = [x y h ins];
        end       
        if seg_I(m,n)==num&&(~isempty(p))
            preNp = np;
            np = np + size(p,1);
            point(preNp+1:np,:) = p;
        end        
    end
end
point = point(1:np,:);
end

