function pointData = correctIntensity(pointData,info,calibdata)
% correct intensity of point clouod
% 
% 
sliceX = info(1);
sliceY = info(2);
sliceK = info(3);

PointSet= struct('data',[],'range',[]);
SliceArray=repmat(PointSet,[1 50]);
x = pointData(:,1);
y = pointData(:,2);
%对点云进行旋转
A = pi/2-sliceK;%旋转角

%将坐标转换到以切点为原点，切线为纵坐标点的坐标系中
x0= (x - sliceX).*cos(A) - (y - sliceY).*sin(A) ;%逆时针旋转A，A是四象限角，旋转后Y轴指向轨迹前进方向
y0= (x - sliceX).*sin(A) + (y - sliceY).*cos(A) ;

%校正
minx0 = min(x0);
miny0= min(y0);
maxx0 = max(x0);
maxy0 = max(y0);
w = 0.05;
for loc=minx0:w:maxx0
    locStart  = (loc-w/2);
    locEnd = (loc+w/2);
    correctvar1= mean(calibdata(calibdata(:,1)>locStart&calibdata(:,1)<=locEnd,2));
    idx = find(x0<=locEnd&x0>locStart);
    if ~isnan(correctvar1)&~isempty(idx)
        pointData(idx,4)=pointData(idx,4)-correctvar1;
    end 
end
end

