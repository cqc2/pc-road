function calibdata = calibratintensity(pointData,sliceInfo)
% calibrat intensity 

      sliceX = sliceInfo(1);
      sliceY = sliceInfo(2);
      sliceK = sliceInfo(3);
x = pointData(:,1);
y = pointData(:,2);
%对点云进行旋转
A = pi/2-sliceK;%旋转角

%将坐标转换到以切点为原点，切线为纵坐标点的坐标系中
x0= (x - sliceX).*cos(A) - (y - sliceY).*sin(A) ;%逆时针旋转A
y0= (x - sliceX).*sin(A) + (y - sliceY).*cos(A) ;

%切片
w = 0.05;
minx0 = min(x0);
maxx0 = max(x0);
IArray = [];
tmp=0;
for loc=minx0:w:maxx0
    loc1 = loc-w/2;
    loc2 = loc1+w;
    IL= mean(pointData(x0<=loc2&x0>loc1,4));
    if isnan(IL)
        IArray = [IArray;loc tmp];
    else
        tmp = IL;
        IArray = [IArray;loc IL];
    end
end
% [imageData ,~] = convertPD2img(pointData,0.05,0.2);
% figure(1);imshow(imageData);
[IArray,~ ]= sortrows(IArray);
calibdata = getbackcurve(IArray);
calibdata(:,2) = calibdata(:,2)-min( calibdata(:,2));

% x = calibdata(:,1);
% y = calibdata(:,2);
% x = -fliplr(x);
% y = fliplr(y);
% calibdata = [x y];
end

function out = getbackcurve(points)
%
x = points(:,1);
y = points(:,2);

% 将数据分为三部分
idx1 = find(x<-3);
idx2 = find(x>=-3&x<3);
idx3 = find(x>=3);
points1 = [x(idx1) y(idx1)];
points2 = [x(idx2) y(idx2)];
points3 = [x(idx3) y(idx3)];
points1 = removepeak(points1,0.5);%暗区域
points2 = removepeak(points2,3);%亮区域
points3 = removepeak(points3,0.5);%暗区域

%  points2(:,2) = points2(:,2)*0.8;

out = [points1;points2;points3];
out(:,2) = smooth(out(:,2),15);
%     figure(10);plot(x,y,'k-');hold on;axis off;
%     figure(10);plot(x,y,'k.');hold on;axis off;
%     plot(out(:,1),out(:,2),'g-');hold on;
%     plot(out(:,1),out(:,2),'g.');hold off
% %     img_grad = smooth(abs(out(2:end,2)-out(1:end-1,2)),30);
% %     plot(out(2:end,1),img_grad*5,'-','Color',[rand rand rand]);
% %     axis([-10,10,-1,60000]);hold off;
%     pause(1);
end

function points = removepeak(points,sigma)
%
% sigma设置的越大越校正曲线越切近实际灰度分布，但去掉更多背景同时标线被去掉的可能性也增大

if ~exist('sigma','var')||isempty(sigma), sigma = 3;end
x = points(:,1);
y = points(:,2);

% 将y异常值用平滑值代替
yy  = smooth(y,5);
idx =  find(isnan(y));
y(idx) = yy(idx);

np = size(x,1);
rate = 1;
while rate>0.01
    yy = smooth(y,15);
    dy = y-yy;
    std0 = std(dy);
    idx =  find(dy>sigma*std0);
    y(idx) = yy(idx);
    rate  =  idx/np;
end
points(:,2) = y;
end







