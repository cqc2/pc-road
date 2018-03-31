function out = imadaptive(img,s,t,sigma,fillpara)
% out = imadaptive(img,s,t,sigma)
% 基于积分图的自适应二值化
%
% Parameters：
% s = 10      - 自适应半径，要大于分割对象的半径，一般s越大越能反映细节，不过噪声也
%               会多些，值越小细节被忽略，噪声也少些
% t = 0.99    - 分割比率,可以大于1，值越大噪声越少，但是信号也会丢失
% sigma = 1.5 - 滤波参数，值越大滤掉的噪声点半径越大,细节也会丢失
% fillpara=1    - 背景填充系数，如果是处理灰度校正后的图像，值设为1即可，如果是
%               背景灰度分布不均匀的图像，可设置为0.5左右
%
%
% Reference：
% Roth D B G. Adaptive Thresholding using the Integral Image[J]. Journal of 
% Graphics Gpu & Game Tools, 2007, 12(2):13-21.

% 说明：
% 自适应阈值分割实现有两种思路，第一种就如参考论文中所叙述那样，通过积分图计算
% 每个像素点的分割阈值(BW=I>T);另一种思路类似顶帽变换，直接对原始图像进行低通
% 滤波，滤波后的图像就就是背景图像(BW = I>t*I2);这两种思路本质是一样的，都是计
% 算每个像素周围的像素，这里使用前者。

if ~exist('s','var')||isempty(s)
    % 图像尺寸较大时效果不是很理想,最好自己设置s大小
%     nhoodSize =  2*floor(size(img)/16)+1;
%     padSize =(nhoodSize-1)/2;
    padSize = [10 10];
elseif size(s,2)==1
     padSize = [s s];
else
    padSize = s;
end
if ~exist('t','var')||isempty(t),t=1;end
if ~exist('sigma','var')||isempty(sigma),sigma=1;end
if ~exist('fillpara','var')||isempty(fillpara),fillpara=1;end

%预处理
if 3==size(img,3)
    img = rgb2gray(img);
end
img = im2double(img);
I = imfillnan(img,[],fillpara);
I = padarray(I,padSize,'replicate','both');%边缘扩展

% 高斯滤波
I = imgaussfilt(I,sigma);%高斯滤波

% 均值滤波
% h = fspecial('average', [10 10]);
% I=filter2(h,I);

% 计算积分图
intImg = integralImage(I);
intImg = intImg(2:end,2:end);

% 积分图盒式滤波,就是均值滤波
% intA, filterSize, normFactor, outType, outSize
% B = integralBoxFilter(intImg,[s s]);

% 自适应二值化
[w,h] = size(intImg);
ws = padSize(1);
hs = padSize(2);
for i=1+ws:w-ws
    for j=1+hs:h-hs
        x1=i-ws;
        x2=i+ws;
        y1=j-hs;
        y2=j+hs;
        count=(x2-x1)*(y2-y1);
        sum=intImg(x2,y2)-intImg(x2,y1)-intImg(x1,y2)+intImg(x1,y1);
        if(I(i,j)*count)<=(sum*t)
            out(i-ws,j-hs)=0;
        else
            out(i-ws,j-hs)=1;
        end
    end
end
% out = imclose(out,strel('disk',3));
% imshow(out);
% T = adaptthresh(img,'Statistic','mean');
% T = adaptthresh(img,0);
% imshow(img>T);
end

function intI = getintimg(I)
% 计算积分图
[w,h]=size(I);
intImg=zeros([w h]);
for i=1:w
    sum=0;
    for j=1:h
        sum=sum+I(i,j);
        if i==1
            intImg(i,j)=sum;
        else
            intImg(i,j)=intImg(i-1,j)+sum;
        end
    end
end
intI = intImg;
end

