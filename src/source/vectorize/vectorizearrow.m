function  [cluster,arrows] = vectorizearrow(cluster)
%
% 箭头标线识别
% [cluster,breakLines] = vectorizearrow(cluster)
% INPUT:
% cluster - 点云的聚类
% 
% OUTPUT:
% cluster - 处理后的点云聚类，暂时无用。
% arrows  - 第一列是箭头在cluster中的索引号，第二列是对应箭头类别编号

readtemplate();
nc = size(cluster,2);
if ~isfield(cluster,'center')
    cluster = getclusterinfo(cluster);
end
types = [];
breakLines = [];
for i = 1:nc
    c = cluster(i);
%         if i>=553
%             plot(c.data(:,1),c.data(:,2),'.');axis equal;
%             type = matchingarrow(c);
%               
%         end
%         continue;
    type = matchingarrow(c);
    if type~=0
        breakLines = [breakLines;i];
        types = [types;type];
    end
    if type~=0
        data = c.data;
      figure(1); plot(data(:,1),data(:,2),'k.');hold on;
    end
end
arrows = [breakLines types];
end

function type = matchingarrow(cluster)
%
% 箭头标线检测

global template;
w = cluster.length;
h = cluster.width;
% 与标线尺寸差的远的直接排除掉
if (w>6.2)||(w<5)||(h<0.3)||(h>2.4)
    type =  0;
    return;
else
    %生成点云影像的默认参数，
    pixelsize = 0.05;
    buffersize = 0.2;
    if cluster.density<100
        buffersize = 4/cluster.density;
    end
    
    [imageData,~] = convertPD2img(cluster.data,pixelsize,buffersize,true);
    imageData(~isnan(imageData)) = 1;
    imageData(imageData~=1) = 0;
    
    se = strel('diamond',2);
    imageData = imclose(imageData,se);
    imageData= imfill(imageData,'holes');% 填补孔洞
end
% figure(2);imshow(imageData);

% 影响识别率和处理速度的原因主要是标线缺损，如果标线缺损较少的话处理效果还是可以的
[type,T,imageData] = findmostmatchbyskel(imageData,0.01/pixelsize);
if type
    % 进行预处理，使模板与匹配图像大小、位置一致
    [tI,mI] =  preprocimage(abs(imresize(template{type}, 0.01/pixelsize)),imageData,T);
    % 匹配
    [regist_perct,defect_perct,exceed_perct,defect_area,exceed_area] =  matching(tI,mI);
    if ~analysispara(type,regist_perct,defect_perct,exceed_perct,defect_area,exceed_area)
        type = 0;
    end
end
end

function isMatched = analysispara(type,regist_perct,defect_perct,exceed_perct,defect_area,exceed_area)
%
% 对结果参数进行分析，最终判断标线类别

isMatched = false;
r07 = (regist_perct>0.7);
r06 = (regist_perct>0.6);
r05 = (regist_perct>0.5);
d05 = (defect_perct<0.5);
d03 = (defect_perct<0.3);
d01 = (defect_perct<0.1);
e01 = (exceed_perct<0.1);
e005 = (exceed_perct<0.05);
e001 = (exceed_perct<0.015);
if r07||(r06&&e01)||(r05&&e001)
    isMatched = true;
end
end

function  [regist_perct,defect_perct,exceed_perct,defect_area,exceed_area]...
    = matching(tImage,image)
%
%这里仅作微调，要求待匹配图像与模板大小一致，不能进行局部匹配识别

% figure(1);imshow(tImage);
% figure(2);imshow(image);
% figure(1);imshow(tImage + image);
[h,w] = size(image);
image2 = imrotate(image,180);
d = sum(sum((tImage - image).^2 - (tImage - image2).^2));
% figure(1);imshow(image);
% figure(2);imshow(image2);
% figure(3);imshow(tImage);

% 确定匹配方向
if d<0
    image3 = image;
else
    image3 = image2;
end

%--------------------------------------------------------------------------
% 骨架匹配已经大致匹配好了，这里再次进行纵、横、旋转匹配是否有必要？
tmp = zeros([h,w]);
r = 10;%在10个像素内进行微调匹配
% 检测纵向最佳匹配位置
for i = 0:r
    image_moveY1 = tmp;
    image_moveY2 = tmp;
    image_moveY1(1:end-i,:) = image(i+1:end,:);
    image_moveY2(i+1:end,:) = image(1:end-i,:);
    dImage = tImage - image_moveY1;
    dImage2 =tImage - image_moveY2;
    d(i+1,1) =  sum(sum(dImage.^2));
    d2(i+1,1) =  sum(sum(dImage2.^2));   
%     figure(1);imshow(tImage + image_moveY1);
%     figure(2);imshow(tImage + image_moveY2);
end 
[d_sort,idx] = sortrows([d;d2]);
pan_y = mod(idx(1),r+1)-1;%像素纵向平移量
if pan_y<0
    pan_y = pan_y+r+1;
end
image_moveY = tmp;
if idx(1)<=r+1
    image_moveY(1:end-pan_y,:) = image(pan_y+1:end,:);
else
    image_moveY(pan_y+1:end,:) = image(1:end-pan_y,:);
end

% 检测横向最佳匹配位置
image = image_moveY;
for i = 0:r
    image_moveX1 = tmp;
    image_moveX2 = tmp;
    image_moveX1(:,1:end-i) = image(:,i+1:end);
    image_moveX2(:,i+1:end) = image(:,1:end-i);
    dImage = tImage - image_moveX1;
    dImage2 =tImage - image_moveX2;
    d(i+1,1) =  sum(sum(dImage.^2));
    d2(i+1,1) =  sum(sum(dImage2.^2));  
%         figure(1);imshow(tImage + image_moveX1);
%         figure(2);imshow(tImage + image_moveX2);
end
[d_sort,idx] = sortrows([d;d2]);
pan_x = mod(idx(1),r+1)-1;%像素横向平移量
if pan_x<0
    pan_x = pan_x+r+1;
end
image_moveX = tmp;
if idx(1)<=r+1
    image_moveX(:,1:end-pan_x) = image(:,pan_x+1:end);
else
    image_moveX(:,pan_x+1:end) = image(:,1:end-pan_x);
end
%--------------------------------------------------------------------------
 
%检测旋转最佳匹配位置
image = image_moveX;
tImage_area =  sum(sum(tImage));%模板像素面积
rotateA = 10;%旋转角度范围
for i = -rotateA:rotateA
    image_rotate = imrotate(image,i);%旋转中心为图像中心
    [rh,rw] = size(image_rotate);
    mh = max([h rh]);
    mw = max([w rw]);
    tImage_ = zeros(mh,mw);% 待匹配模板
    tMatch_ = zeros(mh,mw);% 待匹配图像
    tImage_(ceil(mh/2-h/2+1):ceil(mh/2+h/2),ceil(mw/2-w/2+1):ceil(mw/2+w/2)) = tImage ;
    tMatch_(ceil(mh/2-rh/2+1):ceil(mh/2+rh/2),ceil(mw/2-rw/2+1):ceil(mw/2+rw/2)) = image_rotate;
    
    dImage = tImage_ - tMatch_;
%     figure(5);imshow(tMatch_);
%     figure(6);imshow(tImage_);
%     figure(3);imshow(tMatch_ + tImage_);

    defect_area(i+rotateA+1,1) = sum(dImage(dImage>0));% 待匹配图像相比模板缺损的像素面积
    exceed_area(i+rotateA+1,1) = abs(sum(dImage(dImage<0)));% 待匹配图像相比模多出的像素面积
    d4(i+rotateA+1,1) =  sum(sum(dImage.^2));
end
defect_perct = defect_area./tImage_area;
exceed_perct = exceed_area./tImage_area;
regist_perct = 1-(defect_perct+exceed_perct);
% 找出最大值
defect_area = min(defect_area);
exceed_area = min(exceed_area);
defect_perct = min(defect_perct);
exceed_perct = min(exceed_perct);
[regist_perct,idx] = max(regist_perct);
rA = idx-rotateA-1;
end

function [tI,mI] =  preprocimage(templateImage,matchImage,T)
% 
% 根据旋转矩阵将模板与待匹配图像调整到最佳大小和位置
% [tI,mI] =  preprocimage(templateImage,matchImage,T)
% INPUT：
% templateImage - 模板图像
% matchImage -待匹配图像
% T - 旋转矩阵
%
% OUTPUT:
% tI - 调整大小后的模板图像
% mI - 调整大小及点云位置后的待匹配图像

% %先旋转，在平移
% paraR = T(1:2,1:2);%旋转参数
% paraM = T(4,1:2);%平移参数
%将二值图像转换为点坐标
[my2,mx2] = find(matchImage>0);
[ty2,tx2] = find(templateImage>0);

% 坐标转换
transXY = [mx2 my2 zeros(size(mx2,1),1) ones(size(mx2,1),1)]*T;

% 确定统一边界，使由点云生成的图像大小位置一致
minX = min([min(transXY(:,1)) min(tx2)])-1;
minY = min([min(transXY(:,2)) min(ty2)])-1;
mx3 = transXY(:,1) - minX;
my3 = transXY(:,2) - minY;
tx3 = tx2 - minX;
ty3 = ty2 - minY;
maxX = max([max(mx3) max(tx3)]);
maxY = max([max(my3) max(ty3)]);
tmp = zeros([ceil(maxY) ceil(maxX)]);% 图像大小
tI = tmp;
mI = tmp;
% 由旋转矩阵T计算的图像
tI(sub2ind(size(tI),ceil(ty3),ceil(tx3)))=1;
mI(sub2ind(size(mI),ceil(my3),ceil(mx3)))=1;
mI = imclose(mI,strel('diamond',1));% I2进行了旋转，生成的图像可能有单像素空洞
end

function  [tform,skelRate] =  matchingskel(MovingPcd,fixedPcd)
%
% 骨架点云匹配
% [tform,skelRate] =  matchingskel(MovingPcd,fixedPcd)
% INPUT：
% MovingPcd - 移动点云，生成图像时点云发生旋转
% fixedPcd - 固定点云，生成图像时点云不发生旋转
%
% OUTPUT：
% tform - 将MovingPcd匹配到fixedPcd的旋转矩阵
% skelRate - MovingPcd与fixedPcd的匹配误差
%
% 这里使用的是三维点云的ICP算法，匹配时可能发生镜像旋转导致匹配错误，应该改成二维ICP
%ICP算法对初始位置有一定敏感性，角度差异过大可能使匹配结果首尾颠倒,应该0、180度匹配两次取最小
%会出现两类错误，1、不匹配的图像镜像后匹配了；2、匹配的图像初始误差过大（如角度）过大使结果显示不匹配
%--------------------------------------------------------------------------
%ICP算法假定一个点集是另一个点集的子集，这里模板近似可以看做全集；
%ICP算法要取得好的结果要求两个点集初始位置是近似对齐的，测试发现如果两个点集角
%度或纵向差距过大会使最终匹配结果错误，不过由于标线图像生成的点云都在一个矩形范
%围内，所以进行匹配时角度差在0度和180度附近，只需在0和180度两个方向计算ICP取误
%差较小的结果即可；
%ICP对初始位置敏感原因是以最近欧式距离确定对应点，这是武断的，对应点的不准确导
%致了最终匹配不理想甚至错误。结合ICP确定对应点的方式，可以看出标线匹配时初始位
%置纵向误差较大是导致匹配失败的原因，角度也是因为旋转后产生了纵向误差从而影响匹
%配效果的
%--------------------------------------------------------------------------
if (2==size(fixedPcd,2))
    fixedPcd = [fixedPcd zeros([size(fixedPcd,1) 1])];
end
if (2==size(MovingPcd,2))
    MovingPcd = [MovingPcd zeros([size(MovingPcd,1) 1])];%构造三维骨架点云
end

% plot(MovingPcd(:,1),MovingPcd(:,2),'r.');axis equal;hold on;
% plot(fixedPcd(:,1),fixedPcd(:,2),'b.');axis equal;hold on;

[tform,movingReg] = pcregrigid(pointCloud(MovingPcd),pointCloud(fixedPcd));
matchedPcd = movingReg.Location;
tform = tform.T;
% 骨架匹配度
Mdl = KDTreeSearcher(fixedPcd(:,1:2));
[Idx,D] = knnsearch(Mdl,matchedPcd(:,1:2));
skelRate1 = sqrt(sum(D.*D)/size(Idx,1));%均方误差描述匹配度,单位是像素

Mdl = KDTreeSearcher(matchedPcd(:,1:2));
[Idx,D] = knnsearch(Mdl,fixedPcd(:,1:2));
skelRate2 = sqrt(sum(D.*D)/size(Idx,1));% 模板到待匹配图像的骨架误差
skelRate = [skelRate1;skelRate2];
end

function [type,T,image] =  findmostmatchbyskel(image,scale)
%
% 通过影像生成的骨架确定与其最相似的标线模板
% [type,T,image] =  findmostmatchbyskel(image,scale)
% INPUT：
% image - 待检测的点云聚类影像
% scale - image的像素尺度，比如模板是按1cm/px生成的，image是2cm/pc,则同样大小
%         的image尺度是模板的一半，即scale=0.5
%
% OUTPUT ：
% type - 匹配出的模板类型编号
% T - 影像与模板最佳匹配时image的变换矩阵，通过T可以将影像与模板近似对齐
% image - 处理后的影响，暂时不起作用



% 0.01m一个像素生成的图像需要收缩40次得到骨架，像素越大所需收缩次数越烧少
% 使用'shrink'参数骨架化会导致端点一定程度收缩，其实应该使用'skel'，然后在去除枝
% 杈得到骨干，但是去除枝杈的算法得费点功夫
mSkel = bwmorph(image,'shrink',40*scale); %骨架化
% mSkel = bwmorph(image,'skel',inf); 

[my,mx] = find(imrotate(mSkel,0)>0);%二维骨架点
% figure(2);imshow(mSkel);

%去除离群点，离群点有可能是错误点
TT = clustereuclid([mx my],5);
[C,~,ic] = unique(TT);
tmpy = [];
tmpx = [];
rN = 4;%去掉少于4个点的
for i=1:size(C,1)
   idx =  find(ic==C(i));
%    plot(mx(idx),my(idx),'o','Color',[rand rand rand]);axis equal;hold on
   if size(idx,1)>rN 
       tmpx = [tmpx;mx(idx)];
       tmpy = [tmpy;my(idx)];
   end
end
% plot(mx,my,'b.');axis equal; hold on;
mx = tmpx;
my = tmpy;
% plot(mx,my,'r.');axis equal;

if isempty(mx)
    type = 0;
    T = eye(4);
    return;
end

matchPcd = [mx my zeros([size(mx,1) 1])];%构造三维骨架点云;
% figure(1);imshow(mSkel);
% figure(2);imshow(image);
[type,T] =  findmostmatchbyskelpcd(matchPcd,scale);
if type
     return;%如果已经匹配成功，则不再旋转180度检测
end
matchPcd2 = pcdroateXY(matchPcd,[mean(mx) mean(my)],180);
[type,T] =  findmostmatchbyskelpcd(matchPcd2,scale);
image = imrotate(image,180);
end

function [type,T] =  findmostmatchbyskelpcd(matchPcd,scale)
%
% 通过骨架点云匹配与其最相似的标线模板
% [type,T] =  findmostmatchbyskelpcd(matchPcd,scale)
% INPUT：
% matchPcd - 待匹配的骨架点云
% scale - image的像素尺度，比如模板是按1cm/px生成的，image是2cm/pc,则同样大小
%         的image尺度是模板的一半，即scale=0.5
% 
% OUTPUT ：
% type - 匹配出的模板类型编号
% T - 影像与模板最佳匹配时image的变换矩阵，通过T可以将影像与模板近似对齐
% 
% 注意：limit_R2是个重要参数

global skel;

% limit_R2是决定骨架匹配严格程度的参数，相当于匹配像素误差，值越大，匹配的标准
% 越低，这样缺损比较大标线骨架也可以匹配出结果（进而可以继续进行后面的模板匹配，
% 而不是在骨架匹配这一步就被过滤掉），但同时也会将许多错误的骨架匹配出来，增加
% 后面模板匹配的计算量和识别难度。
limit_R2 = 15;

%估计标线大小
% [h,w] = size(image);
% h = (h*(1/scale))/100;
% w = (w*(1/scale))/100;

%这里暂时不进行进一步判断，而是对所有模板进行匹配，逻辑简单，效率较低
rate = [];
for i=1:8
    pcd = skel{i};
    np = size(skel{i},1);%骨架点个数
    p = randperm(np,ceil(np.*scale));%随机采样
    [tform{i},skelRate(i,1:2)] =  matchingskel(matchPcd,pcd(p,:).*scale);
%     ptCloudOut = pctransform(pointCloud(matchPcd),affine3d(tform{i}));
%     ptCloudOut = ptCloudOut.Location;hold off;
%     plot(pcd(p,1).*scale,pcd(p,2).*scale,'r.','markersize',10);hold on; axis equal;
%     plot(matchPcd(:,1),matchPcd(:,2),'b.','markersize',10); axis equal;
%     plot(ptCloudOut(:,1),ptCloudOut(:,2),'g.','markersize',10); axis equal;
    T = tform{i};
    R = skelRate(i,1:2);
    R1 = R(1);
    R2 = R(2);
    % 如果非直行箭头的图像是镜像匹配，或者正向匹配误差超过2个像素，或者反向匹
    % 配误差超过5个像素,认为匹配无效
    % 由于骨架算法不完善，得到的骨架端点会收缩，所以R2设置的大一些，其实如果骨
    % 架误差小一些的话，R2应该之比R1阈值略大，比如R2=5；
    if (-1==T(3,3)&&(i~=1))||(R1>2||R2>limit_R2)
         rate = [rate;inf];
        continue;
    end
    rate = [rate;sqrt(R*R'/2)];
end
[m,I] = min(rate);
if m<(limit_R2/2)
    type = I;
    T = tform{type};
else
    type = 0;
    T = tform{eye(4)};
end
end

function pcdRoatated = pcdroateXY(pcd,point,rA)
%
% 将点云pcd以point点为轴旋转rA度
% pcdRoatated = pcdroateXY(pcd,point,rA)

if (2==size(pcd,2))
    pcd = [pcd zeros([size(pcd,1) 1])];
end
cx = point(1,1);
cy = point(1,2);
a = cosd(rA);
b =  -sind(rA);
c = sind(rA);
d =  cosd(rA);
rT = [a b 0 0;c d 0 0;0 0 1 0;0 0 0 1];
mT = [1 0 0 0;0 1 0 0;0 0 1 0;-cx -cy 0 1];
mT2 = [1 0 0 0;0 1 0 0;0 0 1 0;cx cy 0 1];
% mT*rT*mT2,即先进行mT平移，在进行rT旋转，在进行mT2平移，mT、mT2大小相等，方向相反
ptCloudOut = pctransform(pointCloud(pcd),affine3d(mT*rT*mT2));
pcdRoatated = ptCloudOut.Location;
end

function readtemplate()
%
%读取模板数据

global template_go;
global template_go_right;
global template_go_left;
global template_right;
global template_left;
global template_around;
global template_merge_left;
global template_merge_right;
template_go = im2double(rgb2gray(255-imread('source\markingimage\go.png')));
template_go_right = im2double(rgb2gray(255-imread('source\markingimage\go_right.png')));
template_go_left = im2double(rgb2gray(255-imread('source\markingimage\go_left.png')));
template_right = im2double(rgb2gray(255-imread('source\markingimage\right.png')));
template_left = im2double(rgb2gray(255-imread('source\markingimage\left.png')));
template_around = im2double(rgb2gray(255-imread('source\markingimage\around.png')));
template_merge_left = im2double(rgb2gray(255-imread('source\markingimage\merge_left.png')));
template_merge_right = im2double(rgb2gray(255-imread('source\markingimage\merge_right.png')));
global template;
template= cell([8 1]);
template(1) = {template_go};
template(2) = {template_go_right};
template(3) = {template_go_left};
template(4) = {template_right};
template(5) = {template_left};
template(6) = {template_around};
template(7) = {template_merge_left};
template(8) = {template_merge_right};


%读取骨架点云
%右为箭头骨架方向，其最小外接矩形长边为x轴，短边为y轴
%注意：生成模板骨架是0.01米一个像素，而点云生成的图像一般用的0.02米，处理过程
%      中注意他们之间的影响关系
global skel_go;
global skel_go_right;
global skel_go_left;
global skel_right;
global skel_left;
global skel_around;
global skel_merge_left;
global skel_merge_right;
global skel;
skel_go = load('source\markingimage\go.mat');skel_go = skel_go.skelPcd;
skel_go_right = load('source\markingimage\go_right.mat');skel_go_right = skel_go_right.skelPcd;
skel_go_left = load('source\markingimage\go_left.mat');skel_go_left = skel_go_left.skelPcd;
skel_right = load('source\markingimage\right.mat');skel_right = skel_right.skelPcd;
skel_left = load('source\markingimage\left.mat');skel_left = skel_left.skelPcd;
skel_around = load('source\markingimage\around.mat');skel_around = skel_around.skelPcd;
skel_merge_left = load('source\markingimage\merge_left.mat');skel_merge_left = skel_merge_left.skelPcd;
skel_merge_right = load('source\markingimage\merge_right.mat');skel_merge_right = skel_merge_right.skelPcd;
skel= cell([8 1]);
skel(1) = {skel_go};
skel(2) = {skel_go_right};
skel(3) = {skel_go_left};
skel(4) = {skel_right};
skel(5) = {skel_left};
skel(6) = {skel_around};
skel(7) = {skel_merge_left};
skel(8) = {skel_merge_right};
end



