function I = imfillnan(I,type,t)
% 将灰度图像中nan或者0值的像素用图像的采样均值填充，避免边界处分割错误
% t - 灰度填充系数，默认值为1，如果是灰度不均匀的图像，可令t=0.5
% 
% 最理想的是做边缘灰度值延拓，这里用均值代替
% 

if ~exist('type','var')||isempty(type),type = 0;end
if ~exist('t','var')||isempty(t),t = 1;end
if 0==type
    %全局均值填充
      [r,c] = size(I);
        num = 0;
        grays = [];
        for i = 1:r
            sampleGray = I(i,ceil(c/2));
            if (sampleGray~=0)&~isnan(sampleGray)
                num = num+1;
                grays(num) = sampleGray;
            end
        end
        if isempty(grays)
            return;
        end
        grays = sort(grays);
        backGray = mean(grays(1:ceil(num)));
else
    %边界处8邻域均值(默认)
    I(isnan(I))=0;
    I2 = I;
    I2(I2~=0)=1;
    [gx,gy ]= gradient(I2);
    gg=gx.*gx+gy.*gy;
    [cx,cy] = find(gg);
    size(cx,1);
    [w,h] = size(I);
    arr = [];
    for i = 1:size(cx,1)
        if(cx(i)>1&&cy(i)>1&&cx(i)<w&&cy(i)<h)
            arr = [arr;I(cx(i)-1,cy(i)-1) I(cx(i)-1,cy(i)) I(cx(i)-1,cy(i)+1) I(cx(i),cy(i)-1)...
                I(cx(i),cy(i)+1) I(cx(i)+1,cy(i)-1) I(cx(i)+1,cy(i)) I(cx(i)+1,cy(i)+1)];
        end
    end
    backGray = mean(arr(arr>0));
end
I(isnan(I)) = backGray*t;
I(I==0) = backGray*t;
end