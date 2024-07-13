function F_score = assess(name,type)
% assess the result of marking extraction
% F_score = assess(name,type)
% name - file name of marking image

addpath(genpath(pwd));
% name = 'erqi';type = 1;
% name = 'fazhan';type = 2;
% name = 'moshuihu';type = 3;
% name = 'yingwuzhou';type = 4;
img_manual = im2double(importdata(strcat(name,'-manual.png')));%人工结果
img = importdata(strcat(name,'.mat'));%原始图像
% imshow(img);
% img = imgaussfilt(img,1);
F_score = [];

% cropping parameter
if type==1
    T = 8;B = 10;L = 15;R = 0;
elseif type==2
    T = 0;B = 0;L = 12;R = 15;
elseif type==3
    T = 8;B = 0;L = 0;R = 10;
elseif type==4
    T = 28;B = 0;L = 0;R = 0;
end
f = calFscore(img,img_manual,14,1.33,T,B,L,R,1);
for s = 5:30
     for t = 0.8:0.1:1.8
          [f,f2,f3]= calFscore(img,img_manual,s,t,T,B,L,R,0.5);%erqi
%           [f,f2,f3]= calFscore(img,img_manual,s,t,0,0,12,15,0.5);%fazhan
%           [f,f2,f3]= calFscore(img,img_manual,s,t,8,0,0,10,0.5);;%moshuihu
%           [f,f2,f3]= calFscore(img,img_manual,s,t,8,0,0,10,0.5);;%yingwuzhou
          F_score = [F_score;s t f f2 f3];
    end
end
end

function [F_score,completeness,correctness]= calFscore(img,img_manual,s,t,T,B,L,R,fillpara)
%
if ~exist('T','var')||isempty(T) T = 0;end
if ~exist('B','var')||isempty(T) B = 0;end
if ~exist('L','var')||isempty(T) L = 0;end
if ~exist('R','var')||isempty(T) R = 0;end
if ~exist('fillpara','var')||isempty(fillpara) fillpara = 0.5;end
%图像裁剪
img = img(1+T:end-B,1+L:end-R);
img_manual = img_manual(1+T:end-B,1+L:end-R);

BW = imadaptive(img,s,t,[],fillpara);
% imshow(BW);
BW=bwareaopen(BW,30);
% se = strel('disk',5);
% BW = imclose(BW,se);
% se = strel('line',5,15);
% BW = imerode(BW,se);

BW = BW.*2;
dta = BW-img_manual;
% imshow(10.*(-dta));
[tp,~] = find(dta==1);
[fn,~] = find(dta==-1);
[fp,~] = find(dta==2);
TP = size(tp,1);%正常
FN = size(fn,1);%残缺
FP = size(fp,1);%溢出
completeness = TP/(TP+FN);
correctness = TP/(TP+FP);
F_score = 2*completeness*correctness/(completeness+correctness);

% % imshow(100*abs(dta));
imshow(BW);
% [r,c] = size(BW);
% pad = zeros([r+T+B,c+L+R]);
% pad(T+1:T+r,L+1:L+c) = BW;
% imwrite(pad,'tmp1.png');
% % figure(2);imshow(img_ywz_manuel);
end
