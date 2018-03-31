function [numarr sarr] =  readmbfile(filePath)
% [numarr sarr] =  readmbfile(filePath)
% INPUT：
% filePath - 文件路径
% 
% OUTPUT：
% numarr - 读取点的数字
% sarr - 读取的字符串
% copyright CQC

data = importdata(filePath);
[num rangeArrayCol] = size(data);
    for n = 1:num
        S = regexp(data(n), '\s+', 'split');
        S = S{1};
         data1 = S{1};
         numarr(n,1) = str2num(data1);
         remian =  S{2};
         S2 = regexp(remian, ',', 'split');
         numarr(n,3) = str2num(S2{2});
         for i=6:15
             numarr(n,i) =  str2num(S2{i-2});
         end
         S3 = regexp(S2{3}, ';', 'split');
         numarr(n,4) =  str2num(S3{1});
         numarr(n,5) =  str2num(S3{2});
         str1 = S2{1};
         str2 = S2{end};
        sarr(n,1) = {str1};
        sarr(n,2) = {str2};
    end
end
