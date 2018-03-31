function originalname = getoriginalname(filename)
%
%提取不带后缀原始文件名
 S = regexp(filename, '_', 'split');
 if size(S,1)==1
    [path,originalname,type]=fileparts(S{1});
    return;
 end
 originalname = S{1};
end