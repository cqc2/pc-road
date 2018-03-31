function caocaocao
    addpath(genpath(pwd));
% hhh('CASEDATA\FazhanAvenue\roaddata-0.08£¬25,0.05,350\data-20170418-091242_road');
% hhh('CASEDATA\FazhanAvenue\roaddata-0.08£¬25,0.05,350\data-20170418-091319_road');

% hhh('CASEDATA\FazhanAvenue\roaddata-0.08£¬25,0.05,350\data-20170418-091358_road');
% hhh('CASEDATA\FazhanAvenue\roaddata-0.08£¬25,0.05,350\data-20170418-091444_road');

hhh('CASEDATA\MoshuiLake\roaddata\data-20170418-092448_road');
hhh('CASEDATA\MoshuiLake\roaddata\data-20170418-092522_road');
hhh('CASEDATA\MoshuiLake\roaddata\data-20170418-092557_road');
hhh('CASEDATA\MoshuiLake\roaddata\data-20170418-092631_road');
hhh('CASEDATA\YingwuzhouBridge\roaddata\data-20170418-093156_road');
hhh('CASEDATA\YingwuzhouBridge\roaddata\data-20170418-093227_road');
hhh('CASEDATA\YingwuzhouBridge\roaddata\data-20170418-093321_road');
hhh('CASEDATA\YingwuzhouBridge\roaddata\data-20170418-093415_road');
end

function hhh(filename)
datetime('now','TimeZone','local','Format','HH:mm:ss Z')
data = readpointcloudfile2(strcat(filename,'.xyz'));
[imageData ,gridArray] = convertPD2img(data,0.05,0.2);
imwrite(imageData,strcat(filename,'1.png'));
BW = imadaptive(imageData,12,1.25);
imwrite(BW,strcat(filename,'2.png'));
BW2=bwareaopen(BW,50);
imwrite(BW2,strcat(filename,'3.png'));
markingpoint = getpointfromgrid(gridArray,BW2,1,12);
savepointcloud2file(markingpoint,strcat(filename,'_marking'),0);
datetime('now','TimeZone','local','Format','HH:mm:ss Z')
end