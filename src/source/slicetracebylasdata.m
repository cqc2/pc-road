function traceDataOutput = slicetracebylasdata(traceData,ladData)
%
%根据时间戳提取出点云对应的轨迹数据
%-----以武汉数据二环单点轨迹数据为例
% traceData = importdata('dataspace_wuhan\dandian.txt');
% ladData = LASreadAll('dataspace_wuhan\origindata\data-20170418-085955.las');
lastime = ladData.time;
startTime = lastime(1);
endTime = lastime(end);

%将轨迹数据的时分秒转换成秒
tracetime = traceData(:,2);
tracetime = mod(tracetime.*1e-9,1).*1e9;%时分秒数字串，9位数
traceH = floor(tracetime.*1e-7);%时，2位数
tracetime = tracetime - traceH.*1e7;
traceMin = floor(tracetime.*1e-5);
tracetime = tracetime - traceMin.*1e5;%5位小数，秒和亚秒
traceScd = tracetime.*1e-3;
traceT = traceH.*3600+traceMin.*60+traceScd;%

dT1 = abs(traceT - startTime);
[~,index] = sortrows(dT1);
startIndex = index(1);

dT2 = abs(traceT - endTime);
[~,index2] = sortrows(dT2);
endIndex = index2(1);
nTrace = size(traceData,1);
if nTrace>index2(1)+4
    endIndex = index2(1)+4;
end
traceDataOutput = traceData(startIndex:endIndex,4:6);
traceDataOutput = densifypoint(traceDataOutput,1/50);
% savepointcloud2file(traceDataOutput,'ahsdjkladlkslk.xyz',0);
end