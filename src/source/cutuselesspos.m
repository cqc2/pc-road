function outpos = cutuselesspos(pointCloudData,posData_in,extendD)
% 
% outpos = cutuselesspos(pointCloudData,posData_in,extendD)
% 去除不需要的pos数据，只留下点云区域内的pos数据
% pointCloudData - 点云数据
% posData_in  - 轨迹数据，结构体类型
% extendD - 轨迹数据切割时两头需要延长的距离
%

if ~exist('extendD','var')||isempty(extendD),extendD = 0;end
posData = [posData_in.x posData_in.y posData_in.h];

np = size(pointCloudData,1);
npos = size(posData,1);
if np<10001
    tracePoint1 = searchtracepoint(pointCloudData);
    tracePoint2 = tracePoint1;
else
    tracePoint1 = searchtracepoint(pointCloudData(1:10000,:));
    tracePoint2 = searchtracepoint(pointCloudData(end-10000:end,:));
end
Mdl = KDTreeSearcher(posData(:,1:2));
[idx1,d1] = knnsearch(Mdl,tracePoint1(:,1:2));
[idx2,d2] = knnsearch(Mdl,tracePoint2(:,1:2));
r1 = sortrows([idx1,d1],2);
r2 = sortrows([idx2,d2],2);
idx1 = r1(1,1);
d1 = r1(1,2);
idx2 = r2(1,1);
d2 = r2(1,2);


 extendN = ceil(extendD/norm(posData(1,:)-posData(2,:)))+10;%延长的pos点个数
 if (idx1-extendN)>0
     idx1 = idx1-extendN;
 end
  if (idx2+extendN)<=npos
     idx2 = idx2+extendN;
  end
  if d1<1&d2<1
      outpos.x = posData(idx1:idx2,1);
      outpos.y = posData(idx1:idx2,2);
      outpos.h = posData(idx1:idx2,3);
  else
      outpos = posData_in;
  end
%   plot(pointCloudData(:,1),pointCloudData(:,2),'g.');hold on;
%   plot(outpos.x,outpos.y,'r.');
end