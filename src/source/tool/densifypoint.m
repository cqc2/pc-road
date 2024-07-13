function densePointData = densifypoint(pointData,step)
 %
 [nPoint,~]= size(pointData);
 interpol = pointData';
% 样条曲线差值  
t=1:nPoint;  
ts = 1:step:nPoint;  
xys = spline(t,interpol,ts); 
 densePointData = xys';
end