function criticalPoint = getroughcriticalpoint(ScanLinePoint,curbHeight,minCurbSlope,fluctuateH)
%detect critical points rooughly by height and slope.
%     curbHeight、minCurbSlope、fluctuateH是三个非常重要的参数，对最终路面点
% 云的提取效果影响显著，curbHeight与需要检测出的路面边界突出物（路砍、挡板）的
% 高度有关，一般设置为比突出物高度值略小（理想值为扫描线打在突出物上的最小长度）；
% minCurbSlope反映了凸出物与路面的夹角，minCurbSlope值设置的越小边界提取越严格，
% 越大边界提取越宽泛，在不使扫描线跳线的情况下越小越好；fluctuateH反映处理的路
% 面点云的高程波动程度，大于fluctuateH值的会认为是边界.
%-------默认参数----------------------
%     -curbHeight ： 0.08;
%     -minCurbSlope ： 25;
%     -fluctuateH ：在扫描线中为0.05，伪扫描线中适当增大如0.15，若伪扫描线是由
%                   多个扫描仪数据生成得，该参数值可设置为0.2~0.3；
%
if ~exist('curbHeight','var') || isempty(curbHeight),curbHeight = 0.08; end
if ~exist('minCurbSlope','var') || isempty(minCurbSlope), minCurbSlope = 25; end
if ~exist('fluctuateH','var') || isempty(fluctuateH), fluctuateH = 0.05; end
 
    x = ScanLinePoint.x;
    y = ScanLinePoint.y;
    h = ScanLinePoint.h;
    ins = ScanLinePoint.ins;
    nScanLinePoint = size(x,1);
    preX = x(1);
    preY = y(1);
    preH = h(1);
    nCriticalPoint = 0;
    criticalPoint = [];
        for m=2:nScanLinePoint,
            currentX = x(m);
            currentY = y(m);
            currentH = h(m);
            dh = abs(currentH-preH);
            dxy = sqrt((currentX-preX)^2+(currentY-preY)^2);
            ds = sqrt(dxy^2+dh^2);
            if (ds>curbHeight)&&(dxy~=0),
                slope = atand(dh/dxy);
                if (slope>minCurbSlope)||abs(dh)>fluctuateH,
                    nCriticalPoint = nCriticalPoint+1;
                    criticalPoint(nCriticalPoint) = m;
                end
                preH = currentH;
                preX = currentX;
                preY = currentY;
            elseif (ds>curbHeight)&&(dxy==0),
                nCriticalPoint = nCriticalPoint+1;
                criticalPoint(nCriticalPoint) = m;
                preH = currentH;
                preX = currentX;
                preY = currentY;
            end           
        end
        criticalPoint = [1 criticalPoint nScanLinePoint];
end