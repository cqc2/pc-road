function roadPoint = getroadbyscanline(ScanLineArray,curbHeight,minCurbSlope,fluctuateH,minRoadPoinNumber)
%abstract road point from scanline points.
%
% -minRoadPoinNumber:路面点最少点个数，小于此个数的扫描线段不认为是路面，此参
%                    数可以虑去个数较少的杂点，也可以滤掉远离路面的建筑物，次
%                    参数设置与扫描密度有关，一般可设置为道路上一条扫描线点个
%                    数的一半
% minRoadPoinNumber = 350;%适合武汉数据
% minRoadPoinNumber = 30;%适合sick扫描仪数据
if ~exist('curbHeight','var') || isempty(curbHeight),curbHeight = []; end
if ~exist('minCurbSlope','var') || isempty(minCurbSlope), minCurbSlope = []; end
if ~exist('fluctuateH','var') || isempty(fluctuateH), fluctuateH = []; end
if ~exist('minRoadPoinNumber','var') || isempty(minRoadPoinNumber), minRoadPoinNumber = 350; end
%由高度、点密度、几何形态综合筛选
    nScanLine = size(ScanLineArray,2);
    nPoint = 0;
    for i=1:nScanLine,
        x = ScanLineArray(i).x;
        nScanLinePoint = size(x,1);
        nPoint = nPoint+nScanLinePoint;
    end
    roadPoint = zeros(nPoint,4);
    nRoadPoint = 0;
    for i=1:nScanLine,
%         if i==278,
%             a=0;
%         end
        ScanLinePoint = ScanLineArray(i);
        x = ScanLinePoint.x;
        y = ScanLinePoint.y;
        h = ScanLinePoint.h;
        ins = ScanLinePoint.ins;   
        criticalPoint = getroughcriticalpoint(ScanLinePoint,curbHeight,minCurbSlope,fluctuateH);
        nScanLinePoint = size(ScanLinePoint.x,1);
        if nScanLinePoint<minRoadPoinNumber
            %此参数与数据点密度有关
            %小于30个点则忽略掉，因为过度切片时可能把少量杂点认为是一条扫描线
            continue;
        end
        ss = nScanLinePoint;
%             hold off;
% %             plot3(x(1:ss),y(1:ss),h(1:ss),'g-');hold on;
%             plot3(x(1:ss),y(1:ss),h(1:ss),'r.');hold on;
% %              plot3(x(1:ss),y(1:ss),h(1:ss),'b-');hold on;
% %             plot3(x(11:100),y(11:100),h(11:100),'r.');hold on;
% %             plot3(x(101:200),y(101:200),h(101:200),'g.');hold on;
% %             plot3(x(201:end),y(201:end),h(201:end),'b.');
%             axis equal;
%             a=0;
       
        nCriticalPoint = size(criticalPoint,2);
%         for o = 1:nCriticalPoint,
%             xx = x(criticalPoint(o));
%             yy = y(criticalPoint(o));
%             hh = h(criticalPoint(o));hold on;
%             plot3(xx,yy,hh,'bo');hold on;
%         end      
        [criticalPoint midHeight]  = revisecriticalpoint(criticalPoint,[x y h ins]);
%         for o = 1:nCriticalPoint,
%             xx = x(criticalPoint(o));
%             yy = y(criticalPoint(o));
%             hh = h(criticalPoint(o));hold on;
%             plot3(xx,yy,hh,'ro');hold on;
%         end   
        
        dn = criticalPoint(2:nCriticalPoint)-criticalPoint(1:nCriticalPoint-1);
        [ nContainPoint locate] = sort(dn,'descend');
        roadStartPointOrder = criticalPoint(locate(1));
        roadEndPointOrder = criticalPoint(locate(1)+1);
        x = x(roadStartPointOrder:roadEndPointOrder);
        y = y(roadStartPointOrder:roadEndPointOrder);
        h = h(roadStartPointOrder:roadEndPointOrder);
        ins = ins(roadStartPointOrder:roadEndPointOrder);
        
%         [criticalPoint midHeight]  = revisecriticalpoint([1 size(x,1)],[x y h ins]);
%         
%         roadStartPointOrder = criticalPoint(1);
%         roadEndPointOrder = criticalPoint(2);
%         x = x(roadStartPointOrder:roadEndPointOrder);
%         y = y(roadStartPointOrder:roadEndPointOrder);
%         h = h(roadStartPointOrder:roadEndPointOrder);
%         ins = ins(roadStartPointOrder:roadEndPointOrder);
        
        preNRoadPoint = nRoadPoint;
        nRoadPoint = preNRoadPoint + size(x,1);      
        roadPoint(preNRoadPoint+1:nRoadPoint,:) = [x y h ins];
%         ss = size(x,1);
%         ss1 = floor(ss/3);
%         ss2 = 2*ss1;
%             hold off;
%             plot3(x(1:ss),y(1:ss),h(1:ss),'r.');hold on;
% %             plot3(x(1:ss1),y(1:ss1),h(1:ss1),'r.');hold on;
% %             plot3(x(ss1:ss2),y(ss1:ss2),h(ss1:ss2),'g.');hold on;
% %             plot3(x(ss2:ss),y(ss2:ss),h(ss2:ss),'b.');hold on;
%             axis equal;
%             a=0;    
    end
    roadPoint = roadPoint(1:nRoadPoint,:);
     a=0;
end