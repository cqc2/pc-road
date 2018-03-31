function clusterArray = clusterpoint(pointData,breakDist,type)
%cluster ordered points by distance between two adjacent points
if ~exist('type','var')||isempty(type),type=2;end
    clusterArray = {};
    nCluster = 0;
    [nPoint,dim] = size(pointData);
    if nPoint==0,
        return;
    elseif nPoint==1,
        clusterArray(1) = {pointData};
        return;
    end
    breakStart = 1;
    breakEnd = 1;
    if dim==2||type==2,
        for iPoint = 2:nPoint,
            breakEnd = iPoint;
            xPre = pointData(iPoint-1,1);
            yPre = pointData(iPoint-1,2);
            x = pointData(iPoint,1);
            y = pointData(iPoint,2);
            dist = norm([xPre-x yPre-y]);
            if dist>=breakDist,
                nCluster = nCluster+1;
                clusterArray(nCluster) = {pointData(breakStart:breakEnd-1,:)};
                breakStart = breakEnd;
            end
        end
    elseif dim==3,
        for iPoint = 2:nPoint,
            breakEnd = iPoint;
            xPre = pointData(iPoint-1,1);
            yPre = pointData(iPoint-1,2);
            hPre = pointData(iPoint-1,3);
            x = pointData(iPoint,1);
            y = pointData(iPoint,2);
            h = pointData(iPoint,3);
            dist = norm([xPre-x yPre-y hPre-h]);
            if dist>=breakDist,
                nCluster = nCluster+1;
                clusterData = pointData(breakStart:breakEnd-1,:);
                clusterArray(nCluster) = {clusterData};
                breakStart = breakEnd;
            end
        end
    else
        return;
    end
    clusterArray(nCluster+1) = {pointData(breakStart:nPoint,:)};
end