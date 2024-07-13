function clusterArray_out = cluster2straightline(clusterArray,mileageArray) 
% divide long winding lines into several short straight lines and drop
% lines which are perpendicular to the direction of trace
nc = size(clusterArray,2);
iCluster = 0;
for i=1:nc
    cluster = clusterArray{i};
    x = cluster(:,1);
    y = cluster(:,2);
    z = cluster(:,3);
    [width,height,A] = getinfominboundrect(x,y);
    minX = min(x);
    minY = min(y);
    
    %小于3米长的丢弃
    if width<3
        continue;
    end
    
    %将线段方向旋转与横轴大致平行
    x0= (x - minX).*cos(-A) - (y - minY).*sin(-A) ;%逆时针旋转A
    y0= (x - minX).*sin(-A) + (y - minY).*cos(-A) ;
%     plot(x - minX,y - minY,'b.');hold on;
%     plot(x0,y0,'r.');axis equal;


    %去掉突变点
     np = size(x0,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %moshuihu 1.4 1.4 15
    %最适合参数 1.4 1.4 14
    W = 0.9;%滑动窗口宽度
    H = 1.5;%限高
    Angle = 15;%角度差限制
    %%%%%%%%%%%%%
    tmpdata= [];
%     f=0;
    for  po = min(x0):W:max(x0)
      [idx,~]= find( x0>=po&x0<(po+W));
      y_1= y(idx);
      x_1= x(idx);
      z_1= z(idx);
      
      [width,height,A,cx,cy] = getinfominboundrect(x_1,y_1);
      [IDX, D]= knnsearch(mileageArray(:,1:2),[cx,cy]);
      direct_trace = mileageArray(IDX,5);
%       
%       for kk=1:10
%           mx = mileageArray(IDX+kk,1);
%           my = mileageArray(IDX+kk,2);
%           plot(mx,my,'r.');      hold on;
%       end
%       plot(x_1,y_1,'k.');        
%       axis equal;
      
      if A<0
          A = A+pi;
      end
      if direct_trace<0
          direct_trace = direct_trace+pi;
      end
      dA = rad2deg(abs(direct_trace-A));
      if dA>90
          dA = 180-dA;
      end
      h = max(y_1)-min(y_1);
      if (h>H&~isempty(tmpdata))||(dA>Angle&~isempty(tmpdata))
          iCluster = iCluster+1;
          clusterArray_out(iCluster) = {tmpdata};
%         plot(x_1,y_1,'r.');hold on
          tmpdata = [];
      elseif h<=H&dA<=Angle
          tmpdata = [tmpdata;x_1 y_1 z_1];
      end
    end
    if ~isempty(tmpdata)
        iCluster = iCluster+1;
        clusterArray_out(iCluster) = {tmpdata};
    end

end
if ~exist('clusterArray_out','var')
    clusterArray_out = cell(0);
end
a=0;
end

function [width,height,A,cx,cy] = getinfominboundrect(x,y)
    
    [rectx,recty,~,~] = minboundrect(x,y);
    cx = mean(rectx);
    cy = mean(recty);
    a = sqrt((rectx(1)-rectx(2))^2+(recty(1)-recty(2))^2);
    b = sqrt((rectx(2)-rectx(3))^2+(recty(2)-recty(3))^2);
    if a>=b
        A = atan2(recty(1)-recty(2),rectx(1)-rectx(2));
        width = a;
        height = b;
    else
        A = atan2(recty(2)-recty(3),rectx(2)-rectx(3));
        width = b;
        height = a;
    end
end
