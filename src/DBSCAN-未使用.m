function [IDX, isnoise]=DBSCAN(X,epsilon,MinPts)
% 密度聚类，将大于一定点密度且联通的区域作为一类
% DBSCAN，Density-Based Spatial Clustering of Applications with Noise
% [IDX, isnoise]=DBSCAN(X,epsilon,MinPts)
%
% INPUT:
% 
% 
% 
% 
% OUTPUT:
%
% REFERENCES:
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPML110
% Project Title: Implementation of DBSCAN Clustering in MATLAB
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%
% data = importdata('ttttt.xyz');
% X = data(40000:60000,1:2);
epsilon = 0.1;
MinPts = 10;
    C=0;
    n=size(X,1);
    IDX=zeros(n,1);
%     D=pdist2(X,X); 
    Mdl = KDTreeSearcher(X);
    
    visited=false(n,1);
    isnoise=false(n,1);
    
    for i=1:n
        if ~visited(i)
            visited(i)=true;
            
            Neighbors=RegionQuery(i);
            if numel(Neighbors)<MinPts
                % X(i,:) is NOISE
                isnoise(i)=true;
            else
                C=C+1;
                ExpandCluster(i,Neighbors,C);
            end
        end 
    end
    
    function ExpandCluster(i,Neighbors,C)
        IDX(i)=C;   
        k = 1;
        while true
            j = Neighbors(k);
            if ~visited(j)
                visited(j)=true;
                Neighbors2=RegionQuery(j);
                if numel(Neighbors2)>=MinPts
                    Neighbors=[Neighbors Neighbors2];   %#ok
                end
            end
            if IDX(j)==0
                IDX(j)=C;
            end
            k = k + 1;
            if k > numel(Neighbors)
                break;
            end
        end
    end
    
    function Neighbors=RegionQuery(i)
        %邻域查找，可以构建kd树代替
        Neighbors = rangesearch(Mdl,X(i,:),epsilon);
        Neighbors = Neighbors{1};
%         Neighbors=find(D(i,:)<=epsilon);
    end
% idx = unique(IDX);
% for i = 0:size(idx,1)
%    figure(1); plot(X(IDX==i,1),X(IDX==i,2),'.','Color',[rand rand rand]);hold on;axis equal;
% end
% 
%  T = clusterdata(X,'maxclust',118);
%  idx = unique(T);
% for i = 0:size(idx,1)
%     figure(2);plot(X(IDX==i,1),X(IDX==i,2),'.','Color',[rand rand rand]);hold on;axis equal;
% end 
a=0;
end


