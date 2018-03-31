function index = clustereuclid(data,dist,maxp)
%
% 欧式聚类
% index = clustereuclid(data,dist,maxp)
%
% INPUT:
% data - 聚类数据，可以是多维
% dist - 聚类距离
% maxp - 最大点个数，原聚类中如果点个数大于maxp，将进行分裂形成两个聚类
%
% OUTPUT:
% index - 每个元素的类别号
% 注意：测试发现，数据量增加一倍，耗时增加大于1倍，所以处理的数据较多时，应分块处理
%       点个数一般不要大于1000
% REFERENCES:
% PCL 1.8.0\include\pcl-1.8\pcl\segmentation\extract_clusters.h

if ~exist('maxp','var')||isempty(maxp), maxp = 9999;end

Mdl = KDTreeSearcher(data);
n = size(data,1);
processed = zeros([n 1]);%是否已经被分类
nCluster  = 0;
index = zeros([n 1]);
for i = 1:n
    if processed(i)
        continue;
    end
    sq_idx = 1;
    seed_queue = i;%搜索队列
    processed(i) = true;
    while size(seed_queue,2)>=sq_idx
        idx = rangesearch(Mdl,data(seed_queue(sq_idx),:),dist);
        idx = idx{1};
        if isempty(idx)
            sq_idx = sq_idx+1;
            continue;
        end
        for j = 1:size(idx,2)
            if processed(idx(j))
                continue;
            end
            seed_queue = [seed_queue idx(j)];
            processed(idx(j)) = true;
        end
        sq_idx = sq_idx+1;      
        if exist('maxp','var')&&size(seed_queue,2)>maxp
            break;%如果多于maxp个点，则进行分裂
        end
    end
    nCluster = nCluster+1;
    index(seed_queue) = nCluster;
end
end