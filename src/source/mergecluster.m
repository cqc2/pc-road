function  cluster = mergecluster(cluster1,cluster2)
    nc = size(cluster1,2);
    nc2 = size(cluster2,2);
    cluster = cluster1;
    for i=1:nc2
        cluster(nc+i) = cluster2(i);
    end
end