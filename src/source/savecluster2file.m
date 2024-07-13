function savecluster2file(index,data,savefilename)
%
if isstruct(data)
    cluster = data;
    nCluster = size(cluster,2);
    indexRange = zeros(nCluster,2);
    clusterPoints = [];
    endIdx = 1+nCluster;
    for i=1:nCluster
        pointdata = cluster(i).data;
        np = size(pointdata,1);
        startIdx = endIdx+1;
        endIdx = startIdx+np-1;
        indexRange(i,:) = [startIdx endIdx];
        clusterPoints = [clusterPoints;pointdata];
    end
    fid1=fopen(strcat(savefilename,'_seg.xyz'),'wt');
    col = size(pointdata,2);
else
    [C,~,ic] = unique(index);
    nCluster = size(C,1); %聚类个数
    indexRange = zeros(nCluster,2);
    clusterPoints = zeros([size(data,1) size(data,2)+1]);
    endIdx = 1+nCluster;
    for i=1:nCluster
        idx =  find(ic==C(i));
        np = size(idx,1);
        startIdx = endIdx+1;
        endIdx = startIdx+np-1;
        indexRange(i,:) = [startIdx endIdx];
        clusterPoints(startIdx-nCluster-1:endIdx-nCluster-1,:) = data(idx,:);
    end
    fid1=fopen(strcat(savefilename,'.seg'),'wt');
    col = size(data,2);
end
fprintf(fid1,'%d\n',nCluster);
fprintf(fid1,'%d %d\n',indexRange');
if col==2
    fprintf(fid1,'%.4f %.4f\n',clusterPoints');
elseif col==3
    %xyz
    fprintf(fid1,'%.4f %.4f %.4f\n',clusterPoints');
elseif col==4
    %xyzi
    fprintf(fid1,'%.4f %.4f %.4f %d\n',clusterPoints');
elseif col==7
    %xyzirgb
    fprintf(fid1,'%.4f %.4f %.4f %d %d %d %d\n',clusterPoints');
end
fclose(fid1);
end