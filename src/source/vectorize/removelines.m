function   [cluster_new,idx_new] = removelines(cluster,breakLines)
%
%È¥³ýclusterÖÐµÄbreakLines
existIdx = [];
for i=1:size(breakLines,2)
    existIdx = [existIdx breakLines{i}];
end

idx = 0;
idx_new = [];
for i=1:size(cluster,2)
    if isempty(find(existIdx==i,1))
        idx = idx+1;
        cluster_new(idx) = cluster(i);
        idx_new = [idx_new i];
    end
end
end