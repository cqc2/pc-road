function draw_Fscore 
name = 'F_score_fazhan';
f1 = importdata(strcat(name,'.mat'));
f2 = importdata(strcat(name,'2.mat'));
draw_Fscore2(f1);
draw_Fscore2(f2)
end

function draw_Fscore2(F_score)
    x = F_score(:,1);
    y = F_score(:,2);
    score = F_score(:,3);
    completeness = F_score(:,4);
    correctness = F_score(:,5);
    [xq,yq] = meshgrid(min(x):max(x),min(y):0.01:max(y));
    vq = griddata(x,y,score,xq,yq);
    surf(xq,yq,vq);
    xlabel('s');
    ylabel('t');
    zlabel('F-score');
end