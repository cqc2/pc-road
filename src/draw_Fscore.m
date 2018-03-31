function draw_Fscore 
%
addpath(genpath(pwd));
name = 'F_score_yingwuzhou2';
% f1 = importdata(strcat(name,'.mat'));
f2 = importdata(strcat(name,'.mat'));
draw_Fscore2(f2);
figure(2);draw_Fscore2(f1);
% 校正后的图像f值算一遍，然后说
end

function draw_Fscore2(F_score)
    s = F_score(:,1);
    t = F_score(:,2);
    score = F_score(:,3);
    completeness = F_score(:,4);
    correctness = F_score(:,5);
    [xq,yq] = meshgrid(min(s):max(s),min(t):0.01:max(t));
    vq = griddata(s,t,score,xq,yq);
    surf(xq,yq,vq,'LineWidth',0.1);
    xlabel('s');
    ylabel('t');
    zlabel('F-score');
    
    idx = [];
    for i= min(s):max(s)
       [r,~] = find(s==i);
       maxF = max(score(r,1));
      [r2,~] =  find(score(r,1)>=maxF);
      idx = [idx;r(r2(1,1),1)];
    end
    hold on;
    plot3(s(idx),t(idx),score(idx),'r-');%绘制脊线
    plot3(s(idx),t(idx),score(idx),'r.');%绘制脊线
    mid_idx = idx(ceil(size(idx,1)/2));
    
%     quiver3(s(mid_idx)+0.01,t(mid_idx)+0.01,score(mid_idx)+0.01,s(mid_idx),t(mid_idx),score(mid_idx),0.01);
    text(s(mid_idx)+0.05,t(mid_idx)+0.05,score(mid_idx)+0.05,'极值脊线');
    maxF = max(score);
    [r,~] = find(score>=maxF);
    plot3(s(r),t(r),score(r),'ko');
    plot3(s(r),t(r),score(r),'g*');
    text(s(r),t(r),score(r)+0.05,['最大值点 (',num2str(s(r)),',',num2str(t(r)),',',num2str(score(r)),')'])
    shading interp;
    
end