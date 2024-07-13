function index = getsampleindex(nPoint,nSample)
%
%在1~nPoint中随机抽取nSample个不重复对像，返回对象索引号
    index = -ones(nSample,1);
    iSample = 0;
    while iSample<nSample,
        rand0 =  floor(1+(nPoint-1)*rand(1,1));
        isSave = true;
        for i = 1:iSample,          
            if rand0==index(i),
                isSave = false;
                break;
            end
        end
        if isSave,
            iSample = iSample+1;
            index(iSample,1) = rand0;
        end
    end
end