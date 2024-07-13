function segment
%test_slice - Sort in ascending or descending order.
    datetime('now','TimeZone','local','Format','HH:mm:ss Z')
    pointCloudFilePath = '#2Final_Point_Cloud_Data.xyz';
%     pointCloudData = readpointcloudfile(pointCloudFilePath);
    segmentArray = segmentdata(pointCloudFilePath,10,1000000);
    datetime('now','TimeZone','local','Format','HH:mm:ss Z')
end

function outputslices2file(sliceArray)
% output point cloud slices data into file
    writetime=datestr(now,'yymmddHHMMSS');
    nSlices = size(sliceArray,2);
    for i=1:nSlices
        temp=[sliceArray(i).x sliceArray(i).y sliceArray(i).h sliceArray(i).ins];
        temp1=strcat('slice',writetime,'\');
        if ~exist(temp1,'dir')
            mkdir(temp1);
        end        
        fid1=fopen(strcat('slice',writetime,'\',num2str(i),'.xyz'),'wt');
        fprintf(fid1,'%f %f %f %d\n',temp');
        fclose(fid1);
    end
end

function segmentArray = segmentdata(filePath,nSegment,interval)
%     %read point cloud file
%     if (isunix) % Linux系统提供了wc命令可以直接使用
%         % 使用syetem函数可以执行操作系统的函数
%         % 比如window中dir，linux中ls等
%         [~, numstr] = system( ['wc -l ', filePath] );
%         row=str2double(numstr);
%     elseif (ispc) % Windows系统可以使用perl命令
%         if exist('countlines.pl','file')~=2
%             % perl文件内容很简单就两行
%             % while (<>) {};
%             % print $.,"\n";
%             fid=fopen('countlines.pl','w');
%             fprintf(fid,'%s\n%s','while (<>) {};','print $.,"\n";');
%             fclose(fid);
%         end
%         % 执行perl脚本
%         row=str2double( perl('countlines.pl',filePath) );
%     end
%     nPoint = row;   
    fid=fopen(filePath,'r');
    writetime=datestr(now,'yymmddHHMM');
    for i=1:nSegment,
%         if i*interval>nPoint,
%             break;
%         end
        data = fscanf(fid,'%f',[4,interval])';
        x = data(:,1);
        y = data(:,2);
        h = data(:,3);
        ins = data(:,4);
        temp=[x y h ins];
        temp1=strcat('segment',writetime,'\');
        if ~exist(temp1,'dir')
            mkdir(temp1);
        end
        fid1=fopen(strcat('segment',writetime,'\',num2str(i),'.xyz'),'wt');
        fprintf(fid1,'%f %f %f %d\n',temp');
        fclose(fid1);
    end
    fclose(fid);
end

function thinnedPointData =  thinpointdata(pointdata,skip)
% thin point data
    [nPoint col] = size(pointdata);
    nThinnedPoints = ceil(nPoint/skip);
    thinnedPointData = zeros(nThinnedPoints,col);
    iThinnedPoint = 0;
    while iThinnedPoint<nThinnedPoints,
        iThinnedPoint = iThinnedPoint+1;
        iPoint = (iThinnedPoint-1)*skip+1;
        thinnedPointData(iThinnedPoint,1:col) = pointdata(iPoint,1:end);
    end
end


















