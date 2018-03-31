function posData = readposfile(filePath,nField,startPos,endPos)
% read pose file 
    
%读取pos数据时注意数据排列格式
    fid=fopen(filePath,'r');
    if nField==16,
        data = fscanf(fid,'%d %d %d %d %d %f %d %f %f %f %f %f %f %f %f %f',[16,inf])';
        posData.roll = data(1:end,15);
        posData.pitch = data(1:end,16);
    elseif nField==14,
        data = fscanf(fid,'%d %d %d %d %d %f %d %f %f %f %f %f %f %f',[14,inf])';
    elseif nField==3
        data = fscanf(fid,'%f %f %f',[3,inf])';
        posData.elevation = data(1:end,3);
        posData.x = data(1:end,1);
        posData.y = data(1:end,2);
        return;
    end
    nRows = size(data,1);
    if ~exist('startPos','var')||isempty(startPos)||startPos<1,startPos=1;end
    if ~exist('endPos','var')||isempty(endPos)||endPos>nRows,endPos=nRows;end
    posData.year = data(startPos:endPos,1);
    posData.month = data(startPos:endPos,2);
    posData.day = data(startPos:endPos,3);
    posData.hour = data(startPos:endPos,4);
    posData.minute = data(startPos:endPos,5);
    posData.second = data(startPos:endPos,6);
    posData.gpsweek = data(startPos:endPos,7);
    posData.gpstime = data(startPos:endPos,8);
    posData.latitude = data(startPos:endPos,9);
    posData.longitude = data(startPos:endPos,10);
    posData.elevation = data(startPos:endPos,11);
    posData.x = data(startPos:endPos,12);
    posData.y = data(startPos:endPos,13);
    posData.azimuth = data(startPos:endPos,14);
    fclose(fid);
end