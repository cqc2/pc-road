function points =  getpointfromstructArray(pointArray)
nArray = size(pointArray,2);
points = [];
for iArray = 1:nArray
    if isempty(pointArray(iArray))
        continue;
    end
    x = pointArray(iArray).x;
    y = pointArray(iArray).y;
    h = pointArray(iArray).h;
    ins = pointArray(iArray).ins;
    if ~isempty(x)
        points = [points;x y h ins];
    else
        continue;
    end
end
end
