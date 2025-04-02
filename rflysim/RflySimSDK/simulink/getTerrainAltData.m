function zz = getTerrainAltData(xin,yin)

%The map for simulation
mapname='MapHeightData';

persistent binmap;
persistent PosOffsetX;
persistent PosScaleX;
persistent PosOffsetY;
persistent PosScaleY;
if isempty(binmap)

%     if ~exist([mapname,'.mat'],'file')
%         LoadPngData(mapname);
%     end
    
    SS=load([mapname,'.mat']);
    binmap=SS.binmap;
    PosOffsetX=SS.PosOffsetX;
    PosScaleX=SS.PosScaleX;
    PosOffsetY=SS.PosOffsetY;
    PosScaleY=SS.PosScaleY; 

end


intCol = (xin*100-PosOffsetX)/PosScaleX+1;
intRow = (yin*100-PosOffsetY)/PosScaleY+1;

intColInt=floor(intCol);
intRowInt = floor(intRow);
a=intCol-intColInt;
b=intRow-intRowInt;

intRowInt1=intRowInt+1; 
intColInt1=intColInt+1; 

[m,n]=size(binmap);
if intColInt<1
    intColInt=1;
    intColInt1=1;
    a=0;
end


if intColInt>=n
    intColInt=n;
    intColInt1=intColInt;
    a=0;
end


if intRowInt<1
    intRowInt=1;
    intRowInt1=1;
    b=0;
end


if intRowInt>=m
    intRowInt=m;
    intRowInt1=intRowInt;
    b=0; 
end

zz=binmap(intRowInt,intColInt)*(1-b)*(1-a)+binmap(intRowInt1,intColInt)*b*(1-a)+binmap(intRowInt,intColInt1)*(1-b)*a+binmap(intRowInt1,intColInt1)*b*a;
