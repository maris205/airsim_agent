function LoadPngData(pngMapName)

if ~exist('pngMapName','var')
    pngMapName='MountainTerrain';
end

filelocPath='map';
fileLocPng = [filelocPath,'\',pngMapName,'.png'];
if ~exist(fileLocPng,'file')
    filelocPath='..\..\..\CopterSim\external\map';
    fileLocPng = [filelocPath,'\',pngMapName,'.png'];
    if ~exist(fileLocPng,'file')
        matPath=[userpath,'\Add-Ons\Toolboxes\PX4PSP\code\+codertarget\+pixhawk\+CMAKE_Utils\FirmwareVersion.mat'];
        if exist(matPath,'file')
            AA=load(matPath);
            filelocPath=[AA.RflyPSP_Px4_Base_Dir,'\CopterSim\external\map'];
            fileLocPng = [filelocPath,'\',pngMapName,'.png']; 
            if ~exist(fileLocPng,'file')
                error(['Cannot find file: ',pngMapName,'.png file']);
            end
        else
            error(['Cannot find file: ',pngMapName,'.png file']);
        end
               
    end
    
end

fileLocTxt = [filelocPath,'\',pngMapName,'.txt']; 
if ~exist(fileLocTxt,'file')
    error(['Cannot find file: ',pngMapName,'.txt']);
end

fileID = fopen(fileLocTxt);
m_readData_cell = textscan(fileID,'%f',9,'Delimiter',',');
m_readData = m_readData_cell{1};
[m,n]=size(m_readData);
if m~=9 || n~=1
    error(['Cannot parse data in ',fileLocTxt]);
end

rowmap  = imread(fileLocPng);
rowmap = double(rowmap)-32768;
[rows, columns] = size(rowmap);

PosScaleX = (m_readData(1)-m_readData(4))/(columns-1);
PosScaleY = (m_readData(2)-m_readData(5))/(rows-1);

PosOffsetX = m_readData(4);
PosOffsetY = m_readData(5);

intCol = int32((m_readData(7)-PosOffsetX)/PosScaleX + 1);
intRow = int32((m_readData(8)-PosOffsetY)/PosScaleY + 1);
% constainXY(intRow,intCol);

heightInit = double(rowmap(1,1)); 
heightFirst = double(rowmap(rows,columns)); 
heightThird = double(rowmap(intRow,intCol));


if abs(heightThird-heightFirst)<=abs(heightThird-heightInit)
    if abs((heightInit-heightThird))>10
        PosScaleZ = (m_readData(6)-m_readData(9))/((heightInit-heightThird));
    else
        PosScaleZ = 1;
    end    
else
    if abs(heightThird-heightFirst)>10
        PosScaleZ = (m_readData(3)-m_readData(9))/((heightFirst-heightThird));
    else
        PosScaleZ = 1;
    end        
end

intPosInitZ = heightInit;
PosOffsetZ = m_readData(6);

xMax=abs(m_readData(1)/100);
yMax=abs(m_readData(2)/100);

binmap= -(PosOffsetZ + ((rowmap)-intPosInitZ)*PosScaleZ)/100.0;

save('MapHeightData','binmap','PosOffsetX','PosScaleX','PosOffsetY','PosScaleY');