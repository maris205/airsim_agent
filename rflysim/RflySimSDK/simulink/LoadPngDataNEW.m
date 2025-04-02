function LoadPngData(pngMapName)

if ~exist('pngMapName', 'var')
    pngMapName = 'MountainTerrain_Ice';
end

filelocPath = 'map';
fileLocPng = fullfile(filelocPath, [pngMapName, '.png']);
if ~exist(fileLocPng, 'file')
    filelocPath = fullfile('..', '..', '..', 'CopterSim', 'external', 'map');
    fileLocPng = fullfile(filelocPath, [pngMapName, '.png']);
    if ~exist(fileLocPng, 'file')
        matPath = fullfile(userpath, 'Add-Ons', 'Toolboxes', 'PX4PSP', 'code', '+codertarget', '+pixhawk', '+CMAKE_Utils', 'FirmwareVersion.mat');
        if exist(matPath, 'file')
            AA = load(matPath);
            filelocPath = fullfile(AA.RflyPSP_Px4_Base_Dir, 'CopterSim', 'external', 'map');
            fileLocPng = fullfile(filelocPath, [pngMapName, '.png']);
            if ~exist(fileLocPng, 'file')
                error(['Cannot find file: ', pngMapName, '.png file']);
            end
        else
            error(['Cannot find file: ', pngMapName, '.png file']);
        end
    end
end

fileLocTxt = fullfile(filelocPath, [pngMapName, '.txt']);
if ~exist(fileLocTxt, 'file')
    error(['Cannot find file: ', pngMapName, '.txt']);
end

fileID = fopen(fileLocTxt);
m_readData_cell = textscan(fileID, '%f', 9, 'Delimiter', ',');
fclose(fileID);
m_readData = m_readData_cell{1};

[m, n] = size(m_readData);
if m ~= 9 || n ~= 1
    error(['Cannot parse data in ', fileLocTxt]);
end

rowmap = imread(fileLocPng);
rowmap = double(rowmap) - 32768;
[rows, columns] = size(rowmap);

PosScaleX = (m_readData(1) - m_readData(4)) / (columns - 1);
PosScaleY = (m_readData(2) - m_readData(5)) / (rows - 1);
PosOffsetX = m_readData(4);
PosOffsetY = m_readData(5);

intCol = int32((m_readData(7) - PosOffsetX) / PosScaleX + 1);
intRow = int32((m_readData(8) - PosOffsetY) / PosScaleY + 1);

heightInit = double(rowmap(1, 1));
heightFirst = double(rowmap(rows, columns));
heightThird = double(rowmap(intRow, intCol));

if abs(heightThird - heightFirst) <= abs(heightThird - heightInit)
    if abs((heightInit - heightThird)) > 10
        PosScaleZ = (m_readData(6) - m_readData(9)) / (heightInit - heightThird);
    else
        PosScaleZ = 1;
    end
else
    if abs(heightThird - heightFirst) > 10
        PosScaleZ = (m_readData(3) - m_readData(9)) / (heightFirst - heightThird);
    else
        PosScaleZ = 1;
    end
end

intPosInitZ = heightInit;
PosOffsetZ = m_readData(6);

xMax = abs(m_readData(1) / 100);
yMax = abs(m_readData(2) / 100);

binmap = -(PosOffsetZ + ((rowmap) - intPosInitZ) * PosScaleZ) / 100.0;

% 初始化 x, y, z 数据存储的数组，并预分配空间
x_data = zeros(rows * columns, 1);
y_data = zeros(rows * columns, 1);
z_data = zeros(rows * columns, 1);
idx = 1;

% 遍历地形网格中的每一行
for row = 0:(rows - 1)
    % 遍历地形网格中的每一列
    for col = 0:(columns - 1)

        % 计算实际的x坐标
        xin = (PosOffsetX + col * PosScaleX) / 100;
        % 计算实际的y坐标
        yin = (PosOffsetY + row * PosScaleY) / 100;

        intCol = (xin * 100 - PosOffsetX) / PosScaleX + 1;
        intRow = (yin * 100 - PosOffsetY) / PosScaleY + 1;

        intColInt = floor(intCol);
        intRowInt = floor(intRow);
        a = intCol - intColInt;
        b = intRow - intRowInt;

        intRowInt1 = intRowInt + 1;
        intColInt1 = intColInt + 1;

        if intColInt < 1
            intColInt = 1;
            intColInt1 = 1;
            a = 0;
        end

        if intColInt >= columns
            intColInt = columns;
            intColInt1 = intColInt;
            a = 0;
        end

        if intRowInt < 1
            intRowInt = 1;
            intRowInt1 = 1;
            b = 0;
        end

        if intRowInt >= rows
            intRowInt = rows;
            intRowInt1 = intRowInt;
            b = 0;
        end

        zz = binmap(intRowInt, intColInt) * (1 - b) * (1 - a) + ...
             binmap(intRowInt1, intColInt) * b * (1 - a) + ...
             binmap(intRowInt, intColInt1) * (1 - b) * a + ...
             binmap(intRowInt1, intColInt1) * b * a;

        % 将 (x, y, z) 数据分别添加到各自的数组中
        x_data(idx) = xin;
        y_data(idx) = yin;
        z_data(idx) = -zz;
        idx = idx + 1;
    end
end

% 将实际的大小调整为有效数据点的数量
x_data = x_data(1:idx-1);
y_data = y_data(1:idx-1);
z_data = z_data(1:idx-1);

% 保存 x, y, z 数据到 MAT 文件
save('TerrainData.mat', 'x_data', 'y_data', 'z_data');



