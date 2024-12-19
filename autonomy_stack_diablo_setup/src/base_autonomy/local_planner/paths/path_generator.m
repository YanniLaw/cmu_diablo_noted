clc;
clear all;
close all;
% 解析参考 https://www.jianshu.com/p/2716b99f2cab
%% generate path
%{.
dis = 1.0;
angle = 27;
deltaAngle = angle / 3;
scale = 0.65;

pathStartAll = zeros(4, 0); % 创建4行0列的空数组，初始值为空([])
pathAll = zeros(5, 0);
pathList = zeros(5, 0);
pathID = 0;
groupID = 0;

figure;
hold on;
box on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');

fprintf('\nGenerating paths\n');
% 离线采样通过采用固定采样距离与调整航向角进行离散路点采样
for shift1 = -angle : deltaAngle : angle % 最外层循环每次循环存储一个角度上的所有StartPath轨迹数据
    wayptsStart = [0, 0, 0;
                   dis, shift1, 0]; % 2行3列
    
    pathStartR = 0 : 0.01 : dis; % 1行101列 [0.0, 0.01, 0.02 ...... 0.99,1.00]

    % wayptsStart(:, 1) 为第一列的值 [0;1]   wayptsStart(:, 2) 为第二列的值 [0;shift1] 
    % 这里的 pathStartShift与pathStartR大小相同, 为 1行101列
    % 每一列存的值为对应pathStartR按照固定间隔插值(三次样条插值)的角度值
    %{
        ----- 函数解析 -----
        s = spline(x,y,xq) 返回与 xq 中的查询点对应的插值向量s. s 的值由 x 和 y 的三次样条插值确定
        输入参数: x: x坐标，指定为向量。向量 x 指定提供数据 y 的点。x 的元素必须是唯一的。三次样条插值需要至少 4 个点，如果分别提供了 2 个或 3 个点，则退回到线性或二次插值
                y: x 坐标处的函数值,可以是向量/矩阵/数组。x 和 y 通常具有相同的长度，但 y 也可以比 x 正好多出两个元素，用以指定端点斜率
        如果 y 是矩阵或数组，则在获取最后一个维度 y(:,...,:,j) 中的值时应使其匹配 x。在此情况下，y 的最后一个维度的长度必须与 x 相同或正好多出两个元素
        
        三次样条的端点斜率遵循以下规则:
        1) 如果 x 和 y 是大小相等的向量，则使用非节点终止条件。
        2) 如果 x 或 y 为标量，则会将该标量扩展为与另一方具有相同的长度并使用非节点终止条件。
        3) 如果 y 是一个包含的值比 x 具有的条目多两个的向量，则 spline 使用 y 中的第一个和最后一个值作为三次样条的端点斜率。例如，如果 y 是一个向量，则：
          - y(2:end-1) 给出 x 中每个点处的函数值
          - y(1) 给出区间开始处 min(x) 的斜率
          - y(end) 给出区间结束处 max(x) 的斜率
        4）同样，如果 y 是一个矩阵或 size(y,N) 等于 length(x)+2 的 N 维数组，则：
          - y(:,...,:,j+1) 给出 x 中每个点的函数值，其中 j = 1:length(x)
          - y(:,:,...:,1) 给出区间开始处 min(x) 的斜率
          - y(:,:,...:,end) 给出区间结束处 max(x) 的斜率
                xq: 查询点。 指定为标量、向量、矩阵或数组。xq 中指定的点是 spline 计算出的插值函数值 yq 的 x 坐标。
        输出参数: s  查询点处的插值，以标量、向量、矩阵或数组形式返回。
            s 的大小与 y 和 xq 的大小相关:
            1) 如果 y 为向量，则 s 的大小与 xq 相同。
            2) 如果 y 是大小为 Ny = size(y) 的数组，则下列条件适用：
              2.1) 如果 xq 为标量或向量，则 size(s) 返回 [Ny(1:end-1) length(xq)]。
              2.2) 如果 xq 是数组，则 size(s) 返回 [Ny(1:end-1) size(xq)]。
        参考 https://ww2.mathworks.cn/help/matlab/ref/spline.html
    %}
    % 第一段路其实只有两个点进行插值，会退化至线性
    pathStartShift = spline(wayptsStart(:, 1), wayptsStart(:, 2), pathStartR);
    % 以下三个变量均为1行101列，表示选定角度插值后的start path的坐标值 极坐标
    pathStartX = pathStartR .* cos(pathStartShift * pi / 180);
    pathStartY = pathStartR .* sin(pathStartShift * pi / 180);
    pathStartZ = zeros(size(pathStartX));
    % ones(size(pathStartX)): 创建一个与 pathStartX 维度相同的全为 1 的数组
    pathStart = [pathStartX; pathStartY; pathStartZ; ones(size(pathStartX)) * groupID]; % 当前轮循环生成的初始path, 4行101列
    pathStartAll = [pathStartAll, pathStart]; % 将当前的pathStart 追加到pathStartAll 中（动态数组扩展,水平拼接）
    % 由于初始path一共有七个组，所以最终有4行707列
    
    % 随着路径的变长，角度分辨率减小，这样能将路径集中在前方，比较密集，朝前走
    for shift2 = -angle * scale + shift1 : deltaAngle * scale : angle * scale + shift1
        for shift3 = -angle * scale^2 + shift2 : deltaAngle * scale^2 : angle * scale^2 + shift2
                % pathStartR' 表示矩阵的转置，即从1行101列 变为 101行1列
                % waypts 104行3列
                % 由于spline()函数拟合路径的时候，利用了第一段路径的所有路径点，所以最后生成整条路径时，前面部分与第一段路径完全重合
                waypts = [pathStartR', pathStartShift', pathStartZ'; % 保持初始路径
                          2 * dis, shift2, 0;
                          3 * dis - 0.001, shift3, 0;   % 保证路径端点曲率不突变
                          3 * dis, shift3, 0];

                % pathR 1行301列 [0, 0.0, ..., 2.90, 3.0]
                pathR = 0 : 0.01 : waypts(end, 1); % waypts(end, 1) 最后一行第一列
                % waypts(:, 1) 第一列 waypts(:, 2) 第二列
                pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                pathX = pathR .* cos(pathShift * pi / 180);
                pathY = pathR .* sin(pathShift * pi / 180);
                pathZ = zeros(size(pathX));

                path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                pathAll = [pathAll, path];
                pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];
                
                pathID = pathID + 1;

                plot3(pathX, pathY, pathZ);
        end
    end
    
    groupID = groupID + 1
end

pathID

fileID = fopen('startPaths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathStartAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d\n', pathStartAll);
fclose(fileID);

fileID = fopen('paths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathAll);
fclose(fileID);

fileID = fopen('pathList.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathList, 2));
fprintf(fileID, 'property float end_x\n');
fprintf(fileID, 'property float end_y\n');
fprintf(fileID, 'property float end_z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathList);
fclose(fileID);

pause(1.0);
%}

%% find correspondence
%{.
% 整个梯形区域构建是从外往里，从最外层每隔0.02米构建一列，一列一列的构建， 梯形区域的坐标原点为右上角，右上角往左voxelNumX递增，右上角往下voxelNumY递增，注意是2倍的Y。
% voxelNumX与voxelNumY是点云ID的索引，不是位置距离。
voxelSize = 0.02; % 体素网格的基本分辨率
searchRadius = 0.45; % 搜索半径，用于调整 Y 坐标的缩放因子，使体素在 Y 方向的分布不是均匀的
offsetX = 3.2; % 体素网格的最大x范围
offsetY = 4.5; % 体素网格的最大y范围 (最终的取值范围 x 方向上 是[0, 3.2] , y 是 [-4.5, +4.5])
voxelNumX = 161; % x方向的体素数
voxelNumY = 451; % y方向的体素数

fprintf('\nPreparing voxels\n');

indPoint = 1;
voxelPointNum = voxelNumX * voxelNumY; % 72611 总的体素点数
voxelPoints = zeros(voxelPointNum, 2);
for indX = 0 : voxelNumX - 1
    x = offsetX - voxelSize * indX; % 从 offsetX 开始，以 voxelSize 为步长向左递减
    % ScaleY是根据三角形相似计算的一个比例，通过计算ScaleY来控制梯形的形状，即梯形斜边上的点，voxelPoints为2列，61 * 451 = 72611行的数据结构尺寸为1。
    scaleY = x / offsetX + searchRadius / offsetY * (offsetX - x) / offsetX;
    for indY = 0 : voxelNumY - 1
        y = scaleY * (offsetY - voxelSize * indY);

        voxelPoints(indPoint, 1) = x;
        voxelPoints(indPoint, 2) = y;
        
        indPoint  = indPoint + 1;
    end
end

plot3(voxelPoints(:, 1), voxelPoints(:, 2), zeros(voxelPointNum, 1), 'k.');
pause(1.0);

fprintf('\nCollision checking\n');

%{
------函数解析------
[Idx,D] = rangesearch(X,Y,r) 用于从参考点集合X中找到每个查询点Y的所有邻居点，这些邻居点的距离在指定范围r内 
参数 X - 输入数据,m x n 矩阵，其中，每行表示一个n维的点，X Y 的列数必须相同(也就是点的维度)
    Y - 同上，行数不一样，列数跟X一样
    r - 查询距离
输出 Idx - 一个 p × 1 的元胞数组。 idx{i} 包含与查询点 Y(i, :) 的距离小于 radius 的参考点在 X 中的索引 类似于与c++ 中的std::vector<std::vector>
    D - 同上对应的查询距离
%}
[ind, dis] = rangesearch(pathAll(1 : 2, :)', voxelPoints, searchRadius);

fprintf('\nSaving correspondences\n');

fileID = fopen('correspondences.txt', 'w');

for i = 1 : voxelPointNum
    fprintf(fileID, '%d ', i - 1);
    
    indVoxel = sort(ind{i}); % 获取当前体素点的邻居点索引并按升序排序
    indVoxelNum = size(indVoxel, 2); % 获取当前体素点的邻居点数量
    % 遍历该体素点的所有邻居点，并获取其路径ID
    pathIndRec = -1;
    for j = 1 : indVoxelNum
        pathInd = pathAll(4, indVoxel(j));
        if pathInd == pathIndRec
            continue;
        end

        fprintf(fileID, '%d ', pathInd);
        pathIndRec = pathInd;
    end
    fprintf(fileID, '-1\n'); % 行结束标志
    
    if mod(i, 1000) == 0
        i
    end
end

fclose(fileID);

fprintf('\nProcessing complete\n');
%}
