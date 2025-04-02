try
    msg = ros2message('mavros_msgs/Mavlink');  % 尝试创建消息类型
    disp('mavros2 registered');
catch ME
    disp('mavros2 install...');

    % 获取MATLAB版本
    matlab_version = ver;
    % 找到MATLAB软件本身的版本信息
    isMatlab = strcmp({matlab_version.Name}, 'MATLAB');
    if any(isMatlab)
       matlab_info = matlab_version(isMatlab);
       release_name = matlab_info.Release;
        
       release_number = release_name(2:7);
       fprintf('MATLAB version: %s\n', release_number);
    else
        warning('MATLAB version information not found.');
    end

    % 获取路径
    PSP_PATH = getenv('PSP_PATH');  % 获取环境变量
    RflyPath = fullfile(PSP_PATH, 'RflySimAPIs', 'RflySimSDK', 'simulink', 'mavros2');
    if isempty(RflyPath)
        warning('Environment variable PSP_PATH is not set.');
    end

    % 根据MATLAB版本选择正确的压缩包并重命名
    switch release_number
        case {'R2024a', 'R2024b'}
            zipFileName = ['matlab_msg_gen_', release_number, '.zip'];
            sourceZipPath = fullfile(RflyPath, zipFileName);
            targetZipPath = fullfile(RflyPath, 'matlab_msg_gen.zip');
            fprintf('MATLAB version: %s\n', release_number);
            if exist(sourceZipPath, 'file') == 2
                copyfile(sourceZipPath, targetZipPath);
                fprintf('Copied and renamed %s to %s.\n', zipFileName, targetZipPath);
                
                % 使用ros2RegisterMessages注册ROS2消息
                ros2RegisterMessages(RflyPath);
                fprintf('Registered ROS2 messages from %s.\n', targetZipPath);
            else
                warning('Source ZIP file does not exist: %s', sourceZipPath);
            end
        otherwise
            warning('MATLAB version does not support MAVROS2.can use 2024a or 2024b instead.');
    end
end
