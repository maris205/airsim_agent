function helpText = sharedhelp(topic)
    % 获取当前 .m 文件的完整路径
    Path = getenv('PSP_PATH'); 
    % 构建相对路径
    relativePath = 'RflySimAPIs/RflySimSDK/html';
    % 组合上一层目录路径和相对路径以获取绝对路径
    Dir = fullfile(Path, relativePath);
    % 根据 topic 参数返回帮助文本
    switch topic
        case 'UDP_20100_PX4SilRecv'
            % 构建相对路径
            file = 'md_ctrl_2md_2UDP__20100__PX4SilRecv.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'UDP_30100_TrueSimRecv'
            % 构建相对路径
            file = 'md_ctrl_2md_2UDP__30100__TrueSimRecv.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'UDP_40100_RflyPx4Recv'
            % 构建相对路径
            file = 'md_ctrl_2md_2UDP__40100__RflyPx4Recv.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PX4SILIntFloatSend'
            % 构建相对路径
            file = 'md_ctrl_2md_2PX4SILIntFloatSend.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PX4FaultInParamsSend'
            % 构建相对路径
            file = 'md_ctrl_2md_2PX4FaultInParamsSend.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyUdpFast'
            % 构建相对路径
            file = 'md_swarm_2md_2RflyUdpFast.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyUdpRaw'
            % 构建相对路径
            file = 'md_swarm_2md_2RflyUdpRaw.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflySerialRaw'
            % 构建相对路径
            file = 'md_swarm_2md_2RflySerialRaw.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyUdpMavlink'
            % 构建相对路径
            file = 'md_swarm_2md_2RflyUdpMavlink.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyVrpnRecv'
            % 构建相对路径
            file = 'md_swarm_2md_2RflyVrpnRecv.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'HighLevelMode'
            % 构建相对路径
            file = 'md_swarm_2md_2HighLevelMode.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);            
        case 'AutoFirmwareAPI'
            % 构建相对路径
            file = 'md_swarm_2md_2AutoFirmwareAPI.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);            
        case 'RealCtrl28D'
            % 构建相对路径
            file = 'md_swarm_2md_2RealCtrl28D.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Offboard_full'
            % 构建相对路径
            file = 'md_swarm_2md_2Offboard__full.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vel_ned_full'
            % 构建相对路径
            file = 'md_swarm_2md_2vel__ned__full.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vel_body_full'
            % 构建相对路径
            file = 'md_swarm_2md_2vel__body__full.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'pos_ned_full'
            % 构建相对路径
            file = 'md_swarm_2md_2pos__ned__full.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vrpn_data_decoder'
            % 构建相对路径
            file = 'md_swarm_2md_2vrpn__data__decoder.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'real_data_decoder'
            % 构建相对路径
            file = 'md_swarm_2md_2real__data__decoder.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'SimpleCtrl4D'
            % 构建相对路径
            file = 'md_swarm_2md_2SimpleCtrl4D.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Full_data_decoder'
            % 构建相对路径
            file = 'md_swarm_2md_2Full_data_decoder.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Simple_data_decoder'
            % 构建相对路径
            file = 'md_swarm_2md_2Simple__data__decoder.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Serial'
            % 构建相对路径
            file = 'md_ctrl_2md_2Serial.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Read ADC Channels'
            % 构建相对路径
            file = 'md_ctrl_2md_2Read_01ADC_01Channels.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'ParamUpdate'
            % 构建相对路径
            file = 'md_ctrl_2md_2ParamUpdate.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'ExamplePrintFcn'
            % 构建相对路径
            file = 'md_ctrl_2md_2ExamplePrintFcn.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'binary_logger'
            % 构建相对路径
            file = 'md_ctrl_2md_2binary__logger.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sensor_combined'
            % 构建相对路径
            file = 'md_ctrl_2md_2sensor__combined.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'input_rc'
            % 构建相对路径
            file = 'md_ctrl_2md_2input__rc.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Speaker_Tune'
            % 构建相对路径
            file = 'md_ctrl_2md_2Speaker__Tune.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'battery_measure'
            % 构建相对路径
            file = 'md_ctrl_2md_2battery__measure.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vehicle_attitude'
            % 构建相对路径
            file = 'md_ctrl_2vehicle__attitude.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vehicle_gps'
            % 构建相对路径
            file = 'md_ctrl_2vehicle__gps.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RGB_LED'
            % 构建相对路径
            file = 'md_ctrl_2md_2RGB__LED.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PWM_output'
            % 构建相对路径
            file = 'md_ctrl_2md_2PWM__output.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'AUX_output'
            % 构建相对路径
            file = 'md_ctrl_2md_2AUX__output.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Read Async'
            % 构建相对路径
            file = 'md_ctrl_2md_2uORB_01Read_01Async.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Read Function-Call Trigger'
            % 构建相对路径
            file = 'md_ctrl_2md_2uORB_01Read_01Function-Call_01Trigger.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Write'
            % 构建相对路径
            file = 'md_ctrl_2md_2uORB_01Write.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Write Advanced'
            % 构建相对路径
            file = 'md_ctrl_2md_2uORB_01Write_01Advanced.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Write Advanced_dai'
            % 构建相对路径
            file = 'md_ctrl_2md_2uORB_01Write_01Advanced__dai.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'HIL16CtrlsNorm'
            % 构建相对路径
            file = 'md_ctrl_2md_2HIL16CtrlsNorm.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'HIL16CtrlsPWM'
            % 构建相对路径
            file = 'md_ctrl_2md_2HIL16CtrlsPWM.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'InputRcNorm'
            % 构建相对路径
            file = 'md_ctrl_2md_2InputRcNorm.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'InputRcCali'
            % 构建相对路径
            file = 'md_ctrl_2md_2InputRcCali.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'OffboardAttCtrlAPI'
            % 构建相对路径
            file = 'md_ctrl_2md_2OffboardAttCtrlAPI.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'OffboardAdvCtrlAPI'
            % 构建相对路径
            file = 'md_ctrl_2md_2OffboardAdvCtrlAPI.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'OffboardPvaCtrlAPI'
            % 构建相对路径
            file = 'md_ctrl_2md_2OffboardPvaCtrlAPI.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'TorqueThrustCtrls'
            % 构建相对路径
            file = 'md_ctrl_2md_2TorqueThrustCtrls.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RCOverCtrlAPI'
            % 构建相对路径
            file = 'md_ctrl_2md_2RCOverCtrlAPI.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RePX4Block'
            % 构建相对路径
            file = 'md_ctrl_2md_2RePX4Block.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'OffCtrlMsgAll'
            % 构建相对路径
            file = 'md_ctrl_2md_2OffCtrlMsgAll.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Msg2SimulinkAPI'
            % 构建相对路径
            file = 'md_ctrl_2md_2Msg2SimulinkAPI.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PosVelAttAll'
            % 构建相对路径
            file = 'md_ctrl_2md_2PosVelAttAll.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
            
        case '6dof'
            % 构建相对路径
            file = 'md_ctrl_2md_26DOF.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'copterforce'
            % 构建相对路径
            file = 'md_ctrl_2md_2CopterForceModel.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'coptermotor'
            % 构建相对路径
            file = 'md_ctrl_2md_2CopterMotorModel.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'groundmodel'
            % 构建相对路径
            file = 'md_ctrl_2md_2GroundModel.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case '3doutput'
            % 构建相对路径
            file = 'md_ctrl_2md_23DOutput.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sensoroutput'
            % 构建相对路径
            file = 'md_ctrl_2md_2SensorOutput.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'MotorFault'
            % 构建相对路径
            file = 'md_phm_2md_2Motor_01FaultModel.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PropFault'
            % 构建相对路径
            file = 'md_phm_2md_2Prop_01FaultModel.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'BatteryFault'
            % 构建相对路径
            file = 'md_phm_2md_2BatteryFaultModel.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'LoadFault'
            % 构建相对路径
            file = 'md_phm_2md_2FailModel.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'WindFault'
            % 构建相对路径
            file = 'md_phm_2md_2Environment_01FaultModel.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'GPSFault'
            % 构建相对路径
            file = 'md_phm_2md_2HILGPSModle.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'SensorFault'
            % 构建相对路径
            file = 'md_phm_2md_2SensorFault.html';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'getTerrainAltData'
            % 构建相对路径
            file = 'classRflySimSDK_1_1ue_1_1UEMapServe_1_1UEMapServe.html#getTerrainAltData';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'LoadPngData'
            % 构建相对路径
            file = 'classRflySimSDK_1_1ue_1_1UEMapServe_1_1UEMapServe.html#LoadPngData';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflySendUE4CMD'
            % 构建相对路径
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4Cmd';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyCameraPosAng'
            % 构建相对路径
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4Cmd';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sendUE4PosFull'
            % 构建相对路径
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4PosFull';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sendUE4Pos2Ground'
            % 构建相对路径
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4Pos2Ground';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sendUE4ExtAct'
            % 构建相对路径
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4ExtAct';
            % 组合上一层目录路径和相对路径以获取绝对路径
            fullPath = fullfile(Dir, file);
            web(fullPath);
        % 其他情况
        otherwise
            helpText = 'No help available for the specified topic.';
    end
end

% simulink模块的mask editor中的maskhelp参数
% maskhelp使用如下语法eval('aerosharedhelp(''sixdof_euler_angles'')');