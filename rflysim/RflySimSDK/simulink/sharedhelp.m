function helpText = sharedhelp(topic)
    % ��ȡ��ǰ .m �ļ�������·��
    Path = getenv('PSP_PATH'); 
    % �������·��
    relativePath = 'RflySimAPIs/RflySimSDK/html';
    % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
    Dir = fullfile(Path, relativePath);
    % ���� topic �������ذ����ı�
    switch topic
        case 'UDP_20100_PX4SilRecv'
            % �������·��
            file = 'md_ctrl_2md_2UDP__20100__PX4SilRecv.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'UDP_30100_TrueSimRecv'
            % �������·��
            file = 'md_ctrl_2md_2UDP__30100__TrueSimRecv.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'UDP_40100_RflyPx4Recv'
            % �������·��
            file = 'md_ctrl_2md_2UDP__40100__RflyPx4Recv.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PX4SILIntFloatSend'
            % �������·��
            file = 'md_ctrl_2md_2PX4SILIntFloatSend.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PX4FaultInParamsSend'
            % �������·��
            file = 'md_ctrl_2md_2PX4FaultInParamsSend.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyUdpFast'
            % �������·��
            file = 'md_swarm_2md_2RflyUdpFast.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyUdpRaw'
            % �������·��
            file = 'md_swarm_2md_2RflyUdpRaw.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflySerialRaw'
            % �������·��
            file = 'md_swarm_2md_2RflySerialRaw.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyUdpMavlink'
            % �������·��
            file = 'md_swarm_2md_2RflyUdpMavlink.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyVrpnRecv'
            % �������·��
            file = 'md_swarm_2md_2RflyVrpnRecv.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'HighLevelMode'
            % �������·��
            file = 'md_swarm_2md_2HighLevelMode.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);            
        case 'AutoFirmwareAPI'
            % �������·��
            file = 'md_swarm_2md_2AutoFirmwareAPI.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);            
        case 'RealCtrl28D'
            % �������·��
            file = 'md_swarm_2md_2RealCtrl28D.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Offboard_full'
            % �������·��
            file = 'md_swarm_2md_2Offboard__full.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vel_ned_full'
            % �������·��
            file = 'md_swarm_2md_2vel__ned__full.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vel_body_full'
            % �������·��
            file = 'md_swarm_2md_2vel__body__full.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'pos_ned_full'
            % �������·��
            file = 'md_swarm_2md_2pos__ned__full.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vrpn_data_decoder'
            % �������·��
            file = 'md_swarm_2md_2vrpn__data__decoder.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'real_data_decoder'
            % �������·��
            file = 'md_swarm_2md_2real__data__decoder.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'SimpleCtrl4D'
            % �������·��
            file = 'md_swarm_2md_2SimpleCtrl4D.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Full_data_decoder'
            % �������·��
            file = 'md_swarm_2md_2Full_data_decoder.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Simple_data_decoder'
            % �������·��
            file = 'md_swarm_2md_2Simple__data__decoder.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Serial'
            % �������·��
            file = 'md_ctrl_2md_2Serial.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Read ADC Channels'
            % �������·��
            file = 'md_ctrl_2md_2Read_01ADC_01Channels.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'ParamUpdate'
            % �������·��
            file = 'md_ctrl_2md_2ParamUpdate.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'ExamplePrintFcn'
            % �������·��
            file = 'md_ctrl_2md_2ExamplePrintFcn.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'binary_logger'
            % �������·��
            file = 'md_ctrl_2md_2binary__logger.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sensor_combined'
            % �������·��
            file = 'md_ctrl_2md_2sensor__combined.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'input_rc'
            % �������·��
            file = 'md_ctrl_2md_2input__rc.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Speaker_Tune'
            % �������·��
            file = 'md_ctrl_2md_2Speaker__Tune.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'battery_measure'
            % �������·��
            file = 'md_ctrl_2md_2battery__measure.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vehicle_attitude'
            % �������·��
            file = 'md_ctrl_2vehicle__attitude.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'vehicle_gps'
            % �������·��
            file = 'md_ctrl_2vehicle__gps.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RGB_LED'
            % �������·��
            file = 'md_ctrl_2md_2RGB__LED.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PWM_output'
            % �������·��
            file = 'md_ctrl_2md_2PWM__output.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'AUX_output'
            % �������·��
            file = 'md_ctrl_2md_2AUX__output.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Read Async'
            % �������·��
            file = 'md_ctrl_2md_2uORB_01Read_01Async.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Read Function-Call Trigger'
            % �������·��
            file = 'md_ctrl_2md_2uORB_01Read_01Function-Call_01Trigger.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Write'
            % �������·��
            file = 'md_ctrl_2md_2uORB_01Write.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Write Advanced'
            % �������·��
            file = 'md_ctrl_2md_2uORB_01Write_01Advanced.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'uORB Write Advanced_dai'
            % �������·��
            file = 'md_ctrl_2md_2uORB_01Write_01Advanced__dai.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'HIL16CtrlsNorm'
            % �������·��
            file = 'md_ctrl_2md_2HIL16CtrlsNorm.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'HIL16CtrlsPWM'
            % �������·��
            file = 'md_ctrl_2md_2HIL16CtrlsPWM.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'InputRcNorm'
            % �������·��
            file = 'md_ctrl_2md_2InputRcNorm.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'InputRcCali'
            % �������·��
            file = 'md_ctrl_2md_2InputRcCali.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'OffboardAttCtrlAPI'
            % �������·��
            file = 'md_ctrl_2md_2OffboardAttCtrlAPI.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'OffboardAdvCtrlAPI'
            % �������·��
            file = 'md_ctrl_2md_2OffboardAdvCtrlAPI.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'OffboardPvaCtrlAPI'
            % �������·��
            file = 'md_ctrl_2md_2OffboardPvaCtrlAPI.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'TorqueThrustCtrls'
            % �������·��
            file = 'md_ctrl_2md_2TorqueThrustCtrls.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RCOverCtrlAPI'
            % �������·��
            file = 'md_ctrl_2md_2RCOverCtrlAPI.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RePX4Block'
            % �������·��
            file = 'md_ctrl_2md_2RePX4Block.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'OffCtrlMsgAll'
            % �������·��
            file = 'md_ctrl_2md_2OffCtrlMsgAll.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'Msg2SimulinkAPI'
            % �������·��
            file = 'md_ctrl_2md_2Msg2SimulinkAPI.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PosVelAttAll'
            % �������·��
            file = 'md_ctrl_2md_2PosVelAttAll.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
            
        case '6dof'
            % �������·��
            file = 'md_ctrl_2md_26DOF.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'copterforce'
            % �������·��
            file = 'md_ctrl_2md_2CopterForceModel.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'coptermotor'
            % �������·��
            file = 'md_ctrl_2md_2CopterMotorModel.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'groundmodel'
            % �������·��
            file = 'md_ctrl_2md_2GroundModel.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case '3doutput'
            % �������·��
            file = 'md_ctrl_2md_23DOutput.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sensoroutput'
            % �������·��
            file = 'md_ctrl_2md_2SensorOutput.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'MotorFault'
            % �������·��
            file = 'md_phm_2md_2Motor_01FaultModel.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'PropFault'
            % �������·��
            file = 'md_phm_2md_2Prop_01FaultModel.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'BatteryFault'
            % �������·��
            file = 'md_phm_2md_2BatteryFaultModel.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'LoadFault'
            % �������·��
            file = 'md_phm_2md_2FailModel.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'WindFault'
            % �������·��
            file = 'md_phm_2md_2Environment_01FaultModel.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'GPSFault'
            % �������·��
            file = 'md_phm_2md_2HILGPSModle.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'SensorFault'
            % �������·��
            file = 'md_phm_2md_2SensorFault.html';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'getTerrainAltData'
            % �������·��
            file = 'classRflySimSDK_1_1ue_1_1UEMapServe_1_1UEMapServe.html#getTerrainAltData';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'LoadPngData'
            % �������·��
            file = 'classRflySimSDK_1_1ue_1_1UEMapServe_1_1UEMapServe.html#LoadPngData';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflySendUE4CMD'
            % �������·��
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4Cmd';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'RflyCameraPosAng'
            % �������·��
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4Cmd';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sendUE4PosFull'
            % �������·��
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4PosFull';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sendUE4Pos2Ground'
            % �������·��
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4Pos2Ground';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        case 'sendUE4ExtAct'
            % �������·��
            file = 'classRflySimSDK_1_1ue_1_1UE4CtrlAPI_1_1UE4CtrlAPI.html#sendUE4ExtAct';
            % �����һ��Ŀ¼·�������·���Ի�ȡ����·��
            fullPath = fullfile(Dir, file);
            web(fullPath);
        % �������
        otherwise
            helpText = 'No help available for the specified topic.';
    end
end

% simulinkģ���mask editor�е�maskhelp����
% maskhelpʹ�������﷨eval('aerosharedhelp(''sixdof_euler_angles'')');