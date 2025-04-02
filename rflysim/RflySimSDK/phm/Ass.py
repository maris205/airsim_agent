import numpy as np
import matplotlib.pyplot as plt
import math
from openpyxl import Workbook

## @file
#  
#  @anchor Ass接口库文件



## @brief 定义了安全评估的核心函数
#  - @anchor SaftyAssessment
#  根据提供的索引、评估名称、数据项列表、数据维度、评估参数和控制命令，执行安全评估。
# 
#  @param Index 索引条目列表，包括起始索引、结束索引和坠落索引。
#  @param EvalName 数据名称。
#  @param EvalData 数据项列表，包括角度、速度、位置等。
#  @param EvalDim 数据维度列表，表示数据的具体维度。
#  @param EvalParam 参数项列表，包括数据频率、地面动能、索引权重等。
#  @param CtrlCmd 控制命令列表，代表位置和速度的目标命令。
#  @return 无返回值，但会设置全局变量以存储评估结果。
def SaftyAssessment(Index,EvalName,EvalData,EvalDim,EvalParam,CtrlCmd=[[0,0,0],[0,0,0]]):
    
    
    
    '''
    Index is the list of index entries:start_index、end_index、fall_index
    EvalName is the name of the data
    EvalData is a data  item list:Ang、Vel、Pos、...
    EvalDim is a dimension item list: Represents the specific dimension of Data
    EvalParam is a parameter item list: Including: data frequency (number of data in 1s), ground kinetic energy, index weight
    CtrlCmd is a Control command item list: Target command representing position and speed
    '''
    # 1、Initialize evaluation parameters
    global WeightNomalLevel,WeightSlightLevel,WeightSeriousLevel,WeightHazardLevel,WeightDisasterLevel,EvalMatrix,SaftyRank,Xsf,ProfustSaftyScoreGroup,DataFreq,SaftyDecFuncSet,CrashProfustSaftyScoreGroup
    global IndexName,RealTimeProfustSaftyScore
    RealTimeProfustSaftyScore = []
    IndexName = EvalName
    DataFreq = EvalParam[0]

    # Weight parameter
    WeightNomalLevel = 1
    WeightSlightLevel = 3
    WeightSeriousLevel = 6
    WeightHazardLevel = 9
    WeightDisasterLevel = 12

    EvalMatrix = np.zeros([len(EvalName),5])
    Xsf = np.array([])
    if Index[2] > 0: 
        Xsf = np.array([0,1])
    else:
        Xsf = np.array([1,0])

    ProfustSaftyScoreGroup = []
    CrashProfustSaftyScoreGroup = []
    W_EvalData = EvalParam[2] 
    SaftyDecFuncSet = np.array([])
    SaftyRank = {
        'Nomal':[0.9,0.999],
        'Slight':[0.7,0.9],       
        'Serious':[0.4,0.7],         
        'Hazard':[0.1,0.4],
        'Disaster':[0,0.1]    
    }
    SaftdataNum = len(EvalData)
    global EvalStand
    EvalStand = []
    DesiredData = []
    for i in range(len(EvalName)):
        if EvalName[i] == 'Ang':  
            AngEvalStand = {
                'Nomal':[0,0.0175],    
                'Slight':[0.0175,0.035], 
                'Serious':[0.035,0.0525],  
                'Hazard':[0.0525,0.07],    
            }
            AngDesiredData = EvalData[i]
            EvalStand.append(AngEvalStand)
            DesiredData.append(AngDesiredData)
        elif EvalName[i] == 'Vel':
            VelEvalStand = {
                'Nomal':[0,0.015],     
                'Slight':[0.015,0.04], 
                'Serious':[0.04,0.16],  
                'Hazard':[0.16,0.5],    
            }
            VelDesiredData = np.zeros(len(EvalData[0]))
            VelDesiredData[:] = CtrlCmd[1][EvalDim[i]]
            EvalStand.append(VelEvalStand)
            DesiredData.append(VelDesiredData)
        elif EvalName[i] == 'Pos':
            PosEvalStand = {
                'Nomal':[0,0.0009],    
                'Slight':[0.0009,0.09], 
                'Serious':[0.09,0.36],  
                'Hazard':[0.36,1],  
            }
            PosDesiredData = np.zeros(len(EvalData[0]))
            PosDesiredData[:] = CtrlCmd[0][EvalDim[i]]
            EvalStand.append(PosEvalStand)
            DesiredData.append(PosDesiredData)

    # 2、Data preprocessing
    Data_index = 0
    while True:
        if  Data_index >= SaftdataNum:
            break

        # 1、Data preprocessing
        # 1.1)Take out the data within the evaluation scope
        Data = EvalData[Data_index][Index[0]:Index[1]]
        # 1.2)Get the required dimension
        DataEval = Data[:,EvalDim[Data_index]]

        # 2、Grouping
        GroupInterval = 2 
        MeanDataEvalSeries = np.array([]) # Storage mean series
        # 3、Find the mean sequence of all groups
        i = 0
        while True:
            # Collect the last set of mean data
            if i + GroupInterval >= len(DataEval):
                MeanGroup = np.mean(DataEval[i:len(DataEval)])
                MeanDataEvalSeries = np.append(MeanDataEvalSeries,MeanGroup)
                i = i + GroupInterval
                break

            MeanGroup = np.mean(DataEval[i:i+GroupInterval])
            MeanDataEvalSeries = np.append(MeanDataEvalSeries,MeanGroup)
            i = i + GroupInterval
        
        # 4、Finding error sequence(Ang:First-order difference; Vel、Pos:Error)
        ErrorDataEvalSeries = np.array([])
        if EvalName[Data_index] == 'Ang':
            index = 1
            while True:
                if index >= len(MeanDataEvalSeries):
                    break
                DetaAng = abs(MeanDataEvalSeries[index] - MeanDataEvalSeries[index - 1])
                ErrorDataEvalSeries = np.append(ErrorDataEvalSeries,DetaAng)
                index = index + 1
        elif EvalName[Data_index] == 'Vel' or EvalName[Data_index] == 'Pos':
            index = 1
            DesiredCmd = np.resize(DesiredData[Data_index],MeanDataEvalSeries.shape)
            while True:
                if index >= len(MeanDataEvalSeries):
                    break
                DetaVel_Pos = (MeanDataEvalSeries[index] - DesiredCmd[index])**2
                ErrorDataEvalSeries = np.append(ErrorDataEvalSeries,DetaVel_Pos)
                index = index + 1 
        '''
        x = np.linspace(1,len(ErrorDataEvalSeries),len(ErrorDataEvalSeries))
        plt.figure(1)
        plt.xlabel(EvalName[Data_index])
        plt.ylabel('deta')
        plt.plot(x,ErrorDataEvalSeries)
        plt.show()
        '''
        
        '''
        # 5、Find Allan variance
        AllanVar = 0
        AllanSum = 0
        for i in range(len(ErrorDataEvalSeries)):
            AllanSum = AllanSum + ErrorDataEvalSeries[i]
        AllanVar = round((1/(2*len(ErrorDataEvalSeries)))*AllanSum,2)
        '''

        # 3、Safty evaluation algorithm
        ProfustSafty(ErrorDataEvalSeries,EvalParam[1],Data_index)
        Data_index = Data_index + 1

    # 4、Profust safety level
    SaftyEvalMatrix = np.dot(W_EvalData,EvalMatrix) 
    # print("SaftyEvalMatrix",SaftyEvalMatrix)
    SaftyLevel = {
        'Unaffected':SaftyEvalMatrix[0],
        'Minor':SaftyEvalMatrix[1],
        'Medium':SaftyEvalMatrix[2],
        'Serious':SaftyEvalMatrix[3],
        'Catastrophic':SaftyEvalMatrix[4]
    }
    global ProfustSaftyLevelUAV,ProfustSaftyScoreUAV
    if Index[2] > 0:
        ProfustSaftyLevelUAV = 'Catastrophic'
    else:
        ProfustSaftyLevelUAV = max(SaftyLevel,key=SaftyLevel.get)
    # print('Profust Safty Level:',ProfustSaftyLevelUAV)

    # 5、Profust safety score
    NoCrashSaftyScoreUAV = 0
    for i in range(len(EvalParam[2])):
        IndexScore = EvalParam[2][i]*ProfustSaftyScoreGroup[i]
        NoCrashSaftyScoreUAV = NoCrashSaftyScoreUAV + IndexScore
    CrashSaftyScoreUAV = np.min(CrashProfustSaftyScoreGroup)
    SaftyDecFuncSet = np.array([NoCrashSaftyScoreUAV,CrashSaftyScoreUAV])
    ProfustSaftyScoreUAV = round(np.dot(Xsf,SaftyDecFuncSet),3)
    # print('Profust Safty Score:',ProfustSaftyScoreUAV)

## @brief 定义了风险评估的辅助函数
#  - @anchor ProfustSafty
#  根据误差数据序列、坠落能量和数据索引，计算风险评估。
# 
#  @param ErrorDataEvalSeries 误差数据序列。
#  @param FallEnergy 坠落能量。
#  @param Data_index 数据索引。
#  @return 无返回值，但会更新全局变量以反映实时安全评分和健康状态。
def ProfustSafty(ErrorDataEvalSeries,FallEnergy,Data_index):

    global EvalMatrix,ProfustSaftyScoreGroup,CrashProfustSaftyScoreGroup,RealTimeProfustSaftyScore
    # Evaluate the risk every 0.1s
    Num = round(DataFreq/10)
    # Crash decision function
    alpha = 10**6
    beta = 100
    E_imp = FallEnergy
    Ps = 0.1
    CrashProfustSaftyScore = 0
    if E_imp == 0:
        CrashProfustSaftyScore = 0.1
        CrashProfustSaftyScoreGroup.append(CrashProfustSaftyScore)
    else:
        P = 1/(1+((alpha/beta)*(beta/E_imp)**(1/(4*Ps)))**0.5)
        R1 = 10**(-8)
        k = 5*10**(-6)
        R = (math.exp((P/R1)*k)-1)/(math.exp((P/R1)*k)+1)
        CrashProfustSaftyScore = round(0.1-(R/10),3)
        CrashProfustSaftyScoreGroup.append(CrashProfustSaftyScore)

    # No crash decision function
    # 1、Find the attenuation model parameters k and a of different health states
    # Nomal
    K_Nomal = (EvalStand[Data_index].get('Nomal')[0]*math.log(SaftyRank.get('Nomal')[0]) - EvalStand[Data_index].get('Nomal')[1]*math.log(SaftyRank.get('Nomal')[1]))/(math.log(SaftyRank.get('Nomal')[1])-math.log(SaftyRank.get('Nomal')[0]))
    A_Nomal = (-math.log(SaftyRank.get('Nomal')[1]))/(EvalStand[Data_index].get('Nomal')[0]+K_Nomal)
    # Slight
    K_Slight = (EvalStand[Data_index].get('Slight')[0]*math.log(SaftyRank.get('Slight')[0]) - EvalStand[Data_index].get('Slight')[1]*math.log(SaftyRank.get('Slight')[1]))/(math.log(SaftyRank.get('Slight')[1])-math.log(SaftyRank.get('Slight')[0]))
    A_Slight = (-math.log(SaftyRank.get('Slight')[1]))/(EvalStand[Data_index].get('Slight')[0]+K_Slight)
    # Serious
    K_Serious = (EvalStand[Data_index].get('Serious')[0]*math.log(SaftyRank.get('Serious')[0]) - EvalStand[Data_index].get('Serious')[1]*math.log(SaftyRank.get('Serious')[1]))/(math.log(SaftyRank.get('Serious')[1])-math.log(SaftyRank.get('Serious')[0]))
    A_Serious = (-math.log(SaftyRank.get('Serious')[1]))/(EvalStand[Data_index].get('Serious')[0]+K_Serious)
    # Hazard
    K_Hazard = (EvalStand[Data_index].get('Hazard')[0]*math.log(SaftyRank.get('Hazard')[0]) - EvalStand[Data_index].get('Hazard')[1]*math.log(SaftyRank.get('Hazard')[1]))/(math.log(SaftyRank.get('Hazard')[1])-math.log(SaftyRank.get('Hazard')[0]))
    A_Hazard = (-math.log(SaftyRank.get('Hazard')[1]))/(EvalStand[Data_index].get('Hazard')[0]+K_Hazard)

    Index = 0

    # 2、Find the safety score corresponding to the attenuation model
    ProfustGroup = np.array([])
    while True:
        if Index + Num >= len(ErrorDataEvalSeries):
            Index = len(ErrorDataEvalSeries) - 1
            if EvalStand[Data_index].get('Nomal')[0] <= ErrorDataEvalSeries[Index] < EvalStand[Data_index].get('Nomal')[1]:
                Score = math.exp((-A_Nomal)*(ErrorDataEvalSeries[Index]+K_Nomal))
                ProfustGroup = np.append(ProfustGroup,Score)
            elif EvalStand[Data_index].get('Slight')[0] <= ErrorDataEvalSeries[Index] < EvalStand[Data_index].get('Slight')[1]:
                Score = math.exp((-A_Slight)*(ErrorDataEvalSeries[Index]+K_Slight))
                ProfustGroup = np.append(ProfustGroup,Score)
            elif EvalStand[Data_index].get('Serious')[0] <= ErrorDataEvalSeries[Index] < EvalStand[Data_index].get('Serious')[1]:
                Score = math.exp((-A_Serious)*(ErrorDataEvalSeries[Index]+K_Serious))
                ProfustGroup = np.append(ProfustGroup,Score)
            elif EvalStand[Data_index].get('Hazard')[0] <= ErrorDataEvalSeries[Index] < EvalStand[Data_index].get('Hazard')[1]:
                Score = math.exp((-A_Hazard)*(ErrorDataEvalSeries[Index]+K_Hazard))
                ProfustGroup = np.append(ProfustGroup,Score)
            else:
                ProfustGroup = np.append(ProfustGroup,CrashProfustSaftyScore)
            break

        if EvalStand[Data_index].get('Nomal')[0] <= ErrorDataEvalSeries[Index] < EvalStand[Data_index].get('Nomal')[1]:
            Score = math.exp((-A_Nomal)*(ErrorDataEvalSeries[Index]+K_Nomal))
            ProfustGroup = np.append(ProfustGroup,Score)
        elif EvalStand[Data_index].get('Slight')[0] <= ErrorDataEvalSeries[Index] < EvalStand[Data_index].get('Slight')[1]:
            Score = math.exp((-A_Slight)*(ErrorDataEvalSeries[Index]+K_Slight))
            ProfustGroup = np.append(ProfustGroup,Score)
        elif EvalStand[Data_index].get('Serious')[0] <= ErrorDataEvalSeries[Index] < EvalStand[Data_index].get('Serious')[1]:
            Score = math.exp((-A_Serious)*(ErrorDataEvalSeries[Index]+K_Serious))
            ProfustGroup = np.append(ProfustGroup,Score)
        elif EvalStand[Data_index].get('Hazard')[0] <= ErrorDataEvalSeries[Index] < EvalStand[Data_index].get('Hazard')[1]:
            Score = math.exp((-A_Hazard)*(ErrorDataEvalSeries[Index]+K_Hazard))
            ProfustGroup = np.append(ProfustGroup,Score)
        else:
            ProfustGroup = np.append(ProfustGroup,CrashProfustSaftyScore)
        
        Index = Index + Num

    # 3、Map the score obtained from the attenuation model according to the membership function
    U_Score = np.array([])
    for i in range(len(ProfustGroup)):
        if ProfustGroup[i]>0.9:
            u = 1
        else:
            u = ProfustGroup[i]
        U_Score = np.append(U_Score,u)
    ProfustHealth = np.array([]) 
    ProfustHealth = np.append(ProfustHealth,U_Score[0])

    # 4、Membership transfer processing (health status transfer processing)
    for i in range(len(U_Score)-1):
        ScoreBegin = U_Score[i]
        ScoreEnd = U_Score[i+1]
        if ScoreEnd <= ScoreBegin: 
            U_Tsf = ScoreBegin - ScoreEnd
            Tsf_score = ScoreBegin*(1-U_Tsf)
        else: 
            Tsf_score = U_Score[i]
        ProfustHealth = np.append(ProfustHealth,Tsf_score)
    RealTimeProfustSaftyScore.append(ProfustHealth)

    # 5、EvalMatrix
    WeightGroup = np.array([])
    for i in range(len(ProfustHealth)):
        if SaftyRank.get('Nomal')[0] < ProfustHealth[i] <= 1:
            WeightGroup = np.append(WeightGroup,WeightNomalLevel)
            EvalMatrix[Data_index][0] += 1 
        elif SaftyRank.get('Slight')[0] < ProfustHealth[i] <= SaftyRank.get('Slight')[1]:
            WeightGroup = np.append(WeightGroup,WeightSlightLevel)
            EvalMatrix[Data_index][1] += 1 
        elif SaftyRank.get('Serious')[0] < ProfustHealth[i] <= SaftyRank.get('Serious')[1]:
            WeightGroup = np.append(WeightGroup,WeightSeriousLevel)
            EvalMatrix[Data_index][2] += 1 
        elif SaftyRank.get('Hazard')[0] < ProfustHealth[i] <= SaftyRank.get('Hazard')[1]:
            WeightGroup = np.append(WeightGroup,WeightHazardLevel)
            EvalMatrix[Data_index][3] += 1 
        else:
            WeightGroup = np.append(WeightGroup,WeightDisasterLevel)
            EvalMatrix[Data_index][4] += 1 

    # 6、Grade weight distribution
    EvalMatrix[Data_index][0] = EvalMatrix[Data_index][0] * 1
    EvalMatrix[Data_index][1] = EvalMatrix[Data_index][1] * 3
    EvalMatrix[Data_index][2] = EvalMatrix[Data_index][2] * 6
    EvalMatrix[Data_index][3] = EvalMatrix[Data_index][3] * 9
    EvalMatrix[Data_index][4] = EvalMatrix[Data_index][4] * 12
    WeightGroup = WeightGroup/sum(WeightGroup)

    # 7、Calculate the weighted rate module safety score
    ProfustSaftyScore = 0
    for i in range(len(ProfustHealth)):
        ProfustSaftyScore = ProfustSaftyScore + WeightGroup[i]*ProfustHealth[i]
    ProfustSaftyScoreGroup.append(ProfustSaftyScore)

    # 8、Standardization
    MatrixSum = np.sum(EvalMatrix[Data_index])
    for i in range(len(EvalMatrix[Data_index])):
        EvalMatrix[Data_index][i] = EvalMatrix[Data_index][i]/MatrixSum
            












        
        



    