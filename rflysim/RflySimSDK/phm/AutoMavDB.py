import threading
import numpy as np
import re
import sqlite3
import os
import time
import sys
import csv
import json
from fnmatch import fnmatch
import lxml.html as lh
import matplotlib.pyplot as plt
import shutil
import pandas as pd
import AutoREG as AutoREG
import QGCCtrlAPI

## @file 
#  该文件定义了与无人机测试数据库交互的类。
#  此类负责管理无人机测试数据的存储、更新和查询操作，以及同步JSON格式的测试用例数据到数据库。
#  @anchor AutoMavDB接口库文件

## @brief 用于将数据库查询结果转换为字典的工厂函数。
# - @anchor dict_factory
# @param cursor 数据库游标对象。
# @param row 查询结果的单行数据。
# @return 返回一个字典，其中包含列名和对应数据。
def dict_factory(cursor, row):  
    d = {}  
    for idx, col in enumerate(cursor.description):  
        d[col[0]] = row[idx]  
    return d 

## @class MAVDB
# @brief 管理无人机数据库的类。
class MAVDB:
    ## @brief构造函数，初始化MAVDB类的实例。
    # - @anchor __init__
    # @param conf 包含无人机配置信息的列表，例如['Quadcopter', 'SITL', 1]。用于设置数据库连接和相关配置。
    def __init__(self, conf): # conf_eg: ['Quadcopter', 'SITL', 1]
        self.cursor = None
        self.mydb = None
        self.conf = conf
        self.is_tested = 0
        self.count = 0
        model_path = os.path.join(sys.path[0],'model',AutoREG.MAV_FRAME_DICT[AutoREG.RFLYSIM_FRAME[conf[0]]][0])
        '''
        conf[0]                      : Quadcopter
        RFLYSIM_FRAME[Quadcopter]    : 1
        AutoREG.MAV_FRAME_DICT[1][0] : Quadcopter
        '''
        dbf = 'db.json'
        dbp = [filename for filename in os.listdir(model_path) if dbf in filename]
        self.jsonpath = os.path.join(model_path, dbp[0])
        MAVDB.JSON_TO_SQL(self)
        self.VISIONFLAG = MAVDB.VISION(self)
    
    ## @brief检查是否开启视觉功能。
    # - @anchor VISION
    # @param self MAVDB类的实例。
    # @return 如果视觉功能开启返回True，否则返回False。
    def VISION(self):
        jsonpath = self.jsonpath
        with open(jsonpath, "r",encoding='utf-8') as f:
            json_data = json.load(f)
        isVision = json_data.get('VISION') 
        if isVision == 'on':
            return True
        else:
            return False
    
    ## @brief将JSON格式的测试用例数据同步到数据库。此方法从JSON文件中读取测试用例数据，并将其转换为数据库中的记录，以便后续操作和查询。
    # - @anchor JSON_TO_SQL
    def JSON_TO_SQL(self):
        '''
        Synchronize test cases in json files and database
        '''
        path = self.jsonpath
        
        with open(path, "r",encoding='utf-8') as f:
            json_data = json.load(f)

        json_case = json_data.get('FAULT_CASE')
        '''
        db_case_eg:
        [
         {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}
        ]
        
        json_case_eg:
        [
         {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 2, 'Subsystem': 'Sensor', 'Component': 'Magnetometer', 'FaultID': '123544', 'FaultType': 'Magnetometer noise', 'FaultMode': 'Magnetometer noise interference', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123544,0;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 3, 'Subsystem': 'Environment', 'Component': 'ConstWind', 'FaultID': '123458', 'FaultType': 'ConstWind interference', 'FaultMode': 'Flight encounters constant winds', 'FaultParams': '15,15,14', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123458,123458,15,15,14;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 4, 'Subsystem': 'Sensor', 'Component': 'Accelerometer', 'FaultID': '123542', 'FaultType': 'Accelerometer noise', 'FaultMode': 'Accelerometer noise interference', 'FaultParams': '1', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123542,1;1,1,10', 'TestStatus': 'Finished'}
        ]
        '''
        '''
        1. Open JSON file and database connection
        '''
        MAVDB.GET_CURSOR(self)
        for item in json_case:
            '''
            2. Traverse JSON data
            '''
            self.cursor.execute("SELECT * FROM TEST_CASE WHERE CaseID=?", (item['CaseID'],))
            '''
            3. Check if the ID exists
            '''
            existing_data = self.cursor.fetchone()
            if existing_data:
                '''
                4. If the ID already exists, update the data in the database using JSON data
                '''
                # print('The same caseID exists, replace')
                self.cursor.execute("UPDATE TEST_CASE SET Subsystem=?, Component=?, FaultID=?, FaultType=?, FaultMode=?, FaultParams=?, ControlSequence=?, TestStatus=? WHERE CaseID=?",
                            (item['Subsystem'], item['Component'], item['FaultID'], item['FaultType'], item['FaultMode'], item['FaultParams'], item['ControlSequence'], item['TestStatus'], item['CaseID']))
            else:
                '''
                5. If the ID does not exist, insert the JSON data into the database
                '''
                # print('The same caseID does not exist, add')
                self.cursor.execute("INSERT INTO TEST_CASE (CaseID, Subsystem, Component, FaultID, FaultType, FaultMode, FaultParams, ControlSequence, TestStatus) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                            (item['CaseID'], item['Subsystem'], item['Component'], item['FaultID'], item['FaultType'], item['FaultMode'], item['FaultParams'], item['ControlSequence'], item['TestStatus']))

            self.mydb.commit()

    ## @brief获取数据库游标。返回数据库游标对象，用于后续的数据库查询和更新操作。
    # - @anchor GET_CURSOR
    def GET_CURSOR(self):
        frame = AutoREG.MAV_FRAME_DICT[AutoREG.RFLYSIM_FRAME[self.conf[0]]][0] + '.db'
        imPath = os.path.join(sys.path[0],'case',frame)
        self.mydb = sqlite3.connect(imPath)
        self.mydb.row_factory = dict_factory
        self.cursor=self.mydb.cursor()
    
    ## @brief获取所有故障测试用例。
    # - @anchor GET_FAULT_CASE
    # @return 返回包含所有故障测试用例的列表。
    def GET_FAULT_CASE(self): 
        '''
        Obtain fault test cases
        '''
        MAVDB.GET_CURSOR(self)
        sql='''
            select   *
            from     TEST_CASE
            '''
        self.cursor.execute(sql)
        result=self.cursor.fetchall()
        return result
    
    ## @brief根据CaseID获取测试用例详细信息。
    # - @anchor GET_CASEINFO
    # @param case_id 测试用例的ID。
    # @return 返回与给定CaseID对应的测试用例详细信息。包括子系统、组件、故障类型等。
    def GET_CASEINFO(self, case_id):
        path = self.jsonpath

        with open(path, "r",encoding='utf-8') as f:
            json_data = json.load(f)
        
        caseInfo = json_data['FAULT_CASE']

        for fault_case in caseInfo:
            if fault_case['CaseID'] == case_id:
                return fault_case

        return None

    ## @brief从指定路径的JSON文件中获取测试用例的详细信息。
    # - @anchor GET_CASEINFO_P
    # @param self MAVDB类的实例。
    # @param case_id 测试用例的ID。
    # @param path JSON文件的路径。
    # @return 返回与给定CaseID对应的测试用例详细信息。
    def GET_CASEINFO_P(self, case_id, path):

        with open(path, "r",encoding='utf-8') as f:
            json_data = json.load(f)
        
        caseInfo = json_data['FAULT_CASE']

        for fault_case in caseInfo:
            if fault_case['CaseID'] == case_id:
                return fault_case

        return None

    ## @brief更新指定路径JSON文件中的测试状态信息。
    # - @anchor MAV_JSONPro_P
    # @param self MAVDB类的实例。
    # @param case_id 测试用例的ID。
    # @param path JSON文件的路径。
    def MAV_JSONPro_P(self, case_id, path): 
        '''
        Change the test status information of json file
        '''

        with open(path, "r",encoding='utf-8') as f:
            db_data = json.load(f)
        json_case = db_data.get('FAULT_CASE')
        '''
        [
        {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}
        ]
        '''
        if len(json_case) >= 1:
            path = self.jsonpath

            with open(path, "r",encoding='utf-8') as f:
                db_data = json.load(f)
                db_data['FAULT_CASE'][case_id-1]['TestStatus'] = 'Finished'
                data = db_data
            f.close()

            with open(path, "w",encoding='utf-8') as w:
                json.dump(data,w,indent=4) 
            w.close()
    
    ## @brief获取测试用例ID列表，，用于测试用例的管理。
    # - @anchor GET_CASEID
    # @return 返回包含所有测试用例ID的列表。
    def GET_CASEID(self): 
        '''
        Obtain the list of fault test case IDs
        '''
        result = MAVDB.GET_FAULT_CASE(self)
        '''
        result_eg:
        [
         {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 2, 'Subsystem': 'Sensor', 'Component': 'Magnetometer', 'FaultID': '123544', 'FaultType': 'Magnetometer noise', 'FaultMode': 'Magnetometer noise interference', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123544,0;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 3, 'Subsystem': 'Environment', 'Component': 'ConstWind', 'FaultID': '123458', 'FaultType': 'ConstWind interference', 'FaultMode': 'Flight encounters constant winds', 'FaultParams': '15,15,14', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123458,123458,15,15,14;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 4, 'Subsystem': 'Sensor', 'Component': 'Accelerometer', 'FaultID': '123542', 'FaultType': 'Accelerometer noise', 'FaultMode': 'Accelerometer noise interference', 'FaultParams': '1', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123542,1;1,1,10', 'TestStatus': 'Finished'}
        ]
        '''
        path = self.jsonpath

        with open(path, "r",encoding='utf-8') as f:
            json_data = json.load(f)

        global caselist
        if json_data.get('TEST_CASE') == 'All':
            caselist = []
            casedata = result
            for data in casedata:
                ID = data['CaseID']
                caselist.append(ID)
            return caselist
        else:
            if self.conf[2] > 1: # Multi Instance
                case = json_data.get('TEST_CASE')
                '''
                case_eg:
                "1,2,5;2,1,1;4,5,6;"
                '''
                
                len_seg = len(re.findall(r';',case))
                if len_seg:
                    mcase = re.findall(r'-?\d+', case.split(";")[0]) # ['1,2', '2,6']
                    '''['1', '2']'''
                    len_case = len(mcase)
                    if len_case > 1:
                        ''' Multi-machine mode multiple test cases, eg: "1,2;3,5" '''
                        segments = case.split(";")
                        '''
                        ['1,2,5', '2,1,1', '4,5,6', '']
                        '''
                        segment_lists = [list(filter(None, segment.split(','))) for segment in segments]
                        '''
                        [['1', '2', '5'], ['2', '1', '1'], ['4', '5', '6'], []]
                        '''
                        max_length = max(len(segment) for segment in segment_lists)

                        result = []
                        for i in range(max_length):
                            result.append([int(segment[i]) for segment in segment_lists if i < len(segment)])
                        '''
                        [[1, 2, 4], [2, 1, 5], [5, 1, 6]]
                        '''
                        return result
                    else:
                        ''' Multi-machine mode single test case, eg:"1;2" ''' 
                        segments = case.split(";")
                        ''' ['1', '2'] '''
                        result = []
                        
                        for segment in segments:
                            elements = list(map(int, segment.split(',')))
                            result.append(elements)

                        return result
                else:
                    warn = 'Sorry, your configuration is wrong, please check as follows:\n1) Whether your UAV num is greater than 1; \n2) Did you forget to add a semicolon(;) in the "TEST_CASE"  of your json file to switch to multi-machine mode?\n'
                    print(warn)
                    AutoREG.BREAK_DOWN = True

            else:
                result = [int(val) for val in re.split(',',json_data.get('TEST_CASE'))]
                '''[1, 2, 5]'''
                return result

    ## @brief将测试结果添加到数据库中，用于记录和分析测试结果。
    # - @anchor GET_MAVCMD
    # @param case_id 测试用例的ID。
    # @return 返回与给定CaseID对应的控制命令序列。
    def GET_MAVCMD(self,case_id): 
        '''
        Process Control Sequence
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            select * from TEST_CASE
            where CaseID = {}
            '''.format(case_id)
        self.cursor.execute(sql)
        data = self.cursor.fetchall()
        case_sequence = data[0].get('ControlSequence')
        case = re.split(';',case_sequence)
        cmd = np.array([])
        for i in range(len(case)):
            cmd = np.append(cmd,case[i])
        '''
        ['2,1' '1,1,5' '2,3,150,0,-30' '1,1,10' '2,4,1000,0,-30' '1,1,10' '2,6,123450,0' '1,1,10']
        '''  
        return cmd   

    ## @brief添加测试结果到数据库。
    # - @anchor RESULT_DBPro
    # @param data 包含测试结果的数据。
    def RESULT_DBPro(self,data): 
        '''
        Process test result library (add test results)
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            insert into TEST_RESULT
            (CaseID, FaultID, CaseDescription, FaultMode, ControlSequence, TestResult)
            values(?, ?, ?, ?, ?, ?) 
            '''
        value = (data[0],data[1],data[2],data[3],data[4],data[5])
        self.cursor.execute(sql,value)
        self.mydb.commit()

    ## @brief更新测试用例的状态，标记测试用例是否已完成。
    # - @anchor TEST_STATEPro
    # @param case_id 测试用例的ID。
    def TEST_STATEPro(self,case_id): 
        '''
        Process test case library (change test status)
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            update TEST_CASE
            set TestStatus = 'Finished'
            where CaseID = {}
            '''.format(case_id)
        self.cursor.execute(sql)
        self.mydb.commit()

    ## @brief判断测试用例是否已测试。
    # - @anchor IS_TESTEDPro
    # @param case_id 测试用例的ID。
    # @return 如果已测试返回1，否则返回0。
    def IS_TESTEDPro(self,case_id): 
        '''
        Judge whether it is a tested case
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            select * from TEST_CASE
            where CaseID = {}
            '''.format(case_id)
    
        self.cursor.execute(sql)
        data = self.cursor.fetchall()

        if data[0].get('TestStatus') == 'Finished':
            self.is_tested = 1
        else:
            self.is_tested = 0
        return self.is_tested

    ## @brief重置测试结果数据库，用于重复使用测试用例。
    # - @anchor RESETR_DB
    # @param case_id 测试用例的ID。
    def RESETR_DB(self,case_id): 
        '''
        Handle the result library of repeated use case test
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            DELETE FROM TEST_CASE
            WHERE CaseID = {}
            '''.format(case_id)
    
        self.cursor.execute(sql)

    ## @brief更新JSON文件中的测试状态信息，反映测试的最新状态。
    # - @anchor MAV_JSONPro
    # @param case_id 测试用例的ID。
    def MAV_JSONPro(self,case_id): 
        '''
        Change the test status information of json file
        '''
         
        path = self.jsonpath
        with open(path, "r",encoding='utf-8') as f:
            db_data = json.load(f)
        json_case = db_data.get('FAULT_CASE')
        '''
        [
        {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}
        ]
        '''
        if len(json_case) >= 1:
            path = self.jsonpath

            with open(path, "r",encoding='utf-8') as f:
                db_data = json.load(f)
                db_data['FAULT_CASE'][case_id-1]['TestStatus'] = 'Finished'
                data = db_data
            f.close()

            with open(path, "w",encoding='utf-8') as w:
                json.dump(data,w,indent=4) 
            w.close()

## @class DataREG
# @brief 用于管理测试数据注册表的类。
class DataREG():
    
    lock = threading.Lock()
    isDeleted = False
    MavNum = 0
    Mode2mavP = []
    Mode2caseP = []
    Mode3frameP = []
    Mode3caseP = []
    Mode4frameP = []
    Mode4caseP = []
    Mode4mavREG = {
        'Quadcopter' : [],
        'Fixedwing'  : [],
        'Vtol'       :[]
    }
    html_file_path = os.path.abspath(os.path.join(sys.path[0],'data','TestInfo.html'))
    TestResult_html_file_path = os.path.abspath(os.path.join(sys.path[0],'data','TestResult.html'))
    bk_path = os.path.abspath(os.path.join(sys.path[0],'conf','material','bk3.png'))
    bk_path = bk_path.replace('\\','//')
    bk_path2 = os.path.abspath(os.path.join(sys.path[0],'conf','material','bk2.png'))
    bk_path2 = bk_path2.replace('\\','//')
    dataPollNum = 0
    Finally_Path_for_Multi = None

## @class DATAAPI
# @brief 用于API数据交互的类。
class DATAAPI():
    ## @brief构造函数，初始化DATAAPI类的实例。
    # - @anchor __init__
    # @param CaseID 测试用例的ID。
    # @param conf 包含无人机配置信息的列表。
    # @param Data 测试数据。
    # @param Info 附加信息。
    def __init__(self,CaseID,conf,Data,Info):
        # conf_eg: ['Quadcopter', 'SITL', 1]
        self.Data = Data
        self.conf = conf
        self.CaseID = CaseID
        self.Info = Info

        model_path = os.path.join(sys.path[0],'model',AutoREG.MAV_FRAME_DICT[AutoREG.RFLYSIM_FRAME[conf[0]]][0])
        dbf = 'db.json'
        dbp = [filename for filename in os.listdir(model_path) if dbf in filename]
        self.jsonpath = os.path.join(model_path, dbp[0])

        conf = self.conf[1] + '.bat'  
        batp = [filename for filename in os.listdir(model_path) if conf in filename]  
        self.batpath = os.path.join(model_path, batp[0])

        self.isTrueDataRecordOver = 0
        self.MacVechileNum = AutoREG.MAV_DATA_FOLDER_REG[AutoREG.RFLYSIM_FRAME[self.conf[0]]]
    
        self.PlatFormpath = self.PX4Path() # 'C:\\PX4PSP'
        self.DataPath = sys.path[0] + '/data'
        self.MavFrameDataPath = self.DataPath + f'/{self.conf[0]}'

        self.mode1sInsp = self.DataPath + '/single' + '/sInstance' 
        self.mode2mInsp = self.DataPath + '/single' + '/mInstance'
        self.mode3sInsp = self.DataPath + '/multi' + '/sInstance'
        self.mode4mInsp = self.DataPath + '/multi' + '/mInstance'
        
        self.DP()
    
    ## @brief处理数据并准备存储。该方法进行数据处理，包括创建必要的目录、处理数据，并准备存储到文件或数据库。
    # - @anchor DP
    def DP(self):
        DataREG.lock.acquire()
        self.MKDPath()
        self.DataPro()
        self.TruedataRecord()
        self.RName()
        DataREG.lock.release()
        self.InfoDown()
        self.Test_result_Record()
    ## @brief创建数据存储所需的目录结构。
    # - @anchor MKDPath
    # @param self DATAAPI类的实例。
    def MKDPath(self):
        DataREG.MavNum += 1
        if AutoREG.TEST_MODE == 1:
            '''Single Instance. Folder struct named with mode/frame/TestCase_''' 
            fp = self.mode1sInsp + f'/{self.conf[0]}'
            cp = fp + f'/TestCase_{self.CaseID}'

            self.Mode1InsP = cp

            if os.path.exists(cp): 
                shutil.rmtree(cp)
            self.logp = cp + '/log' 
            self.truep = cp + '/true'
            self.MFolder(self.logp, self.truep)

        elif AutoREG.TEST_MODE == 2:
            '''Single frame Multi Instance. Folder struct named with mode/frame/TestCase_*_*/'''
            fp = self.mode2mInsp + f'/{self.conf[0]}'
            self.tempCaseP = fp + '/TestCase'

            mavP = self.tempCaseP + f'/mav{AutoREG.DIND_REG[self.conf[0]]+1}'
            self.mavTestID = AutoREG.DIND_REG[self.conf[0]]+1

            DataREG.Mode2mavP.append(f'/mav{AutoREG.DIND_REG[self.conf[0]]+1}')
            DataREG.Mode2caseP.append(self.CaseID)

            AutoREG.DIND_REG[self.conf[0]] += 1

            self.logp = mavP + '/log'
            self.truep = mavP + '/true'
            self.MFolder(self.logp, self.truep)
 
        elif AutoREG.TEST_MODE == 3:
            '''Multi frame Single Instance. Folder struct named with mode/TestCase_/frame/'''
            self.tempCaseP = self.mode3sInsp + f'/TestCase'

            frameP = self.tempCaseP + f'/{self.conf[0]}'

            DataREG.Mode3frameP.append(f'/{self.conf[0]}')
            DataREG.Mode3caseP.append(self.CaseID)

            self.logp = frameP + '/log'
            self.truep = frameP + '/true'
            self.MFolder(self.logp, self.truep)

        elif AutoREG.TEST_MODE == 4:
            '''Multi frame multi Instance. Folder struct named with mode/TestCase_/frame/'''
            self.tempCaseP = self.mode4mInsp + '/TestCase'

            frameP = self.tempCaseP + f'/{self.conf[0]}'
            mavP = frameP + f'/mav{AutoREG.DIND_REG[self.conf[0]]+1}'
            self.mavTestID = AutoREG.DIND_REG[self.conf[0]]+1

            DataREG.Mode4mavREG[self.conf[0]].append(f'/mav{AutoREG.DIND_REG[self.conf[0]]+1}')
            DataREG.Mode4frameP.append(f'/{self.conf[0]}')
            DataREG.Mode4caseP.append(self.CaseID)

            AutoREG.DIND_REG[self.conf[0]] += 1

            self.logp = mavP + '/log'
            self.truep = mavP + '/true'
            self.MFolder(self.logp, self.truep) 
    
    ## @brief获取PX4的路径。
    # - @anchor PX4Path
    # @return 返回PX4控制台的路径，该路径可能用于后续的日志下载或参数配置。
    def PX4Path(self):
        PX4Path=os.environ.get('PSP_PATH')
        if PX4Path is None:  # 如果没有成功获取路径，就使用PX4PSP默认路径
            PX4Path=r'C:\PX4PSP'
            with open(self.batpath, 'r',encoding='UTF-8') as file:
                for line in file:
                    if line.find('SET PSP_PATH=')!=-1:
                        PX4Path=line.replace('SET PSP_PATH=','')
                        PX4Path=PX4Path.strip()
                        PX4Path=PX4Path.replace('\\','/')
                        break
        
        return PX4Path

    ## @brief创建目录。
    # - @anchor MFolder
    # @param TargetFilder_log 日志文件的目标文件夹路径。
    # @param TargetFilder_truedata 真实数据文件的目标文件夹路径。
    def MFolder(self, TargetFilder_log, TargetFilder_truedata):
        os.makedirs(TargetFilder_log) 
        os.makedirs(TargetFilder_truedata)

    ## @brief处理数据。该方法处理从无人机接收到的测试数据，可能包括日志下载、数据解析等操作。
    # - @anchor DataPro
    def DataPro(self):
        
        '''
        # QGC下载接口
        qgc = QGCCtrlAPI.QGCCtrlAPI()
        logName = qgc.ReqQgcLog(AutoREG.LOG_TIMEOUT, self.Info[2])
        if logName!='':
                shutil.copyfile(qgc.QGCPath+'\\'+logName, self.logp + '\\' + logName)
                print('Download log ' + logName+ f' for mav{self.Info[2]} successfully.')
        '''
        
        # 1、List the directories under the file
        log_path = self.PlatFormpath + f'/Firmware/build/px4_sitl_default/instance_{DataREG.MavNum}/log'

        PlatForm_log_dirs = os.listdir(log_path) 
        log_data = PlatForm_log_dirs[len(PlatForm_log_dirs)-1]
        self.path = os.path.join(log_path,log_data) 
        dirs = os.listdir(self.path) 

        # 2、Get the latest ulg file
        ulg = dirs[len(dirs)-1]
        ulgPath = os.path.join(self.path,ulg) 

        # 3、Copy the ulg file to the log folder
        TargetPath_log = self.logp + '/{}'.format(ulg)
        shutil.copyfile(ulgPath, TargetPath_log) 
    
    ## @brief记录真实数据。该方法将无人机的真实测试数据记录到CSV文件中，包括速度、角度和位置等信息。
    # - @anchor TruedataRecord
    def TruedataRecord(self):
            truedata_v_csv_path = self.truep + '//truedata_vel.csv' 
            with open(truedata_v_csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                for row in self.Data[0]:
                    writer.writerow(row)

            truedata_ang_csv_path = self.truep + '//truedata_ang.csv' 
            with open(truedata_ang_csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                for row in self.Data[1]:
                    writer.writerow(row)
            
            truedata_pos_csv_path = self.truep + '//truedata_pos.csv' 
            with open(truedata_pos_csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                for row in self.Data[2]:
                    writer.writerow(row)
                
    ## @brief记录测试信息。该方法将测试信息记录到CSV文件中，包括日期、框架、案例ID、测试信息和数据路径等。
    # - @anchor InfoRecord
    def InfoRecord(self):
        infop = os.path.abspath(os.path.join(sys.path[0],'data','TestInfo.csv'))
        if not os.path.exists(infop):
            header = [
                ['Date','Frame', 'CaseID', 'S/HITL','TestInfo','DataPath','Normal?']
            ]

            df = pd.DataFrame(header)

            df.to_csv(infop, index=False, header=False)

        date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        frame = self.conf[0]
        caseID = self.CaseID
        SHITL = self.conf[1]
        testinfo = self.CMDAna(self.Info[0]) 
        datapath = self.GetInfoP()
        warnInfo = 'Yes'

        if  AutoREG.ARMED_WARN:
            warnInfo = 'No! Armed exception, recommend retest!'
        
        self.dataInfo = [
            [date, frame, caseID, SHITL, testinfo, datapath, warnInfo]
        ]

        ff = pd.DataFrame(self.dataInfo)
        ff.to_csv(infop, mode='a', index=False, header=False)

        self.to_html(self.dataInfo, DataREG.html_file_path, DataREG.bk_path)
    
    ## @brief记录测试结果。该方法将测试结果记录到CSV文件中，包括案例ID、故障ID、案例描述、故障模式、控制序列和测试结果等。
    # - @anchor Test_result_Record
    def Test_result_Record(self):
        infop = os.path.abspath(os.path.join(sys.path[0],'data','TestResultInfo.csv'))
        if not os.path.exists(infop):
            header = [
                ['CaseID','FaultID', 'CaseDescription', 'FaultMode','ControlSequence','TestResult']
            ]

            df = pd.DataFrame(header)

            df.to_csv(infop, index=False, header=False)

        
        TestResultInfo = [
            self.Info[1]
        ]

        ff = pd.DataFrame(TestResultInfo)
        ff.to_csv(infop, mode='a', index=False, header=False)

        DataREG.lock.acquire()
        self.test_result_to_html(TestResultInfo, DataREG.TestResult_html_file_path, DataREG.bk_path2)
        DataREG.lock.release()

    ## @brief分析控制命令。
    # - @anchor CMDAna
    # @param cmd 控制命令序列。
    # @return 返回分析后的控制命令信息，可能包括命令类型、时间戳和参数等。
    def CMDAna(self,cmd):
        subcmd = ';'.join(cmd)
        subcmd = subcmd.split(';')

        mtime = 0
        info = ''

        if subcmd[0][0] == '2':
            for index, cmd in enumerate(subcmd):
                cmd = cmd.split(',')
                if cmd[0] == '2':
                    info += f'{mtime}s: '
                    if cmd[1] == '1' or cmd[1] == '2':
                        info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]]
                    elif cmd[1] == '3' or cmd[1] == '4' or cmd[1] == '5':
                        if not AutoREG.RFLY_SPVO_MODE:
                            info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format(cmd[2],cmd[3],cmd[4])  
                        else:
                            info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format('SPVO','SPVO','SPVO')
                    elif cmd[1] == '6':
                        fp = [float(str) for str in cmd[3:] if str != cmd[2]]
                        info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format(cmd[2],fp)
                    info += ' \n'

                if cmd[0] == '1':
                    mtime += int(cmd[2])
                    if index == len(subcmd) - 1:
                        info += f'{mtime}s: Exit test!'
                
        elif subcmd[0][0] == '1':
            index = 0
            for i in range(len(subcmd)):
                if subcmd[i][0] != '1':
                    index = i
                    break
            
            for index, cmd in enumerate(subcmd[index:], start=index):
                cmd = cmd.split(',')
                if cmd[0] == '2':
                    info += f'{mtime}s: '
                    if cmd[1] == '1' or cmd[1] == '2':
                        info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]]
                    elif cmd[1] == '3' or cmd[1] == '4' or cmd[1] == '5':
                        if not AutoREG.RFLY_SPVO_MODE:
                            info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format(cmd[2],cmd[3],cmd[4])  
                        else:
                            info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format('SPVO','SPVO','SPVO')
                    elif cmd[1] == '6':
                        fp = [float(str) for str in cmd[3:] if str != cmd[2]]
                        info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format(cmd[2],fp)
                    info += ' \n'

                if cmd[0] == '1':
                    mtime += int(cmd[2])
                    if index == len(subcmd) - 1:
                        info += f'{mtime}s: Exit test!'
        return info

    ## @brief用于获取测试信息的路径。
    # - @anchor GetInfoP
    # @return 返回存储测试信息的路径。
    def GetInfoP(self):
        path = None
        if AutoREG.TEST_MODE == 1:
            path = self.Mode1InsP
        elif AutoREG.TEST_MODE == 2:
            path = self.Mode2InsP
        elif AutoREG.TEST_MODE == 3:
            path = self.Mode3InsP
        elif AutoREG.TEST_MODE == 4:
            path = self.Mode4InsP
        
        path = os.path.abspath(path)
        splitstr = os.path.basename(os.path.abspath(os.path.join(sys.path[0])))
        '''splitstr: BETA'''

        spath = path.split(splitstr)[1]
        spath = spath[1:]

        return spath

    ## @brief重命名目录。该方法根据测试结果重命名目录，以便于后续的数据分析和归档。
    # - @anchor RName
    def RName(self):
        if AutoREG.TEST_MODE == 1:
            self.SExecute(self.Mode1InsP)
        if AutoREG.TEST_MODE == 2:
            '''Single frame Multi Instance. Folder struct named with mode/frame/TestCase_*_*/'''
            self.Mode2InsP = self.tempCaseP
            if len(DataREG.Mode2mavP) == AutoREG.MAV_NUM:
                for i in range(len(DataREG.Mode2caseP)):
                    self.Mode2InsP += f'_{DataREG.Mode2caseP[i]}'

                DataREG.Finally_Path_for_Multi = self.Mode2InsP

                if not DataREG.isDeleted:
                    if os.path.exists(self.Mode2InsP): 
                        shutil.rmtree(self.Mode2InsP)
                    DataREG.isDeleted = True

                os.rename(self.tempCaseP,self.Mode2InsP)
                self.SExecute(self.Mode2InsP)
            
        elif AutoREG.TEST_MODE == 3:
            '''Multi frame Single Instance. Folder struct named with mode/TestCase_/frame/'''
            self.Mode3InsP = self.tempCaseP
            if len(DataREG.Mode3frameP) == AutoREG.MAV_NUM:
                for i in range(len(DataREG.Mode3caseP)):
                    self.Mode3InsP += f'_{DataREG.Mode3caseP[i]}'

                DataREG.Finally_Path_for_Multi = self.Mode3InsP

                if not DataREG.isDeleted:
                    if os.path.exists(self.Mode3InsP): 
                        shutil.rmtree(self.Mode3InsP)
                    DataREG.isDeleted = True
                
                os.rename(self.tempCaseP,self.Mode3InsP)
                self.SExecute(self.Mode3InsP)
        
        elif AutoREG.TEST_MODE == 4:
            '''Multi frame multi Instance. Folder struct named with mode/TestCase_/frame/'''
            self.Mode4InsP = self.tempCaseP
            if len(DataREG.Mode4caseP) == AutoREG.MAV_NUM:
                for i in range(len(DataREG.Mode4caseP)):
                    self.Mode4InsP += f'_{DataREG.Mode4caseP[i]}'

                DataREG.Finally_Path_for_Multi = self.Mode4InsP
                
                if not DataREG.isDeleted:
                    # If the destination folder exists, delete the reconstruction
                    if os.path.exists(self.Mode4InsP): 
                        shutil.rmtree(self.Mode4InsP)
                    DataREG.isDeleted = True
                
                os.rename(self.tempCaseP,self.Mode4InsP)
                self.SExecute(self.Mode4InsP)
    
    ## @brief执行数据后处理。
    # - @anchor SExecute
    # @param start_dir 起始目录路径。
    # @details 该方法在数据后处理过程中执行必要的命令，例如日志文件的转换。
    def SExecute(self,start_dir):
        for root, dirs, files in os.walk(start_dir):
            for file in files:
                if file.endswith('.ulg'):
                    current_dir = os.path.abspath(root)
                    os.chdir(current_dir)
                    cmd = "for %i in (*); do ulog2csv %i"
                    os.system(cmd)

    ## @brief将文本中的换行符转换为HTML的换行标签。
    # - @anchor convert_newlines_to_br
    # @param text 需要转换的原始文本。
    # @return 返回转换后的文本，其中的换行符被替换为HTML的<br>标签。
    def convert_newlines_to_br(self, text):
        return re.sub(r'\n', '<br />', text)

    ## @brief生成带有特定样式的HTML表格。
    # - @anchor generate_html_with_style
    # @param html_table HTML表格的字符串表示。
    # @param bk 背景图片的路径。
    # @return 返回包含样式的HTML表格字符串。
    def generate_html_with_style(self, html_table, bk):
        html = f"""
            <html>
            <head>
            <style>
            body {{
                background-image: url('{bk}');
                background-repeat: no-repeat;
                background-size: cover;
            }}
            .d2 {{
                line-height: 50px;
                text-align: center;
                background: rgba(255,255,255,0.5);
                font-weight: bold;
                font-size: 5pt; 
                font-family: sans-serif;
                border-collapse: collapse; 
                border: 1px solid silver;
            }}
            table {{
                table-layout: fixed;
                width: 100%;
                border-collapse: collapse;
                font-family: sans-serif;
                font-size: 12pt;
            }}
            table,th {{
                padding: 10px;
                padding-left:0px;
                font-weight: bold;
                background-color: rgba(0,0,255,0.1);
                color: white;
            }}
            table,td{{
                background-color: rgba(255,0,0,0.2);
                color: white;
                white-space:pre-wrap; 
                word-wrap: break-word;
            }}
            </style>
            </head>
            <body>
            
            <div class="d2"> 
                {html_table}
            </div>

            </body>
            </html>
        """
        return html
    
    ## @brief生成带有特定样式的测试结果HTML表格。
    # - @anchor generate_test_result_html_with_style
    # @param html_table 测试结果HTML表格的字符串表示。
    # @param bk 背景图片的路径。
    # @return 返回包含样式的测试结果HTML表格字符串。
    def generate_test_result_html_with_style(self, html_table, bk):
        html = f"""
            <html>
            <head>
            <style>
            body {{
                background-image: url('{bk}');
                background-repeat: no-repeat;
                background-size: cover;
            }}
            .d2 {{
                line-height: 50px;
                text-align: center;
                background: rgba(255,255,255,0.5);
                font-weight: bold;
                font-size: 5pt; 
                font-family: sans-serif;
                border-collapse: collapse; 
                border: 1px solid silver;
            }}
            table {{
                table-layout: fixed;
                width: 100%;
                border-collapse: collapse;
                font-family: sans-serif;
                font-size: 12pt;
            }}
            table,th {{
                padding: 10px;
                padding-left:0px;
                font-weight: bold;
                background-color: rgba(0,0,255,0.1);
                color: white;
            }}
            table,td{{
                background-color: rgba(255,0,0,0.2);
                color: white;
                white-space:pre-wrap; 
                word-wrap: break-word;
            }}
            </style>
            </head>
            <body>
            
            <div class="d2"> 
                {html_table}
            </div>

            </body>
            </html>
        """
        return html

    ## @brief从HTML文件中读取数据并转换为列表。
    # - @anchor read_data_from_html_tolist
    # @param html_file HTML文件的路径。
    # @return 返回从HTML文件中提取的数据列表。
    def read_data_from_html_tolist(self, html_file):
        # Read data from HTML file
        doc = lh.parse(html_file)
        tables = doc.xpath('//table')

        # Extract data from all tables
        data = []
        for table in tables:
            rows = []
            for tr in table.xpath('.//tr'):
                row = [td.text_content().strip() for td in tr.xpath('.//td')]
                rows.append(row)
            data.extend(rows)
        return data

    ## @brief生成HTML文件的头部，包含表格的列标题。
    # - @anchor generate_header
    # @param html_file HTML文件的路径。
    # @param bk_path 背景图片的路径。
    # @return 如果文件已存在，则返回True；否则，生成头部并返回False。
    def generate_header(self, html_file, bk_path):

        if os.path.exists(html_file):
            return True

        header = [
            ['Date','Frame', 'CaseID', 'S/HITL','TestInfo','DataPath','Normal?']
        ]

        df = pd.DataFrame(header)
        html_table = df.to_html(index=False, header=False, escape=False)
        header = self.generate_html_with_style(html_table, bk_path)
        with open(html_file, 'w') as file:
            file.write(header)

        '''
        Mode2:
        header_html = '<tr><td><font color="white"><b>Frame</b></font></td>  <td><font color="white"><b>CaseID</b></font></td>  <td><font color="white"><b>S/HITL</b></font></td>  <td><font color="white"><b>TestInfo</b></font></td>  <td><font color="white"><b>DataPath</b></font></td> <td><font color="white"><b>Normal?</b></font></td>  </tr>'
        frame_html = '<table border="1" class="dataframe"><tbody>{}</tbody></table>'.format(header_html)
        header = self.generate_html_with_style(frame_html, bk_path)
        with open(html_file, 'w') as file:
            file.write(header)
        '''
        
        return False
    
    ## @brief生成测试结果HTML文件的头部，包含测试结果表格的列标题。
    # - @anchor generate_test_result_header
    # @param html_file 测试结果HTML文件的路径。
    # @param bk_path 背景图片的路径。
    # @return 如果文件已存在，则返回True；否则，生成头部并返回False。
    def generate_test_result_header(self, html_file, bk_path):

        if os.path.exists(html_file):
            return True

        header = [
            ['CaseID','FaultID', 'CaseDescription', 'FaultMode','ControlSequence','TestResult']
        ]

        df = pd.DataFrame(header)
        html_table = df.to_html(index=False, header=False, escape=False)
        header = self.generate_test_result_html_with_style(html_table, bk_path)
        with open(html_file, 'w') as file:
            file.write(header)
        
        return False
    
    ## @brief为HTML数据显示添加换行标签。
    # - @anchor add_br_to_html
    # @param new_html_data_list 新的HTML数据列表。
    # @return 返回包含换行标签的HTML数据列表。
    def add_br_to_html(self, new_html_data_list):
        for index, data in enumerate(new_html_data_list):

            testInfo = data[4]
            new_html_data_list[index][4] = self.add_ControlSequence_br(testInfo)

        return new_html_data_list

    ## @brief将测试信息转换为HTML格式并写入文件。
    # - @anchor to_html
    # @param dataInfo 包含测试信息的数据列表。
    # @param html_file_path HTML文件的路径。
    # @param bk_path 背景图片的路径。
    def to_html(self, dataInfo, html_file_path, bk_path):
        has_header = self.generate_header(html_file_path, bk_path)

        # 1、Get new data and replace newline tags with spaces for comparison
        sub_br = re.sub(r'\n', '', dataInfo[0][4])
        nd_sub_br = dataInfo[0][:]
        nd_sub_br[4] = sub_br[:]

        # 2、Read data in html
        html_data_list = self.read_data_from_html_tolist(html_file_path)
        dataLen = len(html_data_list)

        # 3、Compare row by row, if the same data is found, replace
        frame, caseID, SHITL = nd_sub_br[1], str(nd_sub_br[2]), nd_sub_br[3]
        # print(f'frame:{frame} caseID:{caseID} SHITL:{SHITL} testInfo:{testInfo}')
    
        if dataLen >=2:
            # 3.1、If there is data, first compare the new data and the original data to see if they are the same. If they are the same, replace them.，
            samekey = False
            for index, new in enumerate(html_data_list):
                if new[1] == frame and new[2] == caseID and new[3] == SHITL:
                    # print('The same data exists, start replacing new data')
                    html_data_list[index] = nd_sub_br[:]
                    samekey = True
                    break
            
            # If different, append and write
            if not samekey:
                html_data_list.append(nd_sub_br)

            # 3.2、For all data, add html line break tags
            new_html_data_list = self.add_br_to_html(html_data_list)

            # 3.3、Rewrite html
            df = pd.DataFrame(new_html_data_list)
            html_table = df.to_html(index=False, header=False, escape=False)
            new_html_table = self.generate_html_with_style(html_table, bk_path)
            with open(html_file_path, 'w') as file:
                file.write(new_html_table)
        else:
            # Write the first data
            dataInfo[0][4] = self.convert_newlines_to_br(dataInfo[0][4])
            df = pd.DataFrame(dataInfo)
            html_table = df.to_html(index=False, header=False, escape=False)
            new_html_table = self.generate_html_with_style(html_table, bk_path)
            with open(html_file_path, 'a') as file:
                file.write(new_html_table)

    ## @brief将测试结果转换为HTML格式并写入文件。
    # - @anchor test_result_to_html
    # @param testresultdata 包含测试结果的数据列表。
    # @param html_file_path 测试结果HTML文件的路径。
    # @param bk_path 背景图片的路径。
    def test_result_to_html(self, testresultdata, html_file_path, bk_path):
        has_header = self.generate_test_result_header(html_file_path, bk_path)

        sub_br_CaseDescription = re.sub(r'\n', '', testresultdata[0][2])
        sub_br_ControlSequence = re.sub(r'\n', '', testresultdata[0][4])
        sub_br_TestResult = re.sub(r'\n', '', testresultdata[0][5])

        # 1、Get new data
        nd_sub_br = testresultdata[0][:]
        nd_sub_br[2] = sub_br_CaseDescription[:]
        nd_sub_br[4] = sub_br_ControlSequence[:]
        nd_sub_br[5] = sub_br_TestResult[:]

        # 2、Read data in html
        html_data_list = self.read_data_from_html_tolist(html_file_path)
        dataLen = len(html_data_list)

        # 3、Compare row by row, if the same data is found, replace
        frame, caseID, SHITL = self.conf[0], str(self.CaseID), self.conf[1]
        # print(f'Frame: {frame} caseID: {caseID} SHITL: {SHITL}')

        if dataLen >=2:
            # 3.1、If there is data, first compare the new data and the original data to see if they are the same. If they are the same, replace them.，
            samekey = False
            for index, new in enumerate(html_data_list):
                new_frame = self.Get_test_result_frame(new[2])
                new_caseID = new[0]
                new_SHITL = self.Get_test_result_shitl(new[2])
                if new_frame == frame and new_caseID == caseID and new_SHITL == SHITL:
                    print('The same data exists, start replacing new data')
                    html_data_list[index] = nd_sub_br[:]
                    samekey = True
                    break
            
            # If different, append and write
            if not samekey:
                html_data_list.append(nd_sub_br)

            # 3.2、For all data, add html line break tags
            new_html_data_list = self.add_br_to_test_result_html(html_data_list)

            # 3.3、Rewrite html
            df = pd.DataFrame(new_html_data_list)
            html_table = df.to_html(index=False, header=False, escape=False)
            new_html_table = self.generate_test_result_html_with_style(html_table, bk_path)
            with open(html_file_path, 'w') as file:
                file.write(new_html_table)
        else:
            # Write the first data
            testresultdata[0][2] = self.convert_newlines_to_br(testresultdata[0][2])
            testresultdata[0][4] = self.convert_newlines_to_br(testresultdata[0][4])
            testresultdata[0][5] = self.convert_newlines_to_br(testresultdata[0][5])

            df = pd.DataFrame(testresultdata)
            html_table = df.to_html(index=False, header=False, escape=False)
            new_html_table = self.generate_test_result_html_with_style(html_table, bk_path)
            with open(html_file_path, 'a') as file:
                file.write(new_html_table)
    
    ## @brief为测试结果HTML数据显示添加换行标签。
    # - @anchor add_br_to_test_result_html
    # @param new_html_data_list 新的测试结果HTML数据列表。
    # @return 返回包含换行标签的测试结果HTML数据列表。
    def add_br_to_test_result_html(self, new_html_data_list):
        for index, data in enumerate(new_html_data_list[1:],1):

            CaseDescription = data[2]
            ControlSequence = data[4]
            TestResult = data[5]

            need_add_str = [CaseDescription, ControlSequence, TestResult]

            for ind, add_str in enumerate(need_add_str):
                if ind == 0:
                    variables = [self.Get_test_result_frame(add_str), self.Get_test_result_shitl(add_str)]
                    new_html_data_list[index][2] = self.add_CaseDescription_br(add_str, variables)
                    pass

                if ind == 1:
                    new_html_data_list[index][4] = self.add_ControlSequence_br(add_str)
                    pass

                if ind == 2:
                    new_html_data_list[index][5] = self.add_TestResult_br(add_str)
                    pass
            
        return new_html_data_list

    ## @brief 为测试结果描述添加HTML换行标签。
    # - @anchor add_CaseDescription_br
    # @param self DATAAPI类的实例。
    # @param caseDescription 测试结果描述文本。
    # @param variables 需要添加换行标签的变量列表。
    # @return 返回添加了换行标签的测试结果描述文本。
    def add_CaseDescription_br(self, caseDescription, variables):
        for variable in variables:
            pattern = re.compile(rf"({variable}\s*)")
            caseDescription = re.sub(pattern, r"\1<br />", caseDescription)
        
        return caseDescription

    ## @brief 为控制序列描述添加HTML换行标签。
    # - @anchor add_ControlSequence_br
    # @param self DATAAPI类的实例。
    # @param testInfo 控制序列描述文本。
    # @return 返回添加了换行标签的控制序列描述文本。
    def add_ControlSequence_br(self, testInfo):
        # Prepend a newline character before all times except the first
        pattern = r'(\d+s:)'
        matches = re.findall(pattern, testInfo)
        if matches:
            for match in matches[1:]:
                # Handle the case of mistaken addition
                if re.findall(r'(\d+)\<br />(\d+s)', testInfo):
                    err = re.findall(r'\b(\d+)\<br />(\d+s)\b', testInfo)
                    for er in err:
                        testInfo = testInfo.replace(er[0]+'<br />'+er[1], er[0]+er[1])

                testInfo = testInfo.replace(match, '<br />' + match)
        
        # Add <br> string before Fault injection type and Fault injection parameters
        if 'Fault injection type' in testInfo:
            testInfo = testInfo.replace('Fault injection type:', '<br />Fault injection type:')
        if 'Fault injection parameters' in testInfo:
            testInfo = testInfo.replace('Fault injection parameters:', '<br />Fault injection parameters:')

        return testInfo[:]

    ## @brief 为测试结果添加HTML换行标签。
    # - @anchor add_TestResult_br
    # @param self DATAAPI类的实例。
    # @param testresult 测试结果文本。
    # @return 返回添加了换行标签的测试结果文本。
    def add_TestResult_br(self, testresult):
        keys = [key for key in AutoREG.TEST_RESULT.keys() if key != 'Is_Fall']
        for key in keys:
            testresult = testresult.replace(key, "<br />" + key)
        
        return testresult
        
    ## @brief从测试结果描述中提取无人机框架信息。
    # - @anchor Get_test_result_frame
    # @param text 测试结果描述文本。
    # @return 返回提取的无人机框架信息。
    def Get_test_result_frame(self, text):
        pattern = r"Frame:\s*(Quadcopter|Fixedwing|Vtol)"
        match = re.search(pattern, text)

        if match:
            return match.group(1)

        return [None]

    ## @brief从测试结果描述中提取仿真类型信息。
    # - @anchor Get_test_result_shitl
    # @param text 测试结果描述文本。
    # @return 返回提取的仿真类型信息。
    def Get_test_result_shitl(self, text):
        pattern = r"S/HITL:\s*(SITL|HITL)"
        match = re.search(pattern, text)

        if match:
            return match.group(1)

        return [None]

    ## @brief处理并存储测试信息，等待所有数据处理线程完成后执行。
    # - @anchor InfoDown
    def InfoDown(self):
        DataREG.dataPollNum += 1
        # Wait for all processes to modify the path name
        while True:
            if DataREG.dataPollNum == AutoREG.MAV_NUM:
                break
        
        # Update the latest path
        if AutoREG.TEST_MODE == 2:
            self.Mode2InsP = DataREG.Finally_Path_for_Multi + f'/mav{self.mavTestID}'
        elif AutoREG.TEST_MODE == 3:
            self.Mode3InsP = DataREG.Finally_Path_for_Multi + f'/{self.conf[0]}'
        elif AutoREG.TEST_MODE == 4:
            self.Mode4InsP = DataREG.Finally_Path_for_Multi + f'/mav{self.mavTestID}'
        
        DataREG.lock.acquire()
        # Start writing to html and csv
        self.InfoRecord()
        DataREG.lock.release()






