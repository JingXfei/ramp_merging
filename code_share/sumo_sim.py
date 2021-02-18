"""
Write sumo config file automatically.
"""
import socket
import time
import sys
import os
import numpy as np
import pickle as pkl
import time
# import traci
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'") 
import traci
import traci.constants as tc
import createVehTypeDistribution as cd

# 声明部分仿真参数
SIMU_TIME = 180 # 总时长，秒
STEP_LENGTH = 0.5 # 仿真步长，秒
ROAD_LENGTH = 200 # 道路长度，米
MAIN,RAMP = 1,2 # 区分车辆来源
CAV,HV = 1,2 # 区分车辆类型
MAX_SPEED = 30 # 最大速度
MAX_ACCE_HV, MAX_ACCE_CAV = 1.4976, 1.5 # 最大加速度                                       | norm(1.4976, 0.0555),     | 1.5000
MAX_DECE_HV, MAX_DECE_CAV = 4.0522, 6 # 最大减速度                                         | norm(4.0522, 0.9979),     | 6.0000
VEH_LENGTH = 4.9 # 车长                                                                   | norm(4.9,0.2); [3.5, 5.5] | AV is same
ACTION_STEP_LENGH = 0.1 # 车辆执行其决策逻辑（加速和换道）的时间间隔长度。
VEH_FOLLOW_MODE = 'IDM' # 跟车模型，The Intelligent Driver Model by Martin Treiber
SIGMA = 0.7954 # 跟车模型参数，驾驶员不完美状态，0表示完美，1表示最不完美，默认0.5                  | norm(0.7954, 0.1615),     |  0.5
TAU = 33.6166/40.6236 # 跟车模型参数，驾驶员期望最小车头时距，默认为1                            | gamma(33.6166, 40.6236),  |  0.5
MIN_GAP = 1.5401 # 最小跟车距离，未用到                                                      | norm(1.5401, 0.2188),     |  1.5014
SPEEDFACTOR = 1.2081 # 速度因子                                                            | norm(1.2081, 0.1425),     |  1
POISSON_MAIN = 720 # 车辆到达率，辆/小时
POISSON_RAMP = 720
RATIO = 0.6 # 渗透率
POISSON_MAIN_HV = POISSON_MAIN*(1-RATIO) 
POISSON_MAIN_CAV = POISSON_MAIN*RATIO
POISSON_RAMP_HV = POISSON_RAMP*(1-RATIO)
POISSON_RAMP_CAV = POISSON_RAMP*RATIO
INIT_SPEED_MEAN = 15 # 初始速度
INIT_SPEED_SIGMA = 1 # 初始速度标准差
DEPARTSPEED = 'max'
DEPARTPOS = 'base'
begin = np.sort(np.abs(np.random.randn(4)))

class SumoSim():
    def __init__(self, distributional_vtype=False):
        self.has_addition = False
        self.distributional_vtype = distributional_vtype
        self.co_num, self.co_vehicles = 0, []
        
    def write_vtype_addtional_file(self):
        path = './'

        # create HV vehicles
        config = 'HV.txt'
        out = 'HV.add.xml'
        os.remove(path+out)
        options = vehicleDis(path, config, out, vehDistName='hv', seed=np.random.randint(0, 1000000))
        cd.main(options)
        self.additional_files.append(out)
        print('\nVehicle additional files '+out+' write done!')

        # create CAV vehicles
        config = 'CAV.txt'
        out = 'CAV.add.xml'
        os.remove(path+out)
        options = vehicleDis(path, config, out, vehDistName='cav', seed=np.random.randint(0, 1000000))
        cd.main(options)
        self.additional_files.append(out)
        print('\nVehicle additional files '+out+' write done!')

    def write_flow_config(self, file_path):
        # create veh type and flow
        start = '<routes>\n'
        veh_type_1 = ''
        veh_type_2 = ''
        if not self.has_addition:
            veh_type_1 = ('<vType id="hv" vClass="taxi" accel="'+str(MAX_ACCE_HV)
                            +'" decel="'+str(MAX_DECE_HV)
                            +'" length="'+str(VEH_LENGTH)
                            +'" maxSpeed="'+str(MAX_SPEED)
                            +'" speedFactor="'+str(SPEEDFACTOR)
                            +'" carFollowModel="'+'IDM'
                            # +'" actionStepLength="'+str(ACTION_STEP_LENGH)
                            +'" sigma="'+str(SIGMA)
                            +'" tau="'+str(TAU)
                            +'" miniGap="'+str(MIN_GAP)
                            +'" guiShape="bus" color="red" />\n')
            veh_type_2 = ('<vType id="cav" vClass="taxi" accel="'+str(MAX_ACCE_CAV)
                            +'" decel="'+str(MAX_DECE_CAV)
                            +'" length="'+str(VEH_LENGTH)
                            +'" maxSpeed="'+str(MAX_SPEED)
                            +'" carFollowModel="'+'IDM'
                            +'" sigma="'+str(0.5)
                            +'" tau="'+str(0.5)
                            +'" miniGap="'+str(1.5014)
                            # +'" actionStepLength="'+str(ACTION_STEP_LENGH)
                            +'" guiShape="bus" color="green" />\n')
        route_1 = '<route id="main" edges="left_in merged"/>\n'
        route_2 = '<route id="ramp" edges="ramp_in merged"/>\n'
        flow_1 = ('<flow id="f_main_hv" type="hv" route="main" vehsPerHour="'+str(POISSON_MAIN*(1-self.ratio))
                    +'" begin="'+str(begin[0])
                    +'" departSpeed="'+self.depart_speed
                    +'" departPos="'+self.depart_pos+'"/>\n')
        flow_2 = ('<flow id="f_main_cav" type="cav" route="main" vehsPerHour="'+str(POISSON_MAIN*self.ratio)
                    +'" begin="'+str(begin[1])
                    +'" departSpeed="'+self.depart_speed
                    +'" departPos="'+self.depart_pos+'"/>\n')
        flow_3 = ('<flow id="f_ramp_hv" type="hv" route="ramp" vehsPerHour="'+str(POISSON_RAMP*(1-self.ratio))
                    +'" begin="'+str(begin[2])
                    +'" departSpeed="'+self.depart_speed
                    +'" departPos="'+self.depart_pos+'"/>\n')
        flow_4 = ('<flow id="f_ramp_cav" type="cav" route="ramp" vehsPerHour="'+str(POISSON_RAMP*self.ratio)
                    +'" begin="'+str(begin[3])
                    +'" departSpeed="'+self.depart_speed
                    +'" departPos="'+self.depart_pos+'"/>\n')
        end = '</routes>\n'
        end = '</routes>\n'
        cfg = start+veh_type_1+veh_type_2+route_1+route_2+flow_1+flow_2+flow_3+flow_4+end
        with open(file_path, 'w') as f:
            f.write(cfg)

    def write_sumo_config(self, file_path):
        # sumo cfg
        head = '<?xml version="1.0" encoding="UTF-8"?>\n'
        start = '<configuration>\n'
        input_start = '<input>\n'
        net_file = '<net-file value="ramp.net.xml"/>\n'
        route_file = '<route-files value="av_hv.rou.xml"/>\n'
        if self.has_addition:
            files = ','.join(self.additional_files)
            add_file = '<additional-files value="'+files+'"/>\n'
        else:
            add_file = ''
        input_end = '</input>\n'

        output, co_output = '', ''
        output_start = '<output>\n'
        output = '<summary-output value="./summary/summary_'+str(self.cur_episode_id)+'_'+str(int(time.time()))+'.xml"/>'
        output_end = '</output>'        
        time_start = '<time>\n'
        begin_time = '<begin value="0"/>\n'
        end_time = '<end value="'+str(SIMU_TIME)+'"/>\n'
        time_end = '</time>\n'
        end = '</configuration>\n'
        cfg = head+ (start+ (input_start+net_file+route_file+add_file+input_end) + (output_start+output+co_output+output_end) + (time_start+begin_time+end_time+time_end) +end)
        with open(file_path, 'w') as f:
            f.write(cfg)
    
    def run_sumo(self):
        if self.distributional_vtype:
            self.has_addition = True
            self.additional_files = []
            self.write_vtype_addtional_file()
        flow_path = "./av_hv.rou.xml"
        cfg_path = "./av_hv.sumocfg"
        self.write_flow_config(flow_path)
        self.write_sumo_config(cfg_path)
        time.sleep(1)
        sumoBinary = "sumo"
        # sumoBinary = "sumo-gui"
        sumoCmd = [sumoBinary, "-c", cfg_path , "--step-length", str(STEP_LENGTH)]
        traci.start(sumoCmd)
        traci.junction.subscribeContext('gneJ1', tc.CMD_GET_VEHICLE_VARIABLE, 200, [tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_ACCELERATION])
        assert STEP_LENGTH == traci.simulation.getDeltaT()
        self.systime = 0
        while self.systime < SIMU_TIME:
            # run one step
            traci.simulation.step()
            temp = traci.simulation.getCollidingVehiclesNumber()
            if temp != 0:
                self.co_num += temp
                self.co_vehicles.append([self.cur_episode_id, self.systime, traci.simulation.getCollidingVehiclesIDList()])
            data = traci.junction.getContextSubscriptionResults('gneJ1')
            vehicles_id = list(data.keys())
            for i in vehicles_id:
                speed = data[i][tc.VAR_SPEED]
                self.mean_speed.append(speed)
            self.systime += STEP_LENGTH

    def main(self):
        self.collision_num = []
        ratio = np.array([0.0001, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.9999])
        depart_speed = ['10', 'random', '20', 'max']
        # depart_pos = ['base', 'random']
        depart_pos = ['base']*2
        i = 0
        for s in depart_speed:
            for p in depart_pos:
                for r in ratio:
                    self.mean_speed = []
                    self.depart_speed = s
                    self.depart_pos = p
                    self.ratio = r
                    self.cur_episode_id = i
                    print('cur_episode_id', self.cur_episode_id)
                    print('departspeed', s,'departpos', p, 'ratio', r, 'co_num', self.co_num)
                    self.run_sumo()
                    res = str(time.time())+' cur_episode_id '+str(self.cur_episode_id)+' departspeed '+str(s)+' departpos '+str(p)+' ratio '+str(r)+' cur_collision_num '+str(self.co_num)+' meanspeed '+str(np.mean(self.mean_speed))+'\n'
                    with open('./epi_results.txt','a') as f:
                        f.write(res)
                    traci.close()
                    i += 1

def main():
    sumosim = SumoSim(distributional_vtype=True)
    sumosim.main()
    with open('./result_'+str(int(time.time()))+'.pkl','wb') as f:
        pkl.dump(sumosim.co_vehicles, f)

class vehicleDis():
    '''
        "configFile", help="file path of the config file which defines the car-following parameter distributions"
        "-o", "--output-file", dest="outputFile", default="vTypeDistributions.add.xml", help="file path of the output file (if the file already exists, the script tries to insert the distribution node into it)"
        "-n", "--name", dest="vehDistName", default="vehDist", help="alphanumerical ID used for the created vehicle type distribution"
        "-s", "--size", type=int, default=100, dest="vehicleCount", help="number of vTypes in the distribution"
        "-d", "--decimal-places", type=int, default=3, dest="decimalPlaces", help="number of decimal places for numeric attribute values"
        "--resampling", type=int, default=100, dest="nrSamplingAttempts", help="number of attempts to resample a value until it lies in the specified bounds"
        "--seed", type=int, help="random seed", default=42)
    '''
    def __init__(self, path='./', 
                    config_file='config.txt', 
                    out_file='vehDist.add.xml', 
                    vehDistName='vehDist', 
                    vehCount=100, 
                    decimal=3, 
                    sampleTime=100, 
                    seed=42):
        self.configFile = path+config_file
        self.outputFile = path+out_file
        self.vehDistName = vehDistName
        self.vehicleCount = vehCount
        self.decimalPlaces = decimal
        self.nrSamplingAttempts = sampleTime
        self.seed = seed

if __name__ == '__main__':
    # simulation
    print('Simulation...')
    main()

"""
To Do List AND Debug:
2. 没有实现随机初始速度
"""
