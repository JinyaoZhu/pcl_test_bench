from Tester import Tester
from Benchmark import Benchmark

tester_names = ['SCP','ICP']
tester_result_paths = ['../res/SCP/resultError.csv',
                       '../res/ICP/resultError.csv'
                      ]

# # given names of the testers
# tester_names = ['SCP','ICP','ICPNL','GICP','NDT']
# # csv files path of each tester
# tester_result_paths = ['../res/SCP/resultError.csv',
#                         '../res/ICP/resultError.csv',
#                         '../res/ICPNL/resultError.csv',
#                         '../res/GICP/resultError.csv',
#                         '../res/NDT/resultError.csv']

# # given names of the testers
# tester_names = ['1000','3000','5000','7000','10000']
# # csv files path of each tester
# tester_result_paths = ['../res/SCP_Iter1000/resultError.csv',
#                         '../res/SCP_Iter3000/resultError.csv',
#                         '../res/SCP_Iter5000/resultError.csv',
#                         '../res/SCP_Iter7000/resultError.csv',
#                         '../res/SCP_Iter10000/resultError.csv']

# # given names of the testers
# tester_names = ['50%','60%','70%','80%','90%']
# # csv files path of each tester
# tester_result_paths = [ '../res/SCP_Sim05/resultError.csv',
#                         '../res/SCP_Sim06/resultError.csv',
#                         '../res/SCP_Sim07/resultError.csv',
#                         '../res/SCP_Sim08/resultError.csv',
#                         '../res/SCP_Sim09/resultError.csv']

# #given names of the testers
# tester_names = ['1','2','3','4','5']
# # csv files path of each tester
# tester_result_paths = [ '../res/SCP_Rand1/resultError.csv',
#                         '../res/SCP_Rand2/resultError.csv',
#                         '../res/SCP_Rand3/resultError.csv',
#                         '../res/SCP_Rand4/resultError.csv',
#                         '../res/SCP_Rand5/resultError.csv']


testers = []
# load all results and plot all results
print('Loading...')
for name,path in zip(tester_names,tester_result_paths):
    testers.append(Tester(name))
    testers[-1].loadCsv(path)
    testers[-1].plotAll()
    print('Load:'+ name + ' from:' +path)
# run benchmark
benchmark = Benchmark(testers)
benchmark.plotRotBM()
benchmark.plotTransBM()
benchmark.plotTimeCostBM()
