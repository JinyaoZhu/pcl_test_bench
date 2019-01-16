from Tester import Tester
from Benchmark import Benchmark



# tester_names = ['SCP','ICP','ICP','ICP','ICP','ICP','ICP','ICP','ICP','ICP']
# tester_result_paths = ['../res/SampleConsensusPrerejective/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv',
#                      '../res/IterativeClosestPoint/resultError.csv']
tester_names = ['SCP','ICP']
tester_result_paths = ['../res/SampleConsensusPrerejective/resultError.csv',
                        '../res/IterativeClosestPoint/resultError.csv']

testers = []
# load all results and plot all results
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