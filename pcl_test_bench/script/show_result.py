from Tester import Tester


scp = Tester("SCP")
scp.loadCsv('../res/SampleConsensusPrerejective/resultError.csv')
scp.plotAll()

icp = Tester("ICP")
icp.loadCsv('../res/IterativeClosestPoint/resultError.csv')
icp.plotAll()