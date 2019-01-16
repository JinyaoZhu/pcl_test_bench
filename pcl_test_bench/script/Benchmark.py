from Tester import Tester
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

class Benchmark:
    def __init__(self, testers):
        self.testers = testers
        self.num_tester = len(testers)

    def plotRotBM(self):
        fig_name = "BenchmarkRot"
        fig  = plt.figure(fig_name,figsize=(4+(4.0/3)*(self.num_tester/3),3+(self.num_tester/3)))
        colors = matplotlib.cm.rainbow(np.linspace(0, 1, self.num_tester))
        widths = 1
        positions = np.array([1,1+2+self.num_tester,1+2*(2+self.num_tester)])
        legend_elements = []
        for tester,i in zip(self.testers,range(len(self.testers))):
            box_plot_data=[tester.error_euler[0,:],tester.error_euler[1,:],tester.error_euler[2,:]]
            boxplot = plt.boxplot(box_plot_data,patch_artist=True,positions=positions + i*widths, widths=0.6)
            for patch in boxplot['boxes']:
                patch.set_facecolor(colors[i])
            legend_elements.append(Patch(facecolor=colors[i], edgecolor='k',label=tester.name))
        plt.xlim(0,positions[-1] + self.num_tester*widths)
        plt.xticks(positions+(self.num_tester-1)*widths/2,['yaw','pitch','roll'])
        plt.ylabel('Error(deg)')
        plt.title('Rotational Error')
        plt.legend(handles=legend_elements, loc='best')
        plt.tight_layout()
        fig.savefig(fig_name, dpi=300)

    def plotTransBM(self):
        fig_name = "BenchmarkTrans"
        fig  = plt.figure(fig_name,figsize=(4+(4.0/3)*(self.num_tester/3),3+(self.num_tester/3)))
        colors = matplotlib.cm.rainbow(np.linspace(0, 1, self.num_tester))
        widths = 1
        positions = np.array([1,1+2+self.num_tester,1+2*(2+self.num_tester)])
        legend_elements = []
        for tester,i in zip(self.testers,range(len(self.testers))):
            box_plot_data=[tester.error_trans[0,:],tester.error_trans[1,:],tester.error_trans[2,:]]
            boxplot = plt.boxplot(box_plot_data,patch_artist=True,positions=positions + i*widths, widths=0.6)
            for patch in boxplot['boxes']:
                patch.set_facecolor(colors[i])
            legend_elements.append(Patch(facecolor=colors[i], edgecolor='k',label=tester.name))
        plt.xlim(0,positions[-1] + self.num_tester*widths)
        plt.xticks(positions+(self.num_tester-1)*widths/2,['x','y','z'])
        plt.ylabel('Error(m)')
        plt.title('Translational Error')
        plt.legend(handles=legend_elements, loc='best')
        plt.tight_layout()
        fig.savefig(fig_name, dpi=300)

    def plotTimeCostBM(self):
        fig_name = "BenchmarkTimeCost"
        fig  = plt.figure(fig_name,figsize=(4+(4.0/3)*(self.num_tester/3),3+(self.num_tester/3)))
        colors = matplotlib.cm.rainbow(np.linspace(0, 1, self.num_tester))
        widths = 1
        positions = np.array([1])
        legend_elements = []
        for tester,i in zip(self.testers,range(len(self.testers))):
            box_plot_data= tester.time_cost
            boxplot = plt.boxplot(box_plot_data,patch_artist=True,positions=positions + i*widths, widths=0.6)
            for patch in boxplot['boxes']:
                patch.set_facecolor(colors[i])
            legend_elements.append(Patch(facecolor=colors[i], edgecolor='k',label=tester.name))
        plt.xlim(0,positions[-1] + self.num_tester*widths)
        plt.xticks(positions+(self.num_tester-1)*widths/2,[''])
        plt.ylabel('time(s)')
        plt.title('Time Cost')
        plt.legend(handles=legend_elements, loc='best')
        plt.tight_layout()
        fig.savefig(fig_name, dpi=300)

