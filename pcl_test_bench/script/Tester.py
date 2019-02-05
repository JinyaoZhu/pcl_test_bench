import numpy as np
import matplotlib.pyplot as plt
import os

class Tester:
    def __init__(self, name):
        self.name = name # algorithm name
        # create folder for each tester if not exist
        if not os.path.exists(name):
           os.makedirs(name)

    def loadCsv(self, path):
        self.result_csv = np.genfromtxt(path, dtype="U20,f,f,f,f,f,f,U5,f,i,i,f,U20", 
            skip_header = 2, delimiter=',',names=[
            'trans_name','error_yaw','error_pitch','error_roll','error_x','error_y','error_z',
            'converge','time_cost','src_points','tgt_points','fitness','file_path'
        ])
        data = self.result_csv
        self.index = np.arange(1,data['trans_name'].size+1, dtype="i")
        self.trans_name  = data['trans_name']
        self.error_euler = np.array([data['error_yaw'],data['error_pitch'],data['error_roll']])
        self.error_trans = np.array([data['error_x'],data['error_y'],data['error_z']])
        self.is_converge = data['converge']
        self.time_cost = data['time_cost']
        self.src_points = data['src_points']
        self.tgt_points = data['tgt_points']
        self.src_points_mean = np.mean(self.src_points)
        self.file_path = data['file_path']
        self.error_euler_mean = np.mean(self.error_euler,axis=1)
        self.error_trans_mean = np.mean(self.error_trans,axis=1)
        self.time_cost_mean = np.mean(self.time_cost)
        self.error_trans_std = np.std(self.error_trans,axis=1)
        self.error_euler_std = np.std(self.error_euler,axis=1)
        self.time_cost_std = np.std(self.time_cost)
        self.fitness = data['fitness']

    def plotRotError(self):
        # fig, ax = plt.subplots()
        fig_name = self.name + " RotationError"
        fig = plt.figure(fig_name,figsize=(4,3))
        bar_width = 0.25
        opacity = 0.8
        rects1 = plt.bar(self.index, np.absolute(self.error_euler[0,:]), bar_width,
                        alpha=opacity,
                        color=[('r'if x=='true'else'grey') for x in self.is_converge],
                        label='yaw')
        rects2 = plt.bar(self.index + bar_width, np.absolute(self.error_euler[1,:]), bar_width,
                        alpha=opacity,
                        color=['g'if x=='true'else'grey' for x in self.is_converge],
                        label='pitch')
        rects3 = plt.bar(self.index + 2*bar_width, np.absolute(self.error_euler[2,:]), bar_width,
                        alpha=opacity,
                        color=['b'if x=='true'else'grey' for x in self.is_converge],
                        label='roll')
        plt.xlabel('n th run')
        plt.ylabel('Error(deg)')
        plt.title('Rotational Error(absolute)')
        plt.xticks(self.index+bar_width, np.array([str(x) if x%(5 if self.index.size<=50 else 10)==0 else "" for x in self.index]))
        plt.legend(loc='best')
        # plt.grid(linestyle=':')
        plt.tight_layout()
        fig.savefig(self.name+'/'+fig_name, dpi=300)
        plt.close()

    def plotTransError(self):
        fig_name = self.name + " TranslationError"
        fig  = plt.figure(fig_name,figsize=(4,3))
        bar_width = 0.25
        opacity = 0.8
        rects1 = plt.bar(self.index, np.absolute(self.error_trans[0,:]), bar_width,
                        alpha=opacity,
                        color=['r'if x=='true'else'grey' for x in self.is_converge],
                        label='x')
        rects2 = plt.bar(self.index + bar_width, np.absolute(self.error_trans[1,:]), bar_width,
                        alpha=opacity,
                        color=['g'if x=='true'else'grey' for x in self.is_converge],
                        label='y')
        rects3 = plt.bar(self.index + 2*bar_width, np.absolute(self.error_trans[2,:]), bar_width,
                        alpha=opacity,
                        color=['b'if x=='true'else'grey' for x in self.is_converge],
                        label='z')
        plt.xlabel('n th run')
        plt.ylabel('Error(m)')
        plt.title('Translational Error(absolute)')
        plt.xticks(self.index+bar_width, np.array([str(x) if x%(5 if self.index.size<=50 else 10)==0 else "" for x in self.index]))
        plt.legend(loc='best')
        # plt.grid(linestyle=':')
        plt.tight_layout()
        fig.savefig(self.name+'/'+fig_name, dpi=300)
        plt.close()

    def plotTimeCost(self):
            fig_name = self.name + " TimeCost"
            fig  = plt.figure(fig_name,figsize=(4,3))
            bar_width = 0.5
            opacity = 0.8
            rects1 = plt.bar(self.index, np.absolute(self.time_cost), bar_width,
                            alpha=opacity,
                            color=['b'if x=='true'else'grey' for x in self.is_converge],
                            label='time cost')
            plt.xlabel('n th run')
            plt.ylabel('Time(s)')
            plt.title('Time Cost (avg.=%.4fs)'%np.mean(self.time_cost))
            plt.xticks(self.index, np.array([str(x) if x%(5 if self.index.size<=50 else 10)==0 else "" for x in self.index]))
            plt.axhline(np.mean(self.time_cost),color='r', linestyle='--')
            # plt.grid(linestyle=':')
            plt.tight_layout()
            fig.savefig(self.name+'/'+fig_name, dpi=300)
            plt.close()

    def plotStatTable(self):
        fig_name = self.name + " StatisticsTable"
        fig  = plt.figure(fig_name,figsize=(6,3))
        ax = plt.gca()
        ax.axis('off')
        ax.axis('tight')
        rows = ['mean','std']
        cols = ['error yaw(deg)','error pitch(deg)','error roll(deg)',
        'error x(m)','error y(m)','error z(m)','time cost(s)']
        cell_text = [np.concatenate((self.error_euler_mean,self.error_trans_mean,[self.time_cost_mean])),
                     np.concatenate((self.error_euler_std,self.error_trans_std,[self.time_cost_std]))]
        cell_text = [['%.4f' % j for j in i] for i in cell_text] # format table
        table = plt.table(cellText=cell_text,
                      rowLabels=rows,colLabels=cols,
                      loc='center',cellLoc='center')
        plt.title(self.name)
        fig.savefig(self.name+'/'+fig_name, dpi=300)
        plt.close()

    def plotRotStat(self):
        fig_name = self.name + " RotationStatistics"
        fig  = plt.figure(fig_name,figsize=(4,3))
        box_plot_data=[self.error_euler[0,:],self.error_euler[1,:],self.error_euler[2,:]]
        plt.boxplot(box_plot_data,patch_artist=True,labels=['yaw','pitch','roll'])
        plt.ylabel('Error(deg)')
        plt.title('Rotational Error Statistics (%i runs)' % self.index.size)
        plt.tight_layout()
        fig.savefig(self.name+'/'+fig_name, dpi=300)
        plt.close()

    def plotTransStat(self):
        fig_name = self.name + " TranslationStatistics"
        fig  = plt.figure(fig_name,figsize=(4,3))
        box_plot_data=[self.error_trans[0,:],self.error_trans[1,:],self.error_trans[2,:]]
        plt.boxplot(box_plot_data,patch_artist=True,labels=['x','y','z'])
        plt.ylabel('Error(m)')
        plt.title('Translational Error Statistics (%i runs)' % self.index.size)
        plt.tight_layout()
        fig.savefig(self.name+'/'+fig_name, dpi=300)
        plt.close()

    def plotTimeCostStat(self):
        fig_name = self.name + " TimeCostStatistics"
        fig  = plt.figure(fig_name,figsize=(4,3))
        plt.boxplot(self.time_cost,patch_artist=True,labels=['time cost'])
        plt.ylabel('Time(s)')
        plt.title('Time Cost Statistics (%i runs)' % self.index.size)
        plt.tight_layout()
        fig.savefig(self.name+'/'+fig_name, dpi=300)
        plt.close()

    def plotPointSize(self):
        fig_name = self.name + " SrcPointSize"
        fig  = plt.figure(fig_name,figsize=(4,3))
        bar_width = 0.5
        opacity = 0.8
        rects1 = plt.bar(self.index + bar_width, self.src_points, bar_width,
                        alpha=opacity,
                        color=['b'if x=='true'else'grey' for x in self.is_converge],
                        label='source')
        plt.xlabel('n th run')
        plt.ylabel('size (point)')
        plt.title('Source Size(Target Size = %i)'%self.tgt_points[0])
        plt.xticks(self.index+bar_width, np.array([str(x) if x%(5 if self.index.size<=50 else 10)==0 else "" for x in self.index]))
        # plt.grid(linestyle=':')
        plt.axhline(np.mean(self.src_points),color='r', linestyle='--')
        plt.tight_layout()
        fig.savefig(self.name+'/'+fig_name, dpi=300)
        plt.close()

    def plotFitness(self):
        fig_name = self.name + " Fitness"
        fig  = plt.figure(fig_name,figsize=(4,3))
        bar_width = 0.5
        opacity = 0.8
        rects1 = plt.bar(self.index + bar_width, self.fitness, bar_width,
                        alpha=opacity,
                        color=['b'if x=='true'else'grey' for x in self.is_converge],
                        label='')
        plt.xlabel('n th run')
        plt.ylabel('Fitness ($\mathregular{m^2}$)')
        plt.title('Fitness Score (avg.=%.4f$\mathregular{m^2}$)'%(np.mean(self.fitness)))
        plt.xticks(self.index+bar_width, np.array([str(x) if x%(5 if self.index.size<=50 else 10)==0 else "" for x in self.index]))
        # plt.grid(linestyle=':')
        plt.axhline(np.mean(self.fitness),color='r', linestyle='--')
        plt.tight_layout()
        fig.savefig(self.name+'/'+fig_name, dpi=300)
        plt.close()

    def plotAll(self):
        self.plotRotError()
        self.plotTransError()
        self.plotTimeCost()
        self.plotStatTable()
        self.plotRotStat()
        self.plotTransStat()
        self.plotTimeCostStat()
        self.plotPointSize()
        self.plotFitness()