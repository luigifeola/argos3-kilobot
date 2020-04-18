import matplotlib.pyplot as plt
import numpy as np
import csv
import sys
import os
from scipy import stats
import math
from scipy.optimize import curve_fit
from scipy.stats import norm
import seaborn as sns
import pandas as pd

# default value
my_dict = dict()
def print_help():
    print("usage : folder_path, sim or real ,window size (for windowed_MSD)")


def main():
    number_of_args=len(sys.argv)

    if (number_of_args < 4):
        print_help()
        exit(-1)

    folder=sys.argv[1]
    sim_or_real=sys.argv[2]
    window_size=int(sys.argv[3])

    if(sim_or_real != "sim" and sim_or_real != "real"):
        print("ERROR: you must specify if this is sim or real as third argument")
        exit(-1)

    if(my_dict):
        total_dict=my_dict
    else:
        total_dict=dict()
        number_dict=dict()
        count=1
        # Set the directory you want to start from
        for dirName, subdirList, fileList in os.walk(folder):
            num_robots = "0"
            rho = -1.0
            alpha = -1.0
            elements=dirName.split("_")
            for e in elements:
                if e.startswith("robots"):
                    num_robots=e.split("#")[-1]
                    if(num_robots not in total_dict):
                        total_dict[num_robots]=dict()
                        number_dict[num_robots]=dict()

                if(e.startswith("rho")):
                    rho=float(e.split("#")[-1])
                if(e.startswith("alpha")):
                    alpha=float(e.split("#")[-1])
            
            #print(str(count) + " : " + dirName)
            if(num_robots == "0" or rho == -1.0 or alpha == -1):
                continue

            
            rho_str=str(rho)
            alpha_str=str(alpha)
            # print("rho", rho_str)
            # print("alpha", alpha_str)
            if(rho_str not in total_dict[num_robots]):
                total_dict[num_robots][rho_str]=dict()
                number_dict[num_robots][rho_str]=dict()
        #         print(total_dict)
            mean_wmsd=0.0
            number_of_experiments = 0
            
            for file in fileList:
                if file.endswith('position.tsv'):
        #             print(mean_wmsd)
        #             print('\t\tfile %s' % file)
                    (mean_wmsd, number_of_experiments)=window_displacement(
                        os.path.join(dirName, file), mean_wmsd, number_of_experiments, window_size, int(num_robots))
            print(mean_wmsd)
        #     print(number_of_experiments)
            #Aggiungi i risultati al dizionario, è un po una cagata, correggi
            if(alpha_str in number_dict[num_robots][rho_str]):
                previous_number=number_dict[num_robots][rho_str][alpha_str]
                total_dict[num_robots][rho_str][alpha_str] *= previous_number
                total_dict[num_robots][rho_str][alpha_str] += mean_wmsd *             number_of_experiments
                total_dict[num_robots][rho_str][alpha_str] /= previous_number +             number_of_experiments
                number_dict[num_robots][rho_str][alpha_str] += number_of_experiments
                
            else:
                total_dict[num_robots][rho_str][alpha_str]=mean_wmsd
                number_dict[num_robots][rho_str][alpha_str]=number_of_experiments
            
            count += 1

        print(total_dict)

        for key, value in total_dict.items():
            fig=plt.figure(figsize = (12, 8))
            dataFrame=pd.DataFrame.from_dict(value)
            reversed_df=dataFrame.iloc[::-1]
            ax=sns.heatmap(reversed_df, annot = True, fmt = ".2e")
            ax.set_title("Heatmap of WMSD for %s robots" % (key))
            ax.set_ylabel("alpha")
            ax.set_xlabel("rho")
            plt.show()
            #Salva su file
        #     file_name="%s/WMSD_%s_robots_heatmap.png" % (folder, key)
        #     plt.savefig(file_name)
        #     reversed_df.to_pickle(file_name[:-4] + ".pickle")

def window_displacement(position_filename, mean_wmsd, number_of_experiments, window_size, n_robots):
    #print("mean_wmsd", mean_wmsd)
    displacement_file=open(position_filename, mode = 'r')
    tsvin=csv.reader(displacement_file, delimiter = '\t')
    
    average_w_displacement=[[]]
    expe_length=0
    for row_index,row in enumerate(tsvin):
        if(row[0] == "Robot id"):
            expe_length=len(row) - 1 - window_size
            print("len(row)", len(row))
            print("len(row)-window_size", len(row)-window_size)
            print("expe_length", expe_length)
            average_w_displacement=np.zeros((n_robots, expe_length))
        else:
            print("Range: ",1 + window_size, len(row))
            for i in range(1 + window_size, len(row)):
                [xi, yi]=row[i-window_size].split(",")
                [xi, yi]=[float(xi), float(yi)]

                [xf, yf]=row[i].split(",")
                [xf, yf]=[float(xf), float(yf)]

                w_displacement=((xf - xi)/window_size)**2 + ((yf - yi)/window_size)**2
                

                average_w_displacement[row_index-1][i-1 - window_size] += w_displacement
    mean=0.0
    sum_avg_displ = np.sum(average_w_displacement, axis=0) #dalla matrice sommo tutte le righe
    sum_avg_displ = np.true_divide(sum_avg_displ, n_robots)#divisione per lo scalare numero di robot
    print("average_w_displacement", average_w_displacement)
    print("sum_avg_displ size",sum_avg_displ.size)
    print("sum_avg_displ",sum_avg_displ)
    mean =sum_avg_displ.mean()

    if(number_of_experiments == 0):
        mean_wmsd=mean
        number_of_experiments += 1
    else:
        mean_wmsd *= number_of_experiments
        mean_wmsd += mean
        number_of_experiments += 1
        mean_wmsd /= float(number_of_experiments)
    #print("mean_wmsd", mean_wmsd)
    return (mean_wmsd, number_of_experiments)


if __name__ == '__main__':
    main()
