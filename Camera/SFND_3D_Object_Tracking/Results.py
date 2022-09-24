import math
import random
import sys, os
import numpy as np
from textwrap import wrap
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd

results = os.path.join(os.getcwd(), "Results/comparison_results.csv")

detectors = ["SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"]
descriptors = ["BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"]
matcherTypes = ["MAT_FLANN"]
descriptorTypes = ["DES_BINARY", "DES_HOG"]
selectorTypes = ["SEL_KNN"]

def number_of_false_data(df):
    number_of_nans = 0
    ttc_camera = []
    ttc_lidar = []
    # replace nan with -1
    df.fillna(-1, inplace=True)
    # abnormal values in lidar replaced with -1
    df.loc[df['TTC Lidar'] < 1, 'TTC Lidar'] = -1
    df.loc[df['TTC Lidar'] > 30, 'TTC Lidar'] = -1

    # abnormal values in lidar replaced with -1
    df.loc[df['TTC Camera'] > 100, 'TTC Camera'] = -1


    # for detector in detectors:
    #     for descriptor in descriptors:
    #         for matcherType in matcherTypes:
    #             for descriptorType in descriptorTypes:
    #                 for selectorType in selectorTypes:
    #                     for index, rows in df.iterrows():
    #                         if rows['TTC Camera'] == -1:
    #                             ttc_camera.append([detector, descriptor, matcherType, descriptorType, selectorType, rows['Image pairs']])

    #                         if rows['TTC Lidar'] == -1:
    #                             ttc_lidar.append([detector, descriptor, matcherType, descriptorType, selectorType, rows['Image pairs']])
    # return ttc_camera, ttc_lidar

def plot_sample_camera_ttc(df):
    images = df['Image pairs'].unique().tolist()
    counter = 0
    for detector in detectors:
        fig = plt.figure(counter)
        counter += 1
        df_list = []
        for descriptor in descriptors:
            for matcherType in matcherTypes:
                for descriptorType in descriptorTypes:
                    for selectorType in selectorTypes:
                        df_filtered = df.loc[(df['Detector Type'] == detector)
                                        & (df['Descriptor Type'] == descriptor)
                                        & (df['Matcher Type'] == matcherType)
                                        & (df['Descriptor Content Type'] == descriptorType)
                                        & (df['Selector Type'] == selectorType)]
                        if not df_filtered.empty:
                            df_list.append(df_filtered)
        for df_i in df_list:
            ttc_camera = df_i['TTC Camera'].values.tolist()
            descriptor_i = df_i['Descriptor Type'].unique()
            plt.plot(ttc_camera, label=detector + '+' + descriptor_i[0])
        title = 'TTC Camera for ' + detector + ' and having matcher and selector type fixed to FLANN and KNN respectively'
        plt.title('\n'.join(wrap(title, 60)))
        plt.xlabel('Image pairs')
        plt.ylabel('TTC, s')
        plt.xticks(range(len(images)), images, size='small')
        plt.legend()
        fig.autofmt_xdate(rotation=45)
        plt.savefig('Results_report_plots/' + detector + '.png', bbox_inches='tight')
        plt.show()


def plot_sample_ttc(df):

    # copy_matcherTypes = matcherTypes.copy()
    # copy_descriptorTypes = descriptorTypes.copy()
    # copy_selectorTypes = selectorTypes.copy()
    random_detector = 'SHITOMASI'
    random_descriptor = 'BRISK'
    matcherType = "MAT_FLANN"
    descriptorType = "DES_BINARY"
    selectorType = "SEL_KNN"
    # remove from list so not to be selected again
    
    filtered_data = []
    distance = []
    images = []
    # 0 - detector
    # 1 - descriptor
    # 2 - matcher type
    # 3 - descriptor type
    # 4 - selector type
    # dfobj.loc[(dobj['Name'] == 'Rack') & (dobj['Marks'] == 100)]
    # Header => Detector Type, Descriptor Type,Matcher Type,Descriptor Content Type,Selector Type
    df_filtered = df.loc[(df['Detector Type'] == random_detector)
                            & (df['Descriptor Type'] == random_descriptor)
                            & (df['Matcher Type'] == matcherType)
                            & (df['Descriptor Content Type'] == descriptorType)
                            & (df['Selector Type'] == selectorType)]
    filtered_data = df_filtered['TTC Lidar'].values.tolist()
    images = df_filtered['Image pairs'].values.tolist()
    print(images)
    distance = df_filtered['Lidar distance to car'].values.tolist()


    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    p1 = ax1.plot(filtered_data, label='TTC', color='r')
    p2 = ax2.scatter(range(len(distance)), distance, label='Distance to car', color='g', marker='x')
    ax1.set_xlabel('Image pair indices')
    ax1.set_xticks(np.arange(len(images)))
    ax1.set_xticklabels(images, rotation=45)
    ax1.set_ylabel('TTC, s')
    ax2.set_ylabel('Distance to car, m')
    title = str('TTC using Lidar')
    ax1.set_title('\n'.join(wrap(title,60)))
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1+h2, l1+l2, loc=0)


    plt.show()


def read_csv():
    df = pd.read_csv(results, index_col=False)
    
    number_of_false_data(df)
    plot_sample_camera_ttc(df)
    # plot_sample_ttc(df)


def main():
    read_csv()

if __name__ == "__main__":
    main()

