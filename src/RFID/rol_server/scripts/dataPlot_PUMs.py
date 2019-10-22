import pickle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import csv

from matplotlib.backends.backend_pdf import PdfPages



import yaml


# reads mmap.yaml file and returns regions centroid
def getMeCentroids(file_path, file_name):
    #file_path = '/home/manolofc/catkin_ws/src/RFID/rfid_grid_map/test/pums_test2/'
    #file_name = 'mmap.yaml'
    file_uri = file_path + file_name

    with open(file_uri, 'r') as stream:
        try:
            allData = yaml.load(stream)
            #print(allData)
        except yaml.YAMLError as exc:
            print(exc)

    pointsDict = allData['zoi']['submap_0']

    regionCentroids = dict()

    for pointName,pointEntry in pointsDict.items():
        regionName = pointEntry[1]
        #print regionName
        x = pointEntry[2]
        y = pointEntry[3]
        z = pointEntry[4]
        n = 1
        if regionName in regionCentroids:
            (cx,cy,cz,cn) =regionCentroids[regionName]
            n = cn +1
            x = ( (cx * cn ) + x ) / n
            y = ( (cy * cn ) + y ) / n
            z = ( (cz * cn ) + z ) / n
                
        regionCentroids[regionName] = (x,y,z,n)

    regionCentroids2 = dict()
    for regionName,centroidEntry in regionCentroids.items():
        regionCentroids2[regionName]=(centroidEntry[0],centroidEntry[1])

    return regionCentroids2

def getDataFrame(fullPath, fileName, endTime):

    if fullPath[-1] != '/':
        fullPath = fullPath + '/'

    objectData = pickle.load(open(fullPath + fileName, 'rw'))

    numPoints = len(objectData)
    locations = sorted(objectData[0].keys())
    locations.pop(locations.index('time'))
    # locations=['kitchen', 'lounge', 'bathroom corridor', 
    #'entrance', 'entrance corridor',
    #'kitchen - coffee', 'kitchen - fridge', 'lounge - sofas',
    #'lounge - tv', 'time', 'workzone G', 'workzone P', 
    #'workzone T','workzone Y', 'workzone corridor']

    df2 = pd.DataFrame(objectData)
    df2['seconds'] = df2['time'] - df2['time'][0]
    df2['time'] = pd.to_datetime(df2['time'], unit='s')
    df2.set_index('time', drop=False, append=False, inplace=True)
    df2.set_index('seconds', drop=False, append=False, inplace=True)
    df2.set_index('detects', drop=False, append=False, inplace=True)

    df2 = df2[df2['seconds'] < endTime]
    return df2


def purgueColumns(dataframe, unwanted):
    ans = dataframe
    for i in unwanted:
        try:
            del ans[i]
        except:
            pass
    return ans


def confidence(dataframe, trueLoc, keyList, centr):
    maxNames = []
    index = 0
    maxVals = np.zeros(len(dataframe.index))
    dists = np.zeros(len(dataframe.index))

    trueRegionPos = np.array(centr[trueLoc])

    for i, row in dataframe.iterrows():
        maxName = trueLoc
        try:
            trueRegionConfidence = row[trueLoc]
        except KeyError:
            print 'Wrong true region...'
            return (0,0,0)

        weightPos = trueRegionPos * trueRegionConfidence

        for key in keyList:
            regionConfidence = row[key]
            regionPos = np.array(centr[key])
            weightPos = weightPos + regionPos * regionConfidence

            diff = regionConfidence - trueRegionConfidence
            if diff > maxVals[index]:
                maxVals[index] = diff
                maxName = key
        maxNames.append(maxName)
        dists[index] = np.linalg.norm(trueRegionPos-weightPos)
        index = index+1

    return (maxNames, maxVals, dists)


def changeYaxisToPercent(ax):
    # change labels to show percents
    a = ax.get_yticks().tolist()
    for i in range(0, len(a)):
        a[i] = (str(100.0*a[i]))
    ax.set_yticklabels(a)
    #.........................


def allPlots(fullPath, endTime, minConf, deleteItems, fileName, trueLoc, centr,showFig=False,saveMe=True,offset=11):
    print '........................................................'
    print 'processing: ['+fullPath+fileName+']'

    df = getDataFrame(fullPath, fileName, endTime)

    if df['detects'].max()==0:
        print 'Never detected...'
        return

    if deleteItems is not []:
        purgueColumns(df, deleteItems)


    #print 'Purging regions already subdivided'    
    #del df['bedroom'] 


    useTime=False
    # delete unprobable regions under 10 %
    meanConf = df.mean()
    # meanConf.sort_values(inplace=True)
    dropped = meanConf[meanConf < minConf].keys().tolist()
    
   

    locList = meanConf[meanConf > minConf].keys().tolist()
    locList.remove('seconds')
    locList.remove('detects')
    # print locList

    purgueColumns(df, dropped)

    locs, dife, dist = confidence(df, trueLoc, locList, centr)
    if (locs,dife,dist)==(0,0,0):
        print 'Error'
        return []

    # print locs

    detect = df['detects']
    del df['detects']
    seconds = df['seconds']
    del df['seconds']

    locs = pd.DataFrame(locs)
    goodLocs = locs[locs == trueLoc].count().tolist()[0]
    totalLocs = locs.count().tolist()[0]
    objectName=fileName[offset:-2]
    locConfStr='{0:.1f}'.format(100*(meanConf[trueLoc]))
    avDistStr='{0:.2f}'.format(np.average(dist))
    locAccStr='{0:.1f}'.format(100.0*goodLocs/totalLocs)
    numDetectsStr=str(detect.max())

    print 'Object:', objectName
    print 'Location:', trueLoc
    print 'best region is',  (100*(meanConf[locList].sort_values(ascending=False)))[0:1]
    print 'Region Confidences (%):', locConfStr
    print 'Av. Error (m.):', avDistStr
    print 'Region accuracy (% times):', locAccStr
    print 'Tag detections (#)', numDetectsStr

    #. . . . . . . . . . . . . . . . . . . . . . . . . .
    if useTime:
        ax = df.plot(x=seconds)
        changeYaxisToPercent(ax)
        ax.set_xlabel('Elapsed seconds')
        ax.set_ylabel('Region Confidence (%)')
        ax.legend(title="Regions", loc='best')
        if showFig:
            plt.show()

        fileName=objectName+"_regions_time"
        if saveMe:
            pdfURL = fullPath + '/' + fileName + '.pdf'
            with PdfPages(pdfURL) as pdf:
                fig = ax.get_figure()
                pdf.savefig(fig)
        plt.close(fig)


    #. . . . . . . . . . . . . . . . . . . . . . . . . .
    # que mierda queria hacer yo...
    if False:
        ax = df.plot(x=detect)
        changeYaxisToPercent(ax)
        ax.set_xlabel('Num. of Tag detections')
        ax.set_ylabel('Region Confidence (%)')
        plt.legend(title="Regions", loc='best')
        if showFig:
            plt.show()

        fileName=objectName+"_regions_detect"
        if saveMe:
            pdfURL = fullPath + '/' + fileName + '.pdf'
            with PdfPages(pdfURL) as pdf:
                fig = ax.get_figure()
                pdf.savefig(fig)
            plt.close(fig)

        #. . . . . . . . . . . . . . . . . . . . . . . . . .
        ax = df.plot(kind='box')
        changeYaxisToPercent(ax)
        ax.set_xlabel('Region')
        ax.set_ylabel('Region Confidence (%)')
        if showFig:
            plt.show()

        fileName=objectName+"_hist"
        if saveMe:
            pdfURL = fullPath + '/' + fileName + '.pdf'
            with PdfPages(pdfURL) as pdf:
                fig = ax.get_figure()
                pdf.savefig(fig)
            plt.close(fig)

        #. . . . . . . . . . . . . . . . . . . . . . . . . .
        df['dife'] = dife
        if useTime:
            ax = df.plot(y='dife', x=seconds, legend=None)
            changeYaxisToPercent(ax)
            ax.set_xlabel('Elapsed seconds')
            # plt.legend(title="Regions",loc='best')
            ax.set_ylabel('Confidence error (%)')
            if showFig:
                plt.show()
            fileName=objectName+"_conf_error_t"
            if saveMe:
                pdfURL = fullPath + '/' + fileName + '.pdf'
                with PdfPages(pdfURL) as pdf:
                    fig = ax.get_figure()
                    pdf.savefig(fig)
                plt.close(fig)

        # . . . . . . . . . . . . . . . . . . . . . . . . . .
        ax = df.plot(y='dife', x=detect, legend=None)
        changeYaxisToPercent(ax)
        ax.set_xlabel('Num. of Tag detections')
        # plt.legend(title="Regions",loc='best')
        ax.set_ylabel('Confidence error (%)')
        if showFig:
            plt.show()
        fileName = objectName + "_conf_error_t"
        if saveMe:
            pdfURL = fullPath + '/' + fileName + '.pdf'
            with PdfPages(pdfURL) as pdf:
                fig = ax.get_figure()
                pdf.savefig(fig)
            plt.close(fig)

        #. . . . . . . . . . . . . . . . . . . . . . . . . .
        df['dife'] = dist
        if useTime:
            ax = df.plot(y='dife', x=seconds, legend=None)
            ax.set_xlabel('Elapsed seconds')
            ax.set_ylabel('Distance Error to region centroid (m.)')
            if showFig:
                plt.show()

            fileName=objectName+"_dist_error_t"
            if saveMe:
                pdfURL = fullPath + '/' + fileName + '.pdf'
                with PdfPages(pdfURL) as pdf:
                    fig = ax.get_figure()
                    pdf.savefig(fig)
                plt.close(fig)

        #. . . . . . . . . . . . . . . . . . . . . . . . . .
        ax = df.plot(y='dife', x=detect, legend=None)
        ax.set_xlabel('Num. of Tag detections')
        ax.set_ylabel('Distance Error to region centroid (m.)')
        if showFig:
            plt.show()

        fileName=objectName+"_dist_error_detect"
        if saveMe:
            pdfURL = fullPath + '/' + fileName + '.pdf'
            with PdfPages(pdfURL) as pdf:
                fig = ax.get_figure()
                pdf.savefig(fig)
            plt.close(fig)


    endLineStr = '\\\\'
    ansRow=[objectName, trueLoc,  locConfStr, avDistStr, locAccStr, numDetectsStr, endLineStr]
    return ansRow




# example for pums dataset
def runExpPUMS():

    # where pickle files are
    fullPath = '/home/manolofc/catkin_ws/src/RFID/rfid_grid_map/test/pums_test2/tagMaps/'

    endTime = 324 # seconds
    minConf = 0.1

    # get region centroids
    file_path = '/home/manolofc/catkin_ws/src/RFID/rfid_grid_map/test/pums_test2/'
    file_name = 'mmap.yaml'
    centr = getMeCentroids(file_path, file_name)
    
    #print 'Purging regions already subdivided'    
    #del centr['bedroom']

    #what we save for the csv file
    #Experiment, R, Wi, Wd, objectName, trueLoc,  locConfStr, avDistStr, locAccStr, numDetectsStr, endLineStr

    experiment='PUMS'
    R='3'
    Wi='0.05'
    Wd='0.001'
    drawMe=True

    resultsFileName = 'results.csv'
    resultsFileURI = fullPath + resultsFileName
    csvfile = open(resultsFileURI, 'w+')
    spamwriter = csv.writer(csvfile, delimiter='&')

    #header
    spamwriter.writerow(['Experiment', 'R', 'Wi', 'Wd', 'object', 'Region',  'Av. Reg. Confidence', 'Av. dist. error', 'Accuracy', 'Tag detections', ' '])


    csvRow=[experiment,R,Wi,Wd]


    trueLocsDict = {'glasses': 'bedroom_sofa',
                    'keys': 'livingroom',
                    'pillbox': 'bedroom_dressing',
                    'remote': 'livingroom',
                    'wallet': 'bedroom_tv'}


    deleteItems = []
    offset = len(experiment)+1
    for  objectName,trueLoc in trueLocsDict.items():
        fileName = experiment+'_'+objectName+'.p'

        ansRow = allPlots(fullPath, endTime, minConf, deleteItems, fileName, trueLoc, centr,drawMe,offset)
        if ansRow is not None:
            spamwriter.writerow(csvRow+ansRow)

    


#....................................................................................................................
# goes through all files ...
def runAll():
    basePath = '~/catkin_ws/src/ENRICHME/codes/ais/LibMercuryRFID/src/clients/ros/rfid_grid_map/launch/article/'

    # original option, sweep all
    # r_min = 2.0
    # r_step = 1.0
    # r_max = 5.0
    # r_num = 4
    #
    # wi_min = 0.005
    # wi_step = 0.01
    # wi_max = 0.045
    # wi_num = 5
    #
    # wd_min = 0.001
    # wd_step = 0.002
    # wd_max = 0.009
    # wd_num = 5


# to do only r2.0-i0.15-d0.007
    r_min = 2.0
    r_step = 1.0
    r_max = 2.0
    r_num = 1

    wi_min = 0.015
    wi_step = 0.01
    wi_max = 0.15
    wi_num = 1

    wd_min = 0.007
    wd_step = 0.001
    wd_max = 0.007
    wd_num = 1

    r_vec = np.linspace(r_min, r_max, r_num)
    wi_vec = np.linspace(wi_min, wi_max, wi_num)
    wd_vec = np.linspace(wd_min, wd_max, wd_num)

    # original option, sweep all
    # experiment_vec = ['FDG_1', 'UOL_2', 'UOL_3']
    # test_vec = [0, 1, 2]
    # endTime_vec = [300, 500, 500]
    # minConf_vec =[0.1, 0.1, 0.1]

    experiment_vec = ['FDG_1']
    test_vec = [0]
    endTime_vec = [300]
    minConf_vec =[0.1]


    centr1 = {'kitchen': (3.015, 0.565),
              'kitchen - fridge': (3.015, -0.275),
              'kitchen - coffee': (3.015, 1.300),
              'entrance corridor': (0.910, 3.605),
              'bathroom corridor': (0.655, 0.435),
              'workzone corridor': (0.410, 9.435),
              'workzone G': (-2.620, 6.860),
              'workzone P': (2.935, 6.960),
              'workzone Y': (-2.555, 11.270),
              'workzone T': (3.120, 11.345),
              'entrance': (-3.915, 3.605),
              'lounge': (-2.700, 0.470),
              'lounge - sofas': (-3.825, 0.470),
              'lounge - tv': (-1.395, 0.470)}

    centr0 = {'kitchen': (9.956, 7.461),
              'livingroom': (10.133, 4.004),
              'livingroom - sofa': (9.132, 3.632),
              'livingroom - dining table': (7.424, 5.437),
              'livingroom - window': (10.682, 2.165),
              'livingroom - study': (12.901, 4.183)}

    centr2 = centr1

    # original option, sweep all
    # centr_vec = [centr0, centr1, centr2]
    centr_vec = [centr0]

    # deleteItems for FDG
    deleteItems0 = ['livingroom']

    # delete subregions - keep only regions
    deleteItems1 = ['kitchen - coffee', 'kitchen - fridge',
                    'lounge - sofas', 'lounge - tv']
    # , 'entrance', 'workzone G', 'workzone P', 'workzone T', 'workzone Y', 'workzone corridor']

    deleteItems2 = deleteItems1

    deleteItems_vec = [deleteItems0, deleteItems1, deleteItems2]

    objects0 = ['REPLAY_FDG_1_glasses.p', 'REPLAY_FDG_1_keys.p', 'REPLAY_FDG_1_pillbox.p', 'REPLAY_FDG_1_remote.p',
                'REPLAY_FDG_1_wallet.p']
    trueLocs0 = ['livingroom - dining table', 'livingroom - dining table', 'livingroom - dining table',
                 'livingroom - sofa', 'livingroom - sofa']

    objects1 = ['REPLAY_UOL_kitchen table.p', 'REPLAY_UOL_tape holder.p', 'REPLAY_UOL_lounge table.p']
    trueLocs1 = ['entrance corridor', 'entrance corridor', 'entrance corridor']
    objects2 = objects1
    trueLocs2 = trueLocs1

    # original option, sweep all
    # object_vec = [objects0, objects1, objects2]
    # trueLoc_vec = [trueLocs0, trueLocs1, trueLocs2]
    object_vec = [objects0]
    trueLoc_vec = [trueLocs0]

    # original option, sweep all
    # offsets_vec = [13, 11, 11]
    offsets_vec = [13]

    # original option, sweep all
#    resultsFileName = 'results.csv'
    resultsFileName = 'resultsTEMP.csv'

    resultsFileURI = basePath + resultsFileName
    csvfile = open(resultsFileURI, 'wb')
    spamwriter = csv.writer(csvfile, delimiter='&')
    spamwriter.writerow(['Experiment', 'R', 'Wi', 'Wd', 'object', 'Region',  'Av. Reg. Confidence', 'Av. dist. error', 'Accuracy', 'Tag detections', ' '])
    endLineStr = '\\\\'

    for r in r_vec:
        for i in wi_vec:
            for d in wd_vec:
                for t in test_vec:
                    R = '{0:.1f}'.format(r)
                    Wi = '{0:.3f}'.format(i)
                    Wd = '{0:.3f}'.format(d)

                    # '1/r2.0-i0.005-d0.001'
                    folder = str(t + 1) + '/' \
                                          'r' + R + '-' + \
                             'i' + Wi + '-' + \
                             'd' + Wd
                    fullFolder = basePath + folder

                    endTime = endTime_vec[t]
                    minConf = minConf_vec[t]
                    centr = centr_vec[t]
                    deleteItems = deleteItems_vec[t]

                    # what we save for the csv file
                    # Experiment, R, Wi, Wd, objectName, trueLoc,  locConfStr, avDistStr, locAccStr, numDetectsStr, endLineStr
                    experiment = experiment_vec[t]
                    csvRow = [experiment, R, Wi, Wd]


                    objects = object_vec[t]
                    trueLocs = trueLoc_vec[t]
                    num_objects = len(objects)
                    offset = offsets_vec[t]

                    for it in range(0, num_objects):
                        fileName = objects[it]  # 'REPLAY_UOL_kitchen table.p'
                        trueLoc = trueLocs[it]  # 'entrance corridor'
                        ansRow = allPlots(fullFolder, endTime, minConf, deleteItems, fileName, trueLoc, centr,False,True,offset)
                        if ansRow!=[]:
                            spamwriter.writerow(csvRow + ansRow)



# runs one single experiment
def runExp(fullPath,prefix):


    endTime = 900
    minConf = 0.1

    centr = {'kitchen': (3.015, 0.565),
             'kitchen - fridge': (3.015, -0.275),
             'kitchen - coffee': (3.015, 1.300),
             'entrance corridor': (0.910, 3.605),
             'bathroom corridor': (0.655, 0.435),
             'workzone corridor': (0.410, 9.435),
             'workzone G': (-2.620, 6.860),
             'workzone P': (2.935, 6.960),
             'workzone Y': (-2.555, 11.270),
             'workzone T': (3.120, 11.345),
             'entrance': (-3.915, 3.605),
             'lounge': (-2.700, 0.470),
             'lounge - sofas': (-3.825, 0.470),
             'lounge - tv': (-1.395, 0.470)}


    # delete subregions - keep only regions
    deleteItems = ['kitchen - coffee', 'kitchen - fridge',
                   'lounge - sofas', 'lounge - tv']
    #, 'entrance', 'workzone G', 'workzone P', 'workzone T', 'workzone Y', 'workzone corridor']


    #what we save for the csv file
    #Experiment, R, Wi, Wd, objectName, trueLoc,  locConfStr, avDistStr, locAccStr, numDetectsStr, endLineStr

    experiment='UOL_2'
    drawMe=True
    saveMe=True
    offset = 11

    # 1 ......................................................................
    fileName = prefix+'kitchen table.p'
    trueLoc = 'bathroom corridor'
    ansRow = allPlots(fullPath, endTime, minConf, deleteItems, fileName, trueLoc, centr,drawMe,saveMe,offset)

    #.........................................................................


    # 1 ......................................................................
    fileName = prefix+'tape holder.p'
    trueLoc = 'entrance corridor'
    ansRow = allPlots(fullPath, endTime, minConf, deleteItems, fileName, trueLoc, centr,drawMe,saveMe,offset)
    #.........................................................................


    # 1 ......................................................................
    fileName = prefix+'lounge table.p'
    trueLoc = 'lounge'
    ansRow = allPlots(fullPath, endTime, minConf, deleteItems, fileName, trueLoc, centr,drawMe,saveMe,offset)
    #.........................................................................



    fileName=prefix+'logitech gamepad.p'
    trueLoc = 'entrance corridor'
    ansRow = allPlots(fullPath, endTime, minConf, deleteItems, fileName, trueLoc, centr,drawMe,saveMe,offset)


    #.........................................................................



    fileName=prefix+'chair.p'
    trueLoc = 'entrance corridor'
    ansRow = allPlots(fullPath, endTime, minConf, deleteItems, fileName, trueLoc, centr,drawMe,saveMe,offset)


    #.........................................................................

    fileName=prefix+'remote.p'
    trueLoc = 'entrance corridor'
    ansRow = allPlots(fullPath, endTime, minConf, deleteItems, fileName, trueLoc, centr, drawMe,saveMe,offset)




#....................................................................................................................

# Main function.
if __name__ == '__main__':
    runExpPUMS()
    #runAll()
    # resultsFile='/home/mfcarmona/catkin_ws/src/ENRICHME/codes/ais/LibMercuryRFID/src/clients/ros/rfid_grid_map/launch/article/results.csv'
    # df = pd.read_csv(resultsFile,delimiter='&')s
    # df[(df['Experiment'] == 'FDG_1') & (df['object'] == 'pillbox') & (df['Accuracy'] > 60)].plot()
    # df[(df['Experiment']=='FDG_1') & (df['object']=='pillbox')].plot(x='Av. Reg. Confidence',y='Av. dist. error')
    #runExpThree()
    #runExp('/home/mfcarmona/catkin_ws/src/ENRICHME/codes/ais/LibMercuryRFID/src/clients/ros/rfid_grid_map/launch/article/cone/','REPLAY_UOL_')

