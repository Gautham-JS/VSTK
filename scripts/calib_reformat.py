import cv2
import os
import datetime
import time
import argparse

import numpy as np

def get_datetime():
    ts = time.time()
    return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')

def log(msg : str):
    print("---- [{}] --  {} ".format(get_datetime(), msg))
    
def log_err(msg : str):
    print("xxxx [{}] -- < {} >".format(get_datetime(), msg))

class KittiLoader:
    def __init__(self, calib_file_path:str):
        if ( calib_file_path == None or calib_file_path == "" ):
            log_err("Error : no calibration file specified for KITTI dataset")
            exit(-1)
        self.calib_file_path : str = calib_file_path
        file = open(calib_file_path, "r")
        self.contents_raw : str = file.read()
        file.close()
        self.matdict = dict()

    def parse(self):
        log("Parsing contents of file : " + self.calib_file_path)
        lines : list = self.contents_raw.splitlines()
        line_ptr = 0

        P0, P1, Rt = (np.zeros((3, 4), dtype=np.double) * 3)
        R = np.zeros((3, 3), dtype=np.double)
        t = np.zeros((1, 3), dtype=np.double)

        while(line_ptr < len(lines)):
            line : str = lines[line_ptr]
            if(line != "" or line != None):
                if(line.startswith("P0:")):
                    values = line.split(":")[1]
                    values = values.strip()
                    tokens : list = values.split(" ")
                    P0_list = list()
                    # move i to the end of a row (idx 3), j follows it to the end in an inner loop from i - 3 -> i 
                    for i in range(3, 13, 4):
                        row = list()
                        for j in range(i - 3, i + 1):
                            row.append(tokens[j])
                        P0_list.append(row)
                    P0 = np.asarray(P0_list, dtype=np.double)
                    #log("Read P0 as : ")
                    print(P0)
                if(line.startswith("P1:")):
                    values = line.split(":")[1]
                    values = values.strip()
                    tokens : list = values.split(" ")
                    P1_list = list()
                    # move i to the end of a row (idx 3), j follows it to the end in an inner loop from i - 3 -> i 
                    for i in range(3, 13, 4):
                        row = list()
                        for j in range(i - 3, i + 1):
                            row.append(tokens[j])
                        P1_list.append(row)
                    P1 = np.asarray(P1_list, dtype=np.double)
                    #log("Read P1 as : ")
                    print(P1)
                if(line.startswith("Tr:")):
                    values = line.split(":")[1]
                    values = values.strip()
                    tokens : list = values.split(" ")
                    tr_list = list()
                    # move i to the end of a row (idx 3), j follows it to the end in an inner loop from i - 3 -> i 
                    for i in range(3, 13, 4):
                        row = list()
                        for j in range(i - 3, i + 1):
                            row.append(tokens[j])
                        tr_list.append(row)
                    Rt = np.asarray(tr_list, dtype=np.double)
            line_ptr+=1
        self.P0 : np.array = P0
        self.P1 : np.array = P1
        self.R = Rt[:3, :3]
        self.t = Rt[:, 3]
        log("Parsing complete")
    
    def reformat(self, out_fname:str):
        K : np.array = np.zeros((3, 3), dtype = np.double)
        d : np.array = np.zeros((1, 5), dtype = np.double)
        R : np.array = np.eye(3, 3, dtype = np.double)
        t : np.array = np.zeros((1, 3), dtype = np.double)
        p0 : np.array = self.P0
        p1 : np.array = self.P1
        baseline = (-1 * (p1[0][3] / p1[0][0]))
        t[0][0] = baseline
        log(" Extrinsic camera matrix (K) : ")
        print(p1[:3, :3])
        print("\n")
        log(" baseline : ")
        print(baseline)
        print("\n")
        log("Rotation matrix (R) : ")
        print(R)
        print("\n")
        log("Translation vector (t) : ")
        print(t)
        print("\n")

        f : cv2.FileStorage = cv2.FileStorage(out_fname, flags=1)
        f.startWriteStruct('left_cam', flags=5)
        f.write("K", val = p0[:3, :3])
        f.write("dist_coeff", val=d)
        f.endWriteStruct()
        f.startWriteStruct('right_cam', flags=5)
        f.write("K", val = p1[:3, :3])
        f.write("dist_coeff", val=d)
        f.endWriteStruct()
        f.write("Rs", val = R)
        f.write("ts", val = t)
        




                
                        
                    
                    


        
        



if __name__ == "__main__":
    log("Running calibration reformatter")
    parser = argparse.ArgumentParser(description="Camera calibration reformatter for common dataset's format to VSTK config format")
    parser.add_argument('dataset', type=str, nargs='?', help='Dataset type ("kitti/euroc/tum")')
    parser.add_argument("-c", '--calib', type=str, help='Calibration file in the dataset format', required=True)
    parser.add_argument("-o",  '--out-calib', type=str, help='Output calibration file in VSTK format', default="./vstk.yaml")

    args = parser.parse_args()
    print(args)

    log("Input calibration dataset : {}".format(args.dataset))
    log("Input calibration file path : {}".format(args.calib))
    log("Output calibration file path : {}".format(args.out_calib))

    if(args.dataset == None):
        parser.error("No dataset specified!")
    
    dataset : str = args.dataset
    dataset = dataset.upper()
    if(dataset == "KITTI"):
        log("Loading calibration file in KITTI dataset's formatting.")
        loader = KittiLoader(args.calib)
        loader.parse()
        loader.reformat(args.out_calib)






