#made by Sam Penders (pende061@umn.edu) using existing Mu2e database programs
import csv
import os
import time
from DataLoader import DataLoader, DataQuery
from time import strftime
from datetime import datetime

#have all of the files in the same directory as this program will be run. this
#program checks that the filename starts with "test_name" + "_y_m_d" and is .csv file.
#these .csv files will be generated by workers using appropriate programs.
#the create time given by the tables is the time uploaded, not the time 
#in csv files

#possibly add in comment feature

url = "http://dbweb6.fnal.gov:8080/mu2edev/hdb/loader" #these are the same for all staw tables
queryUrl = "http://dbweb6.fnal.gov:8088/QE/mu2e_hw_dev/app/SQ/query"
group = "Straw Tables"
password = "sdwjmvw"


path = 'C:\\Users\\vold\\Desktop\\Leak Test Results\\' #where files are located C:\Users\vold\Desktop\Leak Test Results
	
#first upload all make_straw files
def makestraw():

    for row in upload_file:

        def createRow():

            if str(row[2]) is not '':
                    return{'straw_barcode': str(row[0]),
                    'batch_number' : str(row[1]),
                    'parent' : str(row[2]),
                    'worker_barcode' : str(row[3]),
                    'create_time' : str(row[4]),}
				
            else: #if there is no parent straw
                    return{'straw_barcode': str(row[0]),
                    'batch_number' : str(row[1]), 
                    'worker_barcode' : str(row[3]),
                    'create_time' : str(row[4]),}
				

	table = "straws"
	dataLoader = DataLoader(password,url,group,table)
	dataLoader.addRow(createRow())
	retVal,code,text =  dataLoader.send()

	if retVal:

		print "succesfuly made straw " + str(row[0])

	elif retVal == False:

		dataLoader = DataLoader(password,url,group,table)

		aRow = createRow()

		dataLoader.addRow(aRow,'update')

		retVal,code,text =  dataLoader.send()

		if retVal:
			print "succesfully updated straw " + str(row[0])
	else:
		print "fail to make or update straw " + str(row[0])

		print code

	dataLoader.clearRows()

def uploadthicknesses():

    for row in upload_file:

        def createRow():
            return{'straw_barcode': str(row[0]),
            'create_time' : str(row[1]),
            'thickness' : str(row[2]),
            'worker_barcode' : str(row[3]),
            'workstation_barcode' : str(row[4]),}

	table = "straw_thicknesses"
	dataLoader = DataLoader(password,url,group,table)
	dataLoader.addRow(createRow())
	retVal,code,text =  dataLoader.send()

	if retVal:

            print "thickness upload success!\n"

            print text

	else:

            print "thickness upload failed!\n"

            print code

            print text

	dataLoader.clearRows()

def uploadleaktests():

    for row in upload_file:
        def createRow():
            return{'straw_barcode': str(row[0]),
            'create_time' : str(row[1]), #Website gets real time somehow.
            'test_type' : str(row[2]),
            'worker_barcode' : str(row[3]),
            'workstation_barcode' : str(row[4]),
            #row[5] is channel where it was tested
            'leak_rate' : str(row[6]),
            'comments': str(row[7])+' (uncertainty)',}
        table = "Straw_Leak_Tests"
        dataLoader = DataLoader(password,url,group,table)
        dataLoader.addRow(createRow())
        retVal,code,text =  dataLoader.send()
        if retVal:
            print "upload leak test success!\n"
            print text
        else:
            print "upload leak test failed!\n"
            print code
            print text
        dataLoader.clearRows()
      
def strawcutlengths():

    for row in upload_file:
        def createRow():
            return{'straw_barcode': str(row[0]),
            'create_time' : str(row[1]), #Website gets real time somehow.
            'worker_barcode' : str(row[2]),
            'workstation_barcode' : str(row[3]),
            'nominal_length' : str(row[4]),
            'measured_length': str(row[5]),
            'temperature' : str(row[6]),
            'humidity' : str(row[7]),
            'comments' : str(row[8]),}
        table = "straw_cut_lengths"
        dataLoader = DataLoader(password,url,group,table)
        dataLoader.addRow(createRow())
        retVal,code,text =  dataLoader.send()
        if retVal:
            print "upload straw length success!\n"
            print text
        else:
            print "upload straw length failed!\n"
            print code
            print text
        dataLoader.clearRows()

def resistanceupload():
    for row in upload_file:
        def createRow():
            return{'straw_barcode': str(row[0]),
            'create_time' : str(row[1]), #Website gets real time somehow.
            'worker_barcode' : str(row[2]),
            'workstation_barcode' : str(row[3]),
            'resistance' : str(row[4]),
            'temperature' : str(row[5]),
            'humidity' : str(row[6]),
            'resistance_measurement_type' : str(row[7]),
            'comments' : str(row[8]),}
        table = "straw_resistance_measurements"
        dataLoader = DataLoader(password,url,group,table)
        dataLoader.addRow(createRow())
        retVal,code,text =  dataLoader.send()
        if retVal:
            print "upload resistance success!\n"
            print text
        else:
            print "upload resistance failed!\n"
            print code
            print text
        dataLoader.clearRows()     
	
def uploadglueup():

    for row in upload_file:

        def createRow():
            return{'straw_barcode': str(row[0]),
                    'glueup_type' : str(row[1]),
                    'worker_barcode' : str(row[2]),
                    'workstation_barcode' : str(row[3]),
                    'comments' : str(row[4]),
                    'glue_batch_number' : str(row[5]),}
	table = "straw_glueups"
	dataLoader = DataLoader(password,url,group,table)
	dataLoader.addRow(createRow())
	retVal,code,text =  dataLoader.send()

	if retVal:

		print "glueup upload success!\n"

		print text

	else:

		print "glueup upload failed!\n"

		print code

		print text

	dataLoader.clearRows()

filelist = os.listdir(path) #directory where things are located

#first upload all make_straw files from that day
filestart= 'make_straw_' + datetime.now().strftime("%Y-%m-%d")

for i in filelist:
	if i.startswith(filestart) & i.endswith(".csv"):
		f = open(path + i)
		upload_file = csv.reader(f)
		makestraw()
		f.close()

#upload thicknesses from appropriately named files		
filestart= 'straw_thickness_' + datetime.now().strftime("%Y-%m-%d")
for i in filelist:
	if i.startswith(filestart) & i.endswith(".csv"):
		f = open(path + i)
		upload_file = csv.reader(f)
		uploadthicknesses()
		f.close()
		
#upload leak test data
filestart= 'leak_test_' + datetime.now().strftime("%Y-%m-%d")
for i in filelist:
	if i.startswith(filestart) & i.endswith(".csv"):
		f = open(path + i)
		upload_file = csv.reader(f)
		uploadleaktests()
		f.close()
		
		
#upload straw cut lengths
filestart= 'straw_cut_length_' + datetime.now().strftime("%Y-%m-%d")
for i in filelist:
	if i.startswith(filestart) & i.endswith(".csv"):
		f = open(path + i)
		upload_file = csv.reader(f)
		strawcutlengths()
		f.close()
		
#upload resistance measurements
filestart= 'resistance_measurements_' + datetime.now().strftime("%Y-%m-%d")
for i in filelist:
	if i.startswith(filestart) & i.endswith(".csv"):
		f = open(path + i)
		upload_file = csv.reader(f)
		resistanceupload()
		f.close()

#upload glueup data
filestart= 'glueup_' + datetime.now().strftime("%Y-%m-%d")
for i in filelist:
	if i.startswith(filestart) & i.endswith(".csv"):
		f = open(path + i)
		upload_file = csv.reader(f)
		uploadglueup()
		f.close()
