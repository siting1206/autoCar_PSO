import psoParameter as psoPm
import pso as pso
import os

path = "./data/"
dataList = []
input = []
output = []
for dirPath, dirNames, fileNames in os.walk(path):
    for file in fileNames:
        file = os.path.join(dirPath, file)
        f = open(file, "r")
        for line in f.readlines():
            line_data = line.split()
            mylist = []
            if type(eval(line_data[0])) == float:
                mylist.append(eval(line_data[0]))
                mylist.append(eval(line_data[1]))
                mylist.append(eval(line_data[2]))
                input.append(mylist)
                output.append(eval(line_data[3]))
            else:
                print(line_data)

for i in range(0, len(output)):  # Normalization(0~1)
    output[i] = (output[i] + 40) / 80

fError_ori = 1e9
fError_now = 1e9
times = 0

pp = psoPm.PSOParameter()
bestPSO = pp.psoIteration(input, output)

print("Best particle's fitness is = ", bestPSO.F)
fError_now = bestPSO.F

if fError_now < fError_ori:
    writeFile = open("./bestPSO.txt", "w")
    PSOList = bestPSO.getPSOList()
    for i in range(len(PSOList)):
        writeFile.write(str(PSOList[i]))
        writeFile.write("     ")
    writeFile.close()
    print("file has be written")
times += 1
print(times)
print("--------------------------------------")
