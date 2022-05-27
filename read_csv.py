import csv
from re import L
import matplotlib.pyplot as plt 
xs=[]
ys=[]
c=0
with open('data_logger.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        c+=1
        h=', '.join(row)
        list_1 = h.split(',')

        #print(list_1)
        #print(list_1[1])
        xs.append(list_1[1])
        ys.append(list_1[3])
        if c==100:
            break
        

plt.plot(xs,ys)
#plt.ylim([340, 380])


plt.show()