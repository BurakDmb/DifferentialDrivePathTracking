import csv

# open the file in the write mode
f = open('data_logger.csv', 'w')

# create the csv writer
writer = csv.writer(f)

# write a row to the csv file
#for i in range(5):
#    row=[1,2*i,"fff"]

 #   writer.writerow(row)

# close the file
f.close()
f = open('data_logger.csv', 'a')
writer = csv.writer(f)
writer.writerow([4,4])
f.close()
