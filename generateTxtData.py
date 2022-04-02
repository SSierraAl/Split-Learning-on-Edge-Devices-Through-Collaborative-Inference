# g++ name.cpp -o name


# Generate random functions for OR operation
import random

f= open("TdataOR.txt","w+")
f.write("topology: 2 4 4 1\n")
for i in range(8000):
  x=random.randint(0, 1)
  y=random.randint(0, 1)
  z= x^y
  f.write("in: "+str(x)+".0 "+str(y)+".0\n")
  f.write("out: "+str(z)+".0"+"\n")

f.close()


#f= open("Tdata.txt","w+")
#f.write("topology: 2 4 4 2\n")
#for i in range(8000):
#  x=int(random.randint(0, 10)/10)
#  y=int(random.randint(0, 10)/10)

#  z1=0
#  z2=0
#  if(x+y>1):
#    z1=1
#  else:
#    z2=1

#  f.write("in: "+str(x)+".0 "+str(y)+".0 \n")
#  f.write("out: "+str(z1)+".0 "+ str(z2)+".0 \n")

#f.close()