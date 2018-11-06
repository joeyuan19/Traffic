import matplotlib.pyplot as plt
import scipy.stats

def linfit(x,y):
    m,b,r,p,e = scipy.stats.linregress(x,y)
    min_x = 0
    max_x = max(x)
    x = (min_x,max_x)
    return x,tuple(m*xi+b for xi in x)


with open('clearance_time','r') as f:
    data = [line.split(',') for line in f]
    data = tuple((int(i),int(j),float(k)) for i,j,k in data)
datasets_lanes = {}
datasets_cars = {}

for cars,lanes,t in data:
    if lanes not in datasets_lanes:
        datasets_lanes[lanes] = [(cars,t)]
    else:
        datasets_lanes[lanes].append((cars,t))
    if cars not in datasets_cars:
        datasets_cars[cars] = [(lanes,t)]
    else:
        datasets_cars[cars].append((lanes,t))
m = 0
c = ['b','r','g','orange','yellow','gray','k']
for j,n in enumerate(datasets_lanes):
    data = datasets_lanes[n]
    x = [i[0] for i in data]
    y = [i[1] for i in data]
    fx,fy = linfit(x,y)
    plt.plot(x,y,c[j]+'o')
    plt.plot(fx,fy,c[j]+'-')
plt.xlabel('N (cars)')
plt.ylabel('time (s)')
plt.show()

