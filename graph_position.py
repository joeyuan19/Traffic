import matplotlib.pyplot as plt

with open('position','r') as f:
    for line in f:
        merge = int(line)
        break
    data = [line.split(',') for line in f]
    data = tuple((int(i),float(j)) for i,j in data)
datasets = {}
for n,pos in data:
    if n not in datasets:
        datasets[n] = [pos]
    else:
        datasets[n].append(pos)
m = 0
for n in datasets:
    y = datasets[n]
    x = [i for i in range(len(y))]
    m = max(m,len(x))
    plt.plot(x,y,label='car '+str(n))
plt.plot([0,m],[merge,merge],color='gray',linestyle='--')

plt.legend(loc='lower right')
plt.show()

