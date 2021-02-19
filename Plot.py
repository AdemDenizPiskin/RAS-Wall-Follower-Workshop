import matplotlib.pyplot as plt
import math

t = [0.01*i for i in range(1000)]

fid = open("data.txt","r")
r = []
theta = []


for lin in fid:
    s = lin.split()
    r.append( float(s[0]) )
    theta.append( 180*float(s[1])/math.pi )

pfid = open("prediction.txt","r")
pred_r = []
pred_theta = []

for lin in pfid:
    s = lin.split()
    pred_r.append( float(s[0]))
    pred_theta.append(180*float(s[1])/math.pi)


show_bound = True

fig, axs = plt.subplots(2)
fig.suptitle('r and theta')


axs[0].plot(t,pred_r)
axs[0].plot(t,r,color='r')

if show_bound:
    axs[0].plot(t,[0 for _ in range(1000)])
    axs[0].plot(t,[20 for _ in range(1000)])

axs[0].legend(["predicted  r","r"])

axs[1].plot(t,pred_theta)
axs[1].plot(t,theta,color='r')
if show_bound:
    axs[1].plot(t,[90 for _ in range(1000)])
    axs[1].plot(t,[-90 for _ in range(1000)])
axs[1].legend(["predicted theta","theta"])

plt.show()

