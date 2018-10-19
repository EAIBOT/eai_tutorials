import numpy
import pylab

n_iter = 50 #number of iterations
sz = (n_iter,) # size of array
x = 24 # truth value (actual temperature)
R = 4e-4 # process variance 
Q = 0.5 # estimate of measurement variance, change to see effect
z = numpy.random.normal(x,Q,size=sz) # observations (normal about x, sigma=0.1)


x_update=numpy.zeros(sz)      # a posteri estimate of x
P=numpy.zeros(sz)         # a posteri error estimate
x_pre=numpy.zeros(sz) # a priori estimate of x
P_pre=numpy.zeros(sz)    # a priori error estimate
K=numpy.zeros(sz)         # gain or blending factor


x_update[0] = 26
P[0] = 2.0


for k in range(1,n_iter):

    x_pre[k] = x_update[k-1]
    P_pre[k] = P[k-1]+R
 
    K[k] = P_pre[k]/( P_pre[k]+Q )
    x_update[k] = x_pre[k]+K[k]*(z[k]-x_pre[k])
    P[k] = (1-K[k])*P_pre[k]


pylab.figure()
pylab.plot(z,'k+',label='noisy measurements') 
pylab.plot(x_update,'b-',label='a posteri estimate') 
pylab.axhline(x,color='g',label='truth value')
pylab.legend()
pylab.xlabel('Iteration')
pylab.ylabel('temperature')


pylab.figure()
valid_iter = range(1,n_iter) # P_pre not valid at step 0
pylab.plot(valid_iter,P_pre[valid_iter],label='a priori error estimate')
pylab.xlabel('Iteration')
pylab.ylabel('Temperature Variance')
pylab.setp(pylab.gca(),'ylim',[0,P[0]])
pylab.show()
