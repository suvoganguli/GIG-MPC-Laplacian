import numpy as np
import probInfo as prob


def measureInitialValue(tmeasure, xmeasure):
    return tmeasure, xmeasure


def solveOptimalControlProblem(N, t0, x0, u0, T, ncons, nu, path, obstacle, posIdx):

    import nlp

    # OPEN LOOP
    # u_new = np.ones([N,1])

    # CLOSED LOOP
    prob = nlp.nlpProb(N, T, t0, x0, ncons, nu, path, obstacle, posIdx)
    probSetup = prob.setup(u0)
    u, info = probSetup.solve(u0.flatten(1))

    # debug
    #tmpCost = prob.objective(u)
    #print(tmpCost)

    nu = len(u)/N
    u_tmp = u.reshape(nu,N)
    u_new = u_tmp.T

    return u_new, info


def applyControl(T, t0, x0, u):
    xapplied = prob.system(u[0,:], x0, T)
    tapplied = t0+T
    return tapplied, xapplied


def shiftHorizon(N, u):
    nu = np.size(u, 1)
    u0 = np.zeros([N, nu])
    for k in range(nu):
        a = u[0:N-1, k]
        b = [u[N-1, k]]
        u0[:,k] = np.concatenate((a,b))
    return u0
