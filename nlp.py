import ipopt
import probInfo as prob
from problemData import *


class nlpProb(object):

    def __init__(self, N, T, t0, x0, ncons, nu, lanes, obstacle, posIdx):
        self.N = N
        self.T = T
        self.t0 = t0
        self.x0 = x0
        self.ncons = ncons  # number of constraints
        self.nu = nu # number of controls
        self.lanes = lanes
        self.obstacle = obstacle
        self.posIdx = posIdx
        pass


    def objective(self, u):
        N = self.N
        T = self.T
        t0 = self.t0
        x0 = self.x0
        lanes = self.lanes
        obstacle = self.obstacle
        posIdx = self.posIdx

        x = prob.computeOpenloopSolution(u, N, T, t0, x0)
        cost = 0.0
        costvec = np.zeros([N, 1])

        for k in range(N):
            uk = np.array([u[k],u[k+N]])
            costvec[k] = prob.runningCosts( uk, x[k], t0 + k*T, lanes, obstacle, posIdx)
            cost = cost + costvec[k]

        return cost


    def gradient(self, u):
        N = self.N
        nu = self.nu

        eps = 1e-2
        obj_grad_u = np.zeros(nu*N)
        for k in range(nu*N):
            uplus = np.copy(u)
            uminus = np.copy(u)

            uplus[k] = uplus[k] + eps
            obj_uplus = self.objective(uplus)

            uminus[k] = uminus[k] - eps
            obj_uminus = self.objective(uminus)

            obj_grad_u[k] = (obj_uplus - obj_uminus) / (2 * eps)

        return obj_grad_u


    def constraints(self, u):
        N = self.N
        T = self.T
        t0 = self.t0
        x0 = self.x0
        lanes = self.lanes
        obstacle = self.obstacle
        posIdx = self.posIdx

        x = prob.computeOpenloopSolution(u, N, T, t0, x0)

        # running constraints
        consR1_R = np.zeros(N)
        consR1_L = np.zeros(N)

        for k in range(N):
            consR1_R[k], consR1_L[k] = prob.runningCons(u, x[k], t0, lanes, obstacle, posIdx)

        consR1 = np.concatenate([consR1_R, consR1_L])

        if ns == 6:
            consR2 = np.array([x[0, idx_V] * x[0, idx_Chidot] * useLatAccelCons])  # lateral acceleration
            consR3 = np.array([x[0, idx_V]])  # velocity

            constmp = np.concatenate([consR1, consR2])
            consR = np.concatenate([constmp, consR3])

            # terminal constraint
            consT1, consT2 = prob.terminalCons(u, x[N - 1], t0, lanes, obstacle, posIdx)
            consT = np.concatenate([consT1, consT2])

        elif ns == 4:
            u_mat = u.reshape(2,-1).T

            consR2 = np.array([x[0, idx_V] * u_mat[0, idx_Chidot] * useLatAccelCons])  # lateral acceleration

            consR = np.concatenate([consR1, consR2])

            # terminal constraint
            consT1, consT2 = prob.terminalCons(u, x[N-1], t0, lanes, obstacle, posIdx)  # ydist, VEnd
            consT = consT1

        # total constraints
        cons = np.concatenate([consR,consT])

        return cons


    def jacobian(self, u):
        N = self.N
        ncons = self.ncons
        nu = self.nu
        jac = np.zeros([ncons,nu*N])
        eps = 1e-2

        for j in range(ncons):

            for k in range(nu*N):
                uplus = np.copy(u)
                uminus = np.copy(u)

                uplus[k] = uplus[k] + eps
                cons_uplus = self.constraints(uplus)

                uminus[k] = uminus[k] - eps
                cons_uminus = self.constraints(uminus)

                jac[j,k] = (cons_uplus[j] - cons_uminus[j]) / (2 * eps)


        return jac.flatten()


    def setup(self, u0):

        N = self.N
        T = self.T
        t0 = self.t0
        x0 = self.x0
        nu = self.nu
        lanes = self.lanes
        obstacle = self.obstacle
        posIdx = self.posIdx

        if ns == 6:

            lb_Vddot = np.ones([N,1])*lb_VddotVal
            lb_Chiddot = np.ones([N,1])*lb_ChiddotVal

            ub_Vddot = np.ones([N,1])*ub_VddotVal
            ub_Chiddot = np.ones([N,1])*ub_ChiddotVal

            lb = np.concatenate([lb_Vddot, lb_Chiddot])
            ub = np.concatenate([ub_Vddot,ub_Chiddot])

        elif ns == 4:

            lb_Vdot = np.ones([N, 1]) * lb_VdotVal
            lb_Chidot = np.ones([N, 1]) * lb_ChidotVal

            ub_Vdot = np.ones([N, 1]) * ub_VdotVal
            ub_Chidot = np.ones([N, 1]) * ub_ChidotVal

            lb = np.concatenate([lb_Vdot, lb_Chidot])
            ub = np.concatenate([ub_Vdot, ub_Chidot])


        lataccel_max = lataccel_maxVal

        # Running Constraints
        u = u0.flatten(1)
        x = prob.computeOpenloopSolution(u, N, T, t0, x0)

        if obstacle.Present == True:

            lane1Lines = lanes.lane1Lines
            lane2Lines = lanes.lane2Lines
            acrossLines = lanes.acrossLines

            idx_Vehicle, laneNo = lanes.insideRoadSegment(x0[0], x0[1], lane1Lines, lane2Lines,
                                                                     acrossLines)

            if (idx_Vehicle < obstacle.idx_StartSafeZone):
                dyRoadL = delta_yRoad
                dyRoadR = delta_yRoad
            elif (idx_Vehicle >= obstacle.idx_StartSafeZone) and (idx_Vehicle < obstacle.idx_EndSafeZone):
                dyRoadL = delta_yRoad
                dyRoadR = delta_yRoadRelaxed
            elif (idx_Vehicle >= obstacle.idx_StartObstacle) and (idx_Vehicle < obstacle.idx_EndObstacle):
                dyRoadL = delta_yRoad
                dyRoadR = delta_yRoad
            else:
                dyRoadL = delta_yRoad
                dyRoadR = delta_yRoad
        else:
            dyRoadL = delta_yRoad
            dyRoadR = delta_yRoad

        # Running Constraint
        #cl_running = np.concatenate([-1*np.ones(N), 0*np.ones(N)])
        #cu_running = np.concatenate([ 0*np.ones(N), 1*np.ones(N)])
        cl_running = np.concatenate([-100*np.ones(N), 0*np.ones(N)])
        cu_running = np.concatenate([ 0*np.ones(N), 100*np.ones(N)])
        cl_tmp1 = np.concatenate([cl_running, [-lataccel_max]])
        cu_tmp1 = np.concatenate([cu_running, [+lataccel_max]])

        if ns == 6:
            # Speed Constraint
            cl_tmp2 = np.concatenate([cl_tmp1, [lb_V]])
            cu_tmp2 = np.concatenate([cu_tmp1, [ub_V]])

            # Terminal Constraint
            cl_tmp3 = np.concatenate([cl_tmp2, [-dyRoadL]])
            cu_tmp3 = np.concatenate([cu_tmp2, [dyRoadR]])

            cl = np.concatenate([cl_tmp3, [-delta_V + V_cmd]])
            cu = np.concatenate([cu_tmp3, [delta_V + V_cmd]])

        elif ns == 4:

            # Terminal Constraint
            cl = np.concatenate([cl_tmp1,[-dyRoadL]])
            cu = np.concatenate([cu_tmp1,[ dyRoadR]])


        if ncons != len(cl) and ncons != len(cu):
            print('Error: resolve number of constraints')

        nlp = ipopt.problem(
            n=nu*N,
            m=len(cl),
            problem_obj=nlpProb(N, T, t0, x0, ncons, nu, lanes, obstacle, posIdx),
            lb=lb,
            ub=ub,
            cl=cl,
            cu=cu
        )
        nlp.addOption('print_level', nlpPrintLevel)
        nlp.addOption('max_iter', nlpMaxIter)
        #nlp.addOption('dual_inf_tol',10.0)  # defaut = 1
        nlp.addOption('constr_viol_tol',0.1)  # default = 1e-4
        nlp.addOption('compl_inf_tol',0.1) # default = 1e-4
        nlp.addOption('acceptable_tol',0.1) # default = 0.01
        nlp.addOption('acceptable_constr_viol_tol',0.1)  # default = 0.01

        return nlp
