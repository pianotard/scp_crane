### Things to change for solve_scp:
# Phases in cranetype, phases in craneloc, phases in cranesupply, phases in demand.
# Change the phases and cost in calc_cost
# rem to change the file name too
# rem to change original schedule in calc_cost too... this is the filter mechanism to check breaks

from gurobipy import *
import numpy as np
import pandas as pd
import math


def solve_curr(demand, supply, craneloc, cranetype, numCrane):

    # Model
    m = Model("Traditional")

    # Variable Declaration
    Q = {}
    # decision variable for demand pt to supply pt given a particular crane at a particular location
    krane = {}
    # decision variable for a crane type at a location
    ay = {}
    # auxiliary variables /indicator variables
    aw = {}
    # another set of auxiliary variables/indicator variables
    cost_per_minute = {}
    # cost vector for cranes
    time = {}

    # List of variables. Better to use list in gurobi, rather than the pandas series.
    l_lst = craneloc['ind'].tolist()  # location
    k_lst = cranetype['ind'].tolist()  # crane
    i_lst = demand['ind'].tolist()  # demand
    j_lst = supply['ind'].tolist()  # supply

    for l in l_lst:
        for k in k_lst:
            strname = "CL_" + str(l) + '_' + str(k)
            krane[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname)
            strname1 = "D1_" + str(l) + '_' + str(k)
            strname2 = "D2_" + str(l) + '_' + str(k)
            aw[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname1)
            ay[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname2)
            for i in i_lst:
                for j in j_lst:
                    strnameq = "Q_" + str(l) + "_" + str(k) + "_" + str(i) + "_" + str(j)
                    Q[l, k, i, j] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strnameq)

    m.update()

    # Calculate Tijkl... the time taken to traverse
    for ind1, locrow in craneloc.iterrows():
        for ind2, typerow in cranetype.iterrows():
            for ind3, demandrow in demand.iterrows():
                for ind4, supplyrow in supply.iterrows():
                    vw = typerow['Vw']
                    vh = typerow['Vh']
                    vz = typerow['Vz']
                    cop = typerow['Ht']
                    rad = typerow['maxR']
                    dx = demandrow['X']
                    dy = demandrow['Y']
                    dz = demandrow['Z']
                    cx = locrow['X']
                    cy = locrow['Y']
                    cz = locrow['Z']
                    sx = supplyrow['X']
                    sy = supplyrow['Y']
                    sz = supplyrow['Z']
                    time[locrow['ind'], typerow['ind'], demandrow['ind'], supplyrow['ind']] = \
                        calc_time(vw, vh, vz, rad, dx, dy, dz, cx, cy, cz+cop, sx, sy, sz)

    # Determine the cost per minute vector
    for l in l_lst:
        for k in k_lst:
            cranecost = cranetype.loc[cranetype.ind == k, 'Cost'].values[0]  # use values to acces the resultant, which is a dict. But inside the dict is still a list. So use [0] to access the value. Fucked up.
            for i in i_lst:
                for j in j_lst:
                    cost_per_minute[l, k, i, j] = cranecost / 43200  # 43200 minutes per month

    # Objective Function
    '''obj = quicksum(Q[l, k, i, j] * time[l, k, i, j] * cost_per_minute[l, k, i, j]
                   for l in l_lst for k in k_lst for i in i_lst for j in j_lst)
    '''
    obj = 0
    for l in l_lst:
        for k in k_lst:
            for i in i_lst:
                for j in j_lst:
                    obj += Q[l, k, i, j] * time[l, k, i, j] * cost_per_minute[l, k, i, j]
    m.setObjective(obj, GRB.MINIMIZE)

    # Constraints
    # For every demand point, there is ONLY one crane lifting it to from any supply point from any crane location
    for i in i_lst:
        m.addConstr(quicksum(Q[l, k, i, j] for l in l_lst for k in k_lst for j in j_lst), GRB.EQUAL, 1, "assignment")

    '''
    # For every location, there is only one crane at that location
    for l in l_lst:
        m.addConstr(quicksum(Q[l, k, i, j] for k in k_lst for i in i_lst for j in j_lst), GRB.LESS_EQUAL, 1, "crane_loc_assign")
    '''

    # Matching krane with assignment
    for l in l_lst:
        for k in k_lst:
            M = 1000000  # arbitraily large
            qtot = 0
            for i in i_lst:
                for j in j_lst:
                    qtot += Q[l, k, i, j]
            m.addConstr(qtot - M * ay[l, k], GRB.LESS_EQUAL, 0, "match_crane")
            m.addConstr(krane[l, k] + M * (1 - aw[l, k]), GRB.GREATER_EQUAL, 1)
            m.addConstr(ay[l, k] - aw[l, k], GRB.LESS_EQUAL, 0)

    # Constraint for ensuring only one crane per location
    for l in l_lst:
        krane_alloc = 0
        for k in k_lst:
            krane_alloc += krane[l, k]
        m.addConstr(krane_alloc, GRB.LESS_EQUAL, 1)

    # Constraint on the numer of cranes
    m.addConstr(quicksum(krane[l, k] for l in l_lst for k in k_lst), GRB.LESS_EQUAL, numCrane)

    # Run the Model
    m.optimize()

    # Output the results
    m.write("traditional_soln.sol")

    # return


def calc_time(vw, vh, vz, rad, dx, dy, dz, cx, cy, cz, sx, sy, sz):
    tz = abs(dz - sz) / vz
    rho_d = ((dx - cx)**2 + (dy - cy)**2)**0.5
    rho_s = ((sx - cx)**2 + (sy - cy)**2)**0.5
    len_ij = ((dx - sx)**2 + (dy - sy)**2)**0.5
    th = abs(rho_s - rho_d) / vh
    cosA = round(((rho_d**2 + rho_s**2) - len_ij**2) / (2 * rho_d * rho_s), 5)
    # use rounding to get rid of the error when 1 or -1. Arises from floating point arith
    tw = math.acos(cosA) / vw
    if rho_d > rad or rho_s > rad or cz < sz or cz < dz:
        maxmax = 1000000000000
    else:
        maxmax = max(th, tw, tz)
    return maxmax


def solve_curr2(demand, supply, craneloc, cranetype):
    # this model has updated to include the Fixed costs of installation/dismantling inside as well.

    # Model
    m = Model("Traditional2_without_fixed_costs")

    # Variable Declaration
    Q = {}
    # decision variable for demand pt to supply pt given a particular crane at a particular location
    krane = {}
    # decision variable for a crane type at a location
    ay = {}
    # auxiliary variables /indicator variables
    aw = {}
    # another set of auxiliary variables/indicator variables

    # Constants to be determined
    cost_per_minute = {}
    # cost vector for cranes
    time = {}

    # List of variables. Better to use list in gurobi, rather than the pandas series.
    l_lst = craneloc['ind'].tolist()  # location
    k_lst = cranetype['ind'].tolist()  # crane
    i_lst = demand['ind'].tolist()  # demand
    j_lst = supply['ind'].tolist()  # supply

    for l in l_lst:
        for k in k_lst:
            strname = "CL_" + str(l) + '_' + str(k)
            krane[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname)
            strname1 = "D1_" + str(l) + '_' + str(k)
            strname2 = "D2_" + str(l) + '_' + str(k)
            aw[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname1)
            ay[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname2)
            for i in i_lst:
                for j in j_lst:
                    strnameq = "Q_" + str(l) + "_" + str(k) + "_" + str(i) + "_" + str(j)
                    Q[l, k, i, j] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strnameq)

    m.update()

    # Calculate Tijkl... the time taken to traverse
    for ind1, locrow in craneloc.iterrows():
        for ind2, typerow in cranetype.iterrows():
            for ind3, demandrow in demand.iterrows():
                for ind4, supplyrow in supply.iterrows():
                    vw = typerow['Vw']
                    vh = typerow['Vh']
                    vz = typerow['Vz']
                    cop = typerow['Ht']
                    rad = typerow['maxR']
                    dx = demandrow['X']
                    dy = demandrow['Y']
                    dz = demandrow['Z']
                    cx = locrow['X']
                    cy = locrow['Y']
                    cz = locrow['Z']
                    sx = supplyrow['X']
                    sy = supplyrow['Y']
                    sz = supplyrow['Z']
                    time[locrow['ind'], typerow['ind'], demandrow['ind'], supplyrow['ind']] = \
                        calc_time(vw, vh, vz, rad, dx, dy, dz, cx, cy, cz+cop, sx, sy, sz)

    # Determine the cost per minute vector
    for l in l_lst:
        for k in k_lst:
            cranecost = cranetype.loc[cranetype.ind == k, 'Cost'].values[0]  # use values to acces the resultant, which is a dict. But inside the dict is still a list. So use [0] to access the value. Fucked up.
            for i in i_lst:
                for j in j_lst:
                    cost_per_minute[l, k, i, j] = cranecost / 2400  # 43800 minutes per month, and doesn't consider working time
                    # Cost is now cost/week. So it will be 5 days * 8 hours * 60min = 2400mins per week

    fixedcost = {}
    for l in l_lst:
        for ind2, typerow in cranetype.iterrows():
            fixedcost[l, typerow['ind']] = typerow['Fixed']

    # Objective Function
    obj = quicksum(krane[l, k] * fixedcost[l, k] for l in l_lst for k in k_lst) * 0 # 0 added in to ignore fixed cost
    for l in l_lst:
        for k in k_lst:
            for i in i_lst:
                for j in j_lst:
                    obj += Q[l, k, i, j] * time[l, k, i, j] * cost_per_minute[l, k, i, j]
    m.setObjective(obj, GRB.MINIMIZE)

    # Constraints
    # For every demand point, there is ONLY one crane lifting it to from any supply point from any crane location
    for i in i_lst:
        m.addConstr(quicksum(Q[l, k, i, j] for l in l_lst for k in k_lst for j in j_lst), GRB.EQUAL, 1, "assignment")

    # Matching krane with assignment
    for l in l_lst:
        for k in k_lst:
            M = 100000000  # arbitraily large
            qtot = 0
            for i in i_lst:
                for j in j_lst:
                    qtot += Q[l, k, i, j]
            m.addConstr(qtot - M * ay[l, k], GRB.LESS_EQUAL, 0, "match_crane")
            m.addConstr(krane[l, k] + M * (1 - aw[l, k]), GRB.GREATER_EQUAL, 1)
            m.addConstr(ay[l, k] - aw[l, k], GRB.LESS_EQUAL, 0)

    # Constraint for ensuring only one crane per location
    for l in l_lst:
        krane_alloc = 0
        for k in k_lst:
            krane_alloc += krane[l, k]
        m.addConstr(krane_alloc, GRB.LESS_EQUAL, 1)

    # Constraint on the numer of cranes
    m.addConstr(quicksum(krane[l, k] for l in l_lst for k in k_lst), GRB.LESS_EQUAL, 2)

    # Run the Model
    m.optimize()

    # Output the results
    m.write("traditional_soln2 wo fixed.sol")


def solve_curr2_with_fixed(demand, supply, craneloc, cranetype):
    # this model has updated to include the Fixed costs of installation/dismantling inside as well.

    # Model
    m = Model("Traditional2_with_fixed_costs")

    # Variable Declaration
    Q = {}
    # decision variable for demand pt to supply pt given a particular crane at a particular location
    krane = {}
    # decision variable for a crane type at a location
    ay = {}
    # auxiliary variables /indicator variables
    aw = {}
    # another set of auxiliary variables/indicator variables

    # Constants to be determined
    cost_per_minute = {}
    # cost vector for cranes
    time = {}

    # List of variables. Better to use list in gurobi, rather than the pandas series.
    l_lst = craneloc['ind'].tolist()  # location
    k_lst = cranetype['ind'].tolist()  # crane
    i_lst = demand['ind'].tolist()  # demand
    j_lst = supply['ind'].tolist()  # supply

    for l in l_lst:
        for k in k_lst:
            strname = "CL_" + str(l) + '_' + str(k)
            krane[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname)
            strname1 = "D1_" + str(l) + '_' + str(k)
            strname2 = "D2_" + str(l) + '_' + str(k)
            aw[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname1)
            ay[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname2)
            for i in i_lst:
                for j in j_lst:
                    strnameq = "Q_" + str(l) + "_" + str(k) + "_" + str(i) + "_" + str(j)
                    Q[l, k, i, j] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strnameq)

    m.update()

    # Calculate Tijkl... the time taken to traverse
    for ind1, locrow in craneloc.iterrows():
        for ind2, typerow in cranetype.iterrows():
            for ind3, demandrow in demand.iterrows():
                for ind4, supplyrow in supply.iterrows():
                    vw = typerow['Vw']
                    vh = typerow['Vh']
                    vz = typerow['Vz']
                    cop = typerow['Ht']
                    rad = typerow['maxR']
                    dx = demandrow['X']
                    dy = demandrow['Y']
                    dz = demandrow['Z']
                    cx = locrow['X']
                    cy = locrow['Y']
                    cz = locrow['Z']
                    sx = supplyrow['X']
                    sy = supplyrow['Y']
                    sz = supplyrow['Z']
                    time[locrow['ind'], typerow['ind'], demandrow['ind'], supplyrow['ind']] = \
                        calc_time(vw, vh, vz, rad, dx, dy, dz, cx, cy, cz+cop, sx, sy, sz)

    # Determine the cost per minute vector
    for l in l_lst:
        for k in k_lst:
            cranecost = cranetype.loc[cranetype.ind == k, 'Cost'].values[0]  # use values to acces the resultant, which is a dict. But inside the dict is still a list. So use [0] to access the value. Fucked up.
            for i in i_lst:
                for j in j_lst:
                    cost_per_minute[l, k, i, j] = cranecost / 2400  # 43800 minutes per month, and doesn't consider working time
                    # Cost is now cost/week. So it will be 5 days * 8 hours * 60min = 2400mins per week

    fixedcost = {}
    for l in l_lst:
        for ind2, typerow in cranetype.iterrows():
            fixedcost[l, typerow['ind']] = typerow['Fixed']

    # Objective Function
    obj = quicksum(krane[l, k] * fixedcost[l, k] for l in l_lst for k in k_lst)
    for l in l_lst:
        for k in k_lst:
            for i in i_lst:
                for j in j_lst:
                    obj += Q[l, k, i, j] * time[l, k, i, j] * cost_per_minute[l, k, i, j]
    m.setObjective(obj, GRB.MINIMIZE)

    # Constraints
    # For every demand point, there is ONLY one crane lifting it to from any supply point from any crane location
    for i in i_lst:
        m.addConstr(quicksum(Q[l, k, i, j] for l in l_lst for k in k_lst for j in j_lst), GRB.EQUAL, 1, "assignment")

    # Matching krane with assignment
    for l in l_lst:
        for k in k_lst:
            M = 100000000  # arbitraily large
            qtot = 0
            for i in i_lst:
                for j in j_lst:
                    qtot += Q[l, k, i, j]
            m.addConstr(qtot - M * ay[l, k], GRB.LESS_EQUAL, 0, "match_crane")
            m.addConstr(krane[l, k] + M * (1 - aw[l, k]), GRB.GREATER_EQUAL, 1)
            m.addConstr(ay[l, k] - aw[l, k], GRB.LESS_EQUAL, 0)

    # Constraint for ensuring only one crane per location
    for l in l_lst:
        krane_alloc = 0
        for k in k_lst:
            krane_alloc += krane[l, k]
        m.addConstr(krane_alloc, GRB.LESS_EQUAL, 1)

    # Constraint on the numer of cranes
    m.addConstr(quicksum(krane[l, k] for l in l_lst for k in k_lst), GRB.LESS_EQUAL, 6)

    # Run the Model
    m.optimize()

    # Output the results
    m.write("phase_ab.sol")


def solve_curr2_wo_cost(demand, supply, craneloc, cranetype):
    # this model has updated to include the Fixed costs of installation/dismantling inside as well.

    # Model
    m = Model("Traditional2_with_fixed_costs")

    # Variable Declaration
    Q = {}
    # decision variable for demand pt to supply pt given a particular crane at a particular location
    krane = {}
    # decision variable for a crane type at a location
    ay = {}
    # auxiliary variables /indicator variables
    aw = {}
    # another set of auxiliary variables/indicator variables

    # Constants to be determined
    cost_per_minute = {}
    # cost vector for cranes
    time = {}

    # List of variables. Better to use list in gurobi, rather than the pandas series.
    l_lst = craneloc['ind'].tolist()  # location
    k_lst = cranetype['ind'].tolist()  # crane
    i_lst = demand['ind'].tolist()  # demand
    j_lst = supply['ind'].tolist()  # supply

    for l in l_lst:
        for k in k_lst:
            strname = "CL_" + str(l) + '_' + str(k)
            krane[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname)
            strname1 = "D1_" + str(l) + '_' + str(k)
            strname2 = "D2_" + str(l) + '_' + str(k)
            aw[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname1)
            ay[l, k] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strname2)
            for i in i_lst:
                for j in j_lst:
                    strnameq = "Q_" + str(l) + "_" + str(k) + "_" + str(i) + "_" + str(j)
                    Q[l, k, i, j] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=strnameq)

    m.update()

    # Calculate Tijkl... the time taken to traverse
    for ind1, locrow in craneloc.iterrows():
        for ind2, typerow in cranetype.iterrows():
            for ind3, demandrow in demand.iterrows():
                for ind4, supplyrow in supply.iterrows():
                    vw = typerow['Vw']
                    vh = typerow['Vh']
                    vz = typerow['Vz']
                    cop = typerow['Ht']
                    rad = typerow['maxR']
                    dx = demandrow['X']
                    dy = demandrow['Y']
                    dz = demandrow['Z']
                    cx = locrow['X']
                    cy = locrow['Y']
                    cz = locrow['Z']
                    sx = supplyrow['X']
                    sy = supplyrow['Y']
                    sz = supplyrow['Z']
                    time[locrow['ind'], typerow['ind'], demandrow['ind'], supplyrow['ind']] = \
                        calc_time(vw, vh, vz, rad, dx, dy, dz, cx, cy, cz+cop, sx, sy, sz)

    # Determine the cost per minute vector
    for l in l_lst:
        for k in k_lst:
            cranecost = cranetype.loc[cranetype.ind == k, 'Cost'].values[0]  # use values to acces the resultant, which is a dict. But inside the dict is still a list. So use [0] to access the value. Fucked up.
            for i in i_lst:
                for j in j_lst:
                    cost_per_minute[l, k, i, j] = cranecost / 2400  # 43800 minutes per month, and doesn't consider working time
                    # Cost is now cost/week. So it will be 5 days * 8 hours * 60min = 2400mins per week
                    #to be honest i don't need ijlk. Only k. this is more for programming convenience.

    fixedcost = {}
    for l in l_lst:
        for ind2, typerow in cranetype.iterrows():
            fixedcost[l, typerow['ind']] = typerow['Fixed']

    # Objective Function
    ### not included... obj = quicksum(krane[l, k] * fixedcost[l, k] for l in l_lst for k in k_lst)
    obj = 0
    for l in l_lst:
        for k in k_lst:
            for i in i_lst:
                for j in j_lst:
                    obj += Q[l, k, i, j] * time[l, k, i, j] # * cost_per_minute[l, k, i, j]
    m.setObjective(obj, GRB.MINIMIZE)

    # Constraints
    # For every demand point, there is ONLY one crane lifting it to from any supply point from any crane location
    for i in i_lst:
        m.addConstr(quicksum(Q[l, k, i, j] for l in l_lst for k in k_lst for j in j_lst), GRB.EQUAL, 1, "assignment")

    # Matching krane with assignment
    for l in l_lst:
        for k in k_lst:
            M = 100000000  # arbitraily large
            qtot = 0
            for i in i_lst:
                for j in j_lst:
                    qtot += Q[l, k, i, j]
            m.addConstr(qtot - M * ay[l, k], GRB.LESS_EQUAL, 0, "match_crane")
            m.addConstr(krane[l, k] + M * (1 - aw[l, k]), GRB.GREATER_EQUAL, 1)
            m.addConstr(ay[l, k] - aw[l, k], GRB.LESS_EQUAL, 0)

    # Constraint for ensuring only one crane per location
    for l in l_lst:
        krane_alloc = 0
        for k in k_lst:
            krane_alloc += krane[l, k]
        m.addConstr(krane_alloc, GRB.LESS_EQUAL, 1)

    # Constraint on the numer of cranes
    m.addConstr(quicksum(krane[l, k] for l in l_lst for k in k_lst), GRB.LESS_EQUAL, 6)

    # Run the Model
    m.optimize()

    # Output the results
    m.write("phase_a_costless.sol")


def solve_scp(demand, supply, craneloc, cranetype):

    # Model
    m = Model("SCP")

    # Variable Declaration
    krane = {}
    # decision variable for a set which is equal to the crane type(k) at a location(l) at a phase(p)

    # Constants to be determined
    total_cost = {}
    # cost vector for cranes


    # Step 1: Update Demand and Supply points
    aug_supply = augment_phases(supply)
    aug_supply = reindex(aug_supply, "j")
    aug_demand = augment_phases(demand)
    aug_demand = reindex(aug_demand, "i")

    # Step 2: Merge crane and location to form kl_list:
    # pandas merge the two databases
    aug_craneloc = augment_phases_powerset(craneloc)
    aug_cranetype = augment_phases_powerset(cranetype)
    aug_craneloc['key'] = 0
    aug_cranetype['key'] = 0
    kl_crane = pd.merge(aug_craneloc, aug_cranetype, on='key')
    kl_crane.drop('key', 1, inplace=True)  # 1 refers to dropping the column, not the row
    kl_crane = kl_crane[kl_crane.Phase_x == kl_crane.Phase_y]  # Nice way of removing rows where Phase x is equal to Phase y. rewrite back to kl_crane
    kl_crane['Rem'] = "F"
    # kl_crane['ind'] = kl_crane['ind_x'] + kl_crane['ind_y']  # Create a new column based on ind_x and ind_y
    ### rationale: K(crane type) and L(location) are merged together to form all possible combinations for crane and location.
    ### Each crane and location has a set of available phases. We take the powerset of these to enumerate possible time combinations.
    ### PErmissible time combinations are when the availability of crane and location concur.

    # Step 3: Construct set via feasible set membership (i.e. crane must be able to handle the demand). Output matrix A
    # STep 3a: Eliminate all crane-loc which cannot meet the supply for all phases. In other words, for every phase in the crane-loc combi,
    for ind5, klrow in kl_crane.iterrows():
        klphase = klrow['Phase_x']
        klx = klrow['X']
        kly = klrow['Y']
        klr = klrow['maxR']
        truthindicator1 = []
        for phase in klphase:
            truthindicator2 = []
            for ind6, augsupplyrow in aug_supply.iterrows():
                sup_phase = augsupplyrow['Phase']
                sup_x = augsupplyrow['X']
                sup_y = augsupplyrow['Y']
                if ((klx-sup_x)**2 + (kly-sup_y)**2) <= (klr**2) and phase == sup_phase:
                    truthindicator2.append('Pass')
                else:
                    truthindicator2.append('Fail')
                # If truthindicator2 is fail, it fails the test of phase or radius.
            if 'Pass' in truthindicator2:
                truthindicator1.append('True')
            else:
                truthindicator1.append('False')
                # if any one of the truthindicator2 is 'fail'. That means that one phase condition fails, and cannot
                # proceed: False for truthindicator 1.
        if 'False' in truthindicator1:
            kl_crane.loc[ind5, 'Rem'] = 'T'
                # if truthindicator 1 is true for any of its elements, then it means that there exists a possible
                # crane combination to proceed

    kl_crane = kl_crane[kl_crane.Rem == 'F']  # remove impossible combinations
    kl_crane.index = range(len(kl_crane))  # df.reindex does not mean create a new index. it just means filter from index! damn you pandas
    # reindex kl_crane
    for ind5, klrow in kl_crane.iterrows():
        klcost = klrow['Cost']
        klfixed = klrow['Fixed']
        klphase = klrow['Phase_x']
        klstr = klrow['ind_x'] + '_' + klrow['ind_y'] + '_' + strlist(klphase) + '_' + str(ind5)
        krane[ind5] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=klstr)
        total_cost[ind5] = calc_cost(klcost, klfixed, klphase)

    m.update()

    # Populate A matrix
    A = []
    for ind3, demandrow in aug_demand.iterrows():
        dx = demandrow['X']
        dy = demandrow['Y']
        dz = demandrow['Z']
        dw = demandrow['Vol']
        dp = demandrow['Phase']
        B = []
        for ind5, klrow in kl_crane.iterrows():
            klx = klrow['X']
            kly = klrow['Y']
            klz = klrow['Z']
            klr = klrow['maxR']
            klphase = klrow['Phase_x']
            klm = klrow['max_moment']
            klht = klrow['Ht']
            B.append(is_set_member(dx, dy, dz, dw, dp, klx, kly, klz, klr, klphase, klm, klht))
        A.append(B)

    # feasibility check
    for i in range(len(aug_demand)):
        C = set(A[i])
        if len(C) == 1 and 0 in C:
            print('Demand pt %s is not feasible' % i)

    # enter the constraints into the model
    for i in range(len(aug_demand)):
        expr = LinExpr()
        for j in range(len(krane)):
            if A[i][j] != 0:
                expr += A[i][j]*krane[j]
        m.addConstr(expr, GRB.GREATER_EQUAL, 1)
    '''
    # only one crane per location
    craneindexlist = list(craneloc['ind'])
    for i in craneindexlist:
        expr = LinExpr()
        for j in krane:
            if i in krane[j].getAttr("VarName"):
                expr += krane[j]
        m.addConstr(expr, GRB.LESS_EQUAL, 1)
    '''

    m.setObjective(quicksum(krane[i]*total_cost[i] for i in range(len(krane))), GRB.MINIMIZE)

    m.optimize()

    ''' maybe just time out this one for now.
    time = {}
    # Calculate Tijkl... the time taken to traverse
    for ind1, locrow in craneloc.iterrows():
        for ind2, typerow in cranetype.iterrows():
            for ind3, demandrow in demand.iterrows():
                for ind4, supplyrow in supply.iterrows():
                    vw = typerow['Vw']
                    vh = typerow['Vh']
                    vz = typerow['Vz']
                    rad = typerow['maxR']
                    dx = demandrow['X']
                    dy = demandrow['Y']
                    dz = demandrow['Z']
                    cx = locrow['X']
                    cy = locrow['Y']
                    cz = locrow['Z']
                    sx = supplyrow['X']
                    sy = supplyrow['Y']
                    sz = supplyrow['Z']
                    time[locrow['ind'], typerow['ind'], demandrow['ind'], supplyrow['ind']] = \
                        calc_time(vw, vh, vz, rad, dx, dy, dz, cx, cy, cz, sx, sy, sz)

    # Retrieve cost per month vector
    '''

    m.write('scp_bca_3phase.sol') #remember to change the file name for every phase/test


def is_set_member(dx, dy, dz, dw, dp, klx, kly, klz, klr, klphase, klm, klht):
    # Determine if a point is part of a set, and return true or false.
    dist = (dx - klx)**2 + (dy - kly)**2
    if dist < klr**2 and (klht + klz >= dz) and nested(dp, klphase):  # and dist*dw <= klm  # moment consideration # height (crane z location + height under hook (klht must be greater than demand z location)
        return 1
    else:
        return 0


# Nice code for checking if an element exists in a nested list... which is what is potentially possible in klphase
def flatten(lst):
    for elem in lst:
        if isinstance(elem, (list, tuple)):
            for nested in flatten(elem):
                yield nested
        else:
            yield elem


def nested(x, ys):
    return any(x == nested for nested in flatten(ys))


def list_powerset(lst):
    # the power set of the empty set has one element, the empty set
    result = [[]]
    for x in lst:
        # for every additional element in our set
        # the power set consists of the subsets that don't
        # contain this element (just take the previous power set)
        # plus the subsets that do contain the element (use list
        # comprehension to add [x] onto everything in the
        # previous power set)
        result.extend([subset + [x] for subset in result])
    return result


def augment_phases(data_frame):
    # return a copy of the dataframe but wi the multiple phases broken up into single phases.
    # e.g. Phase abc = Phase a, Phase b, Phase c... 3 separate entries in the dataframe from the original entry.
    cols = data_frame.columns
    demand1 = pd.DataFrame(columns=list(cols))
    for ind, datarow in data_frame.iterrows():
        if len(datarow['Phase']) == 1:
            srow = data_frame.iloc[ind]
            demand1 = demand1.append(srow, ignore_index=True)  # need to store the output... if not it will still be empty
        else:
            #pslist = list_powerset(list(demandrow['Phase']))  # Works but not what is needed here.
            #pslist.remove([])  # Works here but not what is needed.
            pslist = list(datarow['Phase'])  # split into a list of characters
            for numstr in range(len(pslist)):
                srow = data_frame.iloc[ind].copy()
                srow['Phase'] = pslist[numstr]
                demand1 = demand1.append(srow, ignore_index=True)
    return demand1


def augment_phases_powerset(data_frame):
    cols = data_frame.columns
    demand1 = pd.DataFrame(columns=list(cols))
    for ind, datarow in data_frame.iterrows():
        if len(datarow['Phase']) == 1:
            srow = data_frame.iloc[ind]
            demand1 = demand1.append(srow, ignore_index=True)  # need to store the output... if not it will still be empty
        else:
            pslist = list_powerset(list(datarow['Phase']))
            pslist.remove([])
            for numstr in range(len(pslist)):
                srow = data_frame.iloc[ind].copy()
                srow['Phase'] = pslist[numstr]
                demand1 = demand1.append(srow, ignore_index=True)
    return demand1


def reindex(data_frame, strind):
    for numstr in range(len(data_frame)):
        # data_frame['ind'].iloc[numstr] = strind + str(numstr + 1) raises a setting with copy warning... because of the 'ind'
        data_frame.loc[numstr, 'ind'] = strind + str(numstr + 1)  # much better... setting with copy arises because of wrong use of the iloc.
    return data_frame


def calc_cost(klcost, klfixed, klphase):
    # Predefined: Length of Phases a, b, c etc. Undefined phases are 'free'.

    cost = 0
    if nested('a', klphase):
        cost += klcost * 7  # phase a takes 4 weeks... klcost is cost per week
    if nested('b', klphase):
        cost += klcost * 11  # phase b takes 8 weeks... klcost is cost per week
    if nested('c', klphase):
        cost += klcost * 33
    # to determine number of times to install and dismantle: take the difference of two lists. Each list converted into
    # a set to make each element unique. The first list is the total number of phases available: [a, b, c, d] etc.
    # Subtract the two lists, and find the length of the new list. Add one to the len of this list to get number of
    # install or dismantle.

    original_schedule = ['a', 'b', 'c']      # remember to change this as well...
    start = original_schedule.index(klphase[0])
    stop = original_schedule.index(klphase[-1])
    trunc_sched = original_schedule[start:stop + 1]
    multiples = len(list(set(trunc_sched) - set(klphase))) + 1

    cost += multiples * klfixed

    return cost


def strlist(a):
    str1 = ''.join(a)
    return str1


def solve_scp_new(demand, supply, craneloc, cranetype, filestr, phasedict):

    # Model
    m = Model("SCP")

    # Variable Declaration
    krane = {}
    # decision variable for a set which is equal to the crane type(k) at a location(l) at a phase(p)

    # Constants to be determined
    total_cost = {}
    # cost vector for cranes


    # Step 1: Update Demand and Supply points
    aug_supply = augment_phases(supply)
    aug_supply = reindex(aug_supply, "j")
    aug_demand = augment_phases(demand)
    aug_demand = reindex(aug_demand, "i")

    # Step 2: Merge crane and location to form kl_list:
    # pandas merge the two databases
    aug_craneloc = augment_phases_powerset(craneloc)
    aug_cranetype = augment_phases_powerset(cranetype)
    aug_craneloc['key'] = 0
    aug_cranetype['key'] = 0
    kl_crane = pd.merge(aug_craneloc, aug_cranetype, on='key')
    kl_crane.drop('key', 1, inplace=True)  # 1 refers to dropping the column, not the row
    kl_crane = kl_crane[kl_crane.Phase_x == kl_crane.Phase_y]  # Nice way of removing rows where Phase x is equal to Phase y. rewrite back to kl_crane
    kl_crane['Rem'] = "F"
    # kl_crane['ind'] = kl_crane['ind_x'] + kl_crane['ind_y']  # Create a new column based on ind_x and ind_y
    ### rationale: K(crane type) and L(location) are merged together to form all possible combinations for crane and location.
    ### Each crane and location has a set of available phases. We take the powerset of these to enumerate possible time combinations.
    ### PErmissible time combinations are when the availability of crane and location concur.

    # Step 3: Construct set via feasible set membership (i.e. crane must be able to handle the demand). Output matrix A
    # STep 3a: Eliminate all crane-loc which cannot meet the supply for all phases. In other words, for every phase in the crane-loc combi,
    for ind5, klrow in kl_crane.iterrows():
        klphase = klrow['Phase_x']
        klx = klrow['X']
        kly = klrow['Y']
        klr = klrow['maxR']
        truthindicator1 = []
        for phase in klphase:
            truthindicator2 = []
            for ind6, augsupplyrow in aug_supply.iterrows():
                sup_phase = augsupplyrow['Phase']
                sup_x = augsupplyrow['X']
                sup_y = augsupplyrow['Y']
                if ((klx-sup_x)**2 + (kly-sup_y)**2) <= (klr**2) and phase == sup_phase:
                    truthindicator2.append('Pass')
                else:
                    truthindicator2.append('Fail')
                # If truthindicator2 is fail, it fails the test of phase or radius.
            if 'Pass' in truthindicator2:
                truthindicator1.append('True')
            else:
                truthindicator1.append('False')
                # if any one of the truthindicator2 is 'fail'. That means that one phase condition fails, and cannot
                # proceed: False for truthindicator 1.
        if 'False' in truthindicator1:
            kl_crane.loc[ind5, 'Rem'] = 'T'
                # if truthindicator 1 is true for any of its elements, then it means that there exists a possible
                # crane combination to proceed

    kl_crane = kl_crane[kl_crane.Rem == 'F']  # remove impossible combinations
    kl_crane.index = range(len(kl_crane))  # df.reindex does not mean create a new index. it just means filter from index! damn you pandas
    # reindex kl_crane
    for ind5, klrow in kl_crane.iterrows():
        klcost = klrow['Cost']
        klfixed = klrow['Fixed']
        klphase = klrow['Phase_x']
        klstr = klrow['ind_x'] + '_' + klrow['ind_y'] + '_' + strlist(klphase) + '_' + str(ind5)
        krane[ind5] = m.addVar(lb=0.0, ub=1.0, vtype=GRB.BINARY, name=klstr)
        total_cost[ind5] = calc_cost1(klcost, klfixed, klphase, phasedict)

    m.update()

    # Populate A matrix
    A = []
    for ind3, demandrow in aug_demand.iterrows():
        dx = demandrow['X']
        dy = demandrow['Y']
        dz = demandrow['Z']
        dw = demandrow['Vol']
        dp = demandrow['Phase']
        B = []
        for ind5, klrow in kl_crane.iterrows():
            klx = klrow['X']
            kly = klrow['Y']
            klz = klrow['Z']
            klr = klrow['maxR']
            klphase = klrow['Phase_x']
            klm = klrow['max_moment']
            klht = klrow['Ht']
            B.append(is_set_member(dx, dy, dz, dw, dp, klx, kly, klz, klr, klphase, klm, klht))
        A.append(B)

    # feasibility check
    for i in range(len(aug_demand)):
        C = set(A[i])
        if len(C) == 1 and 0 in C:
            print('Demand pt %s is not feasible' % i)

    # enter the constraints into the model
    for i in range(len(aug_demand)):
        expr = LinExpr()
        for j in range(len(krane)):
            if A[i][j] != 0:
                expr += A[i][j]*krane[j]
        m.addConstr(expr, GRB.GREATER_EQUAL, 1)
    '''
    # only one crane per location
    craneindexlist = list(craneloc['ind'])
    for i in craneindexlist:
        expr = LinExpr()
        for j in krane:
            if i in krane[j].getAttr("VarName"):
                expr += krane[j]
        m.addConstr(expr, GRB.LESS_EQUAL, 1)
    '''

    m.setObjective(quicksum(krane[i]*total_cost[i] for i in range(len(krane))), GRB.MINIMIZE)

    m.optimize()

    ''' maybe just time out this one for now.
    time = {}
    # Calculate Tijkl... the time taken to traverse
    for ind1, locrow in craneloc.iterrows():
        for ind2, typerow in cranetype.iterrows():
            for ind3, demandrow in demand.iterrows():
                for ind4, supplyrow in supply.iterrows():
                    vw = typerow['Vw']
                    vh = typerow['Vh']
                    vz = typerow['Vz']
                    rad = typerow['maxR']
                    dx = demandrow['X']
                    dy = demandrow['Y']
                    dz = demandrow['Z']
                    cx = locrow['X']
                    cy = locrow['Y']
                    cz = locrow['Z']
                    sx = supplyrow['X']
                    sy = supplyrow['Y']
                    sz = supplyrow['Z']
                    time[locrow['ind'], typerow['ind'], demandrow['ind'], supplyrow['ind']] = \
                        calc_time(vw, vh, vz, rad, dx, dy, dz, cx, cy, cz, sx, sy, sz)

    # Retrieve cost per month vector
    '''

    m.write(filestr)


def calc_cost1(klcost, klfixed, klphase, phasedict):
    # Predefined: Length of Phases a, b, c etc. Undefined phases are 'free'.
    # phasedict is a dictionary with the following: {'a':duration of a, 'b':duration of b, 'c':duration of c}

    cost = 0
    for i, j in phasedict.items():
        if nested(i, klphase):
            cost += klcost * j

    '''
    if nested('a', klphase):
        cost += klcost * 7  # phase a takes 4 weeks... klcost is cost per week
    if nested('b', klphase):
        cost += klcost * 11  # phase b takes 8 weeks... klcost is cost per week
    if nested('c', klphase):
        cost += klcost * 33
    '''
    # to determine number of times to install and dismantle: take the difference of two lists. Each list converted into
    # a set to make each element unique. The first list is the total number of phases available: [a, b, c, d] etc.
    # Subtract the two lists, and find the length of the new list. Add one to the len of this list to get number of
    # install or dismantle.

    original_schedule = ['a', 'b', 'c']      # remember to change this as well... use this for ABC, ACB, BAC, BCA, CBA, CAB

    #original_schedule = list(phasedict.keys()) # this has some error. dictionary has no order. So it keeps jumping around.
    start = original_schedule.index(klphase[0])
    stop = original_schedule.index(klphase[-1])
    trunc_sched = original_schedule[start:stop + 1]
    multiples = len(list(set(trunc_sched) - set(klphase))) + 1

    cost += multiples * klfixed

    return cost
