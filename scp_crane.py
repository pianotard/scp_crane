from gurobipy import *
import pandas as pd

DEBUG = False

def solve_scp(demand_points_df, supply_points_df, crane_locs_df, cranes_df, filestr, phasedict, verbose = False):
    
    crane_ids = list(cranes_df['ind'].unique())
    loc_ids = list(crane_locs_df['ind'].unique())

    crane_list = cranes_df.to_dict(orient = 'split')['data']
    loc_list = crane_locs_df.to_dict(orient = 'split')['data']
    
    if verbose: print('Reading data..', end = '')
    kl_dict = {
        crane_id: {
            'loc_ids': {
                loc_id: {
                    'x': 0,
                    'y': 0,
                    'z': 0,
                } for loc_id in loc_ids
            },
            'max_r': 0,
            'height': 0,
            'max_moment': 0,
            'cost': 0,
            'fixed': 0,
            'phases': None
        } for crane_id in crane_ids
    }

    for crane in crane_list:
        crane_id = crane[cranes_df.columns.get_loc('ind')]
        max_r = crane[cranes_df.columns.get_loc('maxR')]
        max_moment = crane[cranes_df.columns.get_loc('max_moment')]
        height = crane[cranes_df.columns.get_loc('Ht')]
        crane_phases = crane[cranes_df.columns.get_loc('Phase')]
        cost = crane[cranes_df.columns.get_loc('Cost')]
        fixed = crane[cranes_df.columns.get_loc('Fixed')]

        kl_dict[crane_id]['max_r'] = max_r
        kl_dict[crane_id]['max_moment'] = max_moment
        kl_dict[crane_id]['height'] = height
        kl_dict[crane_id]['phases'] = crane_phases
        kl_dict[crane_id]['cost'] = cost
        kl_dict[crane_id]['fixed'] = fixed

        for loc in loc_list:
            loc_id = loc[crane_locs_df.columns.get_loc('ind')]
            x = loc[crane_locs_df.columns.get_loc('X')]
            y = loc[crane_locs_df.columns.get_loc('Y')]
            z = loc[crane_locs_df.columns.get_loc('Z')]
            loc_phases = loc[crane_locs_df.columns.get_loc('Phase')]

            assert loc_phases == crane_phases, f'Diff phases provided!'

            kl_dict[crane_id]['loc_ids'][loc_id]['x'] = x
            kl_dict[crane_id]['loc_ids'][loc_id]['y'] = y    
            kl_dict[crane_id]['loc_ids'][loc_id]['z'] = z 

    if verbose: print('Done')
    if DEBUG: print(kl_dict)
    
    def pass_supply(loc, maxR, phases):
        xyz = (supply_points_df['X'].astype(float) - loc['x']) ** 2 + (supply_points_df['Y'].astype(float) - loc['y']) ** 2 < maxR ** 2
        phase = supply_points_df['Phase'].apply(lambda sp: any(p in sp for p in phases))
        return any(xyz & phase)
    
    # Check supply constraints    
    if verbose: print('Checking supply constraints..', end = '')
    kls_dict = {
        crane_id: {
            'loc_ids': {
                loc_id: kl_dict[crane_id]['loc_ids'][loc_id] for loc_id in loc_ids \
                if pass_supply(kl_dict[crane_id]['loc_ids'][loc_id], kl_dict[crane_id]['max_r'], kl_dict[crane_id]['phases'])
            },
            'max_r': kl_dict[crane_id]['max_r'],
            'height': kl_dict[crane_id]['height'],
            'max_moment': kl_dict[crane_id]['max_moment'],
            'cost': kl_dict[crane_id]['cost'],
            'fixed': kl_dict[crane_id]['fixed'],
            'phases': kl_dict[crane_id]['phases']
        } for crane_id in kl_dict.keys() 
    }

    if verbose: print('Done')
    if DEBUG: print(kls_dict)

    if verbose: 
        prev_len = sum([len(v['loc_ids']) * len(list_powerset(v['phases'])[1:]) for v in kl_dict.values()])
        new_len = sum([len(v['loc_ids']) * len(list_powerset(v['phases'])[1:]) for v in kls_dict.values()])
        print(f'Removed {(prev_len - new_len)} crane-loc-phases that fail supply constraints')

    def passed_demand_points(loc, maxR, height, max_moment, phase):
        """
        :param loc: dict of {x, y, z}
        :param phase: a str of phases
        """
        c_sq = [(dx - loc['x']) ** 2 + (dy - loc['y']) ** 2 \
                for dx, dy in zip(demand_points_df['X'], demand_points_df['Y'])]
        pass_dist = [dist < maxR ** 2 for dist in c_sq]
        pass_height = [loc['z'] + height >= dz for dz in demand_points_df['Z']]
        pass_moment = [dist * v <= max_moment for dist, v in zip(c_sq, demand_points_df['Vol'])]
        pass_phase = [any(p in dp for p in phase) for dp in demand_points_df['Phase']]
        return [d and h and m and p for d, h, m, p in zip(pass_dist, pass_height, pass_moment, pass_phase)]

    # Check demand and moment constraints
    if verbose: print('Checking demand, moment and phase constraints..', end = '')
    klsd_dict = {
        crane_id: {
            'loc_ids': {loc_id: {
                    phase: passed_demand_points(kls_dict[crane_id]['loc_ids'][loc_id], kls_dict[crane_id]['max_r'], \
                        kls_dict[crane_id]['height'], kls_dict[crane_id]['max_moment'], phase) \
                    for phase in kls_dict[crane_id]['phases']
                } for loc_id in kls_dict[crane_id]['loc_ids'].keys()
            },
            'cost': kls_dict[crane_id]['cost'],
            'fixed': kls_dict[crane_id]['fixed'],
            'phases': kls_dict[crane_id]['phases']
        } for crane_id in kls_dict.keys()
    }

    if verbose: print('Done')
    if DEBUG: print(klsd_dict)
        
    if verbose: 
        prev_len = sum([len(v['loc_ids']) * len(list_powerset(v['phases'])[1:]) for v in kls_dict.values()])
        new_len = sum([len(v['loc_ids']) * len(list_powerset(v['phases'])[1:]) for v in klsd_dict.values()])
        print(f'Removed {(prev_len - new_len)} crane-loc-phases that fail demand constraints')

    def union_phases(demand_flags, phases):
        """
        Returns union of demand_flags according to phases
        :param demand_flags: dict of list of boolean flags for crane-loc {phase: [True or False]}
        :param phases: list of phases to union demand_flags on
        """
        if len(phases) == 1:
            return demand_flags[phases[0]]

        union = demand_flags[phases[0]]
        for phase in phases[1:]:
            union = [e1 or e2 for e1, e2 in zip(union, demand_flags[phase])]
        return union

    # Check phase constraint
    if verbose: print('Taking powerset of phases..', end = '')
    klsdp_dict = {
        crane_id: {
            'loc_ids': {
                loc_id: {
                    ''.join(phase): union_phases(klsd_dict[crane_id]['loc_ids'][loc_id], phase) \
                    for phase in list_powerset(klsd_dict[crane_id]['phases'])[1:]
                } for loc_id in klsd_dict[crane_id]['loc_ids'].keys()
            },
            'cost': klsd_dict[crane_id]['cost'],
            'fixed': klsd_dict[crane_id]['fixed'],
        } for crane_id in klsd_dict.keys()
    }
    if verbose: print('Done')
    if DEBUG: print(klsdp_dict)
    
    if verbose: 
        passed = sum(
            [sum(
                [sum(
                    [sum(
                        klsdp_dict[k]['loc_ids'][l][p]) for p in klsdp_dict[k]['loc_ids'][l].keys()]) \
                        for l in klsdp_dict[k]['loc_ids'].keys()]) \
                        for k in klsdp_dict.keys()]
        )
        print(f'Total valid crane-loc-phases: {passed}')

    if verbose: print('Binarizing boolean values..', end = '')
    def binarize(l):
        """
        Converts list of boolean to list of 1's and 0's
        """
        return [1 if e else 0 for e in l]

    kl_model = {
        crane_id: {
            loc_id: {
                phase: binarize(klsdp_dict[crane_id]['loc_ids'][loc_id][phase]) \
                for phase in klsdp_dict[crane_id]['loc_ids'][loc_id].keys()
            } for loc_id in klsdp_dict[crane_id]['loc_ids'].keys()
        } for crane_id in klsdp_dict.keys()
    }
    if verbose: print('Done')
    
    m = Model("SCP")

    if verbose: print('Computing costs..', end = '')

    krane = []
    total_cost = []
    A = []
    for crane_id, d1 in klsdp_dict.items():
        cost = d1['cost']
        fixed = d1['fixed']
        for loc_id, d2 in d1['loc_ids'].items():
            for phase, demand_flags in d2.items():
                klstr = loc_id + '_' + crane_id + '_' + ''.join(phase) + '_' + str(len(krane))
                krane.append(m.addVar(lb = 0.0, ub = 1.0, vtype = GRB.BINARY, name = klstr))
                total_cost.append(calc_cost1(cost, fixed, phase, phasedict))
                A.append(demand_flags)

    if verbose: print('Done\nAdding constraints..', end = '')
    if DEBUG: print(len(A))
    m.update()

    for i in range(len(A[0])):
        expr = LinExpr()
        for j in range(len(A)):
            if A[j][i] != 0:
                expr += A[j][i] * krane[j]
        m.addConstr(expr, GRB.GREATER_EQUAL, 1)

    m.setObjective(quicksum(krane[i] * total_cost[i] for i in range(len(krane))), GRB.MINIMIZE)
    if verbose: print('Done\nStart optimize..', end = '')
    m.optimize()
    if verbose: print('Done')
    m.write(filestr)
    
def list_powerset(lst):
    # the power set of the empty set has one element, the empty set
    result = [[]]
    for x in lst:
        result.extend([subset + [x] for subset in result])
    return result

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