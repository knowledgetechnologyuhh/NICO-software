# coding= utf8
import copy
import random
import time  # FIXME remove
from multiprocessing.dummy import Pool as Threadpool

import math3d as m3d
import numpy as np
import scipy.optimize

from . import logs


def inverse_kinematic_optimization(chain, target_frame, starting_nodes_angles, regularization_parameter=None,
                                   max_iter=None, second_chain=None, second_target_frame=None,
                                   include_orientation=False, method="L-BFGS-B", orientation_weight=None):
    """Computes the inverse kinematic on the specified target with an optimization method

    :param ikpy.chain.Chain chain: The chain used for the Inverse kinematics.
    :param numpy.array target: The desired target.
    :param numpy.array starting_nodes_angles: The initial pose of your chain.
    :param float regularization_parameter: The coefficient of the regularization.
    :param int max_iter: Maximum number of iterations for the optimisation algorithm.
    """
    # Only get the position
    target = target_frame[:3, 3]

    if orientation_weight == None:
        orientation_weight = 0.1

    # Get the second target frame if there is one
    if second_target_frame is not None:
        second_target = second_target_frame[:3, 3]

    if starting_nodes_angles is None:
        raise ValueError("starting_nodes_angles must be specified")

    # Compute squared distance to target
    def optimize_target(x):
        # y = np.append(starting_nodes_angles[:chain.first_active_joint], x)
        y = chain.active_to_full(x, starting_nodes_angles)
        squared_distance = np.linalg.norm(
            chain.forward_kinematics(y)[:3, -1] - target)
        # print "Position distance: " + str(squared_distance)
        return squared_distance

    # If a regularization is selected
    if regularization_parameter is not None:
        def optimize_total(x):
            regularization = np.linalg.norm(
                x - starting_nodes_angles[chain.first_active_joint:])
            return optimize_target(x) + regularization_parameter * regularization

    if second_chain is not None:

        initial_second_position = [0] * len(second_chain.links)

        def optimize_total(x):
            # y = np.append(starting_nodes_angles[:chain.first_active_joint], x)
            y1 = chain.active_to_full(x, starting_nodes_angles)
            squared_distance_1 = np.linalg.norm(
                chain.forward_kinematics(y1)[:3, -1] - target)
            y2 = second_chain.active_to_full(x, initial_second_position)
            # squared_distance_2 = np.linalg.norm(second_chain.forward_kinematics(y2)[:3, -1] - second_target)
            squared_distance_2 = np.linalg.norm(second_chain.forward_kinematics(y2)[
                                                :3, -1] - second_target) * 2
            return squared_distance_1 + squared_distance_2

    else:
        # If orientation should be included
        if include_orientation:

            # Calculate UnitQuaternion for target
            targetQuad = m3d.quaternion.UnitQuaternion(
                m3d.orientation.Orientation(target_frame[:3, :3]))

            def optimize_total(x):
                # Calculate position distance
                y = chain.active_to_full(x, starting_nodes_angles)
                fwk = chain.forward_kinematics(y)
                euclidean_distance = np.linalg.norm(fwk[:3, -1] - target)
                #euclidean_distance=scipy.spatial.distance.euclidean(fwk[:3, -1] - target)
                # print "Position distance: " + str(squared_distance)

                # Calculate orientation distance
                import transforms3d
                import math
                # k=transforms3d.euler.EulerFuncs()
                #i_al, i_be, i_ga = transforms3d.taitbryan.mat2euler(fwk[:3, :3])
                #t_al, t_be, t_ga = transforms3d.taitbryan.mat2euler(target_frame[:3, :3])
                #d_al = i_al - t_al
                #d_be = i_be - t_be
                #d_ga = i_ga - t_ga
                #orientation_distance = math.sqrt(d_al * d_al + d_be * d_be + d_ga * d_ga)
                recentQuad = m3d.quaternion.UnitQuaternion(
                    m3d.orientation.Orientation(fwk[:3, :3]))
                orientation_distance = targetQuad.dist_squared(recentQuad)
                # print "Orientation distance: " + str(orientation_distance)

                # return (squared_distance * (1 - orientation_weight) + orientation_distance * orientation_weight)
                return (euclidean_distance * (1 - orientation_weight) + orientation_distance * orientation_weight)

            # return (orientation_distance)

        else:
            def optimize_total(x):
                return optimize_target(x)

    # Compute bounds
    real_bounds = [link.bounds for link in chain.links]
    # real_bounds = real_bounds[chain.first_active_joint:]
    real_bounds = chain.active_from_full(real_bounds)

    options = {}
    # max_iter=1000
    # Manage iterations maximum
    if max_iter is not None:
        options["maxiter"] = max_iter
    # options["gtol"] = 1e-6
    # options["disp"] = True

    # Utilisation d'une optimisation L-BFGS-B
    # print " Real bounds: " + str(real_bounds)
    if (method == "SLSQP"):
        res = scipy.optimize.minimize(optimize_total, chain.active_from_full(starting_nodes_angles), method='SLSQP',
                                      bounds=real_bounds, options=options)
    else:
        res = scipy.optimize.minimize(optimize_total, chain.active_from_full(starting_nodes_angles), method='L-BFGS-B',
                                      bounds=real_bounds, options=options)

    return (chain.active_to_full(res.x, starting_nodes_angles))


def inverse_kinematic_optimization_multi(chain, j_targets, starting_nodes_angles,
                                         max_iter=None, second_target_frame=None,
                                         method="L-BFGS-B"):
    """Computes the inverse kinematic on the specified target with an optimization method

    :param ikpy.chain.Chain chain: The chain used for the Inverse kinematics.
    :param numpy.array target: The desired target.
    :param numpy.array starting_nodes_angles: The initial pose of your chain.
    :param float regularization_parameter: The coefficient of the regularization.
    :param int max_iter: Maximum number of iterations for the optimisation algorithm.
    """
    # Only get the position
    #target = target_frame[:3, 3]

    def optimize_total(x):

        y1 = chain.active_to_full(x, starting_nodes_angles)
        # calculate matrices of the joints
        a_matrices = chain.forward_kinematics(y1, full_kinematics=True)

        # Calculate the difference to all given joints to targets
        # j_targets must be in the format [(number_of_first_joint_to_target,target_of_the_first_joint [:3, 3]-format),
        #                                 (number_of_second_joint_to_target,target_of_the_second_joint [:3, 3]-format),
        #                                   (number_of_third_joint_to_target,target_of_the_third_joint [:3, 3]-format)]
        squared_distances = []
        for j_target in j_targets:
            joint_number, target = j_target
            squared_distances.append(np.linalg.norm(
                a_matrices[joint_number][:3, -1] - target))
            #print str(squared_distances)

        # Sum up all distances
        squared_distance_sum = 0
        for squared_distance in squared_distances:
            squared_distance_sum += squared_distance

        # squared_distance_2 = np.linalg.norm(second_chain.forward_kinematics(y2)[:3, -1] - second_target)
        #squared_distance_2 = np.linalg.norm(second_chain.forward_kinematics(y2)[:3, -1] - second_target) * 2
        # return squared_distance_1 + squared_distance_2
        return squared_distance_sum

    # Compute bounds
    real_bounds = [link.bounds for link in chain.links]
    # real_bounds = real_bounds[chain.first_active_joint:]
    real_bounds = chain.active_from_full(real_bounds)

    options = {}
    # max_iter=1000
    # Manage iterations maximum
    if max_iter is not None:
        options["maxiter"] = max_iter
    # options["gtol"] = 1e-6
    # options["disp"] = True

    # Utilisation d'une optimisation L-BFGS-B
    #print " Real bounds: " + str(real_bounds)
    if (method == "SLSQP"):
        res = scipy.optimize.minimize(optimize_total, chain.active_from_full(starting_nodes_angles), method='SLSQP',
                                      bounds=real_bounds, options=options)
    else:
        res = scipy.optimize.minimize(optimize_total, chain.active_from_full(starting_nodes_angles), method='L-BFGS-B',
                                      bounds=real_bounds, options=options)

    return (chain.active_to_full(res.x, starting_nodes_angles))


def inverse_kinematic_ga(chain, target_frame, starting_nodes_angles,
                         orientation_weight=.4, max_iter=5, second_chain=None,
                         second_target_frame=None, include_orientation=False,
                         method="ga_simple", population_size=12,
                         mutation_rate=.01, num_generations=500, num_elites=3,
                         distance_acc=.001, orientation_acc=.01):

    starting_nodes_angles = chain.inverse_kinematics(
        target_frame, method="L-BFGS-B")

    if starting_nodes_angles is None:
        raise ValueError("starting_nodes_angles must be specified")

    # Only get the position
    target = target_frame[:3, -1]

    targetQuad = m3d.quaternion.UnitQuaternion(
        m3d.orientation.Orientation(target_frame[:3, :3]))

    def fwki_with_orientation(x):

        import math

        # Calculate position distance
        y = chain.active_to_full(x, starting_nodes_angles)
        fwk = chain.forward_kinematics(y)
        distance = math.sqrt(np.linalg.norm(fwk[:3, -1] - target))
        #print "Position distance: " + str(squared_distance)

        # Calculate orientation distance
        recentQuad = m3d.quaternion.UnitQuaternion(
            m3d.orientation.Orientation(fwk[:3, :3]))
        #targetQuad =  m3d.quaternion.UnitQuaternion(m3d.orientation.Orientation(target_frame[:3, :3]))
        orientation_distance = targetQuad.ang_dist(recentQuad)
        #import transforms3d
        #import math
        # k=transforms3d.euler.EulerFuncs()
        #i_al, i_be, i_ga = transforms3d.taitbryan.mat2euler(fwk[:3, :3])
        #t_al, t_be, t_ga = transforms3d.taitbryan.mat2euler(target_frame[:3, :3])
        # d_al=i_al-t_al
        #d_be = i_be - t_be
        #d_ga = i_ga - t_ga
        #orientation_distance = math.sqrt(d_al*d_al+d_be*d_be+d_ga*d_ga)
        #orientation_distance = math.sqrt(d_al*d_al)
        #print "Orientation distance: " + str(orientation_distance)

        # Set the weight random for every calculation
        # orientation_weight=1-random.random()
        #orientation_weight = random.random()
        min(max(0, orientation_weight), 1)
        return ((distance * (1 - orientation_weight) + orientation_distance * orientation_weight, distance, orientation_distance))

    def fwki_only_position(x):
        # Calculate position distance
        y = chain.active_to_full(x, starting_nodes_angles)
        fwk = chain.forward_kinematics(y)
        squared_distance = np.linalg.norm(fwk[:3, -1] - target)
        return (squared_distance)

    def eval_func(chromosome):

        # be in the limits of the joint bounds
        bound_arr = []
        for t in chain.links:
            # print t
            if t.bounds != (None, None):
                bound_arr.append(t.bounds)

        # if gene not in bounds, fit the value into the bounds
        x = []
        for t, value in enumerate(chromosome):
            down, up = bound_arr[t]
            if value < down:
                value = down
            if value > up:
                value = up
            x.append(value)

        if (include_orientation):
            opt, dist, or_dist = fwki_with_orientation(x)
            #opt = fwki_only_position(x)
        else:
            opt = fwki_only_position(x)
        score = float(opt)
        return score, dist, or_dist

    def refresh_fitness(indiv):
        indiv.fitness, indiv.distance, indiv.orientation_distance = eval_func(
            indiv.gene)

    class Individual(object):
        def __init__(self, gene):
            self.gene = gene
            self.fitness, self.distance, self.orientation_distance = eval_func(
                self.gene)

        def change_gene(self, index, value):
            self.gene[index] = value
            #self.fitness = eval_func(self.gene)

        def refresh_fitness(self):
            self.fitness, self.distance, self.orientation_distance = eval_func(
                self.gene)
#        def __cmp__(self, other):
#            if hasattr(other, 'fitness'):
#                return self.fitness.__cmp__(other.fitness)

        def __repr__(self):
            return '{}: {} f: {}'.format(self.__class__.__name__,
                                         self.gene,
                                         self.fitness)

    class Population(object):
        def __init__(self):
            self.individuals = []
            self.sorted = False

    def tournamentSelection(pop, num):
        indiNum = random.randint(0, len(pop.individuals) - 1)
        for i in range(num):
            indiOther = random.randint(0, len(pop.individuals) - 1)
            if pop.individuals[indiOther].fitness < pop.individuals[indiNum].fitness:
                indiNum = indiOther
        return indiNum

    def genCrossover(gMom, gDad):
        off = []
        weight = random.random()
        for i in xrange(len(gMom)):
            off.append((gMom[i] * weight + gDad[i] * (1 - weight)))
            #            if random.random() > 0.5:
            #                brother[i] = ((gMom[i] + gDad[i]) / 2) + (gMom[i] - gDad[i]) - random.random()
            #                # brother=gDad.clone()
            #            else:
            #                brother[i] = ((gMom[i] + gDad[i]) / 2) + (gDad[i] - gMom[i]) + random.random()
            # brother=gMom.clone()

        return (off)

    def mutate(indi):
        off = []
        for i in xrange(len(indi.gene)):
            off.append(indi.gene[i])
            # if random.random()<indi.fitness*5:
            if True:
                if random.random() < mutation_rate:
                    # !!!! ToDo Strength of mutation, dependend on success
                    off[i] = off[i] + (random.random() * 2 - 1)

        indi.gene = off
        # print "Mutant: gene: " + str(off)
        # raw_input()
        return (off)

    def mutation(parent):
        parent.gene = mutate(parent)
        parent.gene = chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(
            parent.gene, starting_nodes_angles), method="SLSQP", include_orientation=True, max_iter=max_iter * 5))

    def initGene(bounds=None):
        import math
        chrome = np.zeros(chromLength)
        for t in range(chromLength):
            if (bounds == None):
                init = random.uniform(-math.pi, math.pi)
            else:
                up, down = bounds[t]
                init = random.uniform(up, down)
            chrome[t] = init
        # Take out or Leave in: Optimize at creation
        #chrome=chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(chrome, starting_nodes_angles), method="SLSQP",include_orientation=True))
        return(chrome)

    chromLength = len(chain.active_from_full(starting_nodes_angles))

    # Build bounds array from URDF data
    bound_arr = []
    for t in chain.links:
        #print t
        if t.bounds != (None, None):
            bound_arr.append(t.bounds)

    # initialize genomes
    pop = Population()
    for i in range(population_size):
        chrome = initGene(bounds=bound_arr)
        #chrome = np.zeros(chromLength)
        # for t in range(chromLength):
        #    init=random.uniform(-3.141, 3.141)
        #    chrome[t]=init
        # !!!! ToDo change parameter of SLSQP optimization. Do not go so deep to save calculation time
        #chrome=chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(chrome, starting_nodes_angles), method="SLSQP",include_orientation=True, max_iter=max_iter))
        # indi=Individual(chrome)
        indi = Individual(initGene(bound_arr))
        pop.individuals.append(indi)
    #chrome = chain.active_from_full(chain.inverse_kinematics(target_frame, method="SLSQP",include_orientation=True, max_iter=max_iter))
    # indi=Individual(chrome)
    indi = Individual(initGene(bound_arr))
    pop.individuals.append(indi)

    pop.individuals = sorted(
        pop.individuals, key=lambda individual: individual.fitness)
    # for individual in pop.individuals:
    #    print individual

    i = 0
    nic = 0  # No improvement counter
    minFitness = 10
    lastFitness = 10
    min_distance = 10
    min_orientation_distance = 10
    acc_reached = False
    # while i < num_generations and minFitness > reachFitness:  # iterate through the generations
    pool = Threadpool()
    while i < num_generations and not acc_reached:  # iterate through the generations
        start_t = time.time()
        #        if lastFitness-minFitness<0.01:
        #		nic+=1
        #        else:
        #			lastFitness=minFitness
        #			nic=0

        i += 1
        # print("(Gen: #%s) Total error: %s\n" % (i, np.sum([ind.fitness for ind in pop.individuals])))
        #print "Min Error: " + str(pop.individuals[0].fitness)
        newPop = Population()
        # winners = np.zeros((params[4], params[3])) #20x2
        # clear population from Individuals with the same fitness
        t = 1
        # Get all individiduals out, that are nearly equal
        for t in range(1, len(pop.individuals)):
            if np.allclose(pop.individuals[t - 1].fitness, pop.individuals[t].fitness, rtol=1e-03, atol=1e-04):
                pop.individuals[t].gene = initGene(bound_arr)
                #print "cleared"
                # raw_input()
        start_mutation = time.time()
        # get the Elites and optimize it with numerical method
        parents = copy.deepcopy(pop.individuals[:num_elites])

        pool.map(mutation, parents)
        # for t in range(num_elites):
        #     start_mutation_it = time.time()
        #     parent = copy.deepcopy(pop.individuals[t])
        #     print("Copy {}s".format(time.time() - start_mutation_it))
        #     parent.gene = mutate(parent)
        #     print("Mutate {}s".format(time.time() - start_mutation_it))
        #     parent.gene = chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(
        #         parent.gene, starting_nodes_angles), method="SLSQP", include_orientation=True, max_iter=max_iter * 5))
        #     print("Gene {}s".format(time.time() - start_mutation_it))
        for parent in parents:
            if len(newPop.individuals) > 0:
                if not np.array_equal(parent.gene, newPop.individuals[0].gene):
                    #print "Elite: " + str(parent.gene)
                    newPop.individuals.append(parent)
            else:
                #print "First Elite: " + str(parent.gene)
                newPop.individuals.append(parent)
        # print("Mutation Process {}s".format(time.time() - start_mutation))
        start_cross = time.time()
        # Crossover the population, select by tournament selection
        while len(newPop.individuals) < population_size and len(pop.individuals) > 2:
            # start_cross_it = time.time()
            momNum = tournamentSelection(pop, 3)
            mom = pop.individuals[momNum].gene
            dadNum = tournamentSelection(pop, 3)
            dad = pop.individuals[dadNum].gene
            # print("Selection {}s".format(time.time() - start_cross_it))
            off = genCrossover(mom, dad)
            indi = Individual(off)
            indi.gene = mutate(indi)

            # invest in optimization dependent from current generations
            if (random.random() * i > num_generations * 0.5):
                indi.gene = chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(
                    off, starting_nodes_angles), method="SLSQP", include_orientation=True, max_iter=max_iter))
            # print("Offspring {}s".format(time.time() - start_cross_it))
            if (indi.fitness < pop.individuals[momNum].fitness) or (indi.fitness < pop.individuals[momNum].fitness):
                if momNum != dadNum:
                    del pop.individuals[dadNum]
                    del pop.individuals[momNum - 1]
                else:
                    del pop.individuals[dadNum]

            newPop.individuals.append(indi)
            # print("Cross_it {}s".format(time.time() - start_cross_it))
        end_cross = time.time()
        # Fill up the rest of the population with new individuals
        while len(newPop.individuals) < population_size:

            indi = Individual(initGene(bound_arr))
            if (random.random() * i > num_generations * 0.5):
                indi.gene = chain.active_from_full(chain.inverse_kinematics(target_frame, initial_position=chain.active_to_full(
                    indi.gene, starting_nodes_angles), method="SLSQP", include_orientation=True, max_iter=max_iter))

            newPop.individuals.append(indi)

        pop = newPop

        start_fit = time.time()
        # Taken out for multithreading
        # for indi in pop.individuals:
        #    indi.refresh_fitness()
        results = pool.map(refresh_fitness, pop.individuals)

        pop.individuals = sorted(
            pop.individuals, key=lambda individual: individual.fitness)
        minFitness = pop.individuals[0].fitness
        min_distance = pop.individuals[0].distance
        min_orientation_distance = pop.individuals[0].orientation_distance
        acc_reached = min_distance < distance_acc and min_orientation_distance < orientation_acc
        print("Elite Mutation: {}s\n Crossover: {}s\n Fill: {}s\n Fit: {}s".format(start_cross - start_t,
                                                                                   end_cross - start_cross, start_fit - end_cross, time.time() - start_fit))
        print("Eval time: {}s".format(time.time() - start_t))
        print "End criteria: " + \
            str(acc_reached) + ' ' + str(min_distance) + \
            ' ' + str(min_orientation_distance)
        # raw_input()
        # for indi in pop.individuals:
        #    print indi
        #print "Give return"
        # raw_input()
    pool.close()
    pool.join()
    return (chain.active_to_full(pop.individuals[0].gene, starting_nodes_angles))


def inverse_kinematic_ccd(chain, target_frame, starting_nodes_angles, orientation_weight=None, max_iter=None,
                          second_chain=None,
                          second_target_frame=None, include_orientation=False, method="ccd_simple"):
    import random

    # Only get the position
    target = target_frame[:3, 3]

    # print starting_nodes_angles

    # if no starting node angles, get for the position with numerical method
    if (starting_nodes_angles == None):
        starting_nodes_angles = chain.inverse_kinematics(
            target_frame, method="SLSQP")

    # print starting_nodes_angles
    # raw_input()

    # bounds
    # real_bounds = [-3.14,3.14]

    if orientation_weight == None:
        orientation_weight = 0.5

    options = {}
    # max_iter=1000
    # Manage iterations maximum
    if max_iter is not None:
        options["maxiter"] = max_iter

    targetQuad = m3d.quaternion.UnitQuaternion(
        m3d.orientation.Orientation(target_frame[:3, :3]))

    import copy
    joint_angles = copy.deepcopy(chain.active_from_full(starting_nodes_angles))
    print "joint_angles " + str(joint_angles)
    raw_input()

    stop = False

    # calculates distance for one joint
    def fwki_with_orientation(x):
        # Calculate position distance
        xa = copy.deepcopy(joint_angles)
        if x is not None:
            xa[joint_num] = x
        y = chain.active_to_full(xa, starting_nodes_angles)
        fwk = chain.forward_kinematics(y)
        squared_distance = np.linalg.norm(fwk[:3, -1] - target)
        # print "Position distance: " + str(squared_distance)

        # Calculate orientation distance
        # recentQuad=m3d.quaternion.UnitQuaternion(m3d.orientation.Orientation(fwk[:3,:3]))
        # orientation_distance=targetQuad.dist_squared(recentQuad)
        # print "Orientation distance: " + str(orientation_distance)

        # calculate the normalized vector from current joint to target (JT)
        currentJointPose = \
            chain.forward_kinematics(chain.active_to_full(joint_angles, starting_nodes_angles), full_kinematics=True)[
                joint_num + 1]
        currentJointPosition = currentJointPose[:3, 3]

        line1 = math3d.geometry.Line(
            point0=currentJointPosition, point1=target)
        print "Line 1:" + str(line1.direction)

        endeff = \
            chain.forward_kinematics(chain.active_to_full(joint_angles, starting_nodes_angles), full_kinematics=True)[
                len(starting_nodes_angles) - 1]
        endeffpo = endeff[:3, 3]
        # calculate the normalized vector from current joint to end effector (JE)
        line2 = math3d.geometry.Line(
            point0=currentJointPosition, point1=endeffpo)
        print "Line 2:" + str(line2.direction)

        angle = line1.direction.angle(line2.direction)

        print "angle: " + str(angle)

        return (angle)

    # Set the weight random for every calculation
    # orientation_weight=1-random.random()

    # return (squared_distance*(1-orientation_weight)+orientation_distance*orientation_weight)

    def fwki_only_position(x_akt):
        # Calculate position distance
        x = copy.deepcopy(joint_angle)
        x[joint_num] = x_akt
        y = chain.active_to_full(x, starting_nodes_angles)
        fwk = chain.forward_kinematics(y)
        squared_distance = np.linalg.norm(fwk[:3, -1] - target)
        return (squared_distance)

    import math3d.geometry
    while not stop:
        for joint_num, joint_angle in reversed(list(enumerate(joint_angles))):
            res = scipy.optimize.minimize(
                fwki_with_orientation, joint_angle, method='SLSQP')

        # joint_angle[joint_num]=res.x

        # from: http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=04904134D4375E6F8F6081E679B2F550?doi=10.1.1.676.6418&rep=rep1&type=pdf

        # calculate the normalized vector from current joint to target (JT)

        # calculate the normalized vector from current joint to end effector (JE)

        # angle = arccos(dot product of JT and JE)

        # axis = cross product of JT and JE

        # Rotate current joint with angle about axis

        if fwki_with_orientation(None) < 0.002:
            stop = True
        print "ccd error: " + \
            str(fwki_with_orientation(None)) + " joints " + str(joint_angles)

    return (chain.active_to_full(joint_angles, starting_nodes_angles))
