# genetic algorithm

import os
import random
from math import sqrt
import math
from random import randint
from scipy.special import comb
import matplotlib.pyplot as plt
import numpy as np
import time
import rospy


def individual (nPoints, start, goal, p2, p3, p4, map_size):
    """
    Create a member of the population

    length: the number of values per individual
    start: start point 
    goal: goal point
    range_min: random number's minimum range_min
    range_min: random number's maximun range_min
    """
    
    p2 = (int(np.random.random()*map_size), int(np.random.random()*map_size))
    p3 = (int(np.random.random()*map_size), int(np.random.random()*map_size))
    p4 = (int(np.random.random()*map_size), int(np.random.random()*map_size))
    
    points = np.random.rand(nPoints,2)*map_size
    points[0] = start       
    points[1] = p2
    points[2] = p3
    points[3] = p4
    points[nPoints -1] = goal
    

    return bezier_curve(points)
    
def bezier_curve(points, nTimes=80):
    """
   Given a set of control points, return the
   bezier curve defined by the control points.

   points should be a list of lists, or list of tuples
   such as [ [1,1], 
             [2,3], 
             [4,5], ..[Xn, Yn] ]
    nTimes is the number of time steps, defaults to 1000

    See http://processingjs.nihongoresources.com/bezierinfo/
    """
    xPoints = []
    yPoints = []
    nPoints = len(points)
    #Creating random points for control points
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])
    
    xPoints.astype(int)
    yPoints.astype(int)
    
    
    '''
    #debug print of the control points
    print('xPoints', xPoints)
    print('yPoints', yPoints)
    print('Start X', xPoints[0])        
    print('Start Y', yPoints[0])
    print('Goal X', xPoints[-1])        
    print('Goal Y', yPoints[-1])
    '''
    t = np.linspace(0.0, 1.0, nTimes)
    
    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)])
    
    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)
    
    xvals.astype(int)
    yvals.astype(int)
    
    #curve = (map(lambda x, y:(x, y), xvals, yvals))
    path = [[xvals[i], yvals[i]] for i in range(0, len(xvals))]
    path.reverse()
    '''
    plt.plot(xvals, yvals)
    plt.plot(xPoints, yPoints, "ro")
    for nr in range(len(xPoints)):
        plt.text(points[nr][0], points[nr][1], nr)
    
    plt.show()
    '''
                                                                                                                                                                                                                                                                                                                                                            
    return path


def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """    
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def Population(populacao_individuos, nPoints, start, goal, p2, p3, p4, map_size):
    """
    Create a number of individuals (i.e. a population).

    count: the number of individuals in the population
    length: the number of values per individual
    start: start point 
    goal: goal point
    """
    populacao_pre = [] #population before fitness
    
    #populacao_pos = [] #population after fitness

    populacao_pre = [ individual(nPoints, start, goal,  p2, p3, p4, map_size) for i in range(populacao_individuos) ]
    
    #print (populacao_pre)
    #print ('populacao shape', len(populacao_pre))
    
    graded = [ (fitness(x, start, goal), x) for x in populacao_pre]
    graded = [ x[1] for x in sorted(graded) if x[0] < 2000]
    
    #if len(graded) < populacao_individuos:
        
    
    
    #print (graded)
    #print ('graded shape', len(graded))
    
    #return [ individual(nPoints, start, goal,  p2, p3, p4, map_size) for i in range(populacao_individuos) ]
    
    return graded
  

# fitness function to be changed later
def fitness(individual,start, goal):
    """
    Determine the fitness of an individual. Lower is better.

    individual: the individual to evaluate
    start: start point 
    goal: goal point
    vertexs_list: a list of vertexs of obstacles
    """
    #TODO
    #Fix this fitness function with the energy function for path planning
    previous_point = (start[0], start[1])
    min_curvature = 5
    dist_optimal = sqrt((start[0] - goal[0])**2 +(start[1] - goal[1])**2 )
    
    
    fitness_euclid = 0
    fitness_curvature = 0
    fitness_vertex = 0
    #fitness = 0.0
    
    #print (individual)
    #vertex fitness
    for point in individual:
        if inside_circle_01(point[0],point[1]):
            fitness_vertex += 1000
    
    # for point in individual:
    #     if inside_circle_02(point[0],point[1]):
    #         fitness_vertex += 1000
            
    # for point in individual:
    #     if inside_circle_03(point[0],point[1]):
    #         fitness_vertex += 1000
            
    # for point in individual:
    #     if inside_circle_04(point[0],point[1]):
    #         fitness_vertex += 1000

    # for point in individual:
    #     if inside_circle_05(point[0],point[1]):
    #         fitness_vertex += 1000

    # for point in individual:
    #     if inside_circle_06(point[0],point[1]):
    #         fitness_vertex += 1000

    # for point in individual:
    #     if inside_circle_07(point[0],point[1]):
    #         fitness_vertex += 1000

    # for point in individual:
    #     if inside_circle_08(point[0],point[1]):
    #         fitness_vertex += 1000

    # for point in individual:
    #     if inside_circle_09(point[0],point[1]):
    #         fitness_vertex += 1000


        
    
    #Fitness about euclidian distance
    for point in individual:
        fitness_euclid += sqrt((point[0] - previous_point[0])**2 +(point[1] - previous_point[1])**2 )
        previous_point = point
        #print (fitness_euclid)
        
    
    curve = individual
    
    x = np.array(curve)[:,0]
    y = np.array(curve)[:,1]
    
    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)

    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)

    curvature = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5
    
    #print (curvature)
        
    #print (curvature)
    
    
    for curve_value in np.ndindex(curvature.shape):
        if curvature[curve_value] > min_curvature:
            #print('the curvature is higher than min_curvature')
            fitness_curvature += 1000
        else:
            fitness_curvature += 0
            
    
            
    
    fitness_final = sqrt(((fitness_curvature/min_curvature)**2) + (fitness_euclid/dist_optimal)**2 + fitness_vertex**2)
    
    #print ('final fitness is equal to: ', fitness_final)
    
    return fitness_final

# 
def evolve(graded, start, goal, retain=1.2, random_select=0.05, mutate=0.01, crossover_pos=None, precise=1):
    """
    evolution operations, include selction, mutation, crossover

    pop: one generation
    start: start point 
    goal: goal point
    vertexs_list: a list of vertexs of obstacles
    retain: percentage of individuals kept every generation
    random_select: select rate 
    mutate: mutation rate
    crossover_pos: crossover's position 
    precise=1
    """
    #TODO verifty the two lines below
    #graded = [ (fitness(x, start, goal, vertexs_list), x) for x in pop] 
    #graded = [ x[1] for x in sorted(graded)] #x[1] take the individual (path)
    #retain_length = int(len(graded)*retain)
    parents = graded
    
    '''
    # randomly add other individuals to promote genetic diversity
    for individual in graded[:retain_length]:
        if random_select < random.random():
            parents.append(individual)
    '''
    
    # mutate some individuals
    for individual in parents:
        if mutate > random.random():
            pos_to_mutate = randint(0, len(individual)-1)
            # this mutation is not ideal, because it
            # restricts the range of possible values,
            # but the function is unaware of the min/max
            # values used to create the individuals,
            individual[pos_to_mutate] = (random.uniform(start[0], goal[0]), \
                                            random.uniform(start[1], goal[1]))

    # crossover parents to create children
    parents_length = len(parents)
    #desired_length = 25
    #desired_length = len(graded) - parents_length
    desired_length = int(len(parents)**retain)
    children = []
    while len(children) < desired_length:
        male = randint(0, parents_length-1)
        female = randint(0, parents_length-1)
        if male != female:
            male = parents[male]
            female = parents[female]
            if crossover_pos == None:
                crossover_pos = int(len(male) / 2)                
            child = male[:crossover_pos] + female[crossover_pos:]
            
            if fitness(child,child[0],child[-1]) < 2000:
                children.append(child)
            crossover_pos = None
            
            check_children = [ (fitness(x, start, goal), x) for x in children]
            graded = [ x[1] for x in sorted(check_children)]
    
    parents.extend(children)
    
    parents_fitness = [ (fitness(x, start, goal), x) for x in parents]
    
    parents = [x[1] for x in sorted(parents_fitness)]
    
    a = int(len(parents[1])/2)
    
    return parents[:a], parents_fitness[0][:a]

def ga_execute(start, goal, p2, p3, p4, population, generation, nPoints, pop_desired_length, map_size):
               #start, goal, p2, p3, p4, population, generation, nPoints,vertexs_list, pop_desired_length
    """
    entrance of genetic algorithm
    this function is executed in supervisor

    start: start point
    goal: goal point
    """
    fitness_evolution = []
    pop = Population(population, nPoints, start, goal, p2, p3, p4, map_size)
    score = 250
    t_ = 0
    t= time.time()
    i =+ 1
    
    while score > 100 and t_ < 1:

       
        pop, best_score = evolve(pop, start, goal)

        t_ = time.time() - t
        #print("--- %s seconds ---" % (t_))
        score = best_score[0]
        
        
    # select the best individual
    best_individual = pop[0]
    
    
    '''
    xs = [x[0] for x in best_individual]
    ys = [x[1] for x in best_individual]
    plt.plot(xs, ys)
    '''
    #print('Best indivudual', best_individual)
    #print('Best score', best_score[0])
    # use optaimal operation
    #best_individual = optimal_operation(best_individual, vertexs_list, start, goal)
    
    # debug information
    #print ('parents[0]', parents[0])
    #print ('best_individual', best_individual)
    
    return best_score[0], best_individual


def inside_circle_01(x, y, a=-50, b=-50, r=1.1):
    return (x - a)*(x - a) + (y - b)*(y - b) < r*r

# def inside_circle_02(x, y, a=4.67, b=4.14, r=1.1):
#     return (x - a)*(x - a) + (y - b)*(y - b) < r*r

# def inside_circle_03(x, y, a=10.54, b=4.12, r=1.1):
#     return (x - a)*(x - a) + (y - b)*(y - b) < r*r

# def inside_circle_04(x, y, a=7.44, b=7.12, r=1.1):
#     return (x - a)*(x - a) + (y - b)*(y - b) < r*r

# def inside_circle_05(x, y, a=4, b=8, r=1.1):
#     return (x - a)*(x - a) + (y - b)*(y - b) < r*r

# def inside_circle_06(x, y, a=7.4, b=7, r=1.1):
#     return (x - a)*(x - a) + (y - b)*(y - b) < r*r

# def inside_circle_07(x, y, a=8.85, b=8.85, r=1.1):
#     return (x - a)*(x - a) + (y - b)*(y - b) < r*r

# def inside_circle_08(x, y, a=11, b=9.5, r=1.1):
#     return (x - a)*(x - a) + (y - b)*(y - b) < r*r

# def inside_circle_09(x, y, a=6.6, b=12.6, r=1.1):
#     return (x - a)*(x - a) + (y - b)*(y - b) < r*r



def getCurve(image_dir="", image_name="path"):
    #vertexs_list = []

    nPoints = 5
    map_size = 16
    points = []
    
    
    for p in range(nPoints+1):
        points.append( (np.random.random()*np.random.uniform(1,map_size), 
                      np.random.random()*np.random.uniform(1,map_size)))
    
    start = (1,1)#points[0]
    p2 = points[1]
    p3 = points[2]
    p4 = points[3]
    goal = (15,15) #points[-1]

    population = 50
    pop_desired_length = 2
    generation = 8
    fitness_threshold = 18
    best_fitness = 1000
    start_time = time.time()
    #for i in range(generation):
    #while best_fitness > fitness_threshold:
    times_to_test = 1
    time_count = []
    fitness_ = []
    
    for i in range(times_to_test):
        best_fitness, best_individual = ga_execute(start, goal, p2, p3, p4, population, generation, nPoints, pop_desired_length, map_size)
        
    #print (best_individual)

    xs = [x[0] for x in best_individual]
    ys = [x[1] for x in best_individual]
    # circle_01=plt.Circle((3.32,2.91),1,color='black')
    # circle_02=plt.Circle((4.67,4.14),1,color='black')
    # circle_03=plt.Circle((10.54,4.12),1,color='black')
    # circle_04=plt.Circle((7.44,7.12),1,color='black')
    # circle_05=plt.Circle((4,8),1,color='black')
    # circle_06=plt.Circle((7.4,7),1,color='black')
    # circle_07=plt.Circle((8.85,8.85),1,color='black')
    # circle_08=plt.Circle((11,9.5),1,color='black')
    # circle_09=plt.Circle((6.6,12.6),1,color='black')


    fig, ax = plt.subplots()
    ax.plot(xs,ys,marker = 'o',markersize=2.5)
    ax.axis('equal')
    # ax.add_artist(circle_01)
    # ax.add_artist(circle_02)
    # ax.add_artist(circle_03)
    # ax.add_artist(circle_04)
    # ax.add_artist(circle_05)
    # ax.add_artist(circle_06)
    # ax.add_artist(circle_07)
    # ax.add_artist(circle_08)
    # ax.add_artist(circle_09)
    ax.set_xbound(lower=0, upper=map_size+1)
    ax.set_ybound(lower=0, upper=map_size+1)

    #plt.text(14, 14, r'Best Path')
    plt.suptitle('Path Planning - Genetic Algorithm', fontsize = 14, color = 'black')
    plt.xlabel('X Position', fontsize = 12, color = 'black')
    plt.ylabel('Y Position', fontsize = 12, color = 'black')
    
    plt.grid(True)
    plt.savefig(os.path.join(image_dir, image_name + "_" + str(time.time()) + ".png"), dpi=300)  
    plt.show()

    return best_individual


if __name__ == '__main__':
    
        run()


    
        '''
        #print('Best ever', best_individual)
        print('Runing...', i)
        print('Best score', best_fitness)
        fitness_.append(best_fitness)
        
        xs = [x[0] for x in best_individual]
        ys = [x[1] for x in best_individual]
        circle_01=plt.Circle((8,1),1,color='black')
        circle_02=plt.Circle((8,4),1,color='black')
        circle_03=plt.Circle((8,7),1,color='black')
        circle_04=plt.Circle((8,10),1,color='black')
        fig, ax = plt.subplots()
        ax.plot(xs,ys,marker = 'o',markersize=2.5)
        ax.axis('equal')
        ax.add_artist(circle_01)
        ax.add_artist(circle_02)
        ax.add_artist(circle_03)
        ax.add_artist(circle_04)
        ax.set_xbound(lower=0, upper=map_size+1)
        ax.set_ybound(lower=0, upper=map_size+1)

        #plt.text(14, 14, r'Best Path')
        plt.suptitle('Path Planning - Genetic Algorithm', fontsize = 14, color = 'black')
        plt.xlabel('X Position', fontsize = 12, color = 'black')
        plt.ylabel('Y Position', fontsize = 12, color = 'black')

        plt.grid(True)
        plt.savefig("Path_%d.png"%i, dpi=300)  
        plt.show()
    
        
        
        time_count.append(round(time.time() - start_time, 2))            
        #print("--- %s seconds ---" % (time.time() - start_time))
        
        
        #plt.show() 
        start_time= time.time()
        

    ax_time = plt.subplots()
    plt.plot(time_count,marker = 'o',markersize=3.5)
    plt.xlim([0, times_to_test-0.5])
    plt.ylim([min(time_count)-.2, max(time_count)+.2])
    plt.suptitle('Path Planning - Time Lapse', fontsize = 14, color = 'black')
    plt.xlabel('Running times', fontsize = 12, color = 'black')
    plt.ylabel('Time Lapse', fontsize = 12, color = 'black')
    plt.savefig("Time_Lapse%d.png"%i, dpi=300)
    

    ax_fitness_ = plt.subplots()
    plt.plot(fitness_,marker = 'x',markersize=3.5)
    plt.xlim([0, max(fitness_)+.2])
    plt.ylim([min(fitness_)-min(fitness_)/100, max(fitness_)+max(fitness_)/100])
    plt.suptitle('Path Planning - Fitness', fontsize = 14, color = 'black')
    plt.xlabel('Running times', fontsize = 12, color = 'black')
    plt.ylabel('Fitness', fontsize = 12, color = 'black')
    plt.savefig("Fitness_%d.png"%i, dpi=300)
    

    
    
    

    ax_hist =  plt.subplots()
    time_count, bins, patches = plt.hist(time_count, 50, density=True, facecolor='g', alpha=0.75)
    plt.suptitle('Path Planning - Time Histogram', fontsize = 14, color = 'black')
    plt.xlabel('Time Lapse', fontsize = 12, color = 'black')
    plt.ylabel('Frequency', fontsize = 12, color = 'black')
    plt.savefig("Time_Lapse%d.png"%i, dpi=300)
    plt.savefig("Time_Histogram%d.png"%i, dpi=300)
    '''
    