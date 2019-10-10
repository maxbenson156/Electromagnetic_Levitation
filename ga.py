"""
Modified GA code from SOURCE
"""

import random
import math
import numpy as np
import csv
import time
import RPi.GPIO as GPIO
import curses
import i2c
import controller
import calibrate
import PID

MAX_TIMESTEPS = 150
POPULATION_SIZE = 50 # 100 (my values are from a paper on GAs for magnetic levitation PID tuning...)
                     # http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.421.36&rep=rep1&type=pdf
MUTATION_PROBABILITY = 0.33 # 0.1
CROSSOVER_RATE = 0.9 # 0.7
MAX_RUNS = 100 # generations
FITNESS_THRESHOLD = 5
MAX_GAIN_VALUE = 1
LINE_SMOOTHNESS = .1

class Chromosome:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
     

"""
1 [Start] Generate random population of n chromosomes (suitable solutions for the problem)
Creates a random genome
"""
def generate_initial_population():
    print("Generating initial population...\n")
    random.seed()
    population = []
    for i in range(POPULATION_SIZE):
        population.append(i)
        # create a random chromosome with a random gain value
        population[i] = Chromosome(random.random() * MAX_GAIN_VALUE * 1000, random.random() * MAX_GAIN_VALUE / 1000, random.random() * MAX_GAIN_VALUE)
    return population
        
""" 
2 [Fitness] Evaluate the fitness f(x) of each chromosome x in the population
returns the fitness value according to the fitness function
Fitness is how long the ball remains inbetween the top and bottom without touching
"""
def _fitness(samples): # **MAKE THIS WORK...**
    # 1. get top and bottom voltage values (as we already do)
    # 2. measure voltage at every time interval (as we already do)
    # 3. while measured voltage != (top or bottom voltages): keep going, don't cap at 1000 samples (iter ++)
    # 4. when measured voltage == (top of bottom voltages): fitness = number of samples before this happened
    #                                                       this might take a while to get going though...
    abs_errors = np.absolute(samples)
    sum_errors = np.sum(abs_errors)
    fitness = 1 / sum_errors 
    
    return fitness
        
"""
Run simulation for a specific chromosome c.
Returns the fitness function value of the simulation
"""
def run_simulation_for_chromosome(population, chromosome): # **MAKE THIS WORK AND DO OTHER IMPORTANT STUFF LIKE LOGGING**
    
    fitness = 0
    pid = PID.PID(pwm, i2c, v_max, v_min, position, population[chromosome].kp, population[chromosome].ki, population[chromosome].kd) # custom PID for chromosome k values
    print("\nkp = {}\nki = {}\nkd = {}\n".format(population[chromosome].kp, population[chromosome].ki, population[chromosome].kd))
    errors = pid.position() # performs PID control and returns location data for ball
    pwm.DC(0)

    fitness = _fitness(errors)

    print('fitness is {}'.format(fitness))
    # logging
    # length = np.transpose(np.linspace(0,999,1000))
    # location = np.transpose(location)
    # myData = [length, location]  
    # date = datetime.datetime.now().strftime("%H-%M-%S-%B-%d-%Y")  
    # filename = './logs/PID-Response-' + chromosome + date +'.csv' # this won't work for multiple generations (it's temporary)
    # myFile = open(filename, 'w')
    # with myFile:  
    #   writer = csv.writer(myFile)
    #   writer.writerows(myData)


    fitness += 1

    return fitness
    
"""
From Emil's code
""" 
def range_check(value):
    if value > 100:
        value = 100
    elif value < 0:
        value = 0
    return value
    
"""
Run the simulation for the set of all chromosomes
"""
def run_simulation(population, generation):
    fitness_values = np.zeros(POPULATION_SIZE)
    for chromosome in range(POPULATION_SIZE):
        print("Generation: {}".format(generation))
        fitness_values[chromosome] = run_simulation_for_chromosome(population, chromosome)
        time.sleep(0.5)
        
    return fitness_values

"""
Pick two parents according to probability represented by normalized fitness values
3a[Selection] Select two parent chromosomes from a population according to their fitness (the better fitness, the bigger chance to be selected)
"""
def selection(fitness_values):
    # normalize the list so we have probabilities to pick parents
    fitness_values = normListSumTo(fitness_values, 1)
    parents = []
    random.seed()
    parent1_probability = random.random()
    parent2_probability = random.random()
    
    sum = 0
    for i in range(POPULATION_SIZE):
        if len(parents) == 2:
            break
        next_sum = sum + fitness_values[i]
        if parent1_probability <= next_sum and parent1_probability >= sum:
            parents.append(i)
        if parent2_probability <= next_sum and parent2_probability >= sum:
            parents.append(i)
        sum = next_sum
    return parents

def normListSumTo(L, sumTo=1):
    '''normalize values of a list to make it sum = sumTo'''

    total = sum(L)
    return [ x/(total*1.0)*total for x in L]

"""
3b[Crossover] With a crossover probability cross over the parents to form a new offspring (children).
If no crossover was performed, offspring is an exact copy of parents.
"""
def crossover(population, parents):
    random.seed()

    # if we dont crossover, offspring is a copy of parents
    if random.random() > CROSSOVER_RATE:
        return population[parents[0]]
    else:
        # random combination crossover
        number = random.random()
        if number < .25:
            return Chromosome(population[parents[1]].kp,population[parents[1]].kd, population[parents[1]].ki)
        elif number < .5:
            return Chromosome(population[parents[0]].kp,population[parents[1]].kd, population[parents[1]].ki)
        elif number < .75:
            return Chromosome(population[parents[0]].kp,population[parents[0]].kd, population[parents[1]].ki)
        else:
            return Chromosome(population[parents[0]].kp,population[parents[0]].kd, population[parents[0]].ki)

"""
3c[Mutation] With a mutation probability mutate new offspring at each locus (position in chromosome).
"""
def mutation(chromosome):
    random.seed()
    
    if random.random() < MUTATION_PROBABILITY / 3:
        #very small real valued mutation
        chromosome.kp = chromosome.kp + random.random()/MAX_GAIN_VALUE 
        if chromosome.kp < 0:
            chromosome.kp = random.random() * MAX_GAIN_VALUE
            
    elif random.random() < MUTATION_PROBABILITY * 2/3:
        chromosome.ki = chromosome.ki + random.random()/MAX_GAIN_VALUE
        if chromosome.ki < 0:
            chromosome.ki = random.random() * MAX_GAIN_VALUE
            
    elif random.random() < MUTATION_PROBABILITY:
        chromosome.kd = chromosome.kd + random.random()/MAX_GAIN_VALUE
        if chromosome.kd < 0:
            chromosome.kd = random.random() * MAX_GAIN_VALUE        
            
    return chromosome
    
"""
3 [New population] Create a new population by repeating following steps until the new population is complete
"""
def generate_new_population(fitness_values, previous_population, generation):
    new_population = []
    print("\nGenerating new population...\n")
    with open(filename, "w") as my_file:
        writer = csv.writer(my_file)
        for i in range(POPULATION_SIZE-1): # saves one space for elitist selection
            new_population.append(i)
            # selection
            parents = selection(fitness_values)
                    
            # crossover
            chromosome = crossover(population, parents)

            # mutation
            chromosome = mutation(chromosome)
            new_population[i] = chromosome
            # log kp, ki, kd, generation, chromosome (relative)
            kp = population[chromosome].kp
            ki = population[chromosome].ki
            kd = population[chromosome].kd
            my_data = [generation, chromosome, kp, ki, kd, 0]
            filename = './logs/GA_PID_parameters' + '.csv'
            writer.writerows(my_data)
    
    
    """
    Perform hybrid elitist selection. Carry the best chromosome over to the new population, unmutated.
    """
    chromosome = population[np.argmax(fitness_values)]
    new_population.append(POPULATION_SIZE-1)
    new_population[POPULATION_SIZE-1] = chromosome
        # log kp, ki, kd, generation, chromosome (relative)
    kp = population[chromosome].kp
    ki = population[chromosome].ki
    kd = population[chromosome].kd
    my_data = [generation, chromosome, kp, ki, kd, 1]
    filename = './logs/GA_PID_parameters' + '.csv'
    with open(filename, "w") as my_file:
        writer = csv.writer(my_file)
        writer.writerows(my_data)
    
    return new_population
    

def file_len(fname):
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
    return i + 1

"""
    Main
    1 [Start] Generate random population of n chromosomes (suitable solutions for the problem)
    2 [Fitness] Evaluate the fitness f(x) of each chromosome x in the population
    3 [New population] Create a new population by repeating following steps until the new population is complete
        3a[Selection] Select two parent chromosomes from a population according to their fitness (the better fitness, the bigger chance to be selected)
        3b[Crossover] With a crossover probability cross over the parents to form a new offspring (children). If no crossover was performed, offspring is an exact copy of parents.
        3c[Mutation] With a mutation probability mutate new offspring at each locus (position in chromosome).
        3d[Accepting] Place new offspring in a new population
    4 [Replace] Use new generated population for a further run of algorithm
    5 [Test] If the end condition is satisfied, stop, and return the best solution in current population
    6 [Loop] Go to step 2
"""
    
#################################################################################################################################################
#   
#   NOTE: Fitness_values is indexed by chromosome number, so if we attempt to sort it, 
#   we will pick the wrong index when generating a new population. Which would be bad.
#
#################################################################################################################################################

# screen = curses.initscr()
# screen.refresh()
# curses.cbreak()
# screen.keypad(True)
# value = 1

generation = 1

i2c = i2c.I2C()
pwm = controller.PWM()
v_min = v_max = 0

try:
    # screen.addstr('press any key to start')
    # key = 0

    cal = calibrate.Calibrate(i2c, pwm)
    v_min, v_max = cal.setup()
    position = (v_min + 2*v_max)/3
    filename = './logs/GA_PID_parameters' + '.csv'
    try:
        f = file.open(filename)
        f.close()
    except FileNotFoundError:
        print('\nFile does not exist, creating new log file\n')
        header = ["generation", "chromosome no.", "kp", "ki", "kd", "elite?"]
        with open(filename, "w") as my_file:
            writer = csv.writer(my_file)
            writer.writerows(header)
    if file_len(filename) > POPULATION_SIZE:
        with open(filename, "r") as my_file:
            reader = csv.reader(my_file)
            generation = reader[-1][0] # final chromosome's generation no. (should be the same for the last 50 lines)
            population = []
            for line in reader:
                kp = line[2]
                ki = line[3]
                kd = line[4]
                chromosome = Chromosome(kp, ki, kd)
                population.append(chromosome) # population is continued from csv file
    else:
        population = generate_initial_population()

    fitness_values = run_simulation(population)
    time.sleep(1)
except Exception as e:
    raise ValueError(e)


for i in range(MAX_RUNS - generation):
    print("Generation {}".format(generation))
    population = generate_new_population(fitness_values, population, generation)
    fitness_values = run_simulation(population, generation)
    
    max_value = max(fitness_values)
    
    print("Maximum fitness of generation {} = {}".format(generation, max_value))

    # if the population sucks, DESTROY THE EARTH
#    if max_value < FITNESS_THRESHOLD:
 #       print("Population sucked so we're starting with a fresh batch lol")
  #      population = generate_initial_population()
   #     generation = 1
    #    continue

    generation += 1