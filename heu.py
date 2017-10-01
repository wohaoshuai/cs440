import timeit
import Queue


# DEBUG
max_cost = 0

# heu
boxes_cache = {}

# change this
filepath = 'data/sokoban4.txt'

# static
walls = set()
goals = set()
# dynamic
man = (0, 0)
cost = 0
boxes = set()
# memory
states = {} # (boxes, position) : (action, cost)
frontiers = Queue.PriorityQueue()
# counters
start_time = timeit.default_timer()

def frozen(new_set):
    return frozenset(list(new_set))

def defrozen(new_set):
    return set(list(new_set))

def save_state():
    global cost
    return (cost, man, frozen(boxes))

def load_state(node):
    global cost
    global man
    global boxes
    cost = node[0]
    man = node[1]
    boxes = defrozen(node[2])

# abstract layer
def calculate_cost(a, b):
    (x1,y1) = a
    (x2,y2) = b
    return abs(x2 - x1) + abs(y2 - y1)

best = -1
coco = 0
def calculate_min(aa, bb, cost):
    global best
    # use immutable instead
    if len(aa) == 0:
        if cost < best or best == -1:
            best = cost
        return

    a = aa[0]
    ac = aa[1:]
    for b in bb:
        bc = list(bb)
        bc.remove(b)
        calculate_min(ac, bc, cost + calculate_cost(a, b))

def heuristic_function(node):
    # prepare the state
    global goals
    global best
    global boxes_cache

    # boxes position
    man = node[1]
    key = node[2]
    boxes = defrozen(key)
    if key in boxes_cache:
        value = boxes_cache[key]
    else:
        best = -1
        aa = list(goals.copy())
        bb = list(boxes)
        calculate_min(aa, bb, 0)
        value = best
        boxes_cache[key] = value

    # calculate from man to shortest boxes
    man_box_distance = []
    for box in boxes:
        man_box_distance.append(calculate_cost(man, box))
    man_cost = min(man_box_distance)

    return value + man_cost

# calculate basic
def heuristic_function_2(node):
    global goals
    global best
    global boxes_cache

# make sure node is immutable
def add_to_queue(q, node):
    cost = node[0]
    heuristic = heuristic_function(node)
    queue_node = (cost + heuristic, node)
    q.put(queue_node)

def get_from_queue(q):
    return q.get()[1]

# (cost, man, boxes)
def add_to_dict(d, node):
    man = node[1]
    boxes = node[2]
    key = (man, boxes)
    cost = node[0]
    value = cost
    d[key] = value

def to_key(node):
    man = node[1]
    boxes = node[2]
    key = (man, boxes)
    return key


count_repeat = 0
count_new = 0
def is_visited(node):
    # fronzen node simply make the node hashable, so that
    # can be checked
    global count_repeat
    global count_new
    key = to_key(node)
    if key in states:
        count_repeat += 1

#        if count_repeat % 100000 == 0:
#            print('==============')
#            print(count_repeat)
#            print(count_new)
#            print('==============')
        return True
    count_new += 1
    return False

def is_okay():
    return True

def is_wall(point):
    if point in walls:
        return True
    return False

def is_box(point):
    if point in boxes:
        return True
    return False

def is_empty(point):
    if point not in walls:
        if point not in boxes:
            return True
    return False

def print_state():
    print('man: ' + str(man))
    print('walls: ' + str(walls))
    print('goals: ' + str(goals))
    print('boxes: ' + str(boxes))

def print_result():
    # solution costs,
    print('solution costs: ' + str(cost))
    # number of expanded nodes,
    count = len(states)
    print('number of expanded nodes: ' + str(count))
    # running time.
    end_time = timeit.default_timer()
    print('running time: ' + str(end_time - start_time))


def check_state():
    if goals == boxes:
        print_result()
        return False
    return True


def generate_nodes(man, boxes, walls):
    new_nodes = []

    # the position from current position 1 step
    up_1 = (man[0], man[1] - 1)
    down_1 = (man[0], man[1] + 1)
    left_1 = (man[0] - 1, man[1])
    right_1 = (man[0] + 1, man[1])
    steps_1 = [up_1, down_1, left_1, right_1]
    # the position from current position 2 step
    up_2 = (man[0], man[1] - 2)
    down_2 = (man[0], man[1] + 2)
    left_2 = (man[0] - 2, man[1])
    right_2 = (man[0] + 2, man[1])
    steps_2 = [up_2, down_2, left_2, right_2]

    steps = zip(steps_1, steps_2)
    for step in steps:
        step_1 = step[0]
        step_2 = step[1]
        if is_empty(step_1):
            new_boxes = boxes.copy()
            new_node = (cost + 1, step_1, frozen(new_boxes))
            new_nodes.append(new_node)
        if is_box(step_1) and is_empty(step_2):
            # init and move boxes
            new_boxes = boxes.copy()
            new_boxes.remove(step_1)
            new_boxes.add(step_2)
            # create new node
            new_node = (cost + 1, step_1, frozen(new_boxes))
            # append node to result
            new_nodes.append(new_node)
    return new_nodes

# read the file
with open(filepath, 'r') as f:
    lines = f.readlines();
    for y, line in enumerate(lines):
        for x, char in enumerate(line):
            position = (x, y)
            if char == '%':
                walls.add(position)
            if char == 'b':
                boxes.add(position)
            if char == '.':
                goals.add(position)
            if char == 'P':
                man = position
            if char == 'B':
                boxes.add(position)
                goals.add(position)

# 2.check state
count = 0
while check_state() and is_okay():
    if cost > max_cost:
        max_cost = cost
        end_time = timeit.default_timer()
        print("!!!" + str(max_cost))
        print(end_time - start_time)
    count+= 1
    if count % 100000 == 0:
        print(count)
        end_time = timeit.default_timer()
        print(end_time - start_time)

    # explore the node
    new_nodes = generate_nodes(man, boxes, walls)

    for new_node in new_nodes:
        if not is_visited(new_node):
            add_to_queue(frontiers, new_node)


    # done with exploration, we now store current node, and load new node
    # we assume there is at least a solution to the problem, hence the queue will
    # always has element
    curr_node = save_state()
    add_to_dict(states, curr_node)
    next_node = get_from_queue(frontiers)
    load_state(next_node)
