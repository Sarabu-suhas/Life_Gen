import math
import random
import heapq
from operator import itemgetter

# Real World Constants
AMBULANCE_SPEED_KMH    = 60
BLOCK_SIZE_METERS      = 100
SIGNAL_WAIT_SECONDS    = 30
BLOCKED_DETOUR_SECONDS = 120
METERS_PER_KM          = 1000
SECONDS_PER_HOUR       = 3600


class block:
    def __init__(self, Name, X, Y, Value):
        self.name  = Name
        self.value = Value
        self.x     = X
        self.y     = Y

    def getX(self):            return self.x
    def getY(self):            return self.y
    def getValue(self):        return self.value
    def setValue(self, Value): self.value = Value
    def getName(self):         return self.name

    def distanceTo(self, block1):
        distX = int(math.fabs(self.getX() - block1.getX()))
        distY = int(math.fabs(self.getY() - block1.getY()))
        return math.sqrt(distX * distX + distY * distY)

    def print(self):
        print(self.getName(), "(", self.getX(), ",",
              self.getY(), "):", self.value)

    def __hash__(self):
        return hash((self.x, self.y, self.name))

    def __eq__(self, other):
        if other is None:
            return False
        return (self.x    == other.x    and
                self.y    == other.y    and
                self.name == other.name and
                self.value== other.value)

    def __lt__(self, other):
        return self.name < other.name


class RouteManager:
    destRoutes = []
    gridLookup = {}

    @staticmethod
    def addBlock(b):
        RouteManager.destRoutes.append(b)
        RouteManager.gridLookup[(b.getX(), b.getY())] = b

    @staticmethod
    def getBlock(index):
        return RouteManager.destRoutes[index]

    @staticmethod
    def numberOfBlocks():
        return len(RouteManager.destRoutes)

    @staticmethod
    def getBlockByXY(x, y):
        return RouteManager.gridLookup.get((x, y), None)

    @staticmethod
    def getNeighbors(b):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nb = RouteManager.getBlockByXY(
                    b.getX() + dx,
                    b.getY() + dy
                )
                if nb is not None:
                    neighbors.append(nb)
        return neighbors

    # ✅ Initial route - picks some signals
    # and suboptimal blocks (not too bad, not too good)
    @staticmethod
    def getInitialCandidates(startBlock, destBlock, count=100):
        candidates = []

        minX = min(startBlock.getX(), destBlock.getX())
        maxX = max(startBlock.getX(), destBlock.getX())
        minY = min(startBlock.getY(), destBlock.getY())
        maxY = max(startBlock.getY(), destBlock.getY())

        # ✅ Bigger padding = more suboptimal detours
        padding = 15

        for b in RouteManager.destRoutes:
            # ✅ Allow signals (not blocked)
            if b.getValue() == "B":
                continue

            inBox = (minX - padding <= b.getX() <= maxX + padding
                     and
                     minY - padding <= b.getY() <= maxY + padding)

            if inBox:
                dist  = b.distanceTo(destBlock)
                score = dist

                # ✅ Slightly prefer signal blocks
                # so initial route has signals in it
                if b.getValue() == "S":
                    score -= 100   # make signals attractive initially

                candidates.append((b, score))

        # ✅ Sort moderately - not best, not worst
        candidates = sorted(candidates, key=lambda x: x[1])

        # ✅ Pick from middle range (not best, not worst)
        total = len(candidates)
        start = total // 4       # skip top 25% best
        end   = total            # include rest
        mid_candidates = candidates[start:end]

        random.shuffle(mid_candidates)
        return [c[0] for c in mid_candidates[:count]]

    @staticmethod
    def getBestNextBlock(currentBlock, destBlock,
                         usedBlocks, topN=10):
        candidates = []
        for b in RouteManager.destRoutes:
            if b in usedBlocks:
                continue
            if b.getValue() == "B":
                continue
            score = b.distanceTo(destBlock)
            if b.getValue() == "S":
                score += 500
            candidates.append((b, score))

        candidates = sorted(candidates, key=lambda x: x[1])
        top = [c[0] for c in candidates[:topN]]
        return random.choice(top) if top else None


# ============================================================
# A* ALGORITHM
# ============================================================
class AStar:

    @staticmethod
    def heuristic(a, b):
        return a.distanceTo(b)

    @staticmethod
    def getMoveCost(fromBlock, toBlock):
        dist_grid  = fromBlock.distanceTo(toBlock)
        meters     = dist_grid * BLOCK_SIZE_METERS
        speed_ms   = (AMBULANCE_SPEED_KMH * METERS_PER_KM /
                      SECONDS_PER_HOUR)
        travelTime = meters / speed_ms

        if toBlock.getValue() == "B":
            travelTime += BLOCKED_DETOUR_SECONDS * 10
        if toBlock.getValue() == "S":
            travelTime += SIGNAL_WAIT_SECONDS

        return travelTime

    @staticmethod
    def findPath(startBlock, destBlock):
        openSet     = []
        heapq.heappush(openSet, (0, startBlock))
        cameFrom    = {}
        gScore      = {startBlock: 0}
        fScore      = {startBlock: AStar.heuristic(
                           startBlock, destBlock)}
        openSetHash = {startBlock}
        iterations  = 0
        maxIter     = 10000

        while openSet and iterations < maxIter:
            iterations += 1
            _, current  = heapq.heappop(openSet)
            openSetHash.discard(current)

            if (current.getX() == destBlock.getX() and
                    current.getY() == destBlock.getY()):
                return AStar.reconstructPath(
                    cameFrom, current, startBlock
                )

            for neighbor in RouteManager.getNeighbors(current):
                tentative_g = (
                    gScore.get(current, float('inf')) +
                    AStar.getMoveCost(current, neighbor)
                )
                if tentative_g < gScore.get(
                        neighbor, float('inf')):
                    cameFrom[neighbor] = current
                    gScore[neighbor]   = tentative_g
                    fScore[neighbor]   = (
                        tentative_g +
                        AStar.heuristic(neighbor, destBlock)
                    )
                    if neighbor not in openSetHash:
                        heapq.heappush(
                            openSet,
                            (fScore[neighbor], neighbor)
                        )
                        openSetHash.add(neighbor)

        return [startBlock, destBlock]

    @staticmethod
    def reconstructPath(cameFrom, current, start):
        path = [current]
        while current in cameFrom:
            current = cameFrom[current]
            path.append(current)
        path.reverse()
        return path


class Route:
    def __init__(self, start, end, Size):
        self.route    = [None] * Size
        self.route[0] = start
        self.route[Size - 1] = end
        self.distance = 0
        self.fitness  = 0

    # ✅ Initial route - suboptimal
    # Has signals, detours, not direct
    def generateInitialIndividual(self):
        destBlock  = self.route[self.routeSize() - 1]
        startBlock = self.route[0]
        usedBlocks = set()
        usedBlocks.add(startBlock)
        usedBlocks.add(destBlock)

        candidates = RouteManager.getInitialCandidates(
            startBlock, destBlock, count=200
        )

        for i in range(1, self.routeSize() - 1):
            added = False
            for candidate in candidates:
                if candidate not in usedBlocks:
                    self.setBlock(i, candidate)
                    usedBlocks.add(candidate)
                    added = True
                    break
            if not added:
                for _ in range(500):
                    t = RouteManager.getBlock(
                        random.randint(
                            0,
                            RouteManager.numberOfBlocks() - 1
                        )
                    )
                    if t not in usedBlocks and t.getValue() != "B":
                        self.setBlock(i, t)
                        usedBlocks.add(t)
                        break

    # ✅ Optimized route via A*
    def generateIndividual(self):
        startBlock = self.route[0]
        destBlock  = self.route[self.routeSize() - 1]

        astarPath = AStar.findPath(startBlock, destBlock)
        routeSize = self.routeSize()

        if len(astarPath) >= routeSize:
            indices = [
                int(i * (len(astarPath) - 1) / (routeSize - 1))
                for i in range(routeSize)
            ]
            for i in range(1, routeSize - 1):
                self.setBlock(i, astarPath[indices[i]])
        else:
            for i in range(1, len(astarPath) - 1):
                if i < routeSize - 1:
                    self.setBlock(i, astarPath[i])

            usedBlocks = set(
                b for b in self.route if b is not None
            )
            for i in range(1, routeSize - 1):
                if self.getBlock(i) is None:
                    nb = RouteManager.getBestNextBlock(
                        self.route[i - 1] or startBlock,
                        destBlock,
                        usedBlocks,
                        topN=5
                    )
                    if nb:
                        self.setBlock(i, nb)
                        usedBlocks.add(nb)

        self._addVariation()

    def _addVariation(self):
        mid_start = max(1, self.routeSize() // 4)
        mid_end   = min(
            self.routeSize() - 1,
            3 * self.routeSize() // 4
        )
        mid = self.route[mid_start:mid_end]
        random.shuffle(mid)
        for i, b in enumerate(mid):
            if b is not None:
                self.route[mid_start + i] = b

    def containBlock(self, thisBlock):
        for i in range(self.routeSize()):
            b = self.getBlock(i)
            if b is not None:
                if (b.getX()    == thisBlock.getX()    and
                        b.getY()    == thisBlock.getY()    and
                        b.getName() == thisBlock.getName() and
                        b.getValue()== thisBlock.getValue()):
                    return True
        return False

    def setBlock(self, routePosition, Block):
        self.route[routePosition] = Block
        self.fitness  = 0
        self.distance = 0

    def getBlock(self, routePosition):
        return self.route[routePosition]

    def routeSize(self):
        return len(self.route)

    def getDistance(self):
        if self.distance == 0:
            routeDist = 0
            for i in range(self.routeSize()):
                fromBlock = self.getBlock(i)
                if fromBlock is None:
                    continue
                if fromBlock.getValue() == "B":
                    routeDist += 2000
                if fromBlock.getValue() == "S":
                    routeDist += 300
                if i + 1 < self.routeSize():
                    destBlock = self.getBlock(i + 1)
                else:
                    destBlock = self.getBlock(0)
                if destBlock is not None:
                    routeDist += int(
                        fromBlock.distanceTo(destBlock)
                    )
            self.distance = int(routeDist)
        return self.distance

    def getFitness(self):
        if self.fitness == 0:
            self.fitness = (
                1 / self.getDistance()
                if self.getDistance() > 0 else 0
            )
        return self.fitness

    def getResponseTime(self):
        total_seconds = 0
        total_meters  = 0
        signal_stops  = 0
        blocked_roads = 0

        for i in range(self.routeSize()):
            fromBlock = self.getBlock(i)
            if fromBlock is None:
                continue

            if fromBlock.getValue() == "S":
                total_seconds += SIGNAL_WAIT_SECONDS
                signal_stops  += 1

            if fromBlock.getValue() == "B":
                total_seconds += BLOCKED_DETOUR_SECONDS
                blocked_roads += 1

            if i + 1 < self.routeSize():
                nextBlock = self.getBlock(i + 1)
            else:
                nextBlock = self.getBlock(0)

            if nextBlock is not None:
                grid_dist     = fromBlock.distanceTo(nextBlock)
                meters        = grid_dist * BLOCK_SIZE_METERS
                speed_ms      = (AMBULANCE_SPEED_KMH *
                                 METERS_PER_KM /
                                 SECONDS_PER_HOUR)
                travel_time   = meters / speed_ms
                total_seconds += travel_time
                total_meters  += meters

        return {
            "total_seconds" : round(total_seconds, 2),
            "total_minutes" : round(total_seconds / 60, 2),
            "total_meters"  : round(total_meters,  2),
            "total_km"      : round(total_meters / 1000, 2),
            "signal_stops"  : signal_stops,
            "blocked_roads" : blocked_roads
        }

    def getRoute(self):  return self.route

    def setRoute(self, routeArray):
        self.route    = routeArray
        self.distance = 0
        self.fitness  = 0

    def PrintMe(self):
        for i in range(self.routeSize()):
            b = self.getBlock(i)
            if b: b.print()


class population:
    def __init__(self, populationSize, initialise,
                 start=0, goal=0, RouteSize=0,
                 initial=False):
        self.routes = [None] * populationSize

        if initialise:
            for i in range(self.populationSize()):
                tmpRoute = Route(start, goal, RouteSize)
                if initial:
                    # ✅ Suboptimal initial route
                    tmpRoute.generateInitialIndividual()
                else:
                    # ✅ Optimized A* route
                    tmpRoute.generateIndividual()
                self.saveRoute(i, tmpRoute)

    def saveRoute(self, index, route1):
        self.routes[index] = route1

    def getRoute(self, index):
        return self.routes[index]

    def populationSize(self):
        return len(self.routes)

    def sort_on_fitness(self):
        temp = [
            [self.getRoute(i), self.getRoute(i).getFitness()]
            for i in range(self.populationSize())
        ]
        temp = sorted(temp, key=itemgetter(1), reverse=True)
        for i in range(self.populationSize()):
            self.saveRoute(i, temp[i][0])

    def routeExists(self, route1):
        for i in range(self.populationSize()):
            route = self.getRoute(i)
            if route is not None:
                exists = True
                for j in range(1, route.routeSize() - 1):
                    bl  = route.getBlock(j)
                    bl2 = route1.getBlock(j)
                    if bl is None or bl2 is None:
                        exists = False
                        break
                    if not (bl.getName()  == bl2.getName()  and
                            bl.getX()     == bl2.getX()     and
                            bl.getY()     == bl2.getY()     and
                            bl.getValue() == bl2.getValue()):
                        exists = False
                        break
                if exists:
                    return True
        return False

    def getFittest(self):
        fittestRoute = self.getRoute(0)
        for i in range(1, self.populationSize()):
            if (fittestRoute.getFitness() <=
                    self.getRoute(i).getFitness()):
                fittestRoute = self.getRoute(i)
        return fittestRoute


class GA:
    mutationRate   = 0.1
    tournamentSize = 8
    elitismCount   = 5

    @staticmethod
    def evolvePopulation(pop):
        newPop = population(pop.populationSize() * 2, False)

        for i in range(GA.elitismCount):
            newPop.saveRoute(i, pop.getRoute(i))

        for i in range(GA.elitismCount, newPop.populationSize()):
            Parent1 = GA.tournamentSelection(pop)
            Parent2 = GA.tournamentSelection(pop)
            child   = GA.crossOver(Parent1, Parent2)
            attempts = 0
            while newPop.routeExists(child) and attempts < 5:
                Parent1 = GA.tournamentSelection(pop)
                Parent2 = GA.tournamentSelection(pop)
                child   = GA.crossOver(Parent1, Parent2)
                attempts += 1
            newPop.saveRoute(i, child)

        for i in range(GA.elitismCount, newPop.populationSize()):
            GA.mutate(newPop.getRoute(i))

        mid = newPop.populationSize() // 2
        for i in range(GA.elitismCount, mid):
            GA.smartMutate(newPop.getRoute(i))

        newPop.sort_on_fitness()

        for i in range(5):
            GA.fastLocalSearch(newPop.getRoute(i))

        newPop.sort_on_fitness()

        newPop2 = population(pop.populationSize(), False)
        for i in range(pop.populationSize()):
            newPop2.saveRoute(i, newPop.getRoute(i))
        return newPop2

    @staticmethod
    def crossOver(route1, route2):
        child    = Route(
            route1.getBlock(0),
            route1.getBlock(route1.routeSize() - 1),
            route1.routeSize()
        )
        startPos = int(random.random() * route1.routeSize())
        endPos   = int(random.random() * route2.routeSize())

        for i in range(1, child.routeSize() - 1):
            if startPos < endPos and startPos < i < endPos:
                b = route1.getBlock(i)
                if b and b.getValue() != "B":
                    child.setBlock(i, b)
            elif startPos > endPos:
                if not (i < startPos and i > endPos):
                    b = route1.getBlock(i)
                    if b and b.getValue() != "B":
                        child.setBlock(i, b)

        for j in range(route2.routeSize()):
            b2 = route2.getBlock(j)
            if b2 and not child.containBlock(b2):
                for k in range(child.routeSize()):
                    if child.getBlock(k) is None:
                        child.setBlock(k, b2)
                        break
        return child

    @staticmethod
    def mutate(myRoute):
        for routePos1 in range(1, myRoute.routeSize() - 1):
            if random.random() < GA.mutationRate:
                routePos2 = int(
                    myRoute.routeSize() * random.random()
                )
                while (routePos2 == 0 or
                       routePos2 == myRoute.routeSize() - 1):
                    routePos2 = int(
                        myRoute.routeSize() * random.random()
                    )
                c1 = myRoute.getBlock(routePos1)
                c2 = myRoute.getBlock(routePos2)
                myRoute.setBlock(routePos2, c1)
                myRoute.setBlock(routePos1, c2)

    @staticmethod
    def smartMutate(myRoute):
        destBlock  = myRoute.getBlock(myRoute.routeSize() - 1)
        usedBlocks = set(
            b for b in myRoute.route if b is not None
        )
        for i in range(1, myRoute.routeSize() - 1):
            currentBlock = myRoute.getBlock(i)
            if currentBlock and currentBlock.getValue() in ["B","S"]:
                if random.random() < 0.9:
                    nextBlock = RouteManager.getBestNextBlock(
                        myRoute.getBlock(i - 1),
                        destBlock,
                        usedBlocks,
                        topN=5
                    )
                    if nextBlock:
                        usedBlocks.discard(currentBlock)
                        myRoute.setBlock(i, nextBlock)
                        usedBlocks.add(nextBlock)

    @staticmethod
    def fastLocalSearch(route):
        for i in range(1, route.routeSize() - 2):
            for j in range(
                    i + 1,
                    min(i + 8, route.routeSize() - 1)):
                oldDist = route.getDistance()
                c1 = route.getBlock(i)
                c2 = route.getBlock(j)
                route.setBlock(i, c2)
                route.setBlock(j, c1)
                newDist = route.getDistance()
                if newDist >= oldDist:
                    route.setBlock(i, c1)
                    route.setBlock(j, c2)
                    route.distance = oldDist

    @staticmethod
    def greedyImprovement(route):
        destBlock  = route.getBlock(route.routeSize() - 1)
        usedBlocks = set(
            b for b in route.route if b is not None
        )
        improved = True
        passes   = 0
        while improved and passes < 10:
            improved = False
            passes  += 1
            for i in range(1, route.routeSize() - 1):
                currentBlock = route.getBlock(i)
                oldDist      = route.getDistance()
                candidate = RouteManager.getBestNextBlock(
                    route.getBlock(i - 1),
                    destBlock,
                    usedBlocks - {currentBlock},
                    topN=10
                )
                if candidate and candidate != currentBlock:
                    usedBlocks.discard(currentBlock)
                    route.setBlock(i, candidate)
                    usedBlocks.add(candidate)
                    newDist = route.getDistance()
                    if newDist >= oldDist:
                        usedBlocks.discard(candidate)
                        route.setBlock(i, currentBlock)
                        usedBlocks.add(currentBlock)
                        route.distance = oldDist
                    else:
                        improved = True

    @staticmethod
    def injectAStarRoutes(pop, startBlock, destBlock, count=5):
        routeSize = pop.getRoute(0).routeSize()
        for c in range(count):
            astarPath = AStar.findPath(startBlock, destBlock)
            if len(astarPath) < 2:
                continue
            newRoute = Route(startBlock, destBlock, routeSize)
            if len(astarPath) >= routeSize:
                indices = [
                    int(i * (len(astarPath) - 1) /
                        (routeSize - 1))
                    for i in range(routeSize)
                ]
                for i in range(1, routeSize - 1):
                    newRoute.setBlock(i, astarPath[indices[i]])
            else:
                for i in range(1, len(astarPath) - 1):
                    if i < routeSize - 1:
                        newRoute.setBlock(i, astarPath[i])
                usedBlocks = set(
                    b for b in newRoute.route if b is not None
                )
                for i in range(1, routeSize - 1):
                    if newRoute.getBlock(i) is None:
                        nb = RouteManager.getBestNextBlock(
                            newRoute.route[i-1] or startBlock,
                            destBlock, usedBlocks, topN=5
                        )
                        if nb:
                            newRoute.setBlock(i, nb)
                            usedBlocks.add(nb)
            worstIdx = pop.populationSize() - 1 - c
            if worstIdx > GA.elitismCount:
                pop.saveRoute(worstIdx, newRoute)

    @staticmethod
    def tournamentSelection(pop):
        popTournament = population(GA.tournamentSize, False)
        for i in range(GA.tournamentSize):
            randomID = random.randint(
                0, pop.populationSize() - 1
            )
            popTournament.saveRoute(i, pop.getRoute(randomID))
        return popTournament.getFittest()


class Ambulance:
    def __init__(self, Start=0, End=0, isFree=True):
        self.start      = RouteManager.getBlock(Start)
        self.end        = RouteManager.getBlock(End)
        self.route      = population(5, False)
        self.isFreeFlag = isFree

    def setStart(self, Start): self.start = Start
    def getStart(self):        return self.start
    def setEnd(self, End):     self.end = End
    def getEnd(self):          return self.end
    def setAllRoutes(self, p): self.route = p
    def getAllRoutes(self):     return self.route
    def setisFree(self, f):    self.isFreeFlag = f
    def getisFree(self):       return self.isFreeFlag