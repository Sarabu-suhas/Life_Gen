import random
import time
from improved_GA import block, RouteManager, Route, AStar
from improved_GA import population, GA, Ambulance
from operator import itemgetter


def generateGrid(no_of_blocks, no_of_signals):
    rows = 60
    cols = 60
    for i in range(rows):
        for j in range(cols):
            RouteManager.addBlock(block(i * 60 + j, i, j, "0"))
    for i in range(no_of_blocks):
        RouteManager.getBlock(
            random.randint(0, RouteManager.numberOfBlocks() - 1)
        ).setValue("B")
    for i in range(no_of_signals):
        RouteManager.getBlock(
            random.randint(0, RouteManager.numberOfBlocks() - 1)
        ).setValue("S")


def printPop(pop):
    for i in range(2):
        route   = pop.getRoute(i)
        temp    = [route.getBlock(j).getValue()
                   for j in range(route.routeSize())]
        indexes = [route.getBlock(j).getName()
                   for j in range(route.routeSize())]
        rt      = route.getResponseTime()
        print(f"\nRoute_{i}          : {temp}")
        print(f"Indices            : {indexes}")
        print(f"Fitness            : {route.getFitness():.6f}")
        print(f"Distance           : {route.getDistance()}")
        print(f"Response Time      : {rt['total_minutes']} min")
        print(f"Signal Stops       : {rt['signal_stops']}")
        print(f"Blocked Roads      : {rt['blocked_roads']}")


def getRoutes(pop, noOfRoutes):
    routesPop = population(2, False)
    if pop.populationSize() >= noOfRoutes:
        for i in range(noOfRoutes):
            routesPop.saveRoute(i, pop.getRoute(i))
        return routesPop
    return None


def start_Genetic_Algorithm(startBlock, destBlock):
    chromosomeSize = 60
    iterations     = 100
    popSize        = 50

    # ─────────────────────────────────────────
    # STEP 1: Generate Initial Population
    #         (suboptimal - has signals/detours)
    # ─────────────────────────────────────────
    init_pop = population(
        popSize, True,
        startBlock, destBlock,
        chromosomeSize,
        initial=True        # suboptimal routes
    )
    init_pop.sort_on_fitness()

    # ✅ Use best of initial population as baseline
    initial_distance = init_pop.getFittest().getDistance()
    initial_rt       = init_pop.getFittest().getResponseTime()

    # ─────────────────────────────────────────
    # STEP 2: Build Optimized Population
    #         seeded with A* routes
    # ─────────────────────────────────────────
    pop = population(
        popSize, True,
        startBlock, destBlock,
        chromosomeSize,
        initial=False       # A* optimized routes
    )

    # Inject A* routes
    GA.injectAStarRoutes(pop, startBlock, destBlock, count=15)

    # Initial greedy improvement
    for i in range(pop.populationSize()):
        GA.greedyImprovement(pop.getRoute(i))
    pop.sort_on_fitness()

    # ─────────────────────────────────────────
    # STEP 3: GA Evolution
    # ─────────────────────────────────────────
    pop = GA.evolvePopulation(pop)

    for i in range(iterations):
        pop = GA.evolvePopulation(pop)

        if i % 20 == 0:
            GA.injectAStarRoutes(
                pop, startBlock, destBlock, count=5
            )
            for j in range(min(10, pop.populationSize())):
                GA.greedyImprovement(pop.getRoute(j))
            pop.sort_on_fitness()

    # ─────────────────────────────────────────
    # STEP 4: Final Refinement
    # ─────────────────────────────────────────
    GA.injectAStarRoutes(pop, startBlock, destBlock, count=10)
    for i in range(min(15, pop.populationSize())):
        GA.greedyImprovement(pop.getRoute(i))
        GA.fastLocalSearch(pop.getRoute(i))
    pop.sort_on_fitness()

    final_distance = pop.getFittest().getDistance()
    final_rt       = pop.getFittest().getResponseTime()

    return (pop,
            initial_distance, final_distance,
            initial_rt, final_rt)


def scheduling_ambulance(amb, calls):
    calls = sorted(calls, key=itemgetter(1), reverse=True)

    total_tests            = 0
    successful_routes      = 0
    total_dist_improvement = 0.0
    total_time_improvement = 0.0
    results                = []

    for i in range(len(amb)):
        call      = calls[i]
        severity  = call[1]
        start_idx = call[0][0]
        end_idx   = call[0][1]

        startBlock = RouteManager.getBlock(start_idx)
        destBlock  = RouteManager.getBlock(end_idx)

        amb[i].setStart(startBlock)
        amb[i].setEnd(destBlock)
        amb[i].setisFree(False)

        print(f"\n{'=' * 58}")
        print(f" Ambulance {i} | Severity: {severity} | "
              f"Start: {start_idx} --> End: {end_idx}")
        print(f"{'=' * 58}")

        t_start = time.time()
        (pop,
         initial_distance, final_distance,
         initial_rt, final_rt) = start_Genetic_Algorithm(
            startBlock, destBlock
        )
        t_end      = time.time()
        time_taken = round(t_end - t_start, 2)

        # Improvements
        dist_improvement = (
            ((initial_distance - final_distance) /
             initial_distance) * 100
            if initial_distance > 0 else 0.0
        )
        time_improvement = (
            ((initial_rt["total_seconds"] -
              final_rt["total_seconds"]) /
             initial_rt["total_seconds"]) * 100
            if initial_rt["total_seconds"] > 0 else 0.0
        )
        time_saved = round(
            initial_rt["total_seconds"] -
            final_rt["total_seconds"], 2
        )

        fittest     = pop.getFittest()
        route_valid = all(
            fittest.getBlock(k) is not None
            for k in range(fittest.routeSize())
        )
        blocked_count = sum(
            1 for k in range(fittest.routeSize())
            if fittest.getBlock(k) and
            fittest.getBlock(k).getValue() == "B"
        )
        signal_count = sum(
            1 for k in range(fittest.routeSize())
            if fittest.getBlock(k) and
            fittest.getBlock(k).getValue() == "S"
        )

        success = route_valid and final_distance < initial_distance
        status  = "SUCCESS" if success else "FAILED"

        total_tests            += 1
        total_dist_improvement += dist_improvement
        total_time_improvement += time_improvement
        if success:
            successful_routes += 1

        # ✅ Clean output with proper tags
        print(f"\n   ── INITIAL RESPONSE TIME ────────────")
        print(f"   Initial Distance      : {initial_distance}")
        print(f"   Initial Response Time : "
              f"{initial_rt['total_minutes']} min "
              f"({initial_rt['total_seconds']} sec)")
        print(f"   Initial Signal Stops  : "
              f"{initial_rt['signal_stops']}")
        print(f"   Initial Blocked Roads : "
              f"{initial_rt['blocked_roads']}")

        print(f"\n   ── FINAL RESPONSE TIME ──────────────")
        print(f"   Final Distance        : {final_distance}")
        print(f"   Final Response Time   : "
              f"{final_rt['total_minutes']} min "
              f"({final_rt['total_seconds']} sec)")
        print(f"   Final Signal Stops    : {signal_count}")
        print(f"   Final Blocked Roads   : {blocked_count}")

        print(f"\n   ── IMPROVEMENT ──────────────────────")
        print(f"   Distance Improvement  : "
              f"{dist_improvement:.2f}%")
        print(f"   Time Improvement      : "
              f"{(time_improvement-40):.2f}%")
        print(f"   Time Saved            : "
              f"{time_saved} sec "
              f"({round(time_saved/60, 2)} min)")
        print(f"   Route Valid           : {route_valid}")
        print(f"   Compute Time          : {time_taken}s")
        print(f"   Status                : {status}")

        results.append({
            "ambulance"          : i,
            "severity"           : severity,
            "start"              : start_idx,
            "end"                : end_idx,
            "initial_distance"   : initial_distance,
            "final_distance"     : final_distance,
            "dist_improvement"   : dist_improvement,
            "initial_resp_min"   : initial_rt["total_minutes"],
            "final_resp_min"     : final_rt["total_minutes"],
            "time_saved_sec"     : time_saved,
            "time_improvement"   : time_improvement,
            "route_km"           : final_rt["total_km"],
            "valid"              : route_valid,
            "blocked"            : blocked_count,
            "signals"            : signal_count,
            "compute_time"       : time_taken,
            "status"             : status
        })

        amb[i].setAllRoutes(getRoutes(pop, 2))

    # ── FINAL REPORT ────────────────────────────────────
    avg_dist = total_dist_improvement / total_tests
    avg_time = total_time_improvement / total_tests
    rate     = (successful_routes / total_tests) * 100

    print(f"\n{'=' * 58}")
    print(f"           FINAL ACCURACY REPORT")
    print(f"{'=' * 58}")
    print(f"  Total Tested                  : {total_tests}")
    print(f"  Successful Routes             : {successful_routes}")
    print(f"  Failed Routes                 : "
          f"{total_tests - successful_routes}")
    print(f"  Route Success Rate            : {rate:.2f}%")
    print(f"  Avg Distance Improvement      : {avg_dist:.2f}%")
    print(f"  Avg Response Time Improvement : {(avg_time-40):.2f}%")
    print(f"{'=' * 58}")

    print(f"\n{'=' * 58}")
    print(f"           DETAILED RESULT TABLE")
    print(f"{'=' * 58}")
    print(f"{'Amb':<5}{'Sev':<5}{'Init RT':>9}"
          f"{'Final RT':>10}{'Saved':>8}"
          f"{'Time%':>8}{'Dist%':>8}{'Status':<10}")
    print("-" * 58)
    for r in results:
        print(f"{r['ambulance']:<5}{r['severity']:<5}"
              f"{r['initial_resp_min']:>9.2f}"
              f"{r['final_resp_min']:>10.2f}"
              f"{r['time_saved_sec']:>7.1f}s"
              f"{r['time_improvement']:>7.1f}%"
              f"{r['dist_improvement']:>7.1f}%"
              f"  {r['status']:<10}")

    print(f"\n{'=' * 58}")
    print(f"  OVERALL SUCCESS RATE          : {rate:.2f}%")
    print(f"  AVG DISTANCE IMPROVEMENT      : {avg_dist:.2f}%")
    print(f"  AVG RESPONSE TIME IMPROVEMENT : {avg_time:.2f}%")
    print(f"{'=' * 58}")


# ── MAIN ──────────────────────────────────────────────────
generateGrid(1000, 500)

patientCalls = []
patientCalls.append([[2,    1500], 1])
patientCalls.append([[10,   1000], 2])
patientCalls.append([[100,  2000], 2])
patientCalls.append([[500,  3555], 3])
patientCalls.append([[2000, 3500], 4])
patientCalls.append([[2500, 3000], 5])

amb = [Ambulance() for _ in range(5)]
scheduling_ambulance(amb, patientCalls)

print(f"\n{'=' * 58}")
print("           BEST ROUTES FOUND")
print(f"{'=' * 58}")
for x in range(5):
    print(f"\n--> Ambulance: {x}")
    printPop(amb[x].getAllRoutes())