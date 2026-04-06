# 🚑 Ambulance Route Optimization using Hybrid Genetic Algorithm

## 📌 Overview

This project focuses on optimizing ambulance routing in a grid-based city model to **minimize response time** while considering:

* Blocked roads
* Traffic signals
* Distance constraints

The system schedules ambulances based on **patient severity** and computes optimal routes using an advanced **Hybrid Genetic Algorithm (GA)**.

---

## ⚙️ Previous Approach (Baseline - ~30% Performance)

### Method Used:

* Pure Genetic Algorithm (GA)

### Characteristics:

* Random initial population
* Fixed small population size (20)
* Limited iterations (10)
* No heuristic guidance
* No local optimization

### Limitations:

* Slow convergence
* High randomness in routes
* Suboptimal paths with unnecessary signals and blocked roads
* No guarantee of improvement in every run

---

## 🚀 Improved Approach (Optimized - ~50% Performance)

### 🔥 Key Enhancements:

#### 1. Hybrid Algorithm (GA + A*)

* Integrated **A*** algorithm to generate near-optimal initial routes
* Injected A* routes into GA population to guide evolution

#### 2. Smart Population Initialization

* Two-stage population:

  * Suboptimal population (baseline measurement)
  * Optimized population seeded with A* routes

#### 3. Increased Search Power

* Population size increased: **20 → 50**
* Chromosome size increased: **20 → 60**
* Iterations increased: **10 → 100**

#### 4. Periodic A* Injection

* Injected fresh A* routes every 20 iterations
* Prevents stagnation and improves convergence

#### 5. Local Optimization Techniques

* **Greedy Improvement**: removes unnecessary detours
* **Fast Local Search**: fine-tunes best routes

#### 6. Multi-Factor Fitness Evaluation

Routes are evaluated based on:

* Distance
* Response time (seconds & minutes)
* Traffic signal delays
* Blocked road penalties

#### 7. Real-Time Performance Tracking

* Tracks:

  * Distance improvement (%)
  * Response time improvement (%)
  * Time saved (seconds/minutes)
  * Route validity
  * Success rate

---

## 📊 Performance Comparison

| Metric                    | Old System | New System            |
| ------------------------- | ---------- | --------------------- |
| Algorithm Type            | GA Only    | Hybrid (GA + A*)      |
| Initial Route Quality     | Random     | A*-Guided             |
| Optimization Techniques   | None       | Greedy + Local Search |
| Response Time Improvement | ~30%       | ~50%                  |


---

## 📈 Results

### ✅ Achievements:

* **Response Time Improvement: ~50%**
* Significant reduction in:

  * Travel distance
  * Traffic signal delays
  * Blocked road encounters
* Higher success rate in generating valid optimal routes

### 📌 Example Improvements:

* Faster ambulance arrival times
* Reduced delays due to traffic signals
* More reliable routing under constraints

---

## 🧠 Approach Summary

The improved system combines:

* **Exploration (Genetic Algorithm)**
* **Exploitation (A* Heuristic)**
* **Refinement (Greedy + Local Search)**

This hybrid strategy ensures:

* Faster convergence
* Better route quality
* Consistent performance improvements

---

## 🏁 Conclusion

The transition from a **basic Genetic Algorithm** to a **Hybrid Optimization Model** resulted in a **significant performance boost**, improving response times from approximately **30% to nearly 50%**.

This demonstrates the effectiveness of combining:

* Heuristic algorithms (A*)
* Evolutionary techniques (GA)
* Local optimization methods

 ## 🏁 Output
 <img width="564" height="200" alt="image" src="https://github.com/user-attachments/assets/daa44369-614c-4c9a-b8ea-bfeb1d79b165" />





