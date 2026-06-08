# IE 313 Assignment 3 — Capacitated Vehicle Routing Problem (CVRP)

**Depot:** Akhisar, Manisa  
**Solver:** Google OR-Tools (Guided Local Search metaheuristic)  
**Implementation:** Python (Pyomo for formulation, OR-Tools for solving)

---

## 1. Mathematical Formulation (MIP)

### Problem Description

A distribution company must deliver goods from a depot in **Akhisar, Manisa** to 62 districts across the Aegean region. Two types of trucks are available:

| Truck Type | Capacity | Fuel Consumption | Cost per km |
|---|---|---|---|
| Small | 10,000 kg | 20 lt/100 km | 13.0 TL/km |
| Large | 15,000 kg | 40 lt/100 km | 26.0 TL/km |

> **Fuel price:** 65 TL/lt

### Vehicle Dominance Analysis

Before formulating the model, we perform a cost-efficiency comparison:

| Configuration | Total Capacity | Total Cost/km |
|---|---|---|
| 1 × Large truck | 15,000 kg | 26.0 TL/km |
| 2 × Small trucks | 20,000 kg | 26.0 TL/km |

Two small trucks provide **33% more capacity** at the **same cost** as one large truck. Therefore, the large truck is **strictly dominated** and will never appear in an optimal solution. We formulate the model using only small trucks.

### Sets

- $N = \{0, 1, \ldots, n-1\}$: Set of all nodes (node 0 = depot Akhisar)
- $C = \{1, 2, \ldots, n-1\}$: Set of customer nodes (62 districts with positive demand)
- $K = \{1, 2, \ldots, K\}$: Set of vehicles (homogeneous small trucks)

### Parameters

| Symbol | Description | Value |
|---|---|---|
| $d_{ij}$ | Distance from node $i$ to node $j$ (km) | From Excel data |
| $q_j$ | Demand at customer $j$ (kg) | From Excel data |
| $Q$ | Vehicle capacity | 10,000 kg |
| $c$ | Cost per km | 13.0 TL/km |

- **Total demand:** 26,424 kg
- **Number of customers (positive demand):** 62

### Decision Variables

- $x_{ijk} \in \{0, 1\}$: 1 if vehicle $k$ travels directly from node $i$ to node $j$
- $u_{ik} \geq 0$: Cumulative load delivered by vehicle $k$ up to and including node $i$ (MTZ variable)

### Objective Function

$$\min \quad c \cdot \sum_{k \in K} \sum_{i \in N} \sum_{\substack{j \in N \\ j \neq i}} d_{ij} \cdot x_{ijk}$$

Minimize total fuel cost across all routes.

### Constraints

**(1) Visit constraint** — Each customer is visited exactly once:
$$\sum_{\substack{i \in N \\ i \neq j}} \sum_{k \in K} x_{ijk} = 1 \qquad \forall j \in C$$

**(2) Flow conservation** — If a vehicle enters a node, it must also leave:
$$\sum_{\substack{j \in N \\ j \neq i}} x_{ijk} = \sum_{\substack{j \in N \\ j \neq i}} x_{jik} \qquad \forall i \in N, \; k \in K$$

**(3) Depot departure** — Each vehicle leaves the depot at most once:
$$\sum_{j \in C} x_{0jk} \leq 1 \qquad \forall k \in K$$

**(4) MTZ subtour elimination + capacity**:
$$u_{ik} + q_j - Q(1 - x_{ijk}) \leq u_{jk} \qquad \forall i, j \in C, \; k \in K$$

**(5) Load bounds**:
$$q_j \leq u_{jk} \leq Q \qquad \forall j \in C, \; k \in K$$

**(6) Variable domains**:
$$x_{ijk} \in \{0, 1\}, \quad u_{ik} \geq 0$$

---

## 2. Implementation

The model was implemented in **Python** using two complementary approaches:

1. **Pyomo + HiGHS**: Exact MIP formulation following the mathematical model above ([cvrp_solution.py](file:///Users/erdogan.kocabas/Desktop/IE313_Assignment_3/cvrp_solution.py))
2. **Google OR-Tools**: Constraint programming with Guided Local Search metaheuristic ([cvrp_ortools.py](file:///Users/erdogan.kocabas/Desktop/IE313_Assignment_3/cvrp_ortools.py))

### Why Two Approaches?

The exact MIP formulation with 62 customers and 6 vehicles generates approximately **24,000 binary decision variables** and over **300,000 constraints**. After running HiGHS for **7 hours**, the solver:

- Failed to prove optimality (terminated at time limit)
- Returned a suboptimal solution of **46,410 TL**
- Produced highly unbalanced routes (one vehicle serving 26 customers, another serving only 1)

Google OR-Tools, using the **Guided Local Search** metaheuristic, found a significantly better solution of **28,964 TL** in just **30 seconds**. While OR-Tools does not guarantee mathematical optimality, the solution quality is demonstrably superior to the best feasible solution found by the exact solver within practical time limits.

> **Note on Optimality:** OR-Tools uses metaheuristic search (PATH_CHEAPEST_ARC for initial solution + GUIDED_LOCAL_SEARCH for improvement). This does not provide a certificate of optimality. However, given that the exact solver (HiGHS) could not find a better solution even after 7 hours of computation, we report the OR-Tools solution as our best-known solution.

---

## 3–4. Solution and Route Interpretation

### Optimal Routes (No Time Constraint)

| Vehicle | Route | Distance | Load | Fuel Cost |
|---|---|---|---|---|
| 1 | AKHİSAR → SOMA → KINIK → BERGAMA → DİKİLİ → ALİAĞA → FOÇA → MENEMEN → ÇİĞLİ → KARŞIYAKA → BAYRAKLI → MERKEZ(İZM) → BORNOVA → MERKEZ(MAN) → ŞEHZADELER → SARUHANLI → AKHİSAR | 377 km | 6,600 kg | 4,901 TL |
| 2 | AKHİSAR → TURGUTLU → KEMALPAŞA → KONAK → KARABAĞLAR → BUCA → GAZİEMİR → BALÇOVA → NARLIDERE → URLA → ÇEŞME → KARABURUN → GÜZELBAHÇE → SEFERİHİSAR → KUŞADASI → SELÇUK → TORBALI → BAYINDIR → TİRE → ÖDEMİŞ → BEYDAĞ → KİRAZ → SALİHLİ → GÖLMARMARA → AKHİSAR | 843 km | 9,864 kg | 10,959 TL |
| 3 | AKHİSAR → GÖRDES → KÖPRÜBAŞI → DEMİRCİ → SELENDİ → KULA → ALAŞEHİR → SARIGÖL → BUHARKENT → KARACASU → NAZİLLİ → BOZDOĞAN → YENİPAZAR → KÖŞK → MERKEZ(AYD) → EFELER → İNCİRLİOVA → KOÇARLI → ÇİNE → KARPUZLU → DİDİM → SÖKE → GERMENCİK → MENDERES → YUNUSEMRE → AKHİSAR | 1,008 km | 9,960 kg | 13,104 TL |

### Summary

| Metric | Value |
|---|---|
| **Vehicles used** | 3 (all small, 10t) |
| **Total distance** | 2,228 km |
| **Total fuel cost** | **28,964 TL** |
| **All demand satisfied** | ✅ Yes |

### Interpretation

The solver efficiently partitions the 62 customers into three geographically coherent clusters:

- **Vehicle 1** (İzmir North + Manisa): Serves northern İzmir (Bergama, Dikili, Aliağa, Foça) and central İzmir (Karşıyaka, Bornova, Çiğli), returning through Manisa. Shortest route with lightest load (6,600 kg).

- **Vehicle 2** (İzmir South + Central): Covers the İzmir metropolitan area (Konak, Buca, Gaziemir), the Çeşme/Karaburun peninsula, and the southern İzmir coast (Kuşadası, Selçuk, Torbalı), looping back through inland areas (Ödemiş, Kiraz). Near-full load (9,864 kg / 10,000 kg).

- **Vehicle 3** (Manisa East + Aydın): Traverses the eastern Manisa districts (Gördes, Demirci, Kula, Alaşehir), then sweeps through all Aydın districts, returning through Menderes. Near-full load (9,960 kg / 10,000 kg).

---

## 5. Tour Time Analysis

Assuming:
- Average speed: **70 km/h** for both truck types
- Unloading time: **10 minutes per 100 kg** (= 600 kg/hour)

| Vehicle | Driving Time | Unloading Time | **Total Tour Time** | Status |
|---|---|---|---|---|
| 1 | 5.39 h (377 km) | 11.00 h (6,600 kg) | **16.39 h** | ⚠️ Exceeds 8h |
| 2 | 12.04 h (843 km) | 16.44 h (9,864 kg) | **28.48 h** | ⚠️ Exceeds 8h |
| 3 | 14.40 h (1,008 km) | 16.60 h (9,960 kg) | **31.00 h** | ⚠️ Exceeds 8h |

**Longest tour: 31.00 hours**

All three tours significantly exceed the 8-hour shift limit. The primary contributor is **unloading time** — even Vehicle 1 with the shortest route (377 km, 5.39 h driving) requires 11 hours for unloading alone. This makes the 8-hour constraint extremely tight for this problem.

---

## 6. 8-Hour Constraint — Revised Solution

### Modified Model

We modify the visit constraint to allow unsatisfied demand:

$$\sum_{\substack{i \in N \\ i \neq j}} \sum_{k \in K} x_{ijk} \leq 1 \qquad \forall j \in C$$

And add a time constraint for each vehicle:

$$\sum_{\substack{i,j \in N \\ i \neq j}} \frac{d_{ij}}{70} \cdot x_{ijk} + \sum_{j \in C} \frac{q_j}{600} \cdot \sum_{\substack{i \in N \\ i \neq j}} x_{ijk} \leq 8 \qquad \forall k \in K$$

Where:
- First term: driving time (distance / speed)
- Second term: unloading time (demand / unloading rate)

### Time-Constrained Routes

| Vehicle | Route | Distance | Load | Cost | Tour Time |
|---|---|---|---|---|---|
| 1 | AKHİSAR → KULA → SELENDİ → DEMİRCİ → KÖPRÜBAŞI → GÖRDES → AKHİSAR | 354 km | 1,566 kg | 4,602 TL | 7.67 h ✅ |
| 2 | AKHİSAR → TURGUTLU → KEMALPAŞA → BORNOVA → MERKEZ(İZM) → AKHİSAR | 211 km | 2,568 kg | 2,743 TL | 7.29 h ✅ |
| 3 | AKHİSAR → MENDERES → SELÇUK → TORBALI → AKHİSAR | 322 km | 1,650 kg | 4,186 TL | 7.35 h ✅ |
| 4 | AKHİSAR → YUNUSEMRE → MERKEZ(MAN) → KARŞIYAKA → ÇİĞLİ → MENEMEN → AKHİSAR | 202 km | 3,000 kg | 2,626 TL | 7.89 h ✅ |
| 5 | AKHİSAR → BAYRAKLI → KARABAĞLAR → BALÇOVA → NARLIDERE → URLA → SEFERİHİSAR → GÜZELBAHÇE → ŞEHZADELER → AKHİSAR | 323 km | 1,920 kg | 4,199 TL | 7.81 h ✅ |
| 6 | AKHİSAR → BUCA → GAZİEMİR → KONAK → SARUHANLI → AKHİSAR | 202 km | 2,916 kg | 2,626 TL | 7.75 h ✅ |
| 7 | AKHİSAR → FOÇA → ALİAĞA → DİKİLİ → BERGAMA → KINIK → SOMA → AKHİSAR | 344 km | 1,770 kg | 4,472 TL | 7.86 h ✅ |
| 8 | AKHİSAR → BAYINDIR → TİRE → ÖDEMİŞ → KİRAZ → SALİHLİ → AKHİSAR | 383 km | 1,464 kg | 4,979 TL | 7.91 h ✅ |
| 9 | AKHİSAR → ALAŞEHİR → SARIGÖL → BUHARKENT → BEYDAĞ → GÖLMARMARA → AKHİSAR | 411 km | 1,260 kg | 5,343 TL | 7.97 h ✅ |

All tours are within the 8-hour limit. Longest tour: **7.97 hours**.

### Unsatisfied Demand

With the 8-hour constraint, **17 districts** cannot be served:

| District | City | Demand (kg) |
|---|---|---|
| KUŞADASI | AYDIN | 1,158 |
| MERKEZ | AYDIN | 942 |
| İNCİRLİOVA | AYDIN | 732 |
| DİDİM | AYDIN | 576 |
| KOÇARLI | AYDIN | 534 |
| NAZİLLİ | AYDIN | 534 |
| KARACASU | AYDIN | 522 |
| BOZDOĞAN | AYDIN | 516 |
| ÇİNE | AYDIN | 504 |
| SÖKE | AYDIN | 468 |
| KARPUZLU | AYDIN | 456 |
| YENİPAZAR | AYDIN | 300 |
| ÇEŞME | İZMİR | 294 |
| KÖŞK | AYDIN | 282 |
| EFELER | AYDIN | 276 |
| GERMENCİK | AYDIN | 144 |
| KARABURUN | İZMİR | 72 |

**Total unsatisfied demand: 8,310 kg (31.4% of total demand)**

### Analysis of Unsatisfied Demand

The unserved districts are overwhelmingly in **Aydın province** (15 out of 17). This is expected because Aydın is the **farthest province from the depot** (Akhisar, Manisa). The round-trip driving time alone to the southern Aydın districts approaches or exceeds 8 hours, leaving insufficient time for unloading.

The two İzmir districts that are unserved (Çeşme, Karaburun) are located at the tip of the Çeşme peninsula, which is geographically isolated and far from main routes.

### Cost Comparison

| Scenario | Vehicles | Distance | Fuel Cost | Demand Satisfied |
|---|---|---|---|---|
| Without time constraint | 3 | 2,228 km | **28,964 TL** | 100% |
| With 8-hour constraint | 9 | 2,752 km | **35,776 TL** | 68.6% |
| | | **+524 km** | **+6,812 TL (+23.5%)** | **−31.4%** |

The 8-hour constraint increases the fuel cost by **23.5%** while simultaneously leaving **31.4%** of demand unsatisfied. The cost increase comes from the need for more vehicles making shorter, less efficient routes, and the increased overhead of returning to the depot more frequently.

---

## Computational Notes

### Model Complexity

The vehicle-indexed CVRP formulation with MTZ subtour elimination generates:

| Parameter | Value |
|---|---|
| Nodes | 63 (1 depot + 62 customers) |
| Binary variables ($x_{ijk}$) | $63 \times 63 \times K$ |
| Continuous variables ($u_{ik}$) | $63 \times K$ |
| MTZ constraints | $\sim 62 \times 62 \times K$ |

For $K = 6$ vehicles: ~24,000 binary variables and ~300,000 constraints.

### Solver Comparison

| Solver | Method | Time | Best Solution | Optimal? |
|---|---|---|---|---|
| HiGHS (exact MIP) | Branch & Bound | 7 hours | 46,410 TL | ❌ Suboptimal |
| OR-Tools | Guided Local Search | 30 seconds | **28,964 TL** | ❌ Not certified |

The exact MIP solver (HiGHS) was unable to find or prove an optimal solution within 7 hours of computation. The OR-Tools metaheuristic found a solution that is **37% better** than the best solution found by HiGHS.

This demonstrates the well-known computational challenge of CVRP: it is NP-hard, and exact methods scale poorly beyond ~20-30 customers. For practical problems of this size (62 customers), metaheuristic approaches such as Guided Local Search, Adaptive Large Neighborhood Search, or Genetic Algorithms are the standard practice in operations research.

While we cannot certify the OR-Tools solution as mathematically optimal, the fact that the exact solver's best feasible solution is significantly worse provides strong evidence that the metaheuristic solution is of high quality.
