#include <bits/stdc++.h>
#include "ga_module.hpp"

using namespace std;

/*
WSN Energy-Aware Coverage Optimizer
-----------------------------------
- Greedy Initialization + Local Search Refinement
- Prints number of active sensors after each phase
*/

// ---------- Data Structures ----------
struct WSN {
    int nSensors;
    int nTargets;
    vector<vector<uint8_t>> cov;  // coverage matrix: cov[i][j] = 1 if sensor i covers target j
    vector<double> energy;        // energy[i] = energy cost of sensor i
};

struct Solution {
    vector<uint8_t> active;       // active[i] = 1 if sensor i is ON
    int coveredTargets = 0;
    double totalEnergy = 0.0;
};

// ---------- Helper Functions ----------
static inline int countCovered(const WSN& w, const vector<uint8_t>& active) {
    int covered = 0;
    for (int j = 0; j < w.nTargets; ++j) {
        bool ok = false;
        for (int i = 0; i < w.nSensors; ++i)
            if (active[i] && w.cov[i][j]) { ok = true; break; }
        covered += ok;
    }
    return covered;
}

static inline double energyUsed(const WSN& w, const vector<uint8_t>& active) {
    double sum = 0.0;
    for (int i = 0; i < w.nSensors; ++i)
        if (active[i]) sum += w.energy[i];
    return sum;
}

// ---------- Coverage Tracker (for fast updates) ----------
struct CoverTracker {
    int m;
    vector<int> count; // count[j] = # of active sensors covering target j
    int coveredCount = 0;
    explicit CoverTracker(int m) : m(m), count(m, 0), coveredCount(0) {}
    void addSensor(const vector<uint8_t>& row) {
        for (int j = 0; j < m; ++j)
            if (row[j]) {
                if (count[j] == 0) coveredCount++;
                count[j]++;
            }
    }
    void removeSensor(const vector<uint8_t>& row) {
        for (int j = 0; j < m; ++j)
            if (row[j]) {
                count[j]--;
                if (count[j] == 0) coveredCount--;
            }
    }
};

// ---------- Greedy Initialization ----------
Solution greedyInitialization(const WSN& w, double alpha = 1.5, double coverageTargetRatio = 1.0) {
    vector<uint8_t> active(w.nSensors, 0);
    CoverTracker ct(w.nTargets);
    const int targetGoal = int(ceil(coverageTargetRatio * w.nTargets));

    auto scoreOf = [&](int i) -> double {
        if (active[i] || w.energy[i] <= 0.0) return 0.0;
        int newCover = 0;
        for (int j = 0; j < w.nTargets; ++j)
            if (w.cov[i][j] && ct.count[j] == 0)
                newCover++;
        if (newCover == 0) return 0.0;
        return double(newCover) / pow(w.energy[i], alpha);
    };

    while (ct.coveredCount < targetGoal) {
        double bestScore = 0.0; int best = -1;
        for (int i = 0; i < w.nSensors; ++i) {
            double s = scoreOf(i);
            if (s > bestScore) { bestScore = s; best = i; }
        }
        if (best == -1) break;
        active[best] = 1;
        ct.addSensor(w.cov[best]);
    }

    Solution sol;
    sol.active = move(active);
    sol.coveredTargets = ct.coveredCount;
    sol.totalEnergy = energyUsed(w, sol.active);
    return sol;
}

// ---------- Local Search Refinement ----------
Solution localSearchRefine(const WSN& w, Solution sol, double coverageTargetRatio = 1.0, int iters = 300) {
    const int targetGoal = int(ceil(coverageTargetRatio * w.nTargets));
    CoverTracker ct(w.nTargets);
    for (int i = 0; i < w.nSensors; ++i)
        if (sol.active[i]) ct.addSensor(w.cov[i]);

    auto meetsCoverage = [&]() { return ct.coveredCount >= targetGoal; };

    // Remove redundant sensors
    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 0; i < w.nSensors; ++i)
            if (sol.active[i]) {
                ct.removeSensor(w.cov[i]);
                if (meetsCoverage()) {
                    sol.active[i] = 0;
                    sol.totalEnergy -= w.energy[i];
                    improved = true;
                } else {
                    ct.addSensor(w.cov[i]); // revert
                }
            }
    }

    // Random swap search
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> uni(0, w.nSensors - 1);
    for (int it = 0; it < iters; ++it) {
        int a = -1, b = -1;
        for (int t = 0; t < 20; ++t) { int c = uni(rng); if (sol.active[c]) { a = c; break; } }
        for (int t = 0; t < 20; ++t) { int c = uni(rng); if (!sol.active[c]) { b = c; break; } }
        if (a == -1 || b == -1) continue;

        ct.removeSensor(w.cov[a]);
        ct.addSensor(w.cov[b]);
        double newEnergy = sol.totalEnergy - w.energy[a] + w.energy[b];
        if (meetsCoverage() && newEnergy < sol.totalEnergy) {
            sol.active[a] = 0;
            sol.active[b] = 1;
            sol.totalEnergy = newEnergy;
        } else {
            ct.removeSensor(w.cov[b]);
            ct.addSensor(w.cov[a]);
        }
    }

    sol.coveredTargets = countCovered(w, sol.active);
    return sol;
}

// ---------- Synthetic Data Generator ----------
WSN generateSynthetic(int nSensors, int nTargets, double radius = 0.25) {
    std::mt19937 rng(123);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    std::uniform_real_distribution<double> E(1.0, 5.0);

    vector<pair<double, double>> sensors(nSensors), targets(nTargets);
    for (auto &p : sensors) p = {U(rng), U(rng)};
    for (auto &p : targets) p = {U(rng), U(rng)};

    WSN w;
    w.nSensors = nSensors;
    w.nTargets = nTargets;
    w.cov.assign(nSensors, vector<uint8_t>(nTargets, 0));
    w.energy.resize(nSensors);

    double R2 = radius * radius;
    for (int i = 0; i < nSensors; ++i) {
        w.energy[i] = E(rng);
        for (int j = 0; j < nTargets; ++j) {
            double dx = sensors[i].first - targets[j].first;
            double dy = sensors[i].second - targets[j].second;
            if (dx * dx + dy * dy <= R2)
                w.cov[i][j] = 1;
        }
    }
    return w;
}

// ---------- Main Function ----------
int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    // Parameters
    int nSensors = 60, nTargets = 80;
    double radius = 0.25;
    double alpha = 1.5;
    double covTarget = 1.0;
    int refineIters = 300;

    // Generate synthetic WSN instance
    WSN w = generateSynthetic(nSensors, nTargets, radius);

    // Run Greedy
    auto t0 = chrono::high_resolution_clock::now();
    Solution greedy = greedyInitialization(w, alpha, covTarget);
    auto t1 = chrono::high_resolution_clock::now();

    // Count active sensors after Greedy
    int greedyActive = 0;
    for (auto v : greedy.active) greedyActive += (v != 0);

    // Run Local Search refinement
    Solution refined = localSearchRefine(w, greedy, covTarget, refineIters);
    auto t2 = chrono::high_resolution_clock::now();

    // Count active sensors after refinement
    int refinedActive = 0;
    for (auto v : refined.active) refinedActive += (v != 0);

    // Compute runtimes
    auto msGreedy = chrono::duration_cast<chrono::milliseconds>(t1 - t0).count();
    auto msRefined = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

    // ---- Output Results ----
    cout << "WSN: sensors=" << w.nSensors << ", targets=" << w.nTargets << "\n";
    cout << "Greedy:  covered=" << greedy.coveredTargets << "/" << w.nTargets
         << ", energy=" << fixed << setprecision(3) << greedy.totalEnergy
         << ", time(ms)=" << msGreedy << "\n";
    cout << "Active sensors after Greedy: " << greedyActive << "\n";

    cout << "Refined: covered=" << refined.coveredTargets << "/" << w.nTargets
         << ", energy=" << fixed << setprecision(3) << refined.totalEnergy
         << ", time(ms)=" << msRefined << "\n";
    cout << "Active sensors after Refine: " << refinedActive << "\n";

    


double lambdaPenalty = 0.01;
auto tGA0 = chrono::high_resolution_clock::now();
Solution ga = runGA(w, covTarget, lambdaPenalty, 40, 60);
auto tGA1 = chrono::high_resolution_clock::now();
int gaActive = 0; for(auto v:ga.active) if(v) gaActive++;
auto msGA = chrono::duration_cast<chrono::milliseconds>(tGA1 - tGA0).count();

cout << "GA:      covered=" << ga.coveredTargets << "/" << w.nTargets
     << ", energy=" << fixed << setprecision(3) << ga.totalEnergy
     << ", time(ms)=" << msGA << "\n";
cout << "Active sensors after GA: " << gaActive << "\n";

return 0;
}
