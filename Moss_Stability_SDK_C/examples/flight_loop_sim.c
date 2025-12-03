#include <stdio.h>
#include "../include/reason_solver.h"

#define SIM_STEPS 1000
#define DT 0.001 // 1000 Hz

int main() {
    printf("[MOSS SDK] Initializing Flight Loop Simulation...\n");

    // Init with J_crit = 500.0 m/s^3
    reason_state_t* solver = reason_init(500.0);
    
    double simulated_acc = 0.0;
    
    for (int i=0; i<SIM_STEPS; i++) {
        // Simulating instability injection at step 500
        if (i > 500) simulated_acc += 1.5; 
        else simulated_acc += 0.1;

        double correction = reason_compute_safety(solver, simulated_acc, DT);
        
        if (solver->resonance_flag == 2) {
            printf("[ALERT] Step %d: Resonance Precursor! Jerk Limit Exceeded. Correction: %.4f\n", 
                   i, correction);
        }
    }

    reason_destroy(solver);
    printf("[MOSS SDK] Simulation Complete.\n");
    return 0;
}
