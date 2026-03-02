#include <stdio.h>
#include <string.h>
#include <time.h>

#include "fp_math.h"
#include "qp_solver.h"

static double fp_to_double(fixed_point_t value)
{
    return FP_TO_DOUBLE(value);
}

static fixed_point_t compute_objective(
    const QuadraticProgramProblem_t *problem,
    const fixed_point_t *u)
{
    int n = problem->variable_count;
    fixed_point_t Hu[QP_MAXIMUM_VARIABLES];
    fp_mat_vec_mul(problem->hessian_matrix, u, Hu, n, n);

    fixed_point_t half = FP_HALF;
    fixed_point_t quad = 0;
    fixed_point_t lin = 0;

    for (int i = 0; i < n; i++)
    {
        quad = fp_add_sat(quad, fp_mul(u[i], Hu[i]));
        lin = fp_add_sat(lin, fp_mul(problem->linear_cost_vector[i], u[i]));
    }

    return fp_add(fp_mul(half, quad), lin);
}

static int check_constraints(const QuadraticProgramProblem_t *problem, const fixed_point_t *u)
{
    for (uint16_t ci = 0; ci < problem->constraint_count; ci++)
    {
        const fixed_point_t *row = &problem->constraint_matrix[ci * problem->variable_count];
        fixed_point_t lhs = 0;

        for (uint16_t vi = 0; vi < problem->variable_count; vi++)
        {
            lhs = fp_add_sat(lhs, fp_mul(row[vi], u[vi]));
        }

        if (lhs > fp_add(problem->constraint_bounds[ci], FP_CONST(0.002)))
        {
            return 0;
        }
    }

    return 1;
}

static void setup_test_problem(QuadraticProgramProblem_t *problem)
{
    qp_solver_initialize_problem(problem);
    problem->variable_count = 4;

    problem->hessian_matrix[0 * 4 + 0] = FP_CONST(8.0);
    problem->hessian_matrix[1 * 4 + 1] = FP_CONST(6.0);
    problem->hessian_matrix[2 * 4 + 2] = FP_CONST(4.0);
    problem->hessian_matrix[3 * 4 + 3] = FP_CONST(2.0);

    problem->linear_cost_vector[0] = FP_CONST(-1.0);
    problem->linear_cost_vector[1] = FP_CONST(2.0);
    problem->linear_cost_vector[2] = FP_CONST(-0.5);
    problem->linear_cost_vector[3] = FP_CONST(0.25);

    int c = 0;
    for (int i = 0; i < 4; i++)
    {
        problem->constraint_matrix[c * 4 + i] = FP_ONE;
        problem->constraint_bounds[c] = FP_CONST(1.0);
        c++;

        problem->constraint_matrix[c * 4 + i] = -FP_ONE;
        problem->constraint_bounds[c] = FP_CONST(1.0);
        c++;
    }

    problem->constraint_matrix[c * 4 + 0] = FP_ONE;
    problem->constraint_matrix[c * 4 + 1] = FP_ONE;
    problem->constraint_bounds[c] = FP_CONST(0.5);
    c++;

    problem->constraint_matrix[c * 4 + 2] = -FP_ONE;
    problem->constraint_matrix[c * 4 + 3] = FP_ONE;
    problem->constraint_bounds[c] = FP_CONST(0.75);
    c++;

    problem->constraint_count = c;
}

int main(void)
{
    QuadraticProgramProblem_t problem;
    QuadraticProgramConfig_t config;
    QuadraticProgramSolution_t solution_cold;
    QuadraticProgramSolution_t solution_warm;

    setup_test_problem(&problem);
    qp_solver_initialize_config(&config);
    config.maximum_iterations = 300;
    config.convergence_tolerance = FP_CONST(0.0015);

    QuadraticProgramStatus_t status_cold = qp_solver_solve(&problem, &config, &solution_cold);

    if (status_cold == QP_STATUS_INFEASIBLE || !check_constraints(&problem, solution_cold.optimal_variables))
    {
        printf("[FAIL] Cold solve failed feasibility checks\n");
        return 1;
    }

    fixed_point_t objective_cold = compute_objective(&problem, solution_cold.optimal_variables);

    memcpy(problem.initial_point, solution_cold.optimal_variables,
           sizeof(fixed_point_t) * problem.variable_count);
    problem.use_warm_start = 1;

    QuadraticProgramStatus_t status_warm = qp_solver_solve(&problem, &config, &solution_warm);

    if (status_warm == QP_STATUS_INFEASIBLE || !check_constraints(&problem, solution_warm.optimal_variables))
    {
        printf("[FAIL] Warm solve failed feasibility checks\n");
        return 1;
    }

    fixed_point_t objective_warm = compute_objective(&problem, solution_warm.optimal_variables);

    if (objective_warm > fp_add(objective_cold, FP_CONST(0.02)))
    {
        printf("[FAIL] Warm start objective regressed: cold=%.6f warm=%.6f\n",
               fp_to_double(objective_cold), fp_to_double(objective_warm));
        return 1;
    }

    const int run_count = 300;
    clock_t start = clock();
    for (int run = 0; run < run_count; run++)
    {
        (void)qp_solver_solve(&problem, &config, &solution_warm);
    }
    clock_t end = clock();

    double elapsed_seconds = (double)(end - start) / (double)CLOCKS_PER_SEC;
    double average_microseconds = (elapsed_seconds * 1e6) / (double)run_count;

    printf("[PASS] Cold status=%d iterations=%d objective=%.6f residual=%.6f\n",
           status_cold,
           solution_cold.iteration_count,
           fp_to_double(objective_cold),
           fp_to_double(solution_cold.constraint_residual));

    printf("[PASS] Warm status=%d iterations=%d objective=%.6f residual=%.6f\n",
           status_warm,
           solution_warm.iteration_count,
           fp_to_double(objective_warm),
           fp_to_double(solution_warm.constraint_residual));

    printf("[INFO] Average runtime over %d solves: %.2f us\n", run_count, average_microseconds);
    return 0;
}
