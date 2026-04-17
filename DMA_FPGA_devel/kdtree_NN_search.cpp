
#include "ap_fixed.h"
#include "ap_int.h"

#define DATA_DIMENSION 3
#define INDEX_DIMENSION 3

#define LEFT_INDEX 0
#define RIGHT_INDEX 1
#define PARENT_INDEX 2

#define X_index 0
#define Y_index 1
#define Z_index 2

#define N_NODES 21000
#define N_QUERY_POINTS 1000
#define N_NODES_in_TOP_URAM 1333
#define N_TOPS 10

typedef unsigned int uint;

int top_kdtree_data1[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data2[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data3[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data4[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data5[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data6[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data7[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data8[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data9[N_NODES_in_TOP_URAM * DATA_DIMENSION];
int top_kdtree_data10[N_NODES_in_TOP_URAM * DATA_DIMENSION];

int kd_tree_data_rest[(N_NODES - N_NODES_in_TOP_URAM) * DATA_DIMENSION];

uint top_kdtree_index1[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index2[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index3[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index4[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index5[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index6[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index7[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index8[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index9[N_NODES_in_TOP_URAM * INDEX_DIMENSION];
uint top_kdtree_index10[N_NODES_in_TOP_URAM * INDEX_DIMENSION];

uint kd_tree_index_rest[(N_NODES - N_NODES_in_TOP_URAM) * INDEX_DIMENSION];
int query_points[N_QUERY_POINTS * DATA_DIMENSION];

uint found_index_FPGA[N_QUERY_POINTS];

void distance_squared(int point1[DATA_DIMENSION], int point2[DATA_DIMENSION], int &distance)
{
    int sum = 0;
    for (uint i = 0; i < DATA_DIMENSION; ++i)
    {
#pragma HLS UNROLL factor = 1
        sum += (point1[i] - point2[i]) * (point1[i] - point2[i]);
    }
    distance = sum;
}

void kdtree_NN_non_recursive_FPGA_separate_start(int *data_arr_start, uint *index_arr_start, int *targets_in, uint *best_index, uint n_query_points)
{
    uint stack_index[N_NODES];
    char stack_dimension[N_NODES];
    int target[DATA_DIMENSION];

    for (uint i = 0; i < n_query_points; ++i)
    {
        for (uint j = 0; j < DATA_DIMENSION; ++j)
        {
            target[j] = targets_in[i * DATA_DIMENSION + j];
        }

        uint stack_ptr = 0;
        stack_index[stack_ptr] = 0;
        stack_dimension[stack_ptr++] = 0;

        best_index[i] = 0;
        int best_dist = 2147483647 - 1; // 2^32/2-1
        bool first_run = true;

        while (stack_ptr > 0)
        {
            // #pragma HLS pipeline
            uint index = stack_index[--stack_ptr];

            uint dimension = stack_dimension[stack_ptr];
            if (index == 0 && !first_run)
            {
                continue;
            }

            uint current_index = index;
            int current_dist;
            if (current_index < N_NODES_in_TOP_URAM)
            {
                distance_squared(&data_arr_start[current_index * DATA_DIMENSION], target, current_dist);
            }
            else
            {
                distance_squared(&kd_tree_data_rest[(current_index - N_NODES_in_TOP_URAM) * DATA_DIMENSION], target, current_dist);
            }

            if (current_dist < best_dist)
            {
                // #pragma HLS pipeline
                best_dist = current_dist;
                best_index[i] = current_index;
            }

            uint next_dimension = (dimension + 1) % DATA_DIMENSION;
            uint index_next, index_other;

            int tmp_data = 0;
            if (current_index < N_NODES_in_TOP_URAM)
            {
                tmp_data = data_arr_start[current_index * DATA_DIMENSION + dimension];
            }
            else
            {
                tmp_data = kd_tree_data_rest[(current_index - N_NODES_in_TOP_URAM) * DATA_DIMENSION + dimension];
            }

            if (target[dimension] < tmp_data)
            {
                // #pragma HLS pipeline
                if (current_index < N_NODES_in_TOP_URAM)
                {
                    index_next = index_arr_start[current_index * INDEX_DIMENSION + LEFT_INDEX];
                    index_other = index_arr_start[current_index * INDEX_DIMENSION + RIGHT_INDEX];
                }
                else
                {
                    index_next = kd_tree_index_rest[(current_index - N_NODES_in_TOP_URAM) * INDEX_DIMENSION + LEFT_INDEX];
                    index_other = kd_tree_index_rest[(current_index - N_NODES_in_TOP_URAM) * INDEX_DIMENSION + RIGHT_INDEX];
                }
            }
            else
            {
                // #pragma HLS pipeline
                if (current_index < N_NODES_in_TOP_URAM)
                {
                    index_next = index_arr_start[current_index * INDEX_DIMENSION + RIGHT_INDEX];
                    index_other = index_arr_start[current_index * INDEX_DIMENSION + LEFT_INDEX];
                }
                else
                {
                    index_next = kd_tree_index_rest[(current_index - N_NODES_in_TOP_URAM) * INDEX_DIMENSION + RIGHT_INDEX];
                    index_other = kd_tree_index_rest[(current_index - N_NODES_in_TOP_URAM) * INDEX_DIMENSION + LEFT_INDEX];
                }
            }

            int plane_dist = target[dimension] - tmp_data;
            plane_dist *= plane_dist;
            // std::cout << "plane_dist: " << plane_dist << std::endl;

            if (plane_dist < best_dist && index_other != 0)
            {
                if (stack_ptr < N_NODES)
                {
                    // #pragma HLS pipeline
                    stack_index[stack_ptr] = index_other;
                    stack_dimension[stack_ptr++] = next_dimension;
                }
            }
            if (index_next != 0)
            {
                if (stack_ptr < N_NODES)
                {
                    // #pragma HLS pipeline
                    stack_index[stack_ptr] = index_next;
                    stack_dimension[stack_ptr++] = next_dimension;
                }
            }
            first_run = false;
        }
    }
}

void memcpy(uint *dest, uint *src, int size)
{
    for (int i = 0; i < size; ++i)
    {
#pragma HLS UNROLL factor = 16
        dest[i] = src[i];
    }
}

void memcpy(int *dest, int *src, int size)
{
    for (int i = 0; i < size; ++i)
    {
#pragma HLS UNROLL factor = 16
        dest[i] = src[i];
    }
}

void load_query_points(int *query_points_in, uint n_query_points)
{
    memcpy(query_points, query_points_in, n_query_points * DATA_DIMENSION);
}

void load_kdtree(int *point_in, uint *index_in, uint n_nodes)
{
    uint tmp_size = n_nodes * DATA_DIMENSION;
    uint tmp_size_top = (tmp_size > N_NODES_in_TOP_URAM * DATA_DIMENSION) ? N_NODES_in_TOP_URAM * INDEX_DIMENSION : tmp_size;

    memcpy(top_kdtree_data1, point_in, tmp_size_top);
    memcpy(top_kdtree_data2, point_in, tmp_size_top);
    memcpy(top_kdtree_data3, point_in, tmp_size_top);
    memcpy(top_kdtree_data4, point_in, tmp_size_top);
    memcpy(top_kdtree_data5, point_in, tmp_size_top);
    memcpy(top_kdtree_data6, point_in, tmp_size_top);
    memcpy(top_kdtree_data7, point_in, tmp_size_top);
    memcpy(top_kdtree_data8, point_in, tmp_size_top);
    memcpy(top_kdtree_data9, point_in, tmp_size_top);
    memcpy(top_kdtree_data10, point_in, tmp_size_top);

    memcpy(top_kdtree_index1, index_in, tmp_size_top);
    memcpy(top_kdtree_index2, index_in, tmp_size_top);
    memcpy(top_kdtree_index3, index_in, tmp_size_top);
    memcpy(top_kdtree_index4, index_in, tmp_size_top);
    memcpy(top_kdtree_index5, index_in, tmp_size_top);
    memcpy(top_kdtree_index6, index_in, tmp_size_top);
    memcpy(top_kdtree_index7, index_in, tmp_size_top);
    memcpy(top_kdtree_index8, index_in, tmp_size_top);
    memcpy(top_kdtree_index9, index_in, tmp_size_top);
    memcpy(top_kdtree_index10, index_in, tmp_size_top);

    if (0 < tmp_size - tmp_size_top)
    {
        memcpy(kd_tree_data_rest, &point_in[N_NODES_in_TOP_URAM * DATA_DIMENSION], tmp_size - tmp_size_top);
        memcpy(kd_tree_index_rest, &index_in[N_NODES_in_TOP_URAM * INDEX_DIMENSION], tmp_size - tmp_size_top);
    }
}
// Returns the index of the closest point not the original index
void find_NNs(uint n_query_points)
{
    uint n_per_impl = n_query_points / N_TOPS;

    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data1, top_kdtree_index1, &query_points[0], &found_index_FPGA[0], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data2, top_kdtree_index2, &query_points[n_per_impl * DATA_DIMENSION], &found_index_FPGA[n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data3, top_kdtree_index3, &query_points[2 * n_per_impl * DATA_DIMENSION], &found_index_FPGA[2 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data4, top_kdtree_index4, &query_points[3 * n_per_impl * DATA_DIMENSION], &found_index_FPGA[3 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data5, top_kdtree_index5, &query_points[4 * n_per_impl * DATA_DIMENSION], &found_index_FPGA[4 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data6, top_kdtree_index6, &query_points[5 * n_per_impl * DATA_DIMENSION], &found_index_FPGA[5 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data7, top_kdtree_index7, &query_points[6 * n_per_impl * DATA_DIMENSION], &found_index_FPGA[6 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data8, top_kdtree_index8, &query_points[7 * n_per_impl * DATA_DIMENSION], &found_index_FPGA[7 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data9, top_kdtree_index9, &query_points[8 * n_per_impl * DATA_DIMENSION], &found_index_FPGA[8 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_FPGA_separate_start(top_kdtree_data10, top_kdtree_index10, &query_points[9 * n_per_impl * DATA_DIMENSION], &found_index_FPGA[9 * n_per_impl], n_per_impl);
}

extern "C"
{
    void kdtree_NN_search(
        int *tree_data_in,
        uint *tree_index_in,
        int *query_points_in,
        uint *found_query_point_index,
        bool rld_tree,
        uint n_kdtree_nodes,
        uint n_query_points
    )
    {
#pragma HLS INTERFACE m_axi port = tree_data_in offset = slave bundle = gmem0
#pragma HLS INTERFACE m_axi port = tree_index_in offset = slave bundle = gmem1
#pragma HLS INTERFACE m_axi port = query_points_in offset = slave bundle = gmem0
#pragma HLS INTERFACE m_axi port = found_query_point_index offset = slave bundle = gmem1

#pragma HLS RESOURCE variable = top_kdtree_data1 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data2 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data3 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data4 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data5 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data6 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data7 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data8 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data9 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_data10 core = RAM_2P_URAM

#pragma HLS RESOURCE variable = top_kdtree_index1 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index2 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index3 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index4 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index5 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index6 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index7 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index8 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index9 core = RAM_2P_URAM
#pragma HLS RESOURCE variable = top_kdtree_index10 core = RAM_2P_URAM

#pragma HLS RESOURCE variable = kd_tree_data_rest core = RAM_2P_URAM
#pragma HLS RESOURCE variable = kd_tree_index_rest core = RAM_2P_URAM

#pragma HLS RESOURCE variable = query_points core = RAM_2P_BRAM
#pragma HLS RESOURCE variable = found_index_FPGA core = RAM_2P_BRAM

        if (rld_tree)
        {
            load_kdtree(tree_data_in, tree_index_in, n_kdtree_nodes);
        }

        load_query_points(query_points_in, n_query_points);

        find_NNs(n_query_points);
        memcpy(found_query_point_index, found_index_FPGA, n_query_points);
    }
}
