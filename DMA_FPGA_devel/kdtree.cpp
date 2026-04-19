// this file will generate a basic kd tree for the point cloud based on the xyz positions of the points
#include <kdtree.h>

#include <stdio.h>
#include <stdlib.h>

// take a number of expected nodes and return 1 if succes and 0 if fail.
// dim is the number of dimensions of each point, that they shout be sorted for
// n is the number of nodes that the tree should be able to hold
int kdtree_allocate(kdtree *tree, uint dim_in, uint n) {
  
  tree->dim = dim_in;
  tree->root = (kdtree_node *)calloc(n, sizeof(kdtree_node));
  uint bytes = sizeof(kdtree_node) * n;
  float mb = (float(bytes)) / 1024.0 / 1024.0;
  if(tree->root == nullptr) {
    std::cerr << "failed to allocate memory for kdtree" << std::endl;
    return 0;
  }
  return 1;
}

// Insert a point into the tree, a point is a 3d point(can contain intensities or other data)
void add_node(kdtree &tree, float data[DATA_DIMENSION]) {
  // if it is the first point
  if(tree.number_of_nodes == 0) {
    kdtree_node *new_node = tree.root;
    new_node->left_index = 0;
    new_node->right_index = 0;
    new_node->parent_index = 0;
    memcpy(new_node->data, data, sizeof(float) * DATA_DIMENSION);
    tree.number_of_nodes++;
    return;
  }

  uint current_index = 0;
  uint parent_index = 0;
  // insert into the kd tree
  bool correctly_inserted = false, left_branch = false;
  uint dimension_to_compare = 0;
  kdtree_node *parent_ptr;

  while(!correctly_inserted) {
    parent_ptr = tree.root + current_index;
    parent_index = current_index;
    if(parent_ptr == nullptr) {
      std::cout << "parent_ptr is null" << std::endl;
    }
    if(data[dimension_to_compare] < parent_ptr->data[dimension_to_compare]) {
      current_index = parent_ptr->left_index;
      left_branch = true;
    } else {
      current_index = parent_ptr->right_index;
      left_branch = false;
    }

    if(current_index == 0) {
      kdtree_node *new_node = tree.root + tree.number_of_nodes;
      new_node->left_index = 0;
      new_node->right_index = 0;
      new_node->parent_index = parent_index;
      memcpy(new_node->data, data, sizeof(float) * DATA_DIMENSION);
      if(left_branch) {
        parent_ptr->left_index = tree.number_of_nodes;
      } else {
        parent_ptr->right_index = tree.number_of_nodes;
      }
      tree.number_of_nodes++;
      correctly_inserted = true;
    }

    dimension_to_compare = (dimension_to_compare + 1) % tree.dim;
  }
}

// Insert an array of points into the tree, an array of points is a set of 3d points(can contain intensities or other data)
// Points = [[],[],...,[]] where each [] is a 3d point
// n is the number of points
// The point will be added from the middle of the points array
void kdtree_insert(kdtree &tree, float points[][DATA_DIMENSION], uint n) {
  int middle_index = n / 2;
  int offset, index;

  add_node(tree, points[middle_index]);

  for(int i = 0; i < n - 1; i++) {
    offset = std::pow(-1, i + 1) * ((i / 2) + 1);
    index = middle_index + offset;
    add_node(tree, points[index]);
    std::cout << "\r\n";
  }
}

void kdtree_insert(kdtree &tree, pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
  uint n = pc->width * pc->height;
  uint middle_index = n / 2;
  uint offset, index;
  float data[DATA_DIMENSION] = {pc->points[middle_index].x, pc->points[middle_index].y, pc->points[middle_index].z, pc->points[middle_index].intensity};
  add_node(tree, data);

  for(uint i = 0; i < n - 1; i++) {
    offset = std::pow(-1, i + 1) * ((i / 2) + 1);
    // Calculate the actual index to access
    index = middle_index + offset;
    data[X_index] = pc->points[index].x;
    data[Y_index] = pc->points[index].y;
    data[Z_index] = pc->points[index].z;
    data[INTENSITY_index] = pc->points[index].intensity;
    data[ORIGINAL_INDEX_index] = index;
    add_node(tree, data);
  }
}

void kdtree_insert(kdtree &tree, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
  uint n = pc->width * pc->height;
  uint middle_index = n / 2;
  uint offset, index;
  float data[DATA_DIMENSION] = {pc->points[middle_index].x, pc->points[middle_index].y, pc->points[middle_index].z, 0};
  add_node(tree, data);
  for(uint i = 1; i < n - 1; i++) {
    offset = std::pow(-1, i + 1) * ((i / 2) + 1);
    index = middle_index + offset;

    data[X_index] = pc->points[index].x;
    data[Y_index] = pc->points[index].y;
    data[Z_index] = pc->points[index].z;
    data[ORIGINAL_INDEX_index] = index;
    add_node(tree, data);
  }
}

float distance_squared(kdtree &tree, float a_data[DATA_DIMENSION], float b_data[DATA_DIMENSION]) {
  float sum = 0;
  for(uint i = 0; i < tree.dim; i++) {
    sum += (a_data[i] - b_data[i]) * (a_data[i] - b_data[i]);
  }
  return sum;
}

float kdtree_distance(kdtree &tree, float a_data[DATA_DIMENSION], float b_data[DATA_DIMENSION]) {
  return sqrt(distance_squared(tree, a_data, b_data));
}

// For the user to use, as the start index should not be inserted by the user
kdtree_node *kdtree_NN(kdtree &tree, float target[DATA_DIMENSION]) {
  return kdtree_NN_intern(tree, target, 0, 0);
}

kdtree_node *kdtree_NN_intern(kdtree &tree, float target[DATA_DIMENSION], uint starting_index, uint dimension_to_compare) {
  kdtree_node *current_node = tree.root + starting_index;
  if(current_node->left_index == 0 && current_node->right_index == 0) {
    return current_node;
  }

  uint index_next, index_other;
  if(target[dimension_to_compare] < current_node->data[dimension_to_compare]) {
    index_next = current_node->left_index;
    index_other = current_node->right_index;
  } else {
    index_next = current_node->right_index;
    index_other = current_node->left_index;
  }

  if(index_next == 0) {
    return current_node;
  }

  kdtree_node *tmp = kdtree_NN_intern(tree, target, index_next, (dimension_to_compare + 1) % tree.dim);
  kdtree_node *best = (distance_squared(tree, tmp->data, target) < distance_squared(tree, current_node->data, target)) ? tmp : current_node;

  float r = distance_squared(tree, best->data, target);
  float r_mark = (target[dimension_to_compare] - current_node->data[dimension_to_compare]);

  if(r_mark * r_mark < r && index_other != 0) {
    tmp = kdtree_NN_intern(tree, target, index_other, (dimension_to_compare + 1) % tree.dim);
    best = (distance_squared(tree, tmp->data, target) < distance_squared(tree, best->data, target)) ? tmp : best;
  }

  return best;
}

void kdtree_print_node_info(kdtree &tree, uint index) {
  kdtree_node *node = tree.root + index;
  std::cout << "data: " << node->data[0] << ", " << node->data[1] << ", " << node->data[2] << ", " << node->data[3] << std::endl;
  std::cout << "parent_index: " << node->parent_index << std::endl;
  std::cout << "left_index: " << node->left_index << std::endl;
  std::cout << "right_index: " << node->right_index << std::endl;
  std::cout << "index: " << index << std::endl;
}

void kdtree_print_node_info(kdtree &tree, kdtree_node *node) {
  std::cout << "data: " << node->data[0] << ", " << node->data[1] << ", " << node->data[2] << ", " << node->data[3] << std::endl;
  std::cout << "parent_index: " << node->parent_index << std::endl;
  std::cout << "left_index: " << node->left_index << std::endl;
  std::cout << "right_index: " << node->right_index << std::endl;
}

// print the depth of all end nodes
void kdtree_print_depth(kdtree &tree) {
  for(int i = 0; i < tree.number_of_nodes; i++) {
    kdtree_node *node = tree.root + i;
    if(node->left_index == 0 && node->right_index == 0) {
      int depth = 0;
      while(node->parent_index != 0) {
        depth++;
        node = tree.root + node->parent_index;
      }
      std::cout << "Node " << i << " is at depth " << depth << std::endl;
    }
  }
}

void kdtree_print_all_nodes(kdtree &tree) {
  for(int i = 0; i < tree.number_of_nodes; i++) {
    kdtree_print_node_info(tree, i);
    std::cout << std::endl;
  }
}

// deletes the tree
void kdtree_free(kdtree *tree) {
  free(tree->root);
}