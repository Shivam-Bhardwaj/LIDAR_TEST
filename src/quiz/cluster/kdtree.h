/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
    : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {


  Node *root;

  KdTree()
    : root(NULL) {}

  void insert_helper(std::vector<float> point, int id, Node **node, int count) {
    if (*node == nullptr) {
      *node = new Node(point, id);
    } else {
      uint depth = count % 2;
      if (point[depth] < (*node)->point[depth]) {
        insert_helper(point, id, &(*node)->left, count + 1);
      } else {
        insert_helper(point, id, &(*node)->right, count + 1);
      }
    }

  }

  void insert(std::vector<float> point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the root
    insert_helper(point, id, &root, 0);
  }

  void search_helper(std::vector<float> target, std::vector<int> &ids, int depth, float distance, Node *node) {
    if (node != nullptr) {
      bool is_in_box =
        (node->point[0] >= target[0] - distance) && (node->point[0] <= target[0] + distance) &&
        (node->point[1] >= target[1] - distance) && (node->point[1] <= target[1] + distance);
      if (is_in_box) {
        float d = sqrt(pow(node->point[0] - target[0], 2.0) + pow(node->point[1] - target[1], 2.0));
        if (d <= distance) { ids.push_back(node->id); }
      }
      int side = depth % 2;
      if ((target[side] - distance) < node->point[side]) {
        search_helper(target, ids, depth + 1, distance, node->left);
      }
      if ((target[side] + distance) > node->point[side]) {
        search_helper(target, ids, depth + 1, distance, node->right);
      }

    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    search_helper(target, ids, 0, distanceTol, root);
    return ids;
  }


};




