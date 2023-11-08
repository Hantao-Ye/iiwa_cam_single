#include <geometry_msgs/Pose.h>

#include <iostream>
#include <stack>
#include <string>
#include <vector>

namespace cam {
class Frame {
 private:
  friend class Kuka;

  int joints_num;
  std::vector<double> joint_pos;
  std::pair<geometry_msgs::Pose, int> cart_pos;

 public:
  Frame() = default;
  Frame(int total_joints) : joints_num(total_joints) {}

  const std::pair<geometry_msgs::Pose, int> &get_cartesian_pos() const {
    return cart_pos;
  }
  const std::vector<double> &get_joint_pos() const { return joint_pos; }

  bool set_joint_pos(const std::vector<double> &joint_position) {
    if (joint_position.size() != joints_num)
      return false;
    joint_pos = joint_position;
    return true;
  }

  bool set_joint_pos(std::vector<double> &&joint_position) {
    if (joint_position.size() != joints_num)
      return false;
    joint_pos = joint_position;
    return true;
  }

  bool set_cart_pos(const std::pair<geometry_msgs::Pose, int> &cart_position) {
    cart_pos = cart_position;
    return true;
  }

  bool set_cart_pos(std::pair<geometry_msgs::Pose, int> &&cart_position) {
    cart_pos = cart_position;
    return true;
  }
};
}  // namespace cam

namespace cam {
class KukaTreeNode {
 private:
  std::string name;
  std::vector<std::string> ancester_names;
  std::vector<KukaTreeNode *> children;
  KukaTreeNode *parent;

 public:
  cam::Frame *frame;

  KukaTreeNode(const std::string &_name,
               const std::vector<std::string> &_ancester_names,
               KukaTreeNode *_parent, cam::Frame *_frame) {
    name = _name;
    ancester_names = _ancester_names;
    parent = _parent;
    frame = _frame;
  }

  KukaTreeNode *get_child(const std::string &childname) {
    for (auto it = children.begin(); it != children.end(); it++) {
      if ((*it)->name == childname) {
        return *it;
      }
    }
    std::cout << childname << " is not a child of " << ((name=="r")?"the world frame":name) << std::endl;
    return nullptr;
  }

  std::vector<KukaTreeNode *> get_children() { return children; }

  std::string get_name() { return name; }

  std::vector<std::string> get_ancester_names() { return ancester_names; }

  KukaTreeNode *get_parent() { return parent; }

  friend KukaTreeNode *insert_node(const std::string &name,
                                   const std::string &abs_path,
                                   KukaTreeNode *prev_node, cam::Frame *frame);
  friend std::vector<std::string> print_tree(KukaTreeNode *tree_root);
};

std::vector<std::string> string2vector(const std::string &abs_paths) {
  std::vector<std::string> ancester_names;
  std::string delim = "/";
  size_t start_loc = 0;
  size_t end_loc = 0;

  while (abs_paths.find(delim, start_loc) == 0) {
    start_loc++;
  }
  while (end_loc != std::string::npos) {
    end_loc = abs_paths.find(delim, start_loc);
    ancester_names.push_back(abs_paths.substr(start_loc, end_loc - start_loc));
    start_loc = end_loc + 1;
  }
  return ancester_names;
}

KukaTreeNode *insert_node(const std::string &name, const std::string &abs_path,
                          KukaTreeNode *prev_node, cam::Frame *frame) {
  // compare abs_path of the new node with abs_path of the prev_node
  // if matches -> set prev_node to be the new node's parent node
  // otherwise -> check if path of the earlier node matches the path
  // once matches, set this node to be the new node's parent node

  KukaTreeNode *new_node;
  KukaTreeNode *parent_node = prev_node;
  bool found = false;

  std::vector<std::string> ancester_names = string2vector(abs_path);
  ancester_names.pop_back();

  while (!found) {
    if (ancester_names == parent_node->ancester_names) {
      found = true;
      break;
    }
    parent_node = parent_node->parent;
  }

  new_node =
      new KukaTreeNode(name, string2vector(abs_path), parent_node, frame);
  parent_node->children.push_back(new_node);

  return new_node;
}

KukaTreeNode *generate_tree(const std::vector<std::string> &frame_names,
                            std::vector<std::string> &abs_paths,
                            std::vector<cam::Frame *> &frames) {
  // add /r to front of all elements in abs_paths
  std::vector<KukaTreeNode *> children;
  KukaTreeNode *parent;
  cam::Frame *frame;
  std::vector<std::string> new_abs_paths(abs_paths.size());
  for (std::vector<std::string>::size_type i = 0; i != abs_paths.size(); i++) {
    abs_paths[i] = "/r" + abs_paths[i];
  }

  // initialize root node
  std::vector<std::string> empty;
  KukaTreeNode *root = new KukaTreeNode("r", empty, nullptr, nullptr);
  KukaTreeNode *prev_node = root;

  // create tree
  for (std::vector<std::string>::size_type i = 0; i != frame_names.size();
       i++) {
    prev_node = insert_node(frame_names[i], abs_paths[i], prev_node, frames[i]);
  }
  return root;
}

// for testing purpose
void print_string_vector(const std::vector<std::string> &v) {
  if (v.size() == 0) {
    std::cout << "empty";
  }
  for (int i = 0; i < v.size(); i++) {
    std::cout << "[" << v[i] << "]";
  }
  std::cout << std::endl;
}

// for testing purpose
std::vector<std::string> print_tree(KukaTreeNode *tree_root) {
  std::stack<KukaTreeNode *> s;
  std::vector<std::string> ans;
  s.push(tree_root);

  while (!s.empty()) {
    KukaTreeNode *temp = s.top();
    s.pop();
    ans.push_back(temp->name);
    for (int i = temp->children.size() - 1; i >= 0; i--) {
      s.push(temp->children[i]);
    }
  }
  print_string_vector(ans);  // preorder trasversal
  return ans;
}

}  // namespace cam
