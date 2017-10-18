#ifndef RRT_H_
#define RRT_H_

#include "utilities.h"

struct RRT_Planner{
  std::vector<Vertex*> tree_;
  int numofDOFs_;
  double* map_;
  int x_size_;
  int y_size_;

  RRT_Planner(int numofDOFs, double*	map, int x_size, int y_size) {
    tree_.clear();
    numofDOFs_ = numofDOFs;
    map_ = map;
    x_size_ = x_size;
    y_size_ = y_size;
  }

  void init() {
    tree_.clear();
  }

  Vertex* getNear(Vertex* q_rand) {
    // find exit from graph and compute heuristic
    Vertex* q_near = NULL;
    double min_dist = HUGE_VAL;
    for(auto v : tree_) {
      double curr_dist = getDistance(v, q_rand);
      if(curr_dist < min_dist) {
        q_near = v;
        min_dist = curr_dist;
      }
    }

    return q_near;
  }

  Vertex* getTreeEscape(Vertex* goal) {
    // find exit from graph and compute heuristic
    Vertex* escape = NULL;
    double min_dist = HUGE_VAL;
    for(auto v : tree_) {
      if(connect(v, goal, numofDOFs_, map_, x_size_, y_size_)) {
        double curr_dist = getDistance(v, goal);
        if(curr_dist < min_dist) {
          escape = v;
          min_dist = curr_dist;
        }
      }
    }

    return escape;
  }

  bool newConfig(Vertex* q_near, Vertex* q_rand, double extend_distance, Vertex* q_new) {
    // maximum extendable config
    double norm = std::min(1.0, extend_distance/getDistance(q_near, q_rand));
    for(int d=0; d<numofDOFs_; d++) {
      q_new->angles_[d] = q_near->angles_[d] + norm * (q_rand->angles_[d] - q_near->angles_[d]);
    }

    // step towards maximum extendable config
    std::vector<double> step(numofDOFs_);
    for(int d=0; d<numofDOFs_; d++) {
      step[d] = (q_new->angles_[d] - q_near->angles_[d]) / 10 ;
    }

    for(int i=1; i<=10; i++) {
      std::vector<double> interpolated_angles(numofDOFs_);
      for(int d=0; d<numofDOFs_; d++) {
        interpolated_angles[d] = q_near->angles_[d] + step[d] * i;
      }
      if(!IsValidArmConfiguration(&interpolated_angles[0], numofDOFs_, map_, x_size_, y_size_)) {
        if(i==1) {
          return false;
        } else {
          // blocked, update config
          for(int d=0; d<numofDOFs_; d++) {
            q_new->angles_[d] = q_near->angles_[d] + step[d] * (i-1);
          }
          break;
        }
      }
    }
    return true;
  }

  int extend(Vertex* q_rand, double extend_distance) {
    int status = 0;
    Vertex* q_near = getNear(q_rand);
    Vertex* q_new = new Vertex(numofDOFs_);
    if(newConfig(q_near, q_rand, extend_distance, q_new)) {
      // add edge
      q_new->parent_ = q_near;
      // add vertex
      tree_.push_back(q_new);

      if(getDistance(q_new, q_rand) < 1e-3) {
        // reached
        status = 2;
      } else {
        // advanced
        status = 1;
      }
    }
    return status;
  }

  void buildRRT(Vertex* start, int numofsamples, double extend_distance) {
    init();
    tree_.push_back(start);
    int i = 0;
    while(i < numofsamples) {
      Vertex* q_rand = new Vertex(numofDOFs_);
      getRandomVertex(numofDOFs_, q_rand);
      if(IsValidArmConfiguration(q_rand->getAnglesPtr(), numofDOFs_, map_, x_size_, y_size_)) {
        extend(q_rand, extend_distance);
        i++;
      }
    }
  }

  void retrievePath(Vertex* goal, std::vector<std::vector<double> >& plan) {
    plan.clear();
    Vertex* escape = getTreeEscape(goal);
    if(escape != NULL) {
      std::cout << "tree exit: " << escape << std::endl;
      goal->parent_ = escape;
    } else {
      std::cout << "cannot find exit from tree" << std::endl;
    }

    for(Vertex* v=goal; v!=NULL; v=v->parent_) {
      std::cout << "[ ";
      for(int i=0; i<numofDOFs_; i++) {
        std::cout << v->angles_[i] << " ";
      }
      std::cout << "]\n";
      plan.push_back(v->angles_);
    }
  }

  void query(double* start_angles, double* goal_angles, std::vector<std::vector<double> >& plan, int numofsamples, double extend_distance) {
    // reverse search
    Vertex* start = new Vertex(goal_angles, numofDOFs_);
    Vertex* goal = new Vertex(start_angles, numofDOFs_);

    // connect start and goal to graph
    buildRRT(start, numofsamples, extend_distance);

    // retrieve path, backward result of reversed search, path is start to goal
    retrievePath(goal, plan);
  }
};

#endif
