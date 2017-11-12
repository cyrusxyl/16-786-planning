#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <algorithm>
#include <cmath>
#include <vector>

using namespace std;

struct Action {
  int index_, item_, base_, target_;
  Action() {
    index_ = -1;
    item_ = -1;
    base_ = -1;
    target_ = -1;
  }

  Action(int index, int item, int base, int target) {
    index_ = index;
    item_ = item;
    base_ = base;
    target_ = target;
  }

  vector<int> flatten() {
    vector<int> flat = {index_, item_, base_, target_};
    return flat;
  }

  void printAction() {
    printf("move %d from %d to %d\n", item_, base_, target_);
  }
};

struct State {
  int on_len_;
  int clear_len_;
  vector<pair<int, int>> on_literals_;
  vector<int> clear_literals_;

  int g_value_ = HUGE_VAL;
  int h_value_ = HUGE_VAL;
  State *parent_ = NULL;
  Action moveToReachHere_;

  State() {
    on_len_ = 0;
    clear_len_ = 0;
  }

  State(const vector<pair<int, int>> &onV, int onV_length,
        const vector<int> &clearV, int numofclear) {
    on_len_ = onV_length;
    clear_len_ = numofclear;
    on_literals_ = onV;
    clear_literals_ = clearV;

    sort(clear_literals_.begin(), clear_literals_.end());
    sort(on_literals_.begin(), on_literals_.end());
  }

  int getBase(int item) {
    for (int i = 0; i < on_len_; i++) {
      if (on_literals_[i].first == item) {
        return on_literals_[i].second;
      }
    }
    return -1;
  }

  static bool isSame(const State &A, const State &B) {
    if (!(A.clear_literals_ == B.clear_literals_)) {
      return false;
    }
    for (int i = 0; i < A.on_len_; i++) {
      if (!(A.on_literals_[i] == B.on_literals_[i])) {
        return false;
      }
    }
    return true;
  }

  static int isIn(State *s, const vector<State *> &list) {
    int index = -1;
    for (int i = 0; i < list.size(); i++) {
      if (isSame(*s, *list[i])) {
        index = i;
        break;
      }
    }
    return index;
  }

  void printState() {
    printf("clear: [");
    for (int i = 0; i < clear_len_; i++) {
      printf("%d, ", clear_literals_[i]);
    }
    printf("], ");

    printf("on: [");
    for (int i = 0; i < on_len_; i++) {
      printf("%d %d, ", on_literals_[i].first, on_literals_[i].second);
    }
    printf("], g=%d, h=%d\n", g_value_, h_value_);
  }
};

bool sortAStar(State *a, State *b) {
  return a->g_value_ + a->h_value_ < b->g_value_ + b->h_value_;
}

struct World {
  int numofblocks_;
  vector<int> blocksV_;
  int numoftriangles_;
  vector<int> trianglesV_;
  int tableIndex_;
  vector<bool> isTriangle_;
  State *start_;
  State *goal_;
  int moveActionIndex_;
  int moveToTableActionIndex_;

  World(const vector<int> &blocksV, int numofblocks,
        const vector<int> &trianglesV, int numoftriangles, int TableIndex,
        const vector<pair<int, int>> &onV_start, int onV_start_length,
        const vector<int> &clearV_start, int numofclear_start,
        const vector<pair<int, int>> &onV_goal, int onV_goal_length,
        const vector<int> &clearV_goal, int numofclear_goal,
        int moveActionIndex, int moveToTableActionIndex) {
    numofblocks_ = numofblocks;
    blocksV_ = blocksV;
    numoftriangles_ = numoftriangles;
    trianglesV_ = trianglesV;
    tableIndex_ = TableIndex;
    moveActionIndex_ = moveActionIndex;
    moveToTableActionIndex_ = moveToTableActionIndex;

    isTriangle_.resize(numofblocks + numoftriangles + 1, false);
    for (int i = 0; i < numoftriangles; i++) {
      int v = trianglesV_[i];
      isTriangle_[v] = true;
    }

    start_ =
        new State(onV_start, onV_start_length, clearV_start, numofclear_start);
    goal_ = new State(onV_goal, onV_goal_length, clearV_goal, numofclear_goal);
  }

  vector<int> getTarget(State *currentState, int item) {
    vector<int> targets;
    for (int i = 0; i < currentState->clear_len_; i++) {
      int target = currentState->clear_literals_[i];
      if (target != item && !isTriangle_[target]) {
        targets.push_back(target);
      }
    }
    return targets;
  }

  vector<Action> getAction(State *currentState) {
    vector<Action> actions;
    for (int i = 0; i < currentState->clear_len_; i++) {
      int item = currentState->clear_literals_[i];
      int base = currentState->getBase(item);
      vector<int> targets = getTarget(currentState, item);
      for (int j = 0; j < targets.size(); j++) {
        Action move(moveActionIndex_, item, base, targets[j]);
        actions.push_back(move);
      }
      if (base != tableIndex_) {
        Action moveToTable(moveToTableActionIndex_, item, base, tableIndex_);
        actions.push_back(moveToTable);
      }
    }
    return actions;
  }

  int getHeuristic(State *currentState) {
    int notSatisfied = 0;
    // for (int i = 0; i < goal_->clear_len_; i++) {
    //   if (!binary_search(currentState->clear_literals_.begin(),
    //                      currentState->clear_literals_.end(),
    //                      goal_->clear_literals_[i])) {
    //     notSatisfied++;
    //   }
    // }

    for (int i = 0; i < goal_->on_len_; i++) {
      if (!binary_search(currentState->on_literals_.begin(),
                         currentState->on_literals_.end(),
                         goal_->on_literals_[i])) {
        notSatisfied++;
      }
    }
    return notSatisfied;
  }

  State *applyAction(State *currentState, const Action &move) {
    State *successorState = new State();
    for (int i = 0; i < currentState->clear_len_; i++) {
      int idx = currentState->clear_literals_[i];
      if (idx != move.target_) {
        successorState->clear_literals_.push_back(idx);
      }
    }
    if (move.base_ != tableIndex_) {
      successorState->clear_literals_.push_back(move.base_);
    }
    sort(successorState->clear_literals_.begin(),
         successorState->clear_literals_.end());

    for (int i = 0; i < currentState->on_len_; i++) {
      int idx = currentState->on_literals_[i].first;
      if (idx != move.item_) {
        successorState->on_literals_.push_back(currentState->on_literals_[i]);
      } else {
        successorState->on_literals_.push_back(
            make_pair(move.item_, move.target_));
      }
    }

    successorState->clear_len_ = successorState->clear_literals_.size();
    successorState->on_len_ = successorState->on_literals_.size();

    successorState->parent_ = currentState;
    successorState->moveToReachHere_ = move;
    successorState->g_value_ = currentState->g_value_ + 1;
    successorState->h_value_ = getHeuristic(successorState);

    return successorState;
  }

  vector<vector<int>> getPlan() {
    vector<vector<int>> plan;

    bool found_path = false;
    vector<State *> OPEN;
    vector<State *> CLOSED;
    start_->g_value_ = 0;
    start_->h_value_ = getHeuristic(start_);
    OPEN.push_back(start_);
    int count = 0;
    while (!OPEN.empty()) {
      State *s = OPEN[0];
      OPEN.erase(OPEN.begin());
      CLOSED.push_back(s);
      count++;
      // printf("expanding state:\n");
      // s->printState();
      if (s->h_value_ == 0) {
        goal_ = s;
        found_path = true;
        break;
      }

      vector<Action> availableMoves = getAction(s);
      vector<State *> successors;

      for (int i = 0; i < availableMoves.size(); i++) {
        successors.push_back(applyAction(s, availableMoves[i]));
      }

      for (int i = 0; i < availableMoves.size(); i++) {
        // printf("\tsuccessor states:\n");
        // printf("\t");
        // availableMoves[i].printAction();
        // printf("\t");
        // successors[i]->printState();
        if (State::isIn(successors[i], CLOSED) == -1) {
          int index = State::isIn(successors[i], OPEN);
          if (index == -1) {
            // printf("\tpush new state\n");
            OPEN.push_back(successors[i]);
          } else if (OPEN[index]->g_value_ > s->g_value_ + 1) {
            OPEN[index]->g_value_ = s->g_value_ + 1;
            OPEN[index]->parent_ = s;
            OPEN[index]->moveToReachHere_ = availableMoves[i];

            delete successors[i];
          }
        } else {
          // printf("\tis closed\n");
        }
      }
      sort(OPEN.begin(), OPEN.end(), sortAStar);
      // printf("OPEN list has %d elements\n", (int)OPEN.size());
    }

    if (found_path) {
      printf("found a path, expanded %d states\n", count);
    } else {
      printf("somethings wrong\n");
      return plan;
    }

    for (State *ptr = goal_; ptr != start_; ptr = ptr->parent_) {
      plan.push_back(ptr->moveToReachHere_.flatten());
    }
    reverse(plan.begin(), plan.end());
    return plan;
  }
};

#endif
