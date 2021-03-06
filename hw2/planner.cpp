/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include "mex.h"
#include "prm.h"
#include "rrt.h"
#include "rrt_connect.h"
#include "rrt_star.h"
#include "utilities.h"

static void plannerPRM(double *map, int x_size, int y_size,
                       double *armstart_anglesV_rad,
                       double *armgoal_anglesV_rad, int numofDOFs,
                       std::vector<std::vector<double>> &plan,
                       int &planlength) {
  // no plan by default

  PRM_Planner prm(numofDOFs, map, x_size, y_size);
  prm.buildRoadMap(1000, 2*PI);
  prm.query(armstart_anglesV_rad, armgoal_anglesV_rad, plan);
  planlength = plan.size();

  return;
}

static void plannerRRT(double *map, int x_size, int y_size,
                       double *armstart_anglesV_rad,
                       double *armgoal_anglesV_rad, int numofDOFs,
                       std::vector<std::vector<double>> &plan,
                       int &planlength) {
  // no plan by default

  RRT_Planner rrt(numofDOFs, map, x_size, y_size, 1000, 2*PI);
  rrt.query(armstart_anglesV_rad, armgoal_anglesV_rad, plan);
  planlength = plan.size();

  return;
}

static void plannerRRTConnect(double *map, int x_size, int y_size,
                              double *armstart_anglesV_rad,
                              double *armgoal_anglesV_rad, int numofDOFs,
                              std::vector<std::vector<double>> &plan,
                              int &planlength) {
  // no plan by default

  RRT_Connect_Planner rrt_connect(numofDOFs, map, x_size, y_size, 1000, 2*PI);
  rrt_connect.query(armstart_anglesV_rad, armgoal_anglesV_rad, plan);
  planlength = plan.size();

  return;
}

static void plannerRRTStar(double *map, int x_size, int y_size,
                              double *armstart_anglesV_rad,
                              double *armgoal_anglesV_rad, int numofDOFs,
                              std::vector<std::vector<double>> &plan,
                              int &planlength) {
  RRT_Star_Planner rrt_star(numofDOFs, map, x_size, y_size, 1000, 2*PI, 3*PI);
  rrt_star.query(armstart_anglesV_rad, armgoal_anglesV_rad, plan);
  planlength = plan.size();

  return;
}

// prhs contains input parameters (3):
// 1st is matrix with all the obstacles
// 2nd is a row vector of start angles for the arm
// 3nd is a row vector of goal angles for the arm
// plhs should contain output parameters (2):
// 1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the
// ith step of the plan (there are D DoF of the arm (that is, D angles). So, j
// can take values from 0 to D-1  2nd is planlength (int)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])

{

  /* Check for proper number of arguments */
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
                      "Four input arguments required.");
  } else if (nlhs != 2) {
    mexErrMsgIdAndTxt("MATLAB:planner:maxlhs", "One output argument required.");
  }

  /* get the dimensions of the map and the map matrix itself*/
  int x_size = (int)mxGetM(MAP_IN);
  int y_size = (int)mxGetN(MAP_IN);
  double *map = mxGetPr(MAP_IN);

  /* get the start and goal angles*/
  int numofDOFs = (int)(MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
  if (numofDOFs <= 1) {
    mexErrMsgIdAndTxt("MATLAB:planner:invalidnumofdofs",
                      "it should be at least 2");
  }
  double *armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
  if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))) {
    mexErrMsgIdAndTxt("MATLAB:planner:invalidnumofdofs",
                      "numofDOFs in startangles is different from goalangles");
  }
  double *armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);

  // get the planner id
  int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
  if (planner_id < 0 || planner_id > 3) {
    mexErrMsgIdAndTxt("MATLAB:planner:invalidplanner_id",
                      "planner id should be between 0 and 3 inclusive");
  }

  // call the planner
  std::vector<std::vector<double>> plan;
  int planlength = 0;

  // you can may be call the corresponding planner function here
  if (planner_id == RRT) {
    plannerRRT(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad,
               numofDOFs, plan, planlength);
  } else if (planner_id == RRTCONNECT) {
    plannerRRTConnect(map, x_size, y_size, armstart_anglesV_rad,
                      armgoal_anglesV_rad, numofDOFs, plan, planlength);
  } else if (planner_id == RRTSTAR) {
    plannerRRTStar(map, x_size, y_size, armstart_anglesV_rad,
                      armgoal_anglesV_rad, numofDOFs, plan, planlength);
  } else if (planner_id == PRM) {
    plannerPRM(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad,
               numofDOFs, plan, planlength);
  } else {
    std::cout << "invalid planner id" << '\n';
  }

  for (int i = 0; i < planlength; i++) {
    double *angles = &plan[i][0];
    if (!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)) {
      std::cout << "configuration " << i << " not valid" << '\n';
    }
  }

  // dummy planner which only computes interpolated path
  // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad,
  //         numofDOFs, plan, planlength);

  printf("%d\n", planlength);

  /* Create return values */
  if (planlength > 0) {
    PLAN_OUT = mxCreateNumericMatrix((mwSize)planlength, (mwSize)numofDOFs,
                                     mxDOUBLE_CLASS, mxREAL);
    double *plan_out = mxGetPr(PLAN_OUT);
    // copy the values
    int i, j;
    for (i = 0; i < planlength; i++) {
      for (j = 0; j < numofDOFs; j++) {
        plan_out[j * planlength + i] = plan[i][j];
      }
    }
  } else {
    PLAN_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)numofDOFs,
                                     mxDOUBLE_CLASS, mxREAL);
    double *plan_out = mxGetPr(PLAN_OUT);
    // copy the values
    int j;
    for (j = 0; j < numofDOFs; j++) {
      plan_out[j] = armstart_anglesV_rad[j];
    }
  }
  PLANLENGTH_OUT =
      mxCreateNumericMatrix((mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL);
  double *planlength_out = mxGetPr(PLANLENGTH_OUT);
  *planlength_out = (double)planlength;

  return;
}
