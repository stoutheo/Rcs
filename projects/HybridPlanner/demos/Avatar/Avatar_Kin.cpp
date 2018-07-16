/*******************************************************************************

  Copyright (C) by
  Honda Research Institute Europe GmbH,
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

*******************************************************************************/


#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_parser.h>

#include <GraphNode.h>
#include <CapsuleNode.h>
#include <HUD.h>
#include <KeyCatcher.h>
#include <JointWidget.h>
#include <RcsViewer.h>
#include <Rcs_guiFactory.h>
#include <BodyPointDragger.h>
#include <VertexArrayNode.h>
#include <MatNdWidget.h>


// for collision visualisation
#include <CapsuleNode.h>
#include <FTSensorNode.h> // for the Arrownode

// for collision detection
#include <Rcs_shape.h>


#include <SegFaultHandler.h>

#include <IkSolverRMR.h>
#include <ControllerWidgetBase.h>

#include <Rcs_sensor.h>
#include <Rcs_math.h>
#include <COSNode.h>


#include <iostream>
#include <csignal>


#include <Rcs_graph.h>
#include <math.h>
#include <Rcs_macros.h>



RCS_INSTALL_ERRORHANDLERS


bool runLoop = true;



/*******************************************************************************
 * Ctrl-C destructor. Tries to quit gracefully with the first Ctrl-C
 * press, then just exits.
 ******************************************************************************/
void quit(int /*sig*/)
{
  static int kHit = 0;
  runLoop = false;
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit + 1);
  kHit++;

  if (kHit == 2)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}


/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  RMSG("Starting Rcs...");

  const char* hgrDir = getenv("RCS_ROBOT_MODELS");
  if (hgrDir != NULL)
  {
    std::string meshDir = std::string(hgrDir);
    Rcs_addResourcePath(meshDir.c_str());
  }

  // Default parameters set
  int mode = 0;
  int simpleGraphics = 0;
  char xmlFileName[128] = "", directory[128] = "";

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  // Parse command line arguments
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode");
  argP.getArgument("-f", xmlFileName, "Configuration file name");
  argP.getArgument("-dir", directory, "Configuration file directory");
  bool valgrind = argP.hasArgument("-valgrind",
                                      "Start without Guis and graphics");
  simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without "
                                       "fancy stuff (shadows, anti-aliasing)");

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  runLoop = true;

  Rcs_addResourcePath("config");

  RPAUSE_DL(5);

  switch (mode)
  {

      // .............................
      // // Initialisation of instances
      // // .............................
      //
      // // linking to appropriate dir
      // Rcs_addResourcePath(directory);
      //
      // // parsing the xml files
      // getModel(directory, xmlFileName);
      //
      // // Create controller
      // Rcs::ControllerBase controller(xmlFileName, true);
      // Rcs::IkSolverRMR ikSolver(&controller);
      //
      //

      //
      // // set all joints to velocity control
      // if (velCntrl == true)
      // {
      //   RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
      //   {
      //     if ((JNT->ctrlType == RCSJOINT_CTRL_VELOCITY) ||
      //         (JNT->ctrlType == RCSJOINT_CTRL_TORQUE))
      //     {
      //       JNT->ctrlType = RCSJOINT_CTRL_VELOCITY;
      //       // JNT->ctrlType = RCSJOINT_CTRL_POSITION;
      //     }
      //   }
      //   RcsGraph_setState(controller.getGraph(), NULL, NULL);
      // }
      //
      // // for the reset of the initial state
      // MatNd* q0 = MatNd_clone(controller.getGraph()->q);
      //
      // // joint position related quantities
      // // replicates for control
      // MatNd* q_des = MatNd_clone(controller.getGraph()->q);             // q positions
      // MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);     // q deltas
      // MatNd* q_dot_des  = MatNd_create(controller.getGraph()->dof, 1);  // q velocities
      //
      // // actuall q values of the robot
      // MatNd* q_curr = MatNd_clone(controller.getGraph()->q);            // q positions
      // MatNd* q_dot_curr = MatNd_create(controller.getGraph()->dof, 1);  // q velocities
      //
      // // variables related to IK
      // MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);  // activation vector
      // MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);      // Gradient mat
      //
      // // task(cartesian) variables that belong to the GUI
      // MatNd* x_des   = MatNd_create(controller.getTaskDim(), 1);        // task final position
      // MatNd* x_goal = MatNd_create(controller.getTaskDim(), 1);         // task intermediate position
      // MatNd* x_dot_goal = MatNd_create(controller.getTaskDim(), 1);     // task velocities
      // MatNd* dx_des  = MatNd_create(controller.getTaskDim(), 1);        // task deltas
      //
      // // actuall task values of the robot
      // MatNd* x_curr  = MatNd_create(controller.getTaskDim(), 1);
      //
      // // set task activation vector
      // controller.readActivationsFromXML(a_des);
      // // controller.readActivationVectorFromXML(a_des, "activation");
      // // make the current activation vector
      // controller.computeX(x_curr);
      // MatNd_copy(x_curr, x_des);
      //
      // // timer and loopcounter
			// Timer* timer = Timer_create(dt);
      // unsigned int loopCount = 0;
      //
			// // Viewer and Gui
			// Rcs::KeyCatcher* kc = NULL;
			// Rcs::Viewer* viewer = NULL;
			// Rcs::HUD* hud = NULL;
      // Rcs::GraphNode* gn  = NULL;

			// if (valgrind==false)
			// {
			//   // HUD
			//   hud = new Rcs::HUD(0,0,500,160);
      //
			//   // keycather
			//   kc = new Rcs::KeyCatcher();
      //
      //   // viewer
      //   viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
      //   gn = new Rcs::GraphNode(controller.getGraph());




        // /////////////////////////////////////////////////////////////////
        // // Compute control input
        // /////////////////////////////////////////////////////////////////
        // controller.computeDX(dx_des, x_goal);
        //
        // // clip dx
        // MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
        // MatNd_saturateSelf(dx_des, &clipArr);
        //
        // // computes the gradient of a cost function w.r.t the joint limits
        // controller.computeJointlimitGradient(dH);
        //
        // // compute cost to collision
        // if (calcDistance==true)
        // {
        //   controller.computeCollisionCost();
        //   MatNd_setZero(dH);
        //   controller.computeCollisionGradient(dH);
        // }
        //
        // // compute manipulability gradient
        // if (manipulability)
        // {
        //   MatNd_setZero(dH);
        //   controller.computeManipulabilityGradient(dH, a_des);
        //   MatNd_constMulSelf(dH, 100.0);
        // }
        //
        // // mult with const
        // MatNd_constMulSelf(dH, alpha);
        //
        // // solve IK right#
        // ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
        //
        // // differentiate in case we want to control velocities
        // MatNd_constMul(q_dot_des, dq_des, 1.0/dt);
        //
        // // update q
        // MatNd_addSelf(q_des, dq_des);
        // // MatNd_addSelf(controller.getGraph()->q, dq_des);
        //
        // //////////////////////////////////////////////////////////////
        // // Forward kinematics ( update graph )
        // //////////////////////////////////////////////////////////////
        // RcsGraph_copyRigidBodyDofs(q_des, controller.getGraph(), q_curr);
        // RcsGraph_setState(controller.getGraph(), q_des, NULL);
        // controller.computeX(x_curr);
        //
        // // update cost to joint limits
        // dJlCost = -jlCost;
        // jlCost = controller.computeJointlimitCost();
        // dJlCost += jlCost;

    // ==============================================================
    // Inverse kinematics
    // ==============================================================
    case 0:
    {
      Rcs::KeyCatcherBase::registerKey("q", "Quit");
      Rcs::KeyCatcherBase::registerKey("T", "Run controller test");
      Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");
      Rcs::KeyCatcherBase::registerKey("a", "Change IK algorithm");
      Rcs::KeyCatcherBase::registerKey("d", "Write q-vector to q.dat");
      Rcs::KeyCatcherBase::registerKey("D", "Set q-vector from file q.dat");
      Rcs::KeyCatcherBase::registerKey("n", "Reset to default state");
      Rcs::KeyCatcherBase::registerKey("C", "Toggle closest point lines");
      Rcs::KeyCatcherBase::registerKey("o", "Toggle distance calculation");
      Rcs::KeyCatcherBase::registerKey("m", "Manipulability null space");
      Rcs::KeyCatcherBase::registerKey("e", "Link generic body");

      int algo = 0;
      double alpha = 0.05, lambda = 1.0e-8, tmc = 0.1, dt = 0.01, dt_calc = 0.0;
      double jlCost = 0.0, dJlCost = 0.0;
      bool calcDistance = true;
      // for scenario and robot
      strcpy(xmlFileName, "cAction-Box.xml");
      strcpy(directory, "config/xml/GenericHumanoid");

      argP.getArgument("-alpha", &alpha,
                       "Null space scaling factor (default is 0.05)");
      argP.getArgument("-lambda", &lambda, "Regularization");
      argP.getArgument("-f", xmlFileName);
      argP.getArgument("-dir", directory);
      argP.getArgument("-tmc", &tmc, "Filter time constant for sliders");
      argP.getArgument("-dt", &dt, "Sampling time interval");

      bool ffwd = argP.hasArgument("-ffwd", "Feed-forward dx only");
      bool pause = argP.hasArgument("-pause", "Pause after each iteration");
      bool launchJointWidget = argP.hasArgument("-jointWidget",
                                                "Launch JointWidget");
      bool manipulability = argP.hasArgument("-manipulability",
                                             "Manipulability criterion in "
                                             "null space");

      if (argP.hasArgument("-h"))
      {
        printf("Resolved motion rate control test\n\n");
        break;
      }

      Rcs_addResourcePath(directory);

      // Create controller
      Rcs::ControllerBase controller(xmlFileName, true);
      Rcs::IkSolverRMR ikSolver(&controller);

      MatNd* dq_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* q_dot_des  = MatNd_create(controller.getGraph()->dof, 1);
      MatNd* a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
      MatNd* x_curr  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_dot_curr = MatNd_create(controller.getTaskDim(), 1);
      MatNd* x_des   = MatNd_create(controller.getTaskDim(), 1);
      // for the collision specific tasks
      MatNd* x_des_coll = MatNd_create(6, 1);

      MatNd* x_des_f = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dx_des  = MatNd_create(controller.getTaskDim(), 1);
      MatNd* dH      = MatNd_create(1, controller.getGraph()->nJ);

      controller.readActivationsFromXML(a_des);
      controller.computeX(x_curr);
      MatNd_copy(x_des, x_curr);
      MatNd_copy(x_des_f, x_curr);

      // Create visualization
      Rcs::Viewer* v           = NULL;
      Rcs::KeyCatcher* kc      = NULL;
      Rcs::GraphNode* gn       = NULL;
      Rcs::HUD* hud            = NULL;
      Rcs::BodyPointDragger* dragger = NULL;
      Rcs::VertexArrayNode* cn = NULL;
      char hudText[2056];

      if (valgrind==false)
      {
        v       = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
        kc      = new Rcs::KeyCatcher();
        gn      = new Rcs::GraphNode(controller.getGraph());
        hud     = new Rcs::HUD();
        dragger = new Rcs::BodyPointDragger();
        dragger->scaleDragForce(0.01);
        v->add(gn);
        v->add(hud);
        v->add(kc);
        v->add(dragger);

        if (controller.getCollisionMdl() != NULL)
        {
          cn = new Rcs::VertexArrayNode(controller.getCollisionMdl()->cp,
                                        osg::PrimitiveSet::LINES, "RED");
          cn->toggle();
          v->add(cn);
        }

        v->runInThread(mtx);

        // Launch the task widget
        if (ffwd == false)
        {
          Rcs::ControllerWidgetBase::create(&controller, a_des, x_des,
                                            x_curr, mtx);
        }
        else
        {
          // Launch the task widget
          MatNdWidget* mw = MatNdWidget::create(dx_des, x_curr,
                                                -1.0, 1.0, "dx",
                                                mtx);

          std::vector<std::string> labels;
          for (size_t id=0; id<controller.getNumberOfTasks(); id++)
          {
            for (unsigned int j=0; j<controller.getTaskDim(id); j++)
              labels.push_back(controller.getTaskName(id) +
                               std::string(": ") +
                               controller.getTask(id)->getParameter(j)->name);
          }

          mw->setLabels(labels);

          mw = MatNdWidget::create(a_des, a_des,
                                   0.0, 1.0, "activation",
                                   &graphLock);
          labels.clear();
          for (size_t id=0; id<controller.getNumberOfTasks(); id++)
          {
            labels.push_back(controller.getTaskName(id));
          }
          mw->setLabels(labels);
        }


        if (launchJointWidget==true)
        {
          Rcs::JointWidget::create(controller.getGraph(), mtx);
        }


      }

      //////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////
      //                           TEMPORALY LOCATION                         //
      //////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////
      // provide contact visualisation points

      // AVATAR 0
      double I_closestPtsAv0R[6];
      VecNd_setZero(I_closestPtsAv0R, 6);

      // RIGHT
      double* cp0Av0R = &I_closestPtsAv0R[0];
      Rcs::CapsuleNode* sphereCP0Av0R = new Rcs::CapsuleNode(cp0Av0R, NULL, 0.015, 0.0);
      sphereCP0Av0R->makeDynamic(cp0Av0R);
      sphereCP0Av0R->setMaterial("GREEN");
      v->add(sphereCP0Av0R);

      double* cp1Av0R = &I_closestPtsAv0R[3];
      Rcs::CapsuleNode* sphereCP1Av0R = new Rcs::CapsuleNode(cp1Av0R, NULL, 0.015, 0.0);
      sphereCP1Av0R->makeDynamic(cp1Av0R);
      sphereCP1Av0R->setMaterial("RED");
      v->add(sphereCP1Av0R);

      // ArrowNode for normal vector
      double n0R[3];
      Vec3d_setZero(n0R);
      double* npAv0R = &n0R[0];
      Rcs::ArrowNode* normalArrow0R = new Rcs::ArrowNode(cp0Av0R, npAv0R, 0.2);
      // v->add(normalArrow0R);

      // LEFT
      double I_closestPtsAv0L[6];
      VecNd_setZero(I_closestPtsAv0L, 6);

      double* cp0Av0L = &I_closestPtsAv0L[0];
      Rcs::CapsuleNode* sphereCP0Av0L = new Rcs::CapsuleNode(cp0Av0L, NULL, 0.015, 0.0);
      sphereCP0Av0L->makeDynamic(cp0Av0L);
      sphereCP0Av0L->setMaterial("BLUE");
      v->add(sphereCP0Av0L);

      double* cp1Av0L = &I_closestPtsAv0L[3];
      Rcs::CapsuleNode* sphereCP1Av0L = new Rcs::CapsuleNode(cp1Av0L, NULL, 0.015, 0.0);
      sphereCP1Av0L->makeDynamic(cp1Av0L);
      sphereCP1Av0L->setMaterial("YELLOW");
      v->add(sphereCP1Av0L);

      // ArrowNode for normal vector
      double n0L[3];
      Vec3d_setZero(n0L);
      double* npAv0L = &n0L[0];
      Rcs::ArrowNode* normalArrow0L = new Rcs::ArrowNode(cp0Av0L, npAv0L, 0.2);
      // v->add(normalArrow0L);

      //////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////
      // object stuff

      // objects' name
      const char* objName = "Man_obj";
      // objects' pointer
      RcsBody* obj = RcsGraph_getBodyByTruncatedName(controller.getGraph(), objName);

      // hand's name
      const char* rHandName = "RightHandTip";
      // hand' pointer
      RcsBody* rHand = RcsGraph_getBodyByTruncatedName(controller.getGraph(), rHandName);

      // hand's name
      const char* lHandName = "LeftHandTip";
      // hand' pointer
      RcsBody* lHand = RcsGraph_getBodyByTruncatedName(controller.getGraph(), lHandName);


      //////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////

      bool coll_flag = false;

      unsigned int loopCount = 0;

      // Endless loop
      while (runLoop == true)
      {
        pthread_mutex_lock(&graphLock);

        dt_calc = Timer_getTime();

        if (ffwd==false)
        {



          if (coll_flag == true){
            for (unsigned int i=6; i<x_des_f->m; i++)
            {
              x_des_f->ele[i] = tmc*x_des->ele[i] +
                                (1.0-tmc)*x_des_f->ele[i];
            }

            // For the tasks that are controlled with the collision hack!
            for (unsigned int i=0; i<6; i++)
            {
              x_des_f->ele[i] = tmc*x_des_coll->ele[i] + (1.0-tmc)*x_des_f->ele[i];
            }
          }
          else{
            for (unsigned int i=0; i<x_des_f->m; i++)
            {
              x_des_f->ele[i] = tmc*x_des->ele[i] +
                                (1.0-tmc)*x_des_f->ele[i];
            }
          }

          controller.computeDX(dx_des, x_des_f);
          double clipLimit = 0.1;
          MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
          MatNd_saturateSelf(dx_des, &clipArr);
        }

        controller.computeJointlimitGradient(dH);

        if (calcDistance==true)
        {
          controller.computeCollisionCost();
        }

        if (manipulability)
        {
          MatNd_setZero(dH);
          controller.computeManipulabilityGradient(dH, a_des);
          MatNd_constMulSelf(dH, 100.0);
        }

        MatNd_constMulSelf(dH, alpha);

        if (valgrind==false)
        {
          dragger->addJointTorque(dH, controller.getGraph());
        }


        ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
        // MatNd_addSelf(controller.getGraph()->q, dq_des);

        MatNd_constMul(q_dot_des, dq_des, 1.0/dt);

        MatNd_addSelf(controller.getGraph()->q, dq_des);
        RcsGraph_setState(controller.getGraph(), NULL, q_dot_des);
        controller.computeX(x_curr);
        //controller.computeXp(x_dot_curr);

        dJlCost = -jlCost;
        jlCost = controller.computeJointlimitCost();
        dJlCost += jlCost;

        dt_calc = Timer_getTime() - dt_calc;



        ////////////////////////////////////////////////////////////////////////
        // fake logic for visually reasonable partner

        // double distL = RcsShape_distance(obj->shape[0], rHand->shape[0],
        //                                  obj->A_BI, rHand->A_BI,
        //                                  cp0Av0R, cp1Av0R, npAv0R);

        double distR = RcsShape_distance(obj->shape[1], rHand->shape[0],
                                        obj->A_BI, rHand->A_BI,
                                        cp0Av0R, cp1Av0R, npAv0R);

        double distL = RcsShape_distance(obj->shape[2], lHand->shape[0],
                                        obj->A_BI, lHand->A_BI,
                                        cp0Av0L, cp1Av0L, npAv0L);
        // RMSG(" Distance is %f \n", distL);
        double dd = 0.0;
        for (unsigned int i=0; i<3; i++)
        {
          double a = (dd*rHand->A_BI->org[i] + (1- dd)*cp0Av0R[i]);
          MatNd_set(x_des_coll, i, 0, a);
        }
        for (unsigned int i=3; i<6; i++)
        {
          double b = (dd*lHand->A_BI->org[i-3] + (1-dd)*cp0Av0L[i-3]);
          MatNd_set(x_des_coll, i, 0, b);
        }

        ////////////////////////////////////////////////////////////////////////



        pthread_mutex_unlock(&graphLock);

        if (kc && kc->getAndResetKey('q'))
        {
          runLoop = false;
        }
        else if (kc && kc->getAndResetKey('T'))
        {
          RLOGS(0, "Running controller test");
          controller.test(true);
        }
        else if (kc && kc->getAndResetKey('p'))
        {
          pause = !pause;
          RMSG("Pause modus is %s", pause ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('d'))
        {
          RMSG("Writing q to file \"q.dat\"");
          MatNd* q_deg = MatNd_clone(controller.getGraph()->q);
          VecNd_constMulSelf(q_deg->ele,180.0/M_PI,q_deg->m);
          MatNd_toFile(q_deg, "q.dat");
          MatNd_destroy(q_deg);
        }
        else if (kc && kc->getAndResetKey('D'))
        {
          bool success = MatNd_fromFile(controller.getGraph()->q, "q.dat");
          RMSG("%s read q from file \"q.dat\"",
               success ? "Successfully" : "Failed to");
          RcsGraph_setState(controller.getGraph(), NULL, NULL);
        }
        else if (kc && kc->getAndResetKey('n'))
        {
          RMSG("Resetting");
          RcsGraph_setDefaultState(controller.getGraph());
        }
        else if (kc && kc->getAndResetKey('C') && cn)
        {
          RMSG("Toggle closest points visualization");
          cn->toggle();
        }
        else if (kc && kc->getAndResetKey('o'))
        {
          calcDistance = !calcDistance;
          RMSG("Distance calculation is %s", calcDistance ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('m'))
        {
          manipulability = !manipulability;
          RMSG("Manipulation index nullspace is %s",
               manipulability ? "ON" : "OFF");
        }
        else if (kc && kc->getAndResetKey('e'))
        {
          std::string bdyName;
          RMSG("Linking GenericBody");
          printf("Enter body to link against: ");
          std::cin >> bdyName;

          RcsBody* lb = RcsGraph_linkGenericBody(controller.getGraph(),
                                                 0, bdyName.c_str());
          RMSG("Linked against \"%s\"", lb ? lb->name : "NULL");
        }
        else if (kc && kc->getAndResetKey('k'))
        {
        coll_flag = true;
        RMSG("Swith from free to Collision follower!");
        }
        else if (kc && kc->getAndResetKey('j'))
        {
        coll_flag = false;
        RMSG("Swith from Collision follower to free!");
        }

        sprintf(hudText, "IK calculation: %.1f us\ndof: %d nJ: %d "
                "nqr: %d nx: %d\nJL-cost: %.6f dJL-cost: %.6f %s %s"
                "\nalgo: %d lambda:%g alpha: %g tmc: %.3f\n"
                "Manipulability index: %.6f\n",
                1.0e6*dt_calc, controller.getGraph()->dof,
                ikSolver.nq, ikSolver.nqr,
                (int) controller.getActiveTaskDim(a_des),
                jlCost, dJlCost,
                ikSolver.getDeterminant()==0.0?"SINGULAR":"",
                ((dJlCost > 1.0e-8) && (MatNd_getNorm(dx_des) == 0.0)) ?
                "COST INCREASE" : "",
                algo, lambda, alpha, tmc,
                controller.computeManipulabilityCost(a_des));


        if (hud != NULL)
        {
          hud->setText(hudText);
        }
        else
        {
          std::cout << hudText;
        }

        if ((valgrind==true) && (loopCount>10))
        {
          runLoop = false;
        }

        if (pause==true)
        {
          RPAUSE();
        }

        loopCount++;
        Timer_waitDT(0.01);
      }



      // Clean up
      if (valgrind==false)
      {
        delete v;
        RcsGuiFactory_shutdown();
      }

      MatNd_destroy(dq_des);
      MatNd_destroy(q_dot_des);
      MatNd_destroy(a_des);
      MatNd_destroy(x_curr);
      MatNd_destroy(x_dot_curr);
      MatNd_destroy(x_des);
      MatNd_destroy(x_des_f);
      MatNd_destroy(dx_des);
      MatNd_destroy(dH);
      break;
    }

	default:
	{
	  RMSG("there is no mode %d", mode);
	}

	}

  if ((mode!=0) && argP.hasArgument("-h"))
  {
    argP.print();
    Rcs::KeyCatcherBase::printRegisteredKeys();
    Rcs_printResourcePath();
  }

  // Clean up global stuff. From the libxml2 documentation:
  // WARNING: if your application is multithreaded or has plugin support
  // calling this may crash the application if another thread or a plugin is
  // still using libxml2. It's sometimes very hard to guess if libxml2 is in
  // use in the application, some libraries or plugins may use it without
  // notice. In case of doubt abstain from calling this function or do it just
  // before calling exit() to avoid leak reports from valgrind !
  xmlCleanupParser();

  fprintf(stderr, "Thanks for using the Rcs libraries\n");

#if defined (_MSC_VER)
  if ((mode==0) || argP.hasArgument("-h"))
  {
    RPAUSE();
  }
#endif

  return 0;
}
