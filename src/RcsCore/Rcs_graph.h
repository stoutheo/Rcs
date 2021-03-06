/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_GRAPH_H
#define RCS_GRAPH_H

typedef struct _RcsGraph          RcsGraph;
typedef struct _RcsBody           RcsBody;
typedef struct _RcsJoint          RcsJoint;
typedef struct _RcsShape          RcsShape;
typedef struct _RcsPair           RcsPair;
typedef struct _RcsCollisionMdl   RcsCollisionMdl;
typedef struct _RcsSensor         RcsSensor;


typedef enum
{
  RcsStateFull = 0,   ///< All dofs
  RcsStateIK   = 1    ///< Only IK-relevant dofs

} RcsStateType;

#include <Rcs_MatNd.h>
#include <Rcs_HTr.h>


#ifdef __cplusplus
extern "C" {
#endif



/*!
 *  \page RcsGraph Kinematics and dynamics
 *
 *  <h1> A graph library for kinematics and dynamics </h1>
 *
 *  \ref XMLParsing
 *
 *  \ref RcsGraphFunctions
 *
 *  \ref RcsKinematicsFunctions
 *
 *  \ref RcsKineticsFunctions
 *
 *  \ref RcsGraphTraversalFunctions
 *
 *  \ref RcsBodyFunctions
 *
 *  \ref RcsJointFunctions
 *
 *  \ref RcsShapeFunctions
 *
 *  \ref RcsParserFunctions
 *
 *  \ref RcsCollisionMdlFunctions
 *
 *  \ref RcsSensorFunctions
 *
 */








/*!
 * \defgroup RcsGraphTraversalFunctions Graph traversal
 *
 */


/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Depth-first traversal through a graph, starting from body. This
 *         function steps through the complete graph, even through the
 *         parents of body.
 */
RcsBody* RcsBody_depthFirstTraversalGetNext(const RcsBody* body);

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief This function returns a body in the reverse direction of
 *         RcsBody_depthFirstTraversalGetNext(). It is so-to-say an
 *         inverse depth-first step.
 */
RcsBody* RcsBody_depthFirstTraversalGetPrevious(const RcsBody* body);

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Returns the leaf-node that comes last in a depth-first traversal.
 */
RcsBody* RcsBody_getLastChild(const RcsBody* body);

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Depth-first traversal through all bodies of the graph, starting
 *         from the root body. The bodies can be accessed with variable
 *         "BODY".
 */
#define RCSGRAPH_TRAVERSE_BODIES(graph) \
  for (RcsBody* BODY = (graph)->root; \
       BODY; \
       BODY = RcsBody_depthFirstTraversalGetNext(BODY))

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Loops through the given body and its children, depth-first. Body
 *         LASTBODY is the last body that should not be reached (outside the
 *         subtree).
 */
#define RCSBODY_TRAVERSE_BODIES(body) \
  for (RcsBody* BODY = (body), \
       *LAST = RcsBody_depthFirstTraversalGetNext(RcsBody_getLastChild(BODY)); \
       BODY && BODY != LAST; \
       BODY = RcsBody_depthFirstTraversalGetNext(BODY))

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief This macro is almost identical to RCSBODY_TRAVERSE_BODIES(),
 *         except that the body argument itself will not be traversed, but
 *         only its children.
 */
#define RCSBODY_TRAVERSE_CHILD_BODIES(body)                             \
  for (RcsBody* BODY = RcsBody_depthFirstTraversalGetNext(body),        \
       *LAST = RcsBody_depthFirstTraversalGetNext(RcsBody_getLastChild(body)); \
       BODY && BODY != LAST;                                            \
       BODY = RcsBody_depthFirstTraversalGetNext(BODY))

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Traverses all joints of the graph, starting from the root body.
 *         The joints can be accessed with variable "JNT".
 */
#define RCSGRAPH_TRAVERSE_JOINTS(graph) \
  RCSGRAPH_TRAVERSE_BODIES(graph) \
  RCSBODY_TRAVERSE_JOINTS(BODY)

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Traverses all sensors of the graph, starting from the root sensor.
 *         The sensors can be accessed with variable "SENSOR".
 */
#define RCSGRAPH_TRAVERSE_SENSORS(graph) \
  for (RcsSensor *SENSOR = (graph)->sensor; SENSOR; SENSOR=SENSOR->next)

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Traverses all joints of a body, starting from the first one.
 *         The joints can be accessed with variable "JNT".
 */
#define RCSBODY_TRAVERSE_JOINTS(body) \
  for (RcsJoint *JNT = (body)->jnt  ; JNT ; JNT=JNT->next)

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Traverses all shapes of a body, starting from the first one.
 *         The joints can be accessed with variable "SHAPE".
 */
#define RCSBODY_TRAVERSE_SHAPES(body) \
  for (RcsShape **sPtr = (RcsShape**)((body)->shape), *SHAPE = *sPtr; *sPtr; \
       sPtr++, SHAPE=*sPtr)

/*! \ingroup RcsGraphTraversalFunctions
 *  \brief Traverses all pairs from a pair list, starting from the first one.
 *         The pairs can be accessed with variable "PAIR".
 */
#define RCSPAIR_TRAVERSE(pairlist) \
  for (RcsPair **pPtr = (RcsPair**)(pairlist), *PAIR = *pPtr; *pPtr; \
       pPtr++, PAIR=*pPtr)





/*!
 * \defgroup RcsGraphFunctions Basic graph functions
 *
 */


/*! \ingroup RcsGraphFunctions
 *  \brief Creates an instance of a graph. The filename refers to an XML
 *         file.
 */
RcsGraph* RcsGraph_create(const char* filename);

/*! \ingroup RcsGraphFunctions
 *  \brief Creates an instance of a graph. The buffer should contain an
 *         XML file.
 */
RcsGraph* RcsGraph_createFromBuffer(const char* buffer, unsigned int size);

/*! \ingroup RcsGraphFunctions
 *  \brief Destroys an instance of a graph. The function does nothing if
 *         self is NULL.
 */
void RcsGraph_destroy(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Scales all elements of the graph with the given scale factor.
 */
void RcsGraph_scale(RcsGraph* self, double scaleFactor);

/*! \ingroup RcsGraphFunctions
 *  \brief Blows the vector of Jacobian coordinates up to the full state
 *         vector.
 */
void RcsGraph_stateVectorFromIK(const RcsGraph* self, const MatNd* q_ik,
                                MatNd* q_full);

/*! \ingroup RcsGraphFunctions
 *  \brief In-place function of RcsGraph_stateVectorFromIK(). If the memory
 *         of q is not large enough, new memory will be allocated. The
 *         constrained elements of q will be set to 0.
 */
void RcsGraph_stateVectorFromIKSelf(const RcsGraph* self, MatNd* q);

/*! \ingroup RcsGraphFunctions
 *  \brief Compresses the full state vector to the vector of Jacobian
 *         relevant coordinates. Vector q_ik will be reshaped by the function.
 *
 *  \param[in]  self Pointer to a valid RcsGraph
 *  \param[in]  q_full Array of dimension dof x 1
 *  \param[out] q_ik Array with memory for at least nJ elements
 */
void RcsGraph_stateVectorToIK(const RcsGraph* self, const MatNd* q_full,
                              MatNd* q_ik);

/*! \ingroup RcsGraphFunctions
 *  \brief In-place function of \ref RcsGraph_stateVectorToIK.
 */
void RcsGraph_stateVectorToIKSelf(const RcsGraph* self, MatNd* q);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the state vector to q, computes the forward kinematics
 *         and recomputes the index vector for the accelerated Jacobian
 *         computation. If the graph contains any coupled joints, their
 *         kinematics will also be updated. If qp is NULL, the velocities are
 *         not updated. The function returns true if the internal q vector
 *         was modified due to kinematic joint couplings, and is not the same
 *         as the q vector given as the argument. If the sizes of the input
 *         arrays are neither RcsGraph::dof x 1 nor RcsGraph::nJ x 1, the
 *         function exits with a fatal error.
 *
 *  \param[in]  self Pointer to a valid RcsGraph
 *  \param[in]  q Joint angles array of dimension RcsGraph::dof x 1 or
 *              RcsGraph::nJ x 1 (the function takes care of the conversion).
 *  \param[in]  qp Joint velocity array of dimension RcsGraph::dof x 1 or
 *              RcsGraph::nJ x 1 (the function takes care of the conversion).
 *              If it is NULL, the velocity array RcsGraph::qp remains
 *              unchanged.
 */
bool RcsGraph_setState(RcsGraph* self, const MatNd* q, const MatNd* qp);

/*! \ingroup RcsGraphFunctions
 *  \brief Computes the forward kinematics of the graph. If vectors q and qp
 *         are not NULL, they are assumed to be of full state vector dimension
 *         (self->dof), and to contain the values for the degrees of freedom
 *         and their velocities. The function will also update the Jacobian
 *         indices considering the constraints, and recompute self->nJ. The
 *         kinematically coupled joints will not be made consistent.
 */
void RcsGraph_computeForwardKinematics(RcsGraph* self, const MatNd* q,
                                       const MatNd* qp);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the state default state vector, computes the forward
 *         kinematics and recomputes the index vector for the accelerated
 *         Jacobian computation. The default state vector corresponds to
 *         the center joint angles q_init of each joint in the XML file.
 *         If it is not specified, the center is assumed to be 0. The
 *         velocities are set to zero.
 */
void RcsGraph_setDefaultState(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Loads the graph's xml file, looks for the tag "model_state" with
 *         the specified time stamp, and if found, sets the state to the
 *         corresponding values.
 *
 *  \return true for success, false otherwise.
 */
bool RcsGraph_setStateFromXML(RcsGraph* self, const char* modelStateName,
                              int timeStamp);

/*! \ingroup RcsGraphFunctions
 * \brief This function overwrites the joint centers with the given values.
 *
 *  \param[in]  self   The graph containing the joint centers to be overwritten
 *  \param[in]  q0     Vector of new joint centers to be written to the joints
 *                     member q0. The vector must be of dimension
 *                     RcsGraph::dof x 1 or nJ x 1, otherwise the function
 *                     will exit fatally. If it is nJ, the constrained dof
 *                     remain unchanged.
 */
void RcsGraph_changeDefaultState(RcsGraph* self, const MatNd* q0);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the default state vector of the graph. It corresponds to
 *         an array holding the center angles of all joints.
 */
void RcsGraph_getDefaultState(const RcsGraph* self, MatNd* q0);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a n x 1 array with elements that correspond to the range
 *         of the respective joint. The elements are aligned in the same
 *         order as in the state vector. If type is RcsStateFull, all
 *         joints are considered (dimension is self->dof), for RcsStateIK
 *         only the relevant ones for the inverse kinematics are considered
 *         (dimension is self->nJ). The function reshapes the array W_inv to
 *         the proper size. The entries of the vector are computed as
 *         <br>
 *         invWq_i = jnt_i->weightMetric * (jnt_i->q_max - jnt_i->q_min)
 */
void RcsGraph_getInvWq(const RcsGraph* self, MatNd* invWq,
                       RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a pointer to the body with the indicated name. If
 *         several bodies have the same name, the closest one to the root
 *         node (depth first traversal) will be taken. If no matching
 *         body is found, NULL will be returned. If name or self are NULL, also
 *         NULL is returned.
 *
 *  \param[in] self   Pointer to graph. If it is NULL, the function returns
 *                    NULL.
 *  \param[in] name   Character array holding the the joint name. If it is NULL,
 *                    the function returns NULL.
 *  \return Pointer to the first (in the depth-first traversal sense) body found
 *          with the name, or NULL if no match has been found.
 */
RcsBody* RcsGraph_getBodyByName(const RcsGraph* self, const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Similar to RcsGraph_RcsGraph_getBodyByName() except that only the
 *         number of characters of argument name are compared. This allows to
 *         find bodies with names that have a suffix.
 *
 *  \param[in] self   Pointer to graph. If it is NULL, the function returns
 *                    NULL.
 *  \param[in] name   Character array holding the first characters of the joint
 *                    name. If it is NULL, the function returns NULL.
 *  \return Pointer to the first (in the depth-first traversal sense) body found
 *          with the truncated name, or NULL if no match has been found.
 */
RcsBody* RcsGraph_getBodyByTruncatedName(const RcsGraph* self,
                                         const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the summed mass of all bodies of the graph.
 */
double RcsGraph_mass(const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the summed mass of all bodies of the graph. If root is
 *         not NULL it refers to the root body
 */
double RcsGraph_massFromBody(const RcsGraph* self, const char* root);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the number of bodies in the graph.
 */
unsigned int RcsGraph_numBodies(const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the torque for all joints that is caused by the mass
 *         of a specific body. Note that parameter torque is not
 *         overwritten, but the bodyTorque is added to it. So set torque
 *         to zero before calling this method if you only want this
 *         specific torque.
 *  \author MM
 */
void RcsGraph_bodyTorque(const RcsGraph* self, const RcsBody* body,
                         MatNd* torque);

/*! \ingroup RcsGraphFunctions
 *  \brief Checks the graph for consistency. The number of errors is
 *         returned.
 *
 *         Currently, the following tests are done:
 *         - self is NULL pointer
 *         - Bodies with rigid body joints have 6 joints in the order
 *           transX-transY-transZ-rotX-rotY-rotZ (rotations are relative to
 *           previous frame, not static axis representation), and NULL or
 *           identity relative transforms A_KV
 *         - duplicate body names
 *         - joint centers out of range
 *         - joint indices out of range
 *         - Jacobian indices out of range
 *         - Joint direction index out of range
 *         - Consistency of coupled joints
 *         - Bodies have a mass >= 0
 *         - Bodies with finite inertia have a mass > 0
 *
 *  \param[in] self  Pointer to the graph to be checked.
 */
int RcsGraph_check(const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Creates a deep copy of a graph.
 *
 *         The following elements are not copied:
 *         - RcsBody::extraInfo
 *         - RcsJoint::extraInfo
 *         - RcsShape::userData
 *         - RcsGraph::userData
 *
 *         These fields are not known on this level, please handle yourself.
 *         Otherwise, the graph's structure is fully preserved.
 *
 *  \param[in] src  Pointer to the graph to be copied.
 *  \return Pointer to copy of the graph. If argument src is NULL, the
 *                  function returns NULL.
 */
RcsGraph* RcsGraph_clone(const RcsGraph* src);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the bdyNum-th generic body pointer to the given body.
 *         Index bdyNum must be [0...9]. The body must be a part of
 *         the graph, otherwise it is unlinked. The function returns the
 *         bodie against which the generic body is linked. It is not possible
 *         to link a generic body to itself or another generic body. In this
 *         case, a warning is issued on debug level 1, and NULL is returned.
 */
RcsBody* RcsGraph_linkGenericBody(RcsGraph* self, int bdyNum,
                                  const char* bdyName);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the pointer to the body that is linked to the generic
 *         body. If no body is linked or value bdyNum is out of range (it is
 *         valid from 0 to 9), NULL is returned.
 */
RcsBody* RcsGraph_getGenericBodyPtr(const RcsGraph* self, int bdyNum);

/*! \ingroup RcsGraphFunctions
 *  \brief Please explain.
 */
void RcsGraph_insertBody(RcsGraph* graph, RcsBody* parent, RcsBody* body);

/*! \ingroup RcsGraphFunctions
 *  \brief Please explain.
 */
void RcsGraph_insertJoint(RcsGraph* graph, RcsBody* body, RcsJoint* jnt);

/*! \ingroup RcsGraphFunctions
 *  \brief Re-order joint indices according to depth-first traversal.
 *         Otherwise, there might be differences in jacobiIndex and jointIndex
 *         after a RcsGraph_setState() function.
 *         Link pointers of coupled joints and initialize range from master
 *         (if its a complex coupling)
 *
 *  \param[in] self  Pointer to a valid RcsGraph (Must not be NULL)
 */
void RcsGraph_makeJointsConsistent(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets all unconstrained joints of the graph to random values and
 *         computes the forward kinematics
 */
void RcsGraph_setRandomState(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Appends a deep copy of the RcsGraph other to the body root of the
 *         graph self. The body root must not have any children, and the graph
 *         that will be appended must have an unique root body (without any next
 *         pointer). If the root body is NULL, the RcsGraph other will be
 *         appended on the top level. The merged graph is set to the state of
 *         both original graphs. The sensors of the other graph are appended to
 *         the sensor array of self.
 *
 *  \param[in] self   Graph to be extended
 *  \param[in] root   Body of attachement. It must belong to RcsGraph self. If
 *                    it doesn't, the behavoir of the function is undefined. The
 *                    body must not have any children. If it does, the function
 *                    fails and returns false.
 *  \param[in] other  Graph that will be attached to the body root. A deep copy
 *                    of this graph will be made internally.
 *  \param[in] suffix Character string that will be appended to all bodies,
 *                    joints and sensors of the appended graph. If it is NULL,
 *                    the body names remain unchanged.
 *  \param[in] A_BP   Optional transformation between other graph and root body.
 *  \return           True for success, false for failure. Only these failure
 *                    cases are checked: The body root must not have any
 *                    children, and the other graph must have an unique root
 *                    body (without next body on the same level).
 */
bool RcsGraph_appendCopyOfGraph(RcsGraph* self, RcsBody* root,
                                const RcsGraph* other, const char* suffix,
                                const HTr* A_BP);



/**
 * @name Joints
 *
 * Functions to access / modify the graph's joints
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Limits the joint angles to the range limits. The function
 *         returns false (0) if the joints are within their range,
 *         true if one or more joints had been clipped.
 */
bool RcsGraph_limitJoints(const RcsGraph* self, MatNd* q, RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the number of joints whose limits are violated.
 */
unsigned int RcsGraph_numJointLimitsViolated(const RcsGraph* self,
                                             bool verbose);

/*! \ingroup RcsGraphFunctions
 *  \brief This function checks all joint and joint speed limits of the
 *         given trajectory q. Array q is assumed to have all
 *         (RcsGraph::dof) or the constrained (RcsGraph::nJ) degrees of
 *         freedom in each row, otherwise false (failure) is returned.
 *         Each row corresponds to the state at one time step.
 *
 *  \param[in] self   Pointer to graph, must not be NULL.
 *  \param[in] q      Trajectory as described above, must not be NULL.
 *  \param[in] dt     Time interval between two consecutive joint vectors. This
 *                    is needed to determine the joint velocities.
 *  \return True if no limit hs been violated, false otherwise.
 */
bool RcsGraph_checkJointTrajectory(const RcsGraph* self, const MatNd* q,
                                   double dt);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the lower and upper joint limits into q_lower and q_upper.
 *         The arrays hold all dof for RcsStateTypeFull, and the unconstrained
 *         ones for RcsStateTypeIK. The arrays are reshaped to column vectors
 *         of the corresponsing size.
 */
void RcsGraph_getJointLimits(const RcsGraph* self, MatNd* q_lower,
                             MatNd* q_upper, RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the joint speed limits into q_dot_limit. The arrays hold all
 *         dof for RcsStateTypeFull, and the unconstrained ones for
 *         RcsStateTypeIK. The array is reshaped to a column vector
 *         of the corresponsing size.
 */
void RcsGraph_getSpeedLimits(const RcsGraph* self, MatNd* q_dot_limit,
                             RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the joint torque limits into T_limit. The arrays hold all
 *         dof for RcsStateTypeFull, and the unconstrained ones for
 *         RcsStateTypeIK. The array is reshaped to a column vector
 *         of the corresponsing size.
 */
void RcsGraph_getTorqueLimits(const RcsGraph* self, MatNd* T_limit,
                              RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the joint angle of the joint with the given name. If this
 *         joint doesn't exist, the function will terminate the application.
 */
double RcsGraph_getJointValue(const RcsGraph* self, const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a pointer to the joint with the indicated name. If
 *         several joints have the same name, the closest one to the root
 *         node (depth first traversal) will be taken. If no matching joint
 *         is found, NULL will be returned.
 *  \param[in] self  Pointer to graph. If it is NULL, the function returns NULL.
 *  \param[in] name  Character array holding the name of the joint. If it is
 *                   NULL, the function returns NULL.
 *  \return Pointer to joint with the given name. If no joint with this name
 *          exist, the function returns NULL.
 */
RcsJoint* RcsGraph_getJointByName(const RcsGraph* self, const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Similar to RcsGraph_getJointByName() except that only the number of
 *         characters of argument name are compared. This allows to find joints
 *         with names that have a suffix.
 *  \param[in] self   Pointer to graph, must not be NULL.
 *  \param[in] name   Character array holding the first characters of the joint
 *                    name, must not be NULL.
 *  \return Pointer to joint with the given name. If several joints with the
 *          same name exist, the first one according to a depth-first
 *          traversal is returned. If no joint with this name exist, the
 *          function returns NULL.
 */
RcsJoint* RcsGraph_getJointByTruncatedName(const RcsGraph* self,
                                           const char* name);

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a pointer to the joint with the given index. If no matching
 *         joint is found, NULL will be returned.
 *
 *  \param[in] self   Pointer to graph. If it is NULL, the function
 *                    returns NULL.
 *  \param[in] idx    Index of the joint, see above description.
 *  \param[in] type   If type is RcsStateIK, the searched index is the
 *                    jacobiIndex, otherwise the jointIndex.
 *  \return Pointer to joint with the corresponsing index. If no joint with
 *          this index exist, the function returns NULL.
 */
RcsJoint* RcsGraph_getJointByIndex(const RcsGraph* self, unsigned int idx,
                                   RcsStateType type);

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the value of the indicated degree of freedom to the value.
 *         If the joint doesn't exist, false will be returned, and a debug
 *         message will be outputted on the console on debug level 1. The
 *         joint angle will then remain unchanged.
 */
bool RcsGraph_setJoint(RcsGraph* self, const char* jntName, double val);

/*! \ingroup RcsGraphFunctions
 *  \brief Limits the joint speeds according to the limits given.
 *         The function returns the scaling factor with which dq is scaled.
 */
double RcsGraph_limitJointSpeeds(const RcsGraph* self, MatNd* dq,
                                 double dt, RcsStateType type);


///@}



/**
 * @name Coupled Joints
 *
 * Functions to handle kinematically coupled joints
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Returns the number of kinematically coupled joints. Coupled
 *         joints of joints that are constrained are not considered.
 *
 *  \param[in] self  Pointer to a valid RcsGraph (Must not be NULL)
 */
int RcsGraph_countCoupledJoints(const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Constraint matrix for coupled joints and its inverse. The matrix
 *         \f$ \mathbf{A^{-1}} \f$ reduces the number of degrees of freedom
 *         by the number of coupled joints:
 *
 *         \f[ \mathbf{ \delta q_{red} = A^{-1} \delta q } \f]
 *
 *         The matrix \f$ \mathbf{A} \f$ expands the number of degrees of
 *         freedom by the number of coupled joints:
 *
 *         \f[ \mathbf{ \delta q = A \delta q_{red}}\f]
 *
 *         Since the Jacobians are holding the dof in the columns, it's the
 *         other way around:
 *
 *         \f[ \mathbf{ J_{red} = J A }\f]
 *
 *         We create a reduced Jacobian JA = J * A with matrix A being the
 *         constraint matrix. The inverse kinematics then becomes
 *
 *         \f[ \mathbf{ \delta q = A (J A)^{\#} \delta x}\f]
 *
 *         Matrix A is of dimension nq x nqr, where
 *         nqr = nq - nCoupledJoints.
 *
 *         The function returns the number of coupled joints.
 *
 *         Note: It is currently not possible to couple a joint to another
 *               coupled joint.
 *
 *  \param[in] self  Pointer to a valid RcsGraph (Must not be NULL)
 *  \param[out] A       Coupled joints reduction matrix. It will be reshaped
 *                      to RcsGraph::nJ x nqr, where nqr is the dimension of
 *                      the reduced state space by the coupled joints.
 *  \param[out] invA    nqr x nq coupled joints reduction matrix. It will be
 *                      reshaped to RcsGraph::nJ x nqr, where nqr is the
 *                      dimension of the reduced state space by the coupled
 *                      joints.
 */
int RcsGraph_coupledJointMatrix(const RcsGraph* self, MatNd* A, MatNd* invA);

/*! \ingroup RcsGraphFunctions
 *  \brief Updates the values of the slave joints to be consistent with the
 *         value of the master joint. The function returns true if one or
 *         more joint values are changed.
 *
 *  \param[in] self  Pointer to a valid RcsGraph (Must not be NULL)
 *  \param[in,out] q State vector (Dimension must be RcsGraph::dof x 1). The
 *                   elements corresponding to slave joints will be updated
 *                   according to the joint coupling equation. All other
 *                   elements remain unchanged.
 *  \param[in,out] qp State velocity vector. If this argument is NULL, the
 *                    velocities will not get updated. Otherwise, qp's
 *                    dimension must be RcsGraph::dof x 1). The velocities
 *                    corresponding to slave joints will be updated according
 *                    to the joint coupling equation. All other elements
 *                    remain unchanged.
 */
bool RcsGraph_updateSlaveJoints(const RcsGraph* self, MatNd* q, MatNd* qp);

///@}



/**
 * @name Rigid body degrees of freedom
 *
 * Functions to access / modify a bodie's rigid body degrees of freedom
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Sets the degrees of freedom of all bodies that have the flag
 *         rigid_body_joints to their initial values (q_init), and their
 *         velocities to zero, and then computes the forward kinematics.
 *         All other internal q and qp vector elements remain unchanged.
 */
void RcsGraph_resetRigidBodyDofs(RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the joint angles of all bodies that have the flag
 *         rigid_body_joints into q. Array q is assumed to have the same
 *         dimensions as the graph's q-vector (RcsGraph::dof).
 *
 *  \param[in,out] q   Array of joint angles. The size of this array must
 *                     match the size of the q-array of the second argument
 *                     (self).
 *  \param[in]  self   The graph against which the rigid body dofs are
 *                     compared and updated.
 *  \param[in]  src    If src is != NULL, then this q array will be used
 *                     as src instead of self->q
 *  \return True if one or more rigid body joint values in q and the graph
 *          differed (and have been updated), false if the values are
 *          identical.
 */
bool RcsGraph_copyRigidBodyDofs(MatNd* q, const RcsGraph* self,
                                const MatNd* src);

/*! \ingroup RcsGraphFunctions
 *  \brief Computes the angles of three translational and three rotational
 *         (euler) joints connecting a rigid body to its parent body (can
 *         be ROOT), if the body is constructed with the "rigid_body_joints"
 *         xml tag. Pass the RcsBody and optional A_KIs for the body and
 *         its parent body. The six joint angles are written to "angles".
 *         The calculation also considers the optional, constant \f$A_{KV}\f$
 *         transformation from after the six joints to the body.
 *
 *  Assuming the given body is body \f$1\f$ and its parent is body
 *  \f$0\f$ with transformations \f$A^1_{KI}\f$ and \f$A^0_{KI}\f$
 *  respectively, and the frame before body \f$1\f$ that is connected
 *  via \f$A_{KV}\f$ be named \f$1'\f$, the computed rotation of the
 *  wanted transformation from frame \f$0\f$ to frame \f$1'\f$ is
 *  given by
 *  \f[
 *    A_{1'0} = A_{1'1} A_{1I} A_{I0}
 *            = A^1_{KV}\mbox{.rot}^T \, A^1_{KI}\mbox{.rot} \,\, A^0_{KI}\mbox{.rot}^T
 *  \f]
 *  and the translation by
 *  \f{eqnarray*}{
 *    _0r_{01'} &=& A_{0I} (_Ir_{I1'} - _Ir_{I0} ) \\
 *              &=& A_{0I} (_Ir_{I1} - _Ir_{1'1} - _Ir_{I0} ) \\
 *              &=& A_{0I} (_Ir_{I1} - A_{I1} A_{11'} \;_{1'}r_{1'1} - _Ir_{I0} ) \\
 *              &=& A^0_{KI}\mbox{.rot} \, (A^1_{KI}\mbox{.org}
 *                  - A^1_{KI}\mbox{.rot} \,\, A^1_{KV}\mbox{.rot} \,\, A^1_{KV}\mbox{.org}
 *                  -  A^0_{KI}\mbox{.org} \,)
 *  \f}
 */
void RcsGraph_relativeRigidBodyDoFs(const RcsBody* body,
                                    const HTr* new_A_KI_body,
                                    const HTr* new_A_KI_parent,
                                    double angles[6]);

/*! \ingroup RcsGraphFunctions
 *  \brief Copies the joint angles into the graph's state vector elements that
 *         correspond to the rigid body degrees of freedom.
 *
 *  \param[in,out] self Pointer to a valid RcsGraph
 *  \param[in] body     Pointer to the body that owns the rigid body joints
 *  \param[in] angles   Six values to be assigned to the rigid body joints in
 *                      the order x, y, z, thx, thy, thz
 *  \return True for success, false for failure:
 *          - self is NULL
 *          - body is NULL
 *          - body has no rigid body degrees of freedom
 */
bool RcsGraph_setRigidBodyDoFs(RcsGraph* self, const RcsBody* body,
                               const double angles[6]);

///@}


/**
 * @name Graph printing and writing to file
 *
 * Printing the graph in various formats to various outputs.
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Prints the graph's xml representation to the given file
 *         descriptor. Both arguments are assumed to be not NULL. Otherwise,
 *         the function exits with a fatal error.
 *
 *  \param[in] out  Valid file descriptor for xml file
 *  \param[in] self  Pointer to a valid RcsGraph
 */
void RcsGraph_fprintXML(FILE* out, const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints out the graph to a file descriptor.
 */
void RcsGraph_fprint(FILE* out, const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints all joints of the graph in consecutive (starting
 *         from root) order to a file.
 */
void RcsGraph_fprintJoints(FILE* out, const RcsGraph* self);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints the Jacobian recursion path starting in the indicated body
 *         to a file descriptor.
 */
void RcsGraph_fprintJointRecursion(FILE* out, const RcsGraph* self,
                                   const char* bdyName);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints out the state vector in a human readeable form. The
 *         function automatically determines if the vector is of type
 *         RcsStateFull or RcsStateIK. If neither is the case or the array
 *         has not exactly one column, it will be printed without human-
 *         readable units, and a warning will be issued.
 */
void RcsGraph_printState(const RcsGraph* self, const MatNd* q);

/*! \ingroup RcsGraphFunctions
 *  \brief Prints out the state vector in the format of the xml model state. The
 *         function automatically determines if the vector is of type
 *         RcsStateFull or RcsStateIK. If neither is the case or the array
 *         has not exactly one column, it will be printed without human-
 *         readable units, and a warning will be issued.
 */
void RcsGraph_fprintModelState(FILE* out, const RcsGraph* self, const MatNd* q);

/*! \ingroup RcsGraphFunctions
 *  \brief Writes the graph to a file in the Graphviz dot format. The graph
 *         can then be visualized with for instance "dotty" or any
 *         other graphviz capable visualization tool. The file will be written
 *         in the same directory as the xml graph file.
 */
void RcsGraph_writeDotFile(const RcsGraph* self, const char* filename);

/*! \ingroup RcsGraphFunctions
 *  \brief Writes the depth-first-traversal structure of the graph to a file in
 *         the Graphviz dot format. The graph can then be visualized with for
 *         instance "dotty" or any other graphviz capable visualization tool.
 *         The file will be written in the same directory as the xml graph file.
 */
void RcsGraph_writeDotFileDfsTraversal(const RcsGraph* self,
                                       const char* filename);


///@}


/**
 * @name Sensor-related graph functions
 *
 */

///@{

/*! \ingroup RcsGraphFunctions
 *  \brief Returns a pointer to the sensor with the indicated name. If
 *         several sensors have the same name, the closest one to the root
 *         node will be taken. If no matching sensor is found, NULL will be
 *         returned.
 */
RcsSensor* RcsGraph_getSensorByName(const RcsGraph* self, const char* name);


///@}


#ifdef __cplusplus
}
#endif

#endif   // RCS_GRAPH_H
