__author__ = 'Cedric Mathou'

from AbstractNavStrategy import AbstractNavStrategy
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetPlan
from actionlib_msgs.msg import GoalID
import time
import tf
import actionlib
import yaml
import math

from common.tools.Lifo import Lifo
from common.tools import MathToolbox
from CmdTwist import CmdTwist
from GoCleanRetryReplayLastNavStrategy import GoCleanRetryReplayLastNavStrategy


class GoCloseToObject(GoCleanRetryReplayLastNavStrategy, object):
    _maxCostMapTolerance = 35
    
    MAKE_PLAN_TOLERANCE = 0.02  # in meter
    MAX_NB_RETRY = 2
    MAX_TIME_ELAPSED_PER_GOAL = 45
    MAX_NB_RETRY_FOR_VALID_MAKE_PLAN = 6
    MAX_NB_RETRY_OVERALL = 4
    TOLERANCE_TO_OBJECTIVE_STEP = 0.8  # in meter
    SLEEP_AFTER_CLEAR_COSTMAP = 1.0  # in second
    POINTS_PER_CIRCLE = 10
    MIN_RANGE_TO_GOAL = 0.60
    MAX_RANGE_TO_GOAL = 0.75

    def __init__(self, actMove_base):
        GoCleanRetryReplayLastNavStrategy.__init__(self, actMove_base)
        self.configure()

        self._tflistener = tf.TransformListener()
        try:
            rospy.wait_for_service('/move_base/make_plan', 5)
            rospy.loginfo("end service make_plan wait time")
            self._makePlan = rospy.ServiceProxy(
                '/move_base/make_plan', GetPlan)
        except Exception as e:
            rospy.logwarn("---INIT-- Service make_plan call failed: %s" % e)

    def configure(self):
        self.setMaxNbRetry(self.MAX_NB_RETRY)
        # self.setMaxTimeElapsed(60)
        self.setMaxTimeElapsedPerGoal(self.MAX_TIME_ELAPSED_PER_GOAL)
        self._maxNbRetryForValidMakePlan = self.MAX_NB_RETRY_FOR_VALID_MAKE_PLAN
        self._nbRetryForValidMakePlan = 0
        # FIXME disable replay last command
        self._isReplyLastCmdActivated = False
        self._circleRadius = self.MIN_RANGE_TO_GOAL
        self._point = 0
        self.lastSourcePose = ''
        self.lastTargetPose = ''
        self.lastRadius = ''
        self._resume = False

    def reset(self):
        super(GoCloseToObject, self).reset()
        self._nbRetryForValidMakePlan = 0

    def resume(self):
        if self.lastSourcePose != '' and self.lastTargetPose != '':
            self._resume = True
        if self.lastRadius:
            result = self.goto(self.lastSourcePose, self.lastTargetPose, self.lastRadius)
        else:
            result = self.goto(self.lastSourcePose, self.lastTargetPose)
        return result

    def goto(self, sourcePose, targetPose, radius=None):
        rospy.loginfo("-------------- GoCloseToObject  NEW NAVIGATION ORDER [%s,%s] -----------------", str(
            targetPose.position.x), str(targetPose.position.y))
        # to do make plan if no plan
        # select new goal close to initial goal and loop

        # saving info for resume command
        self.lastSourcePose = sourcePose
        self.lastTargetPose = targetPose
        self.lastRadius = radius

        # reset costmap before checking plan is valid
        self.resetCostMaps()

        # sleep to wait cost map agin (global) CAUTION global cost map frequency need to be >1hz
        rospy.sleep(self.SLEEP_AFTER_CLEAR_COSTMAP)

        if radius:
            goal = self.getValidGoalWithRadius(targetPose, radius)
        else:
            goal = self.getValidGoal(targetPose)

        if goal == None:
            goal = targetPose

        current_nb_newplan_recovery = 0
        while current_nb_newplan_recovery < self.MAX_NB_RETRY_OVERALL and not rospy.is_shutdown():
            result = super(GoCloseToObject, self).goto(sourcePose, goal)
            # if navigation failed
            if not result:
                self._resume = True

                # reset costmap before checking plan is valid
                self.resetCostMaps()

                # sleep to wait cost map agin (global) CAUTION global cost map frequency need to be >1hz
                rospy.sleep(self.SLEEP_AFTER_CLEAR_COSTMAP)

                # try next point x times
                if radius:
                    goal = self.getValidGoalWithRadius(targetPose, radius)
                else:
                    goal = self.getValidGoal(targetPose)
                # if no valid plan abord
                if goal == None:
                    return False
                else:
                    # if valid plan try again to navigate, CAUTION number of retry is limited to avoid blocking loop
                    current_nb_newplan_recovery = current_nb_newplan_recovery+1
            else:
                # if navigation success return success
                rospy.loginfo("{class_name} : Plan found with: r=%.2f, p=%d".format(
                    class_name=self.__class__.__name__), self._circleRadius, self._point)
                return True
        # navigation failed and max retry reached
        return False

    def getRobotPose(self):
        self._tflistener.waitForTransform(
            "/base_link", "/map", rospy.Time(0), rospy.Duration(5.0))
        robot_p = PoseStamped()
        robot_p.header.frame_id = "/base_link"
        robot_p.pose.position.x = 0
        robot_p.pose.position.y = 0
        robot_p.pose.position.z = 0
        robotPose = self._tflistener.transformPose("map", robot_p)
        return robotPose

    def processNewGoal(self, source, target, r, point):
        targetedPose = Pose()

        targetedPose.position.x = target.position.x + r*math.sin(2*3.14*point/self.POINTS_PER_CIRCLE)
        targetedPose.position.y = target.position.y + r*math.cos(2*3.14*point/self.POINTS_PER_CIRCLE)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.atan2(
            float(target.position.y - targetedPose.position.y), float(target.position.x - targetedPose.position.x)) + 0.785398)
        targetedPose.orientation.x = quaternion[0]
        targetedPose.orientation.y = quaternion[1]
        targetedPose.orientation.z = quaternion[2]
        targetedPose.orientation.w = quaternion[3]

        return targetedPose

    def getValidGoal(self, targetPose):
        isvalidPlan = False
        if not self._resume:
            self._circleRadius = self.MIN_RANGE_TO_GOAL
        while self._circleRadius < self.MAX_RANGE_TO_GOAL:
            rospy.loginfo("Test possible position on circle r=%.2f", self._circleRadius)
            if self._resume:
                self._point += 1
            else:
                self._point = 0
            while self._point < self.POINTS_PER_CIRCLE:
                # get current robot position
                robotPose = self.getRobotPose()

                # process new goal according tolerance
                newGoal = self.processNewGoal(
                    robotPose.pose, targetPose, self._circleRadius, self._point)
                isvalidPlan = self.isValidPlan(robotPose, newGoal)
                self._resume = False
                if isvalidPlan:
                    return newGoal
                self._point += 1
            if not isvalidPlan:
                self._circleRadius += self.MAKE_PLAN_TOLERANCE
            else:
                return newGoal
        return None

    def getValidGoalWithRadius(self, targetPose, radius):
        isvalidPlan = False
        if self._resume:
            self._circleRadius = self.lastRadius
        else:
            self._circleRadius = radius

        if self._resume:
            self._point += 1
        else:
            self._point = 0

        while self._point < self.POINTS_PER_CIRCLE:
            # get current robot position
            robotPose = self.getRobotPose()

            # process new goal according tolerance
            newGoal = self.processNewGoal(
                robotPose.pose, targetPose, self._circleRadius, self._point)
            isvalidPlan = self.isValidPlan(robotPose, newGoal)
            self._resume = False
            if isvalidPlan:
                return newGoal
            self._point += 1
        return None

    def isPtIntoCostMap(self, x, y):
        rospy.loginfo("{class_name} : Is point into cost map ?".format(class_name=self.__class__.__name__))
        global_cost_value = self.getCostMapValue(x, y, self._globalCostMap)
        rospy.loginfo("{class_name} : global cost value isPtIntoCostMap : %s".format(
            class_name=self.__class__.__name__), str(global_cost_value))

        # FIME need to adjust coord for local costmap
        # local_cost_value=self.getCostMapValue(x,y,self._localCostMap)
        local_cost_value = 0
        if global_cost_value >= self._maxCostMapTolerance or local_cost_value >= self._maxCostMapTolerance:
            return True
        else:
            return False

    def isValidPlan(self, startPoseStamped, targetPose):
        # self._makePlan get the new plan and check if ok if plan ok newGoal = Goal
        try:
            start = startPoseStamped
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time.now()

            goal.pose = targetPose
            tolerance = self.MAKE_PLAN_TOLERANCE
            current_plan = self._makePlan(start, goal, tolerance)
            if len(current_plan.plan.poses) == 0:
                return False
            else:
                if self.isPtIntoCostMap(targetPose.position.x, targetPose.position.y):
                    return False
                return True
        except Exception as e:
            rospy.logwarn("{class_name} : Service make plan call failed: %s".format(
                class_name=self.__class__.__name__) % e)
            rospy.logwarn("{class_name} : Service make plan call failed  target goal: %s".format(
                class_name=self.__class__.__name__), str(targetPose.position))
            rospy.logwarn("{class_name} : Service make plan call failed  robot pose: %s".format(
                class_name=self.__class__.__name__), str(start.pose.position))
            return False
