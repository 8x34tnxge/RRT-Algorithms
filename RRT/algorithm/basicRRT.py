from __future__ import annotations

import random
from typing import Union

import numpy as np
from RRT.core.droneInfo import DroneInfo
from RRT.core.mapInfo import MapInfo
from RRT.core.missionInfo import MissionInfo
from RRT.core.sign import Failure, Success
from RRT.core.tree import RRT
from RRT.util.extendmethod import directlyExtend
from RRT.util.samplemethod import randomSample


class BasicRRT:
    """Basic RRT implementation"""

    ######################################
    ############# properties #############
    ######################################
    droneInfo: DroneInfo
    missionInfo: MissionInfo
    mapInfo: MapInfo
    exploreProb: np.float64
    maxAttempts: Union[np.int32, None]
    stepSize: np.float64
    searchTree: RRT

    #########################################
    ############# class methods #############
    #########################################
    def __init__(
        self,
        droneInfo: DroneInfo,
        missionInfo: MissionInfo,
        exploreProb: np.float64,
        stepSize: np.float64,
        maxAttempts: np.int32 = np.Infinity,
    ):
        """the init method of the basic RRT

        Args:
            droneInfo (DroneInfo): the drone infomation
            missionInfo (MissionInfo): the mission infomation
            exploreProb (np.float64): the probability of exploration (1 - the probability of going forward target)
            stepSize (np.float64): the size/length of each step
            maxAttempts (Union[np.int32, None], optional): the maximum number of attempts. Defaults to np.Infinity.
        """
        self.droneInfo = droneInfo
        self.missionInfo = missionInfo
        self.mapInfo = missionInfo.mapInfo
        self.exploreProb: np.float64 = exploreProb
        self.maxAttempts: Union[np.int32, None] = maxAttempts
        self.stepSize: np.float64 = stepSize

        # initialize the forward search tree
        origin, target = missionInfo.origin, missionInfo.target
        self.searchTree: RRT = RRT(origin, target)

    def run(self):
        """the method to run the basic RRT algorithm

        Returns:
            bool: whether basic RRT algorithm reach the target from origin
        """
        attemptCnt = 0
        while not self.searchTree.isReachTarget:
            attemptCnt += 1

            if random.random() < self.exploreProb:
                newSample = randomSample(self.mapInfo.minBorder, self.mapInfo.maxBorder)
            else:
                newSample = self.mapInfo.target

            neighbors = self.searchTree.getNearestNeighbors(newSample, num=1)
            # [ ] ignore the failure of RRT Extension
            directlyExtend(self.searchTree, newSample, neighbors)

            if np.isfinite(self.maxAttempts) and attemptCnt > self.maxAttempts:
                break

        if attemptCnt > self.maxAttempts:
            return Failure
        if not self.searchTree.reachTarget():
            return Failure
        return Success
