# Multi-Target Tracking Unscented Kalman Filter

Authored by [Yoonyoung Cho](jchocholate@gmail.com), 03/16/2018

## Summary of Objectives

[The IARC Mission 7](http://www.aerialroboticscompetition.org/mission7/) is characterized by a relatively general, yet still complex environment, in which many robots interact in a semi-stochastic manner.
That is, with some prior information on the general behavior, the system is nonetheless non-deterministic and the interaction of the robots would change in each iteration.
This is a unique opportunity to exploit *model-based* predictive strategies, while also accounting for the inherent randomness of the system.

As part of the perceptions team, I've been in development of a multi-target tracking UKF with motion models in order to tackle this very issue;
while the pipeline is still under development, it has seen some positive progress that I would like to share.

In essence, we are trying to figure out the position of the roombas at any given moment, in order to faciliate the decision-making processes; in a sense, we are using a filter to preserve a sense of [object permanence](https://en.wikipedia.org/wiki/Object_permanence) for the drone; i.e. both to refine the position of the roombas themselves

## Brief Kalman Filter Overview

In our system, we have the *estimated* positions of the roombas, as well as *observations*; in typical applications, the incentive of applying a Kalman Filter are as follows, where the ones that apply to us have been checked:

- [x] To refine the observations, which are prone to noise;
- [x] To provide predictions, based on pervious known states;
- [x] To deduce additional information, such as velocity and orientation, based on observations without full-state information.
- [ ] To utilize multiple information sources and fuse them to obtain the best estimates (sensor fusion).

This is also true for our model; in order to develop an effective strategy for redirecting the roombas, it is critical to have a fairly accurate world-coordinate positions of the roombas, as well as their orientation, and how they will change over time. Note that there is no need for sensor fusion, given that we only have one realistic source of the roombas' location, which is the downward-facing camera. So far so good!

While that sounds great in *principle*, a shortcoming for a traditional kalman filter is that it is only really effective for *linear* system. In order to mitigate this issue, alternative forms of the kalman filter have been developed, such as the Extended Kalman Filter(EKF) which linearizes a nonlinear system through the jacobians. This is often a close-enough approximation of the system, where the *slope* of the system transition is evaluated at every point to yield a first-order estimate of the predicted behavior.

In our development, we have been using an Unscented Kalman Filter(UKF), as it had the most flexibility in terms of producing the estimates, and it could deal with nonlinearities in a very elegant manner. In a brief description, a UKF employs an [Unscented Transform](https://en.wikipedia.org/wiki/Unscented_transform) in order to achieve an approximation of the best estimate of how a certain probabilistic distribution will evolve, by sampling sigma points based on the prior values.

## Target Associations

### Hungarian Algorithm

### Multivariate Matching

## Motion Model


