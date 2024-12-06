# SFM Mapper

## Resources

- Factor Graphs and GTSAM. https://gtsam.org/tutorials/intro.html#magicparlabel-65377
- Factor Graphs for Robot Perception. Dellaert & Kaess. https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf 
- An Introduction to Factor Graphs. Loeliger. https://people.binf.ku.dk/~thamelry/MLSB08/hal.pdf (a good slide deck)

the TLDR of the TLDR (really. Go skim at least [Factor Graphs and GTSAM](https://gtsam.org/tutorials/intro.html#magicparlabel-65411)) is that a factor graph is a graph of variables (things we want to optimize, like robot poses or landmark pose) connected by "factors" which describe relationships between our variables. For example, this factor graph shows 3 robot state variables (x1, x2, and x3; the white circles). The relationship between these states is described by factors (black filled in circles) which encode information about robot pose delta between states (in the backbone), and an external measurement from a sensor like GPS.

![](https://gtsam.org/tutorials/intro-images/5_Users_dellaert_git_github_doc_images_FactorGraph2.png)

The primary difference between this and tradidional nonlinear least squares is that we don't have to explicitly build a cost function. Instead, we're interested in finding the maximum likelihood solution, given a factor graph and *noise models* (see Dellaert Ch. 2.2), which describe how sure we are about measurements.

## Problem Formulation

We build up a factor graph of landmarks (tags), and estimated camera poses in the world. Our decision variables are:
- Poses of tags in the world (located sufficiently near an initial guess that things don't diverge)
  - This is the pose of the tag origin. The 4 tag corners are offset from this tag origin by +- 3 inches in x and z.
- Poses from where camera observations were recorded from

Our factors:
- From camera observation pose -> each tag corner we can see (in pixels). Noise model is is tag corner location in pixels. The tag corner is a GTSAM "expression" that is (tag origin + (tag origin -> corner N)) for each corner.a
- A "pose prior factor" on up to N many tags describing how sure we are in its pose in SE(3). In this example, we measured tag 8's location with a tape measure, but didn't measure any other ones. We could add prior factors to as many tags as we want - GTSAM will maximize the overall solution likelihood given this information.


```mermaid
graph TD;
    x0[Observation 0]
    x1[Observation 1]
    x2[Observation 2]
    x3[Observation 3]

    l6[Tag 6 Corners]
    l7[Tag 7 Corners]
    l8[Tag 8 Corners]
    l9[Tag 9 Corners]
    l10[Tag 10 Corners]
    l15[Tag 15 Corners]

    l6o[Tag 6 Origin]
    l7o[Tag 7 Origin]
    l8o[Tag 8 Origin]
    l9o[Tag 9 Origin]
    l10o[Tag 10 Origin]
    l15o[Tag 15 Origin]

    prior(Tag 8 pose prior factor)

    l6 --- l6o
    l7 --- l7o
    l8 --- l8o
    l9 --- l9o
    l10 --- l10o
    l15 --- l15o

    x0 <-- F0 --> l6
    x0 <-- F1 --> l7
    x0 <-- F2 --> l8

    x1 <-- F3 --> l6
    x1 <-- F4 --> l7
    x1 <-- F5 --> l8

    x2 <-- F6 --> l7
    x2 <-- F7 --> l8
    x2 <-- F8 --> l9
    x2 <-- F9 --> l10

    x3 <-- F10 --> l9
    x3 <-- F11 --> l10
    x3 <-- F12 --> l15

    prior --- l8o
```
