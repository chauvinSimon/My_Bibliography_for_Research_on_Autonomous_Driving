# My_Bibliography_for_Research_on_Autonomous_Driving

! WORK IN PROGRESS !

I reference additional publications in my other works:

- [Hierarchical Decision-Making for Autonomous Driving](https://github.com/chauvinSimon/Hierarchical-Decision-Making-for-Autonomous-Driving)
- [Educational application of Hidden Markov Model to Autonomous Driving](https://github.com/chauvinSimon/hmm_for_autonomous_driving)
- [My 10 takeaways from the 2019 Intelligent Vehicle Symposium](https://github.com/chauvinSimon/IV19)

## Architecture and Map

Ilievski, M., Sedwards, S., Gaurav, A., Balakrishnan, A., Sarkar, A., Lee, J., Bouchard, F., De Iaco, R., & Czarnecki K. [2019].
**"Design Space of Behaviour Planning for Autonomous Driving"**
[[pdf](https://arxiv.org/abs/1908.07931)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![The focus is on the `BP` module, together with it precessessor (`environment`) and its successor (`LP`) in a modular architecture. [Source](https://arxiv.org/abs/1908.07931).](media/2019_ilievski_5.PNG "The focus is on the `BP` module, together with it precessessor (`environment`) and its successor (`LP`) in a modular architecture. [Source](https://arxiv.org/abs/1908.07931).")  |
|:--:|
| *The focus is on the `BP` module, together with its precessessor (`environment`) and its successor (`LP`) in a modular architecture. [Source](https://arxiv.org/abs/1908.07931).* |

| ![Classification for Question `1` - environment representation. A combination is possible. In black, my notes giving examples. [Source](https://arxiv.org/abs/1908.07931).](media/2019_ilievski_6.PNG "Classification for Question `1` - environment representation. A combination is possible. In black, my notes giving examples. [Source](https://arxiv.org/abs/1908.07931).")  |
|:--:|
| *Classification for Question `1` - environment representation. A combination is possible. In black, my notes giving examples. [Source](https://arxiv.org/abs/1908.07931).* |

| ![Classification for Question `2` - on the architecture. [Source](https://arxiv.org/abs/1908.07931).](media/2019_ilievski_2.PNG "Classification for Question `2` - on the architecture. [Source](https://arxiv.org/abs/1908.07931).")  |
|:--:|
| *Classification for Question `2` - on the architecture. [Source](https://arxiv.org/abs/1908.07931).* |

| ![Classification for Question `3` - on the decision logic representation. [Source](https://arxiv.org/abs/1908.07931).](media/2019_ilievski_1.PNG "Classification for Question `3` - on the decision logic representation. [Source](https://arxiv.org/abs/1908.07931).")  |
|:--:|
| *Classification for Question `3` - on the decision logic representation. [Source](https://arxiv.org/abs/1908.07931).* |

The authors divide their review into three sections:

- Question `1`: **_How to represent the envrionment?_** (relation with _predecessor_ of `BP`)
  - Four representations are compared: `raw data`, `feature-based`, `grid-based` and `latent representation`.
- Question `2`: **_How to communicated with other modules, especially the local planner (`LP`)?_** (relation with _successor_ (`LP`) of `BP`)
  - A first sub-question is the relevance of **separation** `BP` / `LP`.
    - A complete separation (_top-down_) can lead to **computational redundancy** (both have a _collision checker_).
    - One idea, close to **sampling techniques**, could be to **invert the traditional architecture** for planning, i.e. **generate multiple possible local paths** (`~LP`) then selects the best manoeuvre according to a given cost function (`~BP`). But this exasperates the previous point.
  - A second sub-question concerns _prediction_: **_Should the `BP` module have its own dedicated prediction module?_**
    - First, three kind of prediction are listed, depending on what should be predicted (marked with `->`):
      - _Physics_-based (`->` trajectory).
      - _Manoeuvre_-based (`->` low-level motion primitives).
      - _Interaction_-aware (`->` intent).
    - Then, the authors distinghuish between **_explicitly-defined_** and **_implicitly-defined_** prediction models:
      - **_Explicitly-defined_** can be:
        - _Integrated_ with the motion planning process (called **_Internal prediction models_**) such as **belief-based** decision making (e.g. `POMDP`). Ideal for **planning under uncertainty**.
        - _Decoupled_ from the planning process (called **_External prediction models_**). There is a **clear interface** between prediction and planning, which aids **modularity**.
      - **_Implicitly-defined_**, such as `RL` techniques.
- Question `3`: **_How to make `BP` decisions?_** (`BP` itself)
  - A first distinction in _representation of decision logic_ is made depending based on _non-learnt_ / _learnt_:
    - Using a set of **explicitly programmed** production rules can be divided into:
      - `Imperative` approaches, e.g. _state machines_.
      - `Declarative` approaches often based on some **probabilistic system**.
        - The **decision-tree** structure and the (`PO`)`MDP` formulation makes it **more robust to uncertainty**.
        - Examples include **MCTS** and **online POMDP solvers**.
    - Logic representation can also rely on mathematical models with **parameters learned** a priori.
      - A distinction is made depending on _"where does the training data come from and when is it created?"_.
      - In other words, one could think of **supervised learning** (_learning from example_) versus **reinforcement learning** (_learning from interaction_).
      - The combination of both seems benefitial:
        - An initial behaviour is obtained through **imitation learning** (_learning from example_). Also possible with `IRL`.
        - But **improvements are made through interaction** with a simulated environment (_learning from interaction_).
          - By the way, the _learning from interaction_ techniques raise the question of the **origin of the experience** (e.g. realism of the simulator) and its **sampling efficiency**.
      - Another promising direction is **hierarchical RL** where the MDP is divided into sub-problems (the **lower for `LP`** and the **higher for `BP`**)
        - The _lowest level_ implementation of the hierarchy approximates a solution to the **control and LP problem** ...
        - ... while the _higher level_ **selects a maneuver** to be executed by the lower level implementations.
  - As mentioned my the section on [Scenarios and Datasets](https://github.com/chauvinSimon/IV19#scenarios-and-datasets), the authors mention the **lack of benchmark** to compare and **evaluate** the performance of BP technologies.

One quote about the _representation of decision logic_:

- As identified in [my notes about IV19](https://github.com/chauvinSimon/IV19), the **combination** of _learnt_ and _non-learnt_ approaches looks the most promising.
- > "Without learning, traditional robotic solutions cannot adequately handle **complex, dynamic human environments**, but **ensuring the safety** of learned systems remains a significant challenge."
- > "Hence, we speculate that future high performance and safe behaviour planning solutions will be **_hybrid_** and **_heterogeneous_**, incorporating modules consisting of learned systems supervised by programmed logic."

---

</details>

Wei, J., Snider, J. M., & Dolan, J. M. [2014].
**"A Behavioral Planning Framework for Autonomous Driving"**
[[pdf](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![Comparison and fusion of the **hierarchical** and **parallel** architectures. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).](media/2014_wei_1.PNG "Comparison and fusion of the **hierarchical** and **parallel** architectures. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).")  |
|:--:|
| *Comparison and fusion of the **hierarchical** and **parallel** architectures. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).* |

| ![The `PCB` algorithm implemented in the `BP` module. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).](media/2014_wei_2.PNG "The `PCB` algorithm implemented in the `BP` module. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).")  |
|:--:|
| *The `PCB` algorithm implemented in the `BP` module. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).* |

| ![Related work by (Xu, Pan, Wei, & Dolan, 2014) - Grey ellipses indicate the magnitude of the uncertainty of state. [Source](https://ri.cmu.edu/pub_files/2014/6/ICRA14_0863_Final.pdf).](media/2014_xu_1.PNG "Related work by (Xu, Pan, Wei, & Dolan, 2014) - Grey ellipses indicate the magnitude of the uncertainty of state. [Source](https://ri.cmu.edu/pub_files/2014/6/ICRA14_0863_Final.pdf).")  |
|:--:|
| *Related work by (Xu, Pan, Wei, & Dolan, 2014) - Grey ellipses indicate the magnitude of the uncertainty of state. [Source](https://ri.cmu.edu/pub_files/2014/6/ICRA14_0863_Final.pdf).* |

- Some related concepts:
  - `behavioural planning`, `sampling-based planner`, `decision under uncertainty`, [`TORCS`](https://en.wikipedia.org/wiki/TORCS)

Note: I find very valuable to get insights from the **CMU** (Carnegie Mellon University) Team, based on their **experience of the DARPA Urban Challenges**.

- Related works:
  - [_A prediction- and cost function-based algorithm for robust autonomous freeway driving_. 2010](https://ri.cmu.edu/pub_files/2010/6/2010_IV.pdf) by (Wei, Dolan, & Litkouhi, 2010).
    - They introduced the **_"Prediction- and Cost-function Based (`PCB`) algorithm"_** used.
    - The idea is to `generate`-`forward_simulate`-`evaluate` a set of manoeuvres.
    - The planner can therefore take **surrounding vehicles’ reactions** into account **in the cost function** when it searches for the best strategy.
    - At the time, the authors rejected the option of a `POMDP` forumlation (_computing the control policy over the space of the belief state, which is a probability distribution over all the possible states_) deemed as computationally expensive. Improvements in hardware and algorithmic have been made since 2014.
  - [_Motion planning under uncertainty for on-road autonomous driving_. 2014](https://ri.cmu.edu/pub_files/2014/6/ICRA14_0863_Final.pdf) by (Xu, Pan, Wei, & Dolan, 2014).
    - An extension of the framework to **consider uncertainty** (both for _environment_ and the _others participants_) in the decision-making.
    - The prediction module is using a **Kalman Filter** (assuming constant velocity).
    - For each candidate trajectory, the **uncertainty** can be estimated using a **_Linear-Quadratic Gaussian_** (`LQG`) framework (based on the the noise characteristics of the localization and control).
    - Their Gaussian-based method gives some **probabilistic safety guaranty** (e.g. likelihood `2%` of collision to occur).
- Proposed architecture for _decision-making_:
  - First ingredient: **Hierarchical** architecture.
    - The hierachy `mission` `->` `manoeuver` `->` `motion` [`3M` concept](https://github.com/chauvinSimon/Hierarchical-Decision-Making-for-Autonomous-Driving) makes it very modular but can raise limitations:
    - > "the higher-level decision making module usually **does not have enough detailed information**, and the lower-level layer **does not have authority** to **reevaluate the decision**."
  - Second ingredient: **Parallel** architecture.
    - This is inspired from **ADAS** engineering.
    - The control modules (`ACC`, `Merge Assist`, `Lane Centering`) are relatively **independent** and **work in parallel**.
    - In some **complicated cases** needing cooperation, this framework may not perform well.
      - _This probably shows that just_ **_extending the common ADAS architectures_** _cannot be enough to reach the_ **_level-`5` of autonomy_**.
  - Idea of the proposed framework: **combine the strengths** of the **_hierarchical_** and **_parallel_** architectures.
    - This relieves the path planner and the control module (the search space is reduced).
    - Hence the **computational cost** shrinks (by over 90% compared to a **_sample-based planner_** in the **spatio-temporal space**).
- One module worth mentioning: **_Traffic-free Reference Planner_**.
  - Its input: lane-level **sub-missions** from the _Mission Planning_.
  - Its output: kinematically and dynamically feasible **paths** and a **speed profiles** for the **_Behavioural Planner_** (`BP`).
    - It assumes there is **no traffic** on the road, i.e. **ignores dynamic obstacles**.
    - It also applies **traffic rules** such as _speed limits_.
  - This **guides** the `BP` layer which considers both static and dynamic obstacles to generate so-called **_"controller directives"_** such as:
    - The **lateral driving bias**.
    - The **desired leading vehicle** to follow.
    - The **aggressiveness** of distance keeping.
    - The **maximum speed**.

---

</details>

## Behavior Cloning, End-To-End (E2E), Imitation Learning

Buhet, T., Wirbel, E., & Perrotton, X. [2019].
**"Conditional Vehicle Trajectories Prediction in CARLA Urban Environment"**
[[pdf](https://arxiv.org/abs/1909.00792)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![`End-to-Mid` approach: `3` inputs with **different levels of abstraction** are used to predict the future positions on a fixed `2s` horizon of the ego vehicle and the neighbors. The ego trajectory is be **implemented by an external** `PID` controller. [Source](https://arxiv.org/abs/1909.00792).](media/2019_buhet_1.PNG "`End-to-Mid` approach: `3` inputs with **different levels of abstraction** are used to predict the future positions on a fixed `2s` horizon of the ego vehicle and the neighbors. The ego trajectory is be **implemented by an external** `PID` controller. [Source](https://arxiv.org/abs/1909.00792).")  |
|:--:|
| *`End-to-Mid` approach: `3` inputs with **different levels of abstraction** are used to predict the future positions on a fixed `2s` horizon of the ego vehicle and the neighbors. The ego trajectory is be **implemented by an external** `PID` controller. [Source](https://arxiv.org/abs/1909.00792).* |

| ![The past **3D-bounding boxes** of the road users in the current reference are **projected back in the current camera space**. The **past positions** of ego and other vehicles are projected into some grid-map called **"proximity map"**. The image and the **proximity map** are concatenated to form context feature vector `C`. This **context encoding** is concatenated with the **ego encoding**, then fed into **branches** corresponding to the different high level goals - `conditional navigation goal`. [Source](https://arxiv.org/abs/1909.00792).](media/2019_buhet_2.PNG "The past **3D-bounding boxes** of the road users in the current reference are **projected back in the current camera space**. The **past positions** of ego and other vehicles are projected into some grid-map called **"proximity map"**. The image and the **proximity map** are concatenated to form context feature vector `C`. This **context encoding** is concatenated with the **ego encoding**, then fed into **branches** corresponding to the different high level goals - `conditional navigation goal`. [Source](https://arxiv.org/abs/1909.00792).")  |
|:--:|
| *The past **3D-bounding boxes** of the road users in the current reference are **projected back in the current camera space**. The **past positions** of ego and other vehicles are projected into some grid-map called **"proximity map"**. The image and the **proximity map** are concatenated to form context feature vector `C`. This **context encoding** is concatenated with the **ego encoding**, then fed into **branches** corresponding to the different high level goals - `conditional navigation goal`. [Source](https://arxiv.org/abs/1909.00792).* |

| ![Illustration of the **distribution shift** in **imitation learning** `IL`. [Source](https://arxiv.org/abs/1909.00792).](media/2019_buhet_3.PNG "Illustration of the **distribution shift** in **imitation learning** `IL`. [Source](https://arxiv.org/abs/1909.00792).")  |
|:--:|
| *Illustration of the **distribution shift** in **imitation learning** `IL`. [Source](https://arxiv.org/abs/1909.00792).* |

| ![[`VisualBackProp`](https://arxiv.org/abs/1611.05418) highlights the **image pixels which contributed the most** to the final results - **Traffic lights** and their colors are important, together with with highlights lane markings and curbs when there is a significant lateral deviation. [Source](https://arxiv.org/abs/1909.00792).](media/2019_buhet_4.PNG "[`VisualBackProp`](https://arxiv.org/abs/1611.05418) highlights the **image pixels which contributed the most** to the final results - **Traffic lights** and their colors are important, together with with highlights lane markings and curbs when there is a significant lateral deviation. [Source](https://arxiv.org/abs/1909.00792).")  |
|:--:|
| *[`VisualBackProp`](https://arxiv.org/abs/1611.05418) highlights the **image pixels which contributed the most** to the final results - **Traffic lights** and their colors are important, together with with highlights lane markings and curbs when there is a significant lateral deviation. [Source](https://arxiv.org/abs/1909.00792).* |

- Some related concepts:
  - `conditional IL`, [`CARLA`](http://carla.org/), `distributional shift problem`
- Previous works:
  - ["Imitation Learning for End to End Vehicle Longitudinal Control with Forward Camera"](https://arxiv.org/abs/1812.05841) - (George, Buhet, Wirbel, Le-Gall, & Perrotton, 2018).
- One term: **_"Conditional navigation goal"_**.
  - Together with the RGB images and the **past positions**, the network takes as input a **navigation command** to describe the **desired behavior** of the ego vehicle at intersections.
  - Hence, the future trajectory of the ego vehicle is **conditionned** by a **navigation command**.
    - If the ego-car is approaching an intersection, the **goal** can be `left`, `right` or `cross`, else the goal is `keep lane`.
    - That means `lane-change` is not an option.
  - > "The last layers of the network are **split into branches** which are **masked with the current navigation command**, thus allowing the network to learn specific behaviors for each goal".
- Ingredients to improve vanilla end-to-end imitation learning (`IL`):
  - Mix of `high` and `low`-level data (i.e. _hybrid_ input):
    - Both **raw signal** (_images_) and **partial environment abstraction** (_navigation commands_) are used.
  - **Auxiliary tasks**:
    - One head of the network predicts the future trajectories of the **surrounding vehicles**.
      - **It differs from the primary task** which should decide the `2s`-ahead trajectory for the ego car.
      - Nevertheless, this **secondary task helps**: _"Adding the neighbors prediction makes the ego prediction more compliant to traffic rules."_
    - This refers to the concept of **_"Priviledged learning"_**:
      - > "The network is **partly trained with an auxiliary task** on a ground truth which is **useful to driving**, and on the rest is only trained for IL".
  - **Label augmentation**:
    - The main challenge of `IL` is the difference between **train** and **online test** distributions. This is due to the difference between
      - **`Open-loop`** control: decisions are not implemented.
      - **`Close-loop`** control: decisions are implemented and the vehicle can end in a state absent from the _train distribution_, potentially causing _"error accumulation"_.
    - **Data augmentation** is used to reduce the gap between _train_ and _test_ distributions.
      - Classical randomization is combined with **`label augmentation`**: data similar to **failure cases** is generated a posteriori.
    - Three findings:
    - > "There is a **significant gap** in performance when introducing the **augmentation**."
    - > "The effect is much more noticeable on **complex navigation tasks**." _(Errors accumulate quicker)_.
    - > "**Online test** is the real significant indicator for `IL` when it is used for active control." _(The common offline evaluation metrics may not be correlated to the online performance)_.
- Baselines:
  - **Conditional Imitation** learning (`CIL`).
    - `CIL` produces **instantaneous commands**.
  - **Conditional Affordance** Learning (`CAL`).
    - `CAL` produces **_"affordances"_** which are then given to a controller.
- One word about the choice of the simulator.
  - A possible alternative to [CARLA](http://carla.org/) could be the promising [**`LGSVL`** simulator](https://www.lgsvlsimulator.com) developed by the Advanced Platform Lab at the **LG Electronics** America R&D Center.
- One term: **_"End-To-Middle"_**.
  - It is opposed to **_"End-To-End"_**, i.e. it **does not output "end" control signals** such as throttle or steering but rather some **desired trajectory**, i.e. a mid-level representation.
    - Each trajectory is described by **two polynomial functions** (one for `x`, the other for `y`), therefore the network has to **predict a vector** (`x0`, ..., `x4`, `y0`, ..., `y4`) for each vehicle.
    - The desired ego-trajectory is then implemented by an **external controller** (`PID`). Therefore not `end-to-end`.
  - Advantages of `end-to-mid`: **interpretability** for the control part + less to be learnt by the net.
  - This approach is also an instance of **"Direct perception"**:
    - > "Instead of commands, the network predicts hand-picked parameters relevant to the driving (distance to the lines, to other vehicles), which are then fed to an independent controller".
  - Small digression: if the raw perception measurements were first processed to form a **mid-level** input representation, the approach would be said `mid-to-mid`. An example is [ChauffeurNet](https://arxiv.org/abs/1812.03079), detailed on this page as well.
- About Ground truth:
  - The expert demonstrations do not come from human recordings but rather from CARLA autopilot.
  - `15` hours of driving in `Town01` were collected.
  - As for human demonstrations, **no annotation is needed**.

---

</details>

Michelmore, R., Wicker, M., Laurenti, L., Cardelli, L., Gal, Y., & Kwiatkowska, M. [2019].
**"Uncertainty Quantification with Statistical Guarantees in End-to-End Autonomous Driving Control"**
[[pdf](https://arxiv.org/abs/1909.09884)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![The __trust__ or __uncertainty__ in one decision can be measured based on the `probability mass function` around its mode. [Source](https://arxiv.org/abs/1904.08980).](media/2019_michelmore_1.PNG "The __trust__ or __uncertainty__ in one decision can be measured based on the `probability mass function` around its mode. [Source](https://arxiv.org/abs/1909.09884).")  |
|:--:|
| *The __trust__ or __uncertainty__ in one decision can be measured based on the `probability mass function` around its mode. [Source](https://arxiv.org/abs/1904.08980).* |

| ![The measures of uncertainty based on __mutual information__ can be used to issue warnings to the driver and perform safety / emergency manoeuvres. [Source](https://arxiv.org/abs/1904.08980).](media/2019_michelmore_2.PNG "The measures of uncertainty based on __mutual information__ can be used to issue warnings to the driver and perform safety / emergency manoeuvres. [Source](https://arxiv.org/abs/1909.09884).")  |
|:--:|
| *The measures of uncertainty based on __mutual information__ can be used to issue warnings to the driver and perform safety / emergency manoeuvres. [Source](https://arxiv.org/abs/1904.08980).* |

| ![As noted by the authors: while the `variance` can be useful in __collision avoidance__, the wide variance of `HMC` causes a larger proportion of trajectories to fall __outside of the safety boundary__ when a _new weather_ is applied. [Source](https://arxiv.org/abs/1904.08980).](media/2019_michelmore_3.PNG "As noted by the authors: while the `variance` can be useful in __collision avoidance__, the wide variance of `HMC` causes a larger proportion of trajectories to fall __outside of the safety boundary__ when a _new weather_ is applied. [Source](https://arxiv.org/abs/1909.09884).")  |
|:--:|
| *As noted by the authors: while the `variance` can be useful in __collision avoidance__, the wide variance of `HMC` causes a larger proportion of trajectories to fall __outside of the safety boundary__ when a _new weather_ is applied. [Source](https://arxiv.org/abs/1904.08980).* |

- Some related concepts:
  - `uncertainty-aware decision`, `Bayesian inference`, [`CARLA`](http://carla.org/)
- One related work:
  - NVIDIA’s [**`PilotNet`**](https://arxiv.org/abs/1704.07911) [`DAVE-2`] where **expert demonstrations** are used together with **supervised learning** to map from **images** (front camera) to **steering command**.
  - Here, human demonstrations are collected in the **CARLA** simulator.
- One idea: use **distribution** in weights.
  - The difference with `PilotNet` is that the neural network applies the **_"Bayesian"_** paradigm, i.e. each **weight is described by a distribution** (not just a single value).
  - The authors illustrate the benefits of that paradigm, imagining an **obstacle in the middle** of the road.
    - The Bayesian controller may be **uncertain on the steering angle** to apply (e.g. a `2`-tail or `M`-shape distribution).
    - A first option is to **sample** angles, which turns the car either _right_ or _left_, with **equal probability**.
    - Another option would be to simply select the **mean value** of the distribution, which **aims straight** at the obstacle.
    - The motivation of this work is based on that example: *"derive some __precise quantitative measures__ of the BNN uncertainty to facilitate the __detection of such ambiguous situation__"*.
- One definition: **"real-time decision confidence"**.
  - This is the **probability** that the BNN controller is **certain of its decision** at the current time.
  - The notion of **trust** can therefore be introduced: the idea it to compute the probability mass in a `ε`−ball around the decision `π`(`observation`) and classify it as _certain_ if the resulting probability is greater than a **threshold**.
    - _It reminds me the concept of `trust-region` optimisation in RL_.
    - In extreme cases, all actions are _equally distributed_, `π`(`observation`) has a very **high variance**, the agent does not know what to do (_no trust_) and will **randomly sample** an action.
- _How to get these estimates?_ Three Bayesian inference methods are compared:
  - **Monte Carlo dropout** (`MCD`).
  - **Mean-field variational inference** (`VI`).
  - **Hamiltonian Monte Carlo** (`HMC`).
- _What to do with this information?_
  - > "This measure of uncertainty can be employed together with commonly employed _measures of uncertainty_, such as __mutual information__, to __quantify__ in __real time__ the degree that the model is __confident in its predictions__ and can offer a notion of __trust__ in its predictions."
    - I did not know about **"mutual information"** and liked the explanation of [wikipedia](https://en.wikipedia.org/wiki/Mutual_information) about the link of `MI` to `entropy` and `KL-div`.
      - *I am a little bit confused: in what I read, the `MI` is function of **two random variables**. What are they here? The authors rather speak about the __predictive distribution__ exhibited by the predictive distribution*.
  - Depending on the **uncertainty level**, several actions are taken:
    - `mutual information warnings` **slow down** the vehicle.
    - `standard warnings` **slow down** the vehicle and **alert** the operator of potential hazard.
    - `severe warnings` cause the car to **safely brake** and ask the operator to **take control back**.
- Another definition: **_"probabilistic safety"_**, i.e. the probability that a BNN controller will keep the car _"safe"_.
  - _Nice, but what is "safe"?_
  - It all relies on the assumption that **expert demonstrations** were all _"safe"_, and measures the how much of the trajectory belongs to this **_"safe set"_**.
  - I must admit I did not fully understand the measure on _"safety"_ for some _continuous_ trajectory and _discrete_ demonstration set:
    - A car can drive with a large lateral offset from the demonstration on a wide road while being _"safe"_, while a thin lateral shift in a narrow street can lead to an _"unsafe"_ situation.
    - Not to mention that the scenario (e.g. _configuration of obstacles_) has probably changed in-between.
    - This leads to the following point with an interesting application for _scenario coverage_.
- One idea: apply **changes in scenery and weather conditions** to evaluate **model robustness**.
  - To check the **generalization** ability of a model, the _safety analysis_ is re-run (_offline_) with **other weather conditions**.
  - As noted in conclusion, this _offline safety probability_ can be used as a **guide for active learning** in order to increase **data coverage** and **scenario representation** in training data.

---

</details>

Codevilla, F., Santana, E., Antonio, M. L., & Gaidon, A. [2019].
**"Exploring the Limitations of Behavior Cloning for Autonomous Driving"**
[[pdf](https://arxiv.org/abs/1904.08980)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![Conditional Imitation Learning is extended with a ResNet architecture and Speed prediction (`CILRS`). [Source](https://arxiv.org/abs/1904.08980).](media/2019_codevilla.PNG "Conditional Imitation Learning is extended with a ResNet architecture and Speed prediction (`CILRS`). [Source](https://arxiv.org/abs/1904.08980).")  |
|:--:|
| *Conditional Imitation Learning is extended with a ResNet architecture and Speed prediction (`CILRS`). [Source](https://arxiv.org/abs/1904.08980).* |

- Some related concepts:
  - `distributional shift problem`, `off-policy data collection`, [`CARLA`](http://carla.org/), `conditional imitation learning`, `residual architecture`, `reproducibility issue`, `variance caused by initialization and sampling`
- One term: **“CILRS”** = **Conditional Imitation Learning** extended with a **ResNet** architecture and **Speed prediction**.
- One Q&A: _How to include in E2E learning information about the destination, i.e. to disambiguate imitation around multiple types of intersections?_
  - Add a high-level `navigational command` (e.g. _take the next right_, _left_, or _stay in lane_) to the tuple <`observation`, `expert action`> when building the dataset.
- One idea: learn to predict the ego speed ([`mediated perception`](http://deepdriving.cs.princeton.edu/paper.pdf)) to address the _inertia problem_ stemming from [**causal confusion**](https://arxiv.org/pdf/1905.11979.pdf) (**biased correlation** between _low speed_ and _no acceleration_ - when the ego vehicle is stopped, e.g. at a red traffic light, the probability it stays static is indeed overwhelming in the training data).
- Another idea: The off-policy (expert) driving demonstration is not produced by a human, but rather generated from an **omniscient "AI" agent**.
- One quote:

> "The more common the vehicle model and color, the better the trained agent reacts to it. This raises ethical challenges in automated driving".

---

</details>

Bansal, M., Krizhevsky, A., & Ogale, A. [2018].
**"ChauffeurNet: Learning to Drive by Imitating the Best and Synthesizing the Worst"**
[[pdf](https://arxiv.org/abs/1812.03079)]
[[videos](https://sites.google.com/view/waymo-learn-to-drive)]
[[presentation](https://www.youtube.com/watch?v=mxqdVO462HU)]

<details>
  <summary>Click to expand</summary>

Two figures:

| ![Different layers composing the `mid-level representation`. [Source](https://arxiv.org/abs/1812.03079).](media/2018_bansal_1.PNG "Different layers composing the `mid-level representation`. [Source](https://arxiv.org/abs/1812.03079).")  |
|:--:|
| *Different layers composing the `mid-level representation`. [Source](https://arxiv.org/abs/1812.03079).* |

| ![Training architecture around `ChauffeurNet` with the different losses terms, that can be grouped into `environment` and `imitation` losses. [Source](https://arxiv.org/abs/1812.03079).](media/2018_bansal_2.PNG "Training architecture around `ChauffeurNet` with the different losses terms, that can be grouped into `environment` and `imitation` losses. [Source](https://arxiv.org/abs/1812.03079).")  |
|:--:|
| *Training architecture around `ChauffeurNet` with the different losses terms, that can be grouped into `environment` and `imitation` losses. [Source](https://arxiv.org/abs/1812.03079).* |

- Some related concepts:
  - `imitation learning`, `distributional shift problem`
- One term: **_"mid-level representation"_**
  - The decision-making task (between `perception` and `control`) is packed into one single "learnable" module.
    - Input: the representation divided into **several image-like layers**:
      - `Map features` such as _lanes_, _stop signs_, _cross-walks_...; `Traffic lights`; `Speed Limit`; `Intended route`; `Current agent box`; `Dynamic objects`; `Past agent poses`.
      - Such a representation is **generic**, i.e. independant of the **number of dynamic objects** and independant of the **road geometry/topology**.
      - I discuss some [equivalent representations](https://github.com/chauvinSimon/IV19#generic-scene-representation) seen at IV19.
    - Output: **intended route**, i.e. the future poses recurrently predicted by the introduced **`ChauffeurNet`** model.
  - This architecture lays between **E2E** (from `pixels` directly to `control`) and **fully decomposed modular pipelines** (decomposing `planning` in multiple modules).
  - Two notable advantages over **E2E**:
    - It alleviates the **burdens of learning perception** and **control**:
      - The desired trajectory is passed to a **controls optimizer** that takes care of creating the low-level control signals.
      - Not to mention that **different types of vehicles** may possibly utilize different control outputs to achieve the **same driving trajectory**.
    - **Perturbations** and input data from simulation are easier to generate.
- One key finding: **_"pure imitation learning is not sufficient"_**, despite the `60` days of continual driving (`30 million` examples).
  - One quote about the "famous" **_distribution shift_** (deviation from the training distribution) in **imitation learning**:
  > "The key challenge is that we need to run the system closed-loop, where errors accumulate and induce a shift from the training distribution."
  - The **training data** does not have any **real collisions**. _How can the agent efficiently learn to avoid them if it has never been exposed during training?_
  - One solution consists in **exposing the model to non-expert behaviours**, such as collisions and off-road driving, and in **adding extra loss functions**.
    - Going **beyond vanilla cloning**.
      - **_Trajectory perturbation_**: Expose the learner to synthesized data in the form of **perturbations to the expert’s driving** (e.g. jitter the midpoint pose and heading)
        - One idea for future works is to use **more complex augmentations**, e.g. with RL, especially for highly interactive scenarios.
      - **_Past dropout_**: to prevent **using the history to cheat** by **just extrapolating** from the past rather than finding the **underlying causes** of the behavior.
      - Hence the concept of tweaking the training data in order to **_“simulate the bad rather than just imitate the good”._**
    - Going **beyond the vanially imitation loss**.
      - Extend imitation losses.
      - Add **_environment losses_** to discourage undesirable behavior, e.g. measuring the overlap of predicted agent positions with the _non-road_ regions.
      - Use **_imitation dropout_**, i.e. sometimes favor the _environment loss_ over the _imitation loss_.

---

</details>

## Inverse Reinforcement Learning, Inverse Optimal Control, Game Theory

Sankar, G. S., & Han, K. [2019].
**"Adaptive Robust Game-Theoretic Decision Making for Autonomous Vehicles"**
[[pdf](https://arxiv.org/abs/1909.02586)]
[[code - simulator](https://github.com/gokulsivasankar/RobustDecisionMaking)]

<details>
  <summary>Click to expand</summary>

| ![The agent maintain belief on the `k` parameter for other vehicles and updates it at each step. [Source](https://arxiv.org/abs/1902.09068).](media/2019_sankar_1.PNG "The agent maintain belief on the `k` parameter for other vehicles and updates it at each step. [Source](https://arxiv.org/abs/1902.09068).")  |
|:--:|
| *The agent maintain belief on the `k` parameter for other vehicles and updates it at each step. [Source](https://arxiv.org/abs/1902.09068).* |

- Some related concepts:
  - `k-level strategy`, `MPC`, `interaction-aware prediction`
- One related work (described further below): [_Decision making in dynamic and interactive environments based on cognitive hierarchy theory: Formulation, solution, and application to autonomous driving_](https://arxiv.org/abs/1908.04005) by (Li, S., Li, N., Girard, A., & Kolmanovsky, I. 2019).
- One framework: **"_level-`k` game-theoretic framework_"**.
  - It is used to **model the interactions** between vehicles, taking into account the **rationality** of the other agents.
  - The agents are categorized into hierarchical structure of their **cognitive abilities**, parametrized with a **reasoning depth** `k` in [`0`, `1`, `2`].
    - A level-`0` vehicle considers the other vehicles in the traffic scenario as **stationary obstacles**, hence being **"aggressive"**.
    - A level-`1` agent assumes other agents are at level-`0`. ...
  - This parameter `k` is what the agent **must estimate** to **model the interaction** with the other vehicles.
- One term: **"_disturbance set_"**.
  - This set, denoted `W`, describe the uncertainty in the position estimate of other vehicle (with some `delta`, similar to the variance in Kalman filters).
  - It should capture both the uncertainty about the **transition model** and the uncertainty about the **driver models**.
  - This set is considered when taking action using a **"_feedback min-max strategy_"**.
    - I must admit I did not fully understand the concept. Here is a quote:
    - > "The min-max strategy considers the **worst-case disturbance** affecting the behavior/performance of the system and provides control actions to mitigate the effect of the worst-case disturbance."
  - The important idea is to **adapt the size of this `W` set** in order to **avoid over-conservative behaviours** (compared to _reachable-set_ methods).
    - This is done based on the **confidence** in the **estimated driver model** (_probability distribution of the estimated `k`_) for the other vehicles.
      - If the agent is sure that the other car follows model `0`, then it should be **"fully" conservative**.
      - If the agent is sure it follows level `1`, then it could **relax its conservatism** (i.e. reduce the size of the **disturbance set**) since it is taken into consideration.
- I would like to draw some parallels:
  - With **`(PO)MDP`** formulation: for the use of a **_transition model_** (or _transition function_) that is hard to define.
  - With **`POMDP`** formulation: for the **tracking of believes about the driver model** (or intention) of other vehicles.
    - The estimate of the probability distribution (for `k`) is updated at every step.
  - With **`IRL`**: where the agent can **predict the reaction** of other vehicles assuming they act optimally w.r.t a **reward function it is estimating**.
  - With **`MPC`**: the choice of the optimal control following a **receding horizon** strategy.

---

</details>

Sierra Gonzalez, D. [2019].
**"Towards Human-Like Prediction and Decision-Making for Automated Vehicles in Highway Scenarios"**
[[pdf](https://tel.archives-ouvertes.fr/tel-02184362/document)]
[[video - simulator](https://www.youtube.com/watch?v=Xx5OmV86CsM)]
[[code - simulator](https://github.com/marioney/hybrid_simulation/tree/decision-making)]

<details>
  <summary>Click to expand</summary>

- Note:
  - this **`190`-page thesis** is also referenced in the sections for **prediction** and **planning**.
  - I really like how the author organizes synergies between three modules that are split and made indepedent in most modular architectures:
    - **`(1)` driver model**
    - **`(2)` behavior prediction**
    - **`(3)` decision-making**

- Some related concepts:
  - `Maximum Entropy IRL`

- Related work: there are close concepts to the approach of `(Kuderer et al., 2015)` referenced below.
- One idea: **encode the driving preferences** of a human driver with a **reward function** (or **cost function**), mentioning a quote from Abbeel, Ng and Russell:

> “The reward function, rather than the policy or the value function, is the most succinct, robust, and transferable definition of a task”.

- Other ideas:
  - Use IRL to **avoid the manual tuning** of the parameters of the reward model. Hence learn a cost/reward function from demonstrations.
  - Include **dynamic features**, such as the `time-headway`, in the linear combination of the cost function, to take the **interactions** between traffic participants into account.
  - Combine IRL with a trajectory planner based on **_"conformal spatiotemporal state lattices"_**.
    - The motivation is to deal with _continuous_ state and action spaces and handle the presence of _dynamic obstacles_.
    - Several advantages (_I honnestly did not understand that point_): the ability to exploit the structure of the environment, to **consider time as part of the state-space** and respect the non-holonomic motion constraints of the vehicle.

- One term: **"_planning-based motion prediction_"**.
  - The resulting reward function can be used to **generate trajectory** (for prediction), using optimal control.
  - Simply put, it can be assumed that each vehicle in the scene behaves in the **"risk-averse"** manner **encoded by the model**, i.e. choosing actions leading to the lowest cost / highest reward.
  - This method is also called "**model**-based prediction" since it relies on a reward function or on the models of a MDP.
  - This prediction tool is not used alone but rather coupled with some **DBN-based maneuver estimation** (detailed in the [section on prediction](#Prediction_and_Manoeuvre_Recognition)).

---

</details>

He, X., Xu, D., Zhao, H., Moze, M., Aioun, F., & Franck, G. [2018].
**"A Human-like Trajectory Planning Method by Learning from Naturalistic Driving Data"**
[[html](https://ieeexplore.ieee.org/document/8500448)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://ieeexplore.ieee.org/document/8500448).](media/2018_he.PNG "[Source](https://ieeexplore.ieee.org/document/8500448).")  |
|:--:|
| *[Source](https://ieeexplore.ieee.org/document/8500448).* |

- Some related concepts:
  - `sampling-based trajectory planning`
- One idea: couple **_learning_** and **_sampling_** for **motion planning**.
  - More precisely, learn from human demonstrations (offline) how to **weight** different contributions in a **cost function** (as opposed to _hand-crafted_ approaches).
  - This cost function is then used for **trajectory planning** (online) to evaluate and select one trajectory to follow, among a set of candidates generated by **sampling methods**.
- One detail: the weights of the optimal cost function minimise the sum of [`prob`(`candidate`) * `similarities`(`demonstration`, `candidate`)].
  - It is clear to me how a _cost_ can be converted to _some probability_, using `softmax()`.
  - But for the **similarity measure** of a trajectory candidate, how to compute "its distance to the human driven one at the _same driving situation_"?
  - Should the expert car have driven _exactly on the same track_ before or is there any abstraction in the representation of the _situation_?
  - How can it **generalize at all** if the similarity is estimated only on the **location** and **velocity**? The traffic situation will be different between two drives.
- One quote:

> "The more similarity (i.e. less distance) the trajectory has with the human driven one, the higher probability it has to be selected."

---

</details>

Kuderer, M., Gulati, S., & Burgard, W. [2015].
**"Learning driving styles for autonomous vehicles from demonstration"**
[[pdf](http://ais.informatik.uni-freiburg.de/publications/papers/kuderer15icra.pdf)]

<details>
  <summary>Click to expand</summary>

- Some related concepts:
  - `inverse optimal control`, `IRL`
- One term: **"ME-IRL"** = **Maximum Entropy** IRL. The probability distribution over trajectories is in the form `exp`(`-cost[f(traj), θ]`), to model that **agents are exponentially more likely to select trajectories with lower cost**.
- One Q&A: The trajectory object is first mapped to some feature vector (`speed`, `acceleration` ...). How to then derive a cost (or reward) from these features?
  - The authors assume the cost function to be a **linear combination of the features**. The goal is then to **learn the weights**. But they acknowledge in the conclusion that it may be a too simple model. Maybe **neural nets** could help to capture some more complex relations.
- One quote about the _maximum likelihood approximation_ in ME-IRL:

> "We assume that the demonstrations are in fact generated by minimizing a cost function (IOC), in contrast to the assumption that demonstrations are samples from a probability distribution (IRL)".

---

</details>

## Prediction and Manoeuvre Recognition

Leon, F., & Gavrilescu, M. [2019].
**"A Review of Tracking, Prediction and Decision Making Methods for Autonomous Driving"**
[[pdf](https://arxiv.org/abs/1909.07707)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![Classification of __motion models__ based on three increasingly abstract levels - adapted from (Lefèvre, S., Vasquez. D. & Laugier C. - 2014). [Source](https://arxiv.org/abs/1909.07707).](media/2019_leon_1.PNG " Classification of motion __motion models__ on three increasingly abstract levels - adapted from (Lefèvre, S., Vasquez. D. & Laugier C. - 2014). [Source](https://arxiv.org/abs/1909.07707).")  |
|:--:|
| *Classification of __motion models__ based on three increasingly abstract levels - adapted from [(Lefèvre, S., Vasquez. D. & Laugier C. - 2014)](https://robomechjournal.springeropen.com/track/pdf/10.1186/s40648-014-0001-z). [Source](https://arxiv.org/abs/1909.07707).* |

- A reference to one white paper: [**"Safety first for automated driving"**](https://www.aptiv.com/docs/default-source/white-papers/safety-first-for-automated-driving-aptiv-white-paper.pdf) 2019 - from Aptiv, Audi, Baidu, BMW, Continental, Daimler, Fiat Chrysler Automobiles, HERE, Infineon, Intel and Volkswagen (alphabetical order). The authors quotes some of the good practices about **_Interpretation  and  Prediction_**:
  - Predict only a **short time** into the future (_the further the predicted state is in the future, the less likely it is that the prediction is correct_).
  - Rely on **physics** where possible (_a vehicle driving in front of the automated vehicle will not stop in zero time on its own_).
  - Consider the **compliance** of other road users with traffic rules.
- Miscellaneous notes about prediction:
  - The authors point the need of **high-level reasoning** (the more **abstract** the feature, the more reliable it is **long term**), mentioning both _"affinity"_ and _"attention"_ mechanisms.
  - They also call for **jointly** addressing vehicle **motion modeling** and **risk estimation** (_criticality assessment_).
  - **Gaussian Processed** is found to be a flexible tool for **modeling motion patterns** and is compared to Markov Models for prediction.
    - In particular, GP regressions have the ability to **quantify uncertainty** (e.g. **occlusion**).
  - > "**CNNs** can be **superior to LSTMs** for **temporal modeling** since trajectories are continuous in nature, do not have complicated "state", and have high spatial and temporal correlations".

---

</details>

Cho, K., Ha, T., Lee, G., & Oh, S. [2019].
**"Deep Predictive Autonomous Driving Using Multi-Agent Joint Trajectory Prediction and Traffic Rules"**
[[pdf](http://cpslab.snu.ac.kr/publications/papers/2019_iros_predstl.pdf)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![The framework consists of four modules: _encoder module_, _interaction module_, _prediction module_ and _control module_. [Source](http://cpslab.snu.ac.kr/publications/papers/2019_iros_predstl.pdf).](media/2019_cho_1.PNG "The framework consists of four modules: _encoder module_, _interaction module_, _prediction module_ and _control module_. [Source](http://cpslab.snu.ac.kr/publications/papers/2019_iros_predstl.pdf).")  |
|:--:|
| *The framework consists of four modules: _encoder module_, _interaction module_, _prediction module_ and _control module_. [Source](http://cpslab.snu.ac.kr/publications/papers/2019_iros_predstl.pdf).* |

- Some related concepts:
  - `LTL`, `STL`, `CVAE for prediction`, `MPC`, `NGSIM`
- One previous work: [_"Learning-Based Model Predictive Control under Signal Temporal Logic Specifications"_](http://cpslab.snu.ac.kr/publications/papers/2018_icra_mpcstl.pdf) by (Cho & Ho, 2018).
- One term: **_"robustness slackness"_** for `STL`-formula.
  - The motivation is to solve _dilemma situations_ (inherent to **strict compliance** when all rules cannot be satisfied) by **disobeying certain rules** based on their **predicted degree of satisfaction**.
  - The idea is to **filter out** non-plausible trajectories in the prediction step to only consider **valid prediction candidates** during planning.
  - The filter considers some **"rules"** such as `Lane keeping` and `Collision avoidance of front vehicle` or `Speed limit` (_I did not understand why they are equally considered_).
  - These rules are represented by **Signal Temporal Logic** (`STL`) formulas.
    - Note: `STL` is an extension of _Linear Temporal Logic_ (with _boolean predicates_ and _discrete-time_) with _real-time_ and _real-valued_ constraints.
  - A metric can be introduced to measure how well a given signal (_here, a trajectory candidate_) satisfies a `STL` formula.
    - This is called **_"robustness slackness"_** and acts as a **margin to satisfaction** of `STL`-formula.
  - This enables a **"control under temporal logic specification"** as mentioned by the authors.
- Architecture
  - **Encoder** module: The observed trajectories are fed to some `LSTM` whose internal state is used by the two subsequent modules.
  - **Interaction** module: To consider interaction, all `LSTM` states are **concatenated** (**_joint_** state) together with a feature vector of **relative distances**. In addition, a **CVAE** is used for multi-modality (several possible trajectories are **generated**) and **capture interactions** (_I did not fully understand that point_), as stated by the authors:
    - > "The latent variable `z` models inherent structure in the interaction of multiple vehicles, and it also helps to describe underlying ambiguity of future behaviors of other vehicles."
  - **Prediction** module: Based on the `LSTM` states, the **concatenated vector** and the **latent variable**, both **future trajectories** and **margins to the satisfaction** of each rule are predicted.
  - **Control** module: A `MPC` optimizes the control of the ego car, deciding **which rules should be prioritized** based on the two predicted objects (_trajectories_ and _robustness slackness_).

---

</details>

Han, T., Filev, D., & Ozguner, U. [2019].
**"An Online Evolving Framework for Modeling the Safe Autonomous Vehicle Control System via Online Recognition of Latent Risks"**
[[pdf](https://arxiv.org/abs/1908.10823)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![Both the **state space** and the **transition model** are adapted online, offering two features: **prediction** about the next state and **detection of unknown (i.e. `risky`) situations**. [Source](https://arxiv.org/abs/1908.10823).](media/2019_han_1.PNG "Both the **state space** and the **transition model** are adapted online, offering two features: **prediction** about the next state and **detection of unknown (i.e. `risky`) situations**. [Source](https://arxiv.org/abs/1908.10823).")  |
|:--:|
| *Both the **state space** and the **transition model** are adapted online, offering two features: **prediction** about the next state and **detection of unknown (i.e. `risky`) situations**. [Source](https://arxiv.org/abs/1908.10823).* |

- Some related concepts:
  - `MDP`, `action-state transitions matrix`, `SUMO`, `risk assessment`
- Motivation
  - _"_**_Rule-based_** and **_supervised-learning_** _methods cannot_ **_recognize unexpected situations_** _so that the AV controller cannot react appropriately under_ **_unknown circumstances_**_."_
  - Based on their previous work on RL [“Highway Traffic Modeling and Decision Making for Autonomous Vehicle Using Reinforcement Learning”](http://dcsl.gatech.edu/papers/iv2018.pdf) by (You, Lu, Filev, & Tsiotras, 2018).
- Main ideas: Both the **state space** and the the **transition model** (here discrete state space so transition matricies) of a MDP are **adapted online**.
  - I understand it as trying to **learn the transition model** (experience is generated using `SUMO`), hence to some extent going toward **_model-based RL_**.
  - The motivation is to assist any AV **control framework** with a so-called **_"evolving Finite State Machine"_** (**`e`-`FSM`**).
    - By **identifing state-transitions** precisely, the **future states** can be **predicted**.
    - By determining states uniquely (using **online-clustering** methods) and recognizing the state consistently (expressed by a probability distribution), initially **unexpected dangerous situations** can be detected.
    - It reminds some [ideas about risk assessment](https://github.com/chauvinSimon/IV19#risk-assessment-and-safety-checkers) discussed during IV19: the **discrepency between expected outcome and observed outcome** is use to **quantify risk**, i.e. the _surprise_ or _misinterpretation_ of the current situation).
- Some concerns:
  - _"The_ **_dimension_** _of transition matrices should_ **_be expanded_** _to represent state-transitions between all existing states"_
    - What when the scenario gets more complex than the presented _"simple car-following"_ and that the **state space** (treated as **_discrete_**) becomes huge?
  - In addition, _"the total **_number of transition matrices_** _is identical to the total_ **_number of actions_**"_.
    - Alone for the simple example, the acceleration command was sampled into `17` bins. **Continuous action spaces** are not an option.

---

</details>

Liu, S., Zheng, K., Member, S., Zhao, L., & Fan, P. [2019].
**"A Driving Intention Prediction Method Based on Hidden Markov Model for Autonomous Driving"**
[[pdf](https://arxiv.org/abs/1902.09068)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://arxiv.org/abs/1902.09068).](media/2019_lui.PNG "[Source](https://arxiv.org/abs/1902.09068).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1902.09068).* |

- Some related concepts:
  - `HMM`, `Baum-Welch algorithm`, `forward algorithm`
- One term: **"mobility feature matrix"**
  - The recorded data (e.g. absolute positions, timestamps ...) are processed to form the _mobility feature matrix_ (e.g. speed, relative position, lateral gap in lane ...).
  - Its size is `T × L × N`: `T` time steps, `L` vehicles, `N` types of mobility features.
  - In the _discrete characterization_, this matrix is then turned into a **set of observations** using K-means clustering.
  - In the _continuous case_, mobility features are modelled as Gaussian mixture models (GMMs).
- This work implements HMM concepts presented in my project [Educational application of Hidden Markov Model to Autonomous Driving](https://github.com/chauvinSimon/hmm_for_autonomous_driving).

---

</details>

Huang, X., Hong, S., Hofmann, A., & Williams, B. [2019].
**"Online Risk-Bounded Motion Planning for Autonomous Vehicles in Dynamic Environments"**
[[pdf](https://arxiv.org/abs/1904.02341)]

<details>
  <summary>Click to expand</summary>

- Some related concepts:
  - `intention-aware planning`, `manoeuvre-based motion prediction`, `POMDP`, `probabilistic risk assessment`, `CARLA`

One figure:

| ![[Source](https://arxiv.org/abs/1904.02341).](media/2019_huang.PNG "[Source](https://arxiv.org/abs/1904.02341).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1904.02341).* |

- One term: [**"Probabilistic Flow Tubes"**](https://dspace.mit.edu/handle/1721.1/76824) (PFT)
  - A *motion representation* used in the **"Motion Model Generator"**.
  - Instead of using **hand-crafted** rules for the transition model, the idea is to **learns human behaviours** from demonstration.
  - The inferred models are encoded with **PFTs** and are used to generate **probabilistic predictions** for both _manoeuvre_ (long-term reasoning) and _motion_ of the other vehicles.
  - The advantage of **belief-based probabilistic planning** is that it can avoid **over-conservative** behaviours while offering **probabilistic safety guarantees**.
- Another term: **"Risk-bounded POMDP Planner"**
  - The **uncertainty** in the intention estimation is then propagated to the decision module.
  - Some notion of **risk**, defined as the _probability of collision_, is evaluated and considered when taking actions, leading to the introduction of a **"chance-constrained POMDP"** (`CC-POMDP`).
  - The **online solver** uses a heuristic-search algorithm, [**Risk-Bounded AO\***](https://www.aaai.org/ocs/index.php/AAAI/AAAI16/paper/download/12321/12095) (**RAO\***), takes advantage of the **risk estimation** to prune the over-risky branches that violate the **risk constraints** and eventually outputs a plan with a **guarantee over the probability of success**.
- One quote (_this could apply to many other works_):

> "One possible future work is to test our work in real systems".

---

</details>

Sierra Gonzalez, D. [2019].
**"Towards Human-Like Prediction and Decision-Making for Automated Vehicles in Highway Scenarios"**
[[pdf](https://tel.archives-ouvertes.fr/tel-02184362/document)]
[[video - simulator](https://www.youtube.com/watch?v=Xx5OmV86CsM)]
[[code - simulator](https://github.com/marioney/hybrid_simulation/tree/decision-making)]

<details>
  <summary>Click to expand</summary>

- Some related concepts:
  - `planning-based motion prediction`, `manoeuvre-based motion prediction`

- Prediction techniques are often classified into three types:
  - `physics-based`
  - `manoeuvre-based` (and `goal-based`).
  - `interaction-aware`

- As I understood, the main idea here is to **combine prediction techniques** (and their advantages).
  - The **driver-models** (i.e. the reward functions previously learnt with IRL) can be used to identify the most likely, risk-aversive, anticipatory maneuvers. This is called the `model-based` prediction by the author since it relies on one _model_.
    - But relying only on **driver models** to predict the behavior of surrounding traffic might fail to predict dangerous maneuvers.
    - As stated, _"the model-based method is not a reliable alternative for the_ **_short-term_** estimation of behavior, since it cannot predict_ **_dangerous actions that deviate_** _from_ **_what is encoded in the model_**_"_.
    - One solution is to add a term that represents **how the observed movement of the target _matches_ a given maneuver**.
    - In other words, to **consider the noisy observation of the dynamics of the targets** and include theses so-called `dynamic evidence` into the prediction.

- Usage:
  - The resulting approach is used in the _probabilistic filtering framework_ to **update the belief** in the POMDP and in its **rollout** (to bias the **construction of the history tree** towards likely situations given the state and intention estimations of the surrounding vehicles).
  - It improves the inference of manoeuvers, **reducing rate of false positives** in the detection of lane change maneuvers and enables the exploration of situations in which the surrounding vehicles behave dangerously (not possible if relying on **safe generative models** such as `IDM`).

- One quote about this combination:

> "This model mimics the reasoning process of human drivers: they can guess what a given vehicle is likely to do given the situation (the **model-based prediction**), but they closely **monitor its dynamics** to detect deviations from the expected behavior".

- One idea: use this combination for **risk assessment**.
  - As stated, _"if the_ **_intended_** _and_ **_expected_** _maneuver of a vehicle_ **_do not match_**_, the situation is classified as dangerous and an alert is triggered"_.
  - This is a important concept of **risk assessement** I could [identify at IV19](https://github.com/chauvinSimon/IV19#risk-assessment-and-safety-checkers): a situation is dangerous if there is a discrepency between _what is expected_ (given the context) and _what is observed_.

- One term: **"_Interacting Multiple Model_"** (`IMM`), used as baseline in the comparison.
  - The idea is to consider a **group of motion models** (e.g. `lane keeping with CV`, `lane change with CV`) and continuously estimate which of them captures more accurately the dynamics exhibited by the target.
  - The final predictions are produced as a **weighted combination** of the **individual predictions of each filter**.
  - `IMM` belongs to the **_physics-based_** predictions approaches and could be extended for `maneuver inference` (called _dynamics matching_). It is often used to **maintain the beliefs** and **guide the observation sampling** in POMDP.
  - But the issue is that IMM completely **disregards the interactions between vehicles**.

---

</details>

Li, S., Li, N., Girard, A., & Kolmanovsky, I. [2019].
**"Decision making in dynamic and interactive environments based on cognitive hierarchy theory: Formulation, solution, and application to autonomous driving"**
[[pdf](https://arxiv.org/abs/1908.04005)]

<details>
  <summary>Click to expand</summary>

- Some related concepts:
  - `level-k game theory`, `cognitive hierarchy theory`, `interaction modelling`, `interaction-aware decision making`
- One concept: **`cognitive hierarchy`**.
  - Other drivers are assumed to follow some **"cognitive behavioral models"**, parametrized with a so called **"_cognitive level_"** `σ`.
  - The goal is to **obtain and maintain belief about `σ`** based on observation in order to **optimally respond** (using a `MPC`).
  - Three levels are considered:
    - level-`0`: driver that treats other vehicles on road as **stationary obstacles**.
    - level-`1`: **cautious/conservative** driver.
    - level-`2`: **aggressive** driver.

- One quote about the **"_cognitive level_"** of human drivers:

> "Humans are most commonly level-1 and level-2 reasoners".

Related works:

- Li, N., Oyler, D., Zhang, M., Yildiz, Y., Kolmanovsky, I., & Girard, A. [2016]. **"Game-theoretic modeling of driver and vehicle interactions for verification and validation of autonomous vehicle control systems"** [[pdf](https://arxiv.org/abs/1608.08589)]
  - > "If a driver assumes that the other drivers are level-`1` and **takes an action accordingly**, this driver is a level-`2` driver".
  - Use RL with **hierarchical assignment** to learn the policy:
    - First, the `π-0` (for level-`0`) is learnt for the ego-agent.
    - Then `π-1` with all the other participants following `π-0`.
    - Then `π-2` ...
  - **Action masking**: "If a car in the _left lane_ is in a parallel position, the controlled car _cannot change lane to the left_".
    - _"The use of these_ _**hard constrains**_ _eliminates the clearly undesirable behaviors better than through penalizing them in the reward function, and also_ **_increases the learning speed during training_**_"_

- Ren, Y., Elliott, S., Wang, Y., Yang, Y., & Zhang, W. [2019]. **"How Shall I Drive ? Interaction Modeling and Motion Planning towards Empathetic and Socially-Graceful Driving"** [[pdf](https://arxiv.org/abs/1901.10013)] [[code](https://github.com/scaperoth/carmachinelearning)]

| ![[Source](https://arxiv.org/abs/1901.10013).](media/2019_ren_1.PNG "[Source](https://arxiv.org/abs/1901.10013).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1901.10013).* |

| ![[Source](https://arxiv.org/abs/1901.10013).](media/2019_ren_2.PNG "[Source](https://arxiv.org/abs/1901.10013).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1901.10013).* |

---

</details>

## Rule-based Decision Making

Censi, A., Slutsky, K., Wongpiromsarn, T., Yershov, D., Pendleton, S., Fu, J., & Frazzoli, E. [2019].
**"Liability, Ethics, and Culture-Aware Behavior Specification using Rulebooks"**
[[pdf](https://arxiv.org/abs/1902.09355)]
[[videos and additional content](http://rulebooks.tech/)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![Defining the `rulebook`. [Source](https://arxiv.org/abs/1902.09355).](media/2019_censi_1.PNG "Defining the `rulebook`. [Source](https://arxiv.org/abs/1902.09355).")  |
|:--:|
| *Defining the `rulebook`. [Source](https://arxiv.org/abs/1902.09355).* |

| ![The `rulebook` is associated to an operator `=<` to prioritize between rules. [Source](https://arxiv.org/abs/1902.09355).](media/2019_censi_2.PNG "The `rulebook` is associated to an operator `=<` to prioritize between rules. [Source](https://arxiv.org/abs/1902.09355).")  |
|:--:|
| *The `rulebook` is associated to an operator `=<` to prioritize between rules. [Source](https://arxiv.org/abs/1902.09355).* |

| ![The `rulebook` serves for __deciding which trajectory to take__ and can be __adapted__ using a series of operations. [Source](https://arxiv.org/abs/1902.09355).](media/2019_censi_3.PNG "The `rulebook` serves for __deciding which trajectory to take__ and can be __adapted__ using a series of operations. [Source](https://arxiv.org/abs/1902.09355).")  |
|:--:|
| *The `rulebook` serves for __deciding which trajectory to take__ and can be __adapted__ using a series of operations. [Source](https://arxiv.org/abs/1902.09355).* |

- Some related concepts:
  - `sampling-based planning`, `safety validation`, `reward function`, `RSS`
- Allegedly how **nuTonomy** (an [**Aptiv**](https://www.aptiv.com/autonomous-mobility) company) cars work.

- One main concept: **_"rulebook"_**.
  - It contains multiple **`rules`**, that specify the **desired behavior** of the self-driving cars.
  - A rule is simply a **scoring function**, or **“violation metric”**, on the _realizations_ (= trajectories).
  - The **degree of violation** acts like some **penalty term**: here some examples of evalution of a realization `x` evaluated by a rule `r`:
    - For speed limit: `r`(`x`) = interval for which the car was above `45 km/h`.
    - For minimizing harm: `r`(`x`) = kinetic energy transferred to human bodies.
  - Based on Use as a comparison operator to rank candidate trajectories.
- One idea: **Hierarchy of rules**.
  - With many rules being defined, it may be impossible to find a realization (e.g. trajectory) that satisfies all.
  - But even in critical situation, the algorithm must make a choice - the least catastrophic option (hence no concept of _infeasibility_.)
  - To deal with this concept of **_"Unfeasibility"_**, priorities between conflicting rules which are therefore hierarchically ordered.
  - Hence a rulebook `R` comes with some operator `<`: <**`R`**, **`<`**>.
  - This leads to some concepts:
  - **Safety** vs. **infractions**.
    - Ex.: a rule _"not to collide with other objects"_ will have a **higher priority** than the rule _"not crossing the double line"_.
  - **Liability-aware** specification.
    - Ex.: (_edge-case_): Instruct the agent to _collide with the object on its lane_, rather than _collide with the object on the opposite lane_, since _changing lane_ will **provoke** an accident for which **it would be at fault**.
    - This is close to the [RSS](https://github.com/chauvinSimon/IV19#rss) ("responsibility-sensitive safety" model) of Mobileye.
  - **Hierarchy** between rules:
    - **Top**: Guarante **safety** of humans.
      - This is written **analytically** (e.g. a precise expression for the kinetic energy to minimize harm to people).
    - **Bottom**: _Comfort_ constraints and _progress_ goals.
      - Can be **learnt** based on observed behavior (and also tend to be platform- and implementation- specific).
    - **Middle**: All the other priorities among rule groups
      - There are somehow **open for discussion**.
- How to build a rulebook:
  - Rules can be defined **analytically** (e.g. `LTL` formalism) or **learnt** from data (for non-safety-critical rules).
  - **Violation functions** can be **learned from data** (e.g. `IRL`).
  - **Priorities** between rules can also be learnt.
- One idea: **manipulation** of rulebooks.
  - **Regulations** and cultures differ depending on the country and the state.
  - A rulebook <**`R`**, **`<`**> can easily be **adapted** using three operations (`priority refinement`, `rule augmentation`, `rule aggregation`).

- Related work: Several topics raised in this papers reminds me subjects addressed in [Emilio Frazzoli, CTO, nuTonomy - 09.03.2018](https://www.youtube.com/watch?v=dWSbItd0HEA)
  - 1- Decision making with **FSM**:
    - Too **complex** to code. Easy to make mistake. Difficult to adjust. Impossible to **debug** (:cry:).
  - 2- Decision making with **E2E learning**:
    - Appealing since there are too many possible scenarios.
    - But _how to prove that and justify it to the authorities?_
      - One solution is to **revert** such **imitation** strategy: **start by defining the rules**.
  - 3- Decision making **"cost-function-based"** methods
    - 3-1- **`RL`** / **`MCTS`**: not addressed here.
    - 3-2- **Rule**-based (not the `if`-`else`-`then` logic but rather **traffic/behaviour** rules).
  - First note:
    - Number of **rules**: small (`15` are enough for level-`4`).
    - Number of **possible scenarios**: huge (combinational).
  - Second note:
    - Driving bahaviours are **hard to code**.
    - Driving bahaviours are **hard to learn**.
    - But driving bahaviours are **easy to assess**.
  - Strategy:
    - 1- **Generate candidate** trajectories
      - Not only in _time_ and _space_.
      - Also in term of **semantic** (_logical trajectories in Kripke structure_).
    - 2- **Check** if they satisfy the constraints and **pick the best**.
      - This involves _linear_ operations.
  - Conclusion:
    - > "Rules and rules priorities, especially those that concern safety and liability, must be part of nation-wide regulations to be developed after an informed public discourse; it should not be up to engineers to choose these important aspects."
    - This reminds me [the discussion about social-acceptance](https://github.com/chauvinSimon/IV19#social-acceptance) I had at IV19.^
    - As E. Frazzoli concluded during his talk, the remaining question is:
      - _"We do not know how we want human-driven vehicle to behave?"_
      - **Once we have the answer, that is easy**.

Some figures from this related presentation:

| ![Candidate trajectories are not just __spatio-temporal__ but also __semantic__. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).](media/2018_frazzoli_1.PNG "Candidate trajectories are not just __spatio-temporal__ but also __semantic__. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).")  |
|:--:|
| *Candidate trajectories are not just __spatio-temporal__ but also __semantic__. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).* |

| ![Define __priorities__ between rules, as Asimov did for his laws. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).](media/2018_frazzoli_2.PNG "Define __priorities__ between rules, as Asimov did for his laws. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).")  |
|:--:|
| *Define __priorities__ between rules, as Asimov did for his laws. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).* |

| ![As raised here by the main author of the paper, I am still wondering how the presented framework deals with the different __sources of uncertainties__. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).](media/2018_censi_1.PNG "As raised here by the main author of the paper, I am still wondering how the presented framework deals with the different __sources of uncertainties__. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).")  |
|:--:|
| *As raised here by the main author of the paper, I am still wondering how the presented framework deals with the different __sources of uncertainties__. [Source](https://www.youtube.com/watch?v=dWSbItd0HEA).* |

---

</details>

Naumann, M., Königshof, H., & Stiller, C. [2019].
**"Provably Safe and Smooth Lane Changes in Mixed Traffic"**
[[pdf](https://www.mrt.kit.edu/z/publ/download/2019/Naumann2019LaneChange.pdf)]
[[video](https://www.mrt.kit.edu/z/publ/download/2019_LaneChange_Naumann.mp4)]
[[Code for Simulator](https://github.com/coincar-sim)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![The first `safe?` check might lead to conservative behaviours (huge gaps would be needed for safe lane changes). Hence it is relaxed with some `Probably Safe?` condition. [Source](https://www.mrt.kit.edu/z/publ/download/2019/Naumann2019LaneChange.pdf).](media/2019_naumann_1.PNG "The first `safe?` check might lead to conservative behaviours (huge gaps would be needed for safe lane changes). Hence it is relaxed with some `Probably Safe?` condition. [Source](https://www.mrt.kit.edu/z/publ/download/2019/Naumann2019LaneChange.pdf).")  |
|:--:|
| *The first `safe?` check might lead to conservative behaviours (huge gaps would be needed for safe lane changes). Hence it is relaxed with some `Probably Safe?` condition. [Source](https://www.mrt.kit.edu/z/publ/download/2019/Naumann2019LaneChange.pdf).* |

| ![[Source](https://www.mrt.kit.edu/z/publ/download/2019/Naumann2019LaneChange.pdf).](media/2019_naumann_2.PNG "[Source](https://www.mrt.kit.edu/z/publ/download/2019/Naumann2019LaneChange.pdf).")  |
|:--:|
| *[Source](https://www.mrt.kit.edu/z/publ/download/2019/Naumann2019LaneChange.pdf).* |

| ![Formulation by Pek, Zahn, & Althoff, 2017. [Source](https://mediatum.ub.tum.de/doc/1379669/794156.pdf).](media/2017_pek.PNG "Formulation by Pek, Zahn, & Althoff, 2017. [Source](https://mediatum.ub.tum.de/doc/1379669/794156.pdf).")  |
|:--:|
| *Formulation by Pek, Zahn, & Althoff, 2017. [Source](https://mediatum.ub.tum.de/doc/1379669/794156.pdf).* |

- Some related concepts:
  - `path-velocity decomposition`, `IDM`, `RSS`

- Main ideas:
  - The notion of **_safety_** is based on the **responsibility sensitive safety** (`RSS`) definition.
    - As stated by the authors, _"A **`safe`** lane change is guaranteed not to **`cause`** a collision according to the previously defined rules, while a single vehicle cannot ensure that it will never be involved in a collision."_
  - Use _set-based reachability analysis_ to prove the "RSS-safety" of lane change maneuvers based on **gap evaluation**.
    - In other words, it is the **responsibility** of the ego vehicle to **maintain safe distances** during the lane change manoeuvre.

- Related works: A couple of safe distances are defined, building on
  - **`RSS`** principles (_after IV19, I tried to summarize some of the RSS concepts [here](https://github.com/chauvinSimon/IV19#rss)_).
  - [_"Verifying the Safety of Lane Change Maneuvers of Self-driving Vehicles Based on Formalized Traffic Rules"_](https://mediatum.ub.tum.de/doc/1379669/794156.pdf), (Pek, Zahn, & Althoff, 2017)

---

</details>

Noh, S. [2018].
**"Decision-Making Framework for Autonomous Driving at Road Intersections: Safeguarding Against Collision, Overly Conservative Behavior, and Violation Vehicles"**
[[html](https://ieeexplore.ieee.org/document/8370800)]
[[video](https://www.youtube.com/watch?v=h7ExZ040wyk)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://ieeexplore.ieee.org/document/8370800).](media/2018_noh.PNG "[Source](https://ieeexplore.ieee.org/document/8370800).")  |
|:--:|
| *[Source](https://ieeexplore.ieee.org/document/8370800).* |

- Some related concepts:
  - `probabilistic risk assessment`, `rule-based probabilistic decision making`
- Many ML-based works criticize rule-based approaches (_over-conservative_, _no generalization capability_ and _painful parameter tuning_).
  - True, the presented framework contains **many parameters whose tuning may be tedious**.
  - But this approach just works! At least they **go out of the simulator** and show some experiments on a **real car**.
  - I really like [**their video**](https://www.youtube.com/watch?v=h7ExZ040wyk), especially the multiple camera views together with the `RViz` representation.
  - It can be seen that **probabilistic reasoning** and **uncertainty-aware** decision making are essential for **robustness**.
- One term: **"Time-to-Enter"** (tte).
  - It represents the time it takes a _relevant vehicle_ to reach the potential _collision area_ (CA), from its current position at its current speed.
  - To deal with uncertainty in the measurements, a variant of this heuristic is coupled with a Bayesian network for **probabilistic threat-assessment**.
- One Q&A: What is the difference between **_situation awareness_** and **_situation assessment_**?
  - In **_situation awareness_**, all possible routes are considered for the detected vehicles using a map. The vehicles whose potential route **intersect with the ego-path** are classified as _relevant vehicles_.
  - In **_situation assessment_**, a threat level in {`Dangerous`, `Attentive`, `Safe`} is inferred for each _relevant vehicle_.
- One quote:

> "The existing literature mostly focuses on motion prediction, threat assessment, or decision-making problems, but not all three in combination."

---

</details>

## Model Free Reinforcement Learning

Saxena, D. M., Bae, S., Nakhaei, A., Fujimura, K., & Likhachev, M. [2019].
**"Driving in Dense Traffic with Model-Free Reinforcement Learning"**.
[[pdf](https://arxiv.org/abs/1909.06710)]
[[simulator](https://github.com/sisl/AutomotiveDrivingModels.jl)]
[[code](https://github.com/honda-research-institute/DRLDriving)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![[Source](https://arxiv.org/abs/1909.06710).](media/2019_saxena_5.PNG "[Source](https://arxiv.org/abs/1909.06710).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1909.06710).* |

| ![The __occupancy-grid-like observation space__ is divided into `4` channels, each containing `3` lanes. A _ego-vehicle_ specific __feature vector__ is also considered. The authors use **policy-gradient** Proximal Policy Optimisation - `PPO` - method and decided not to share parameters between the _actor_ and the _critic_. [Source](https://arxiv.org/abs/1909.06710).](media/2019_saxena_1.PNG "The __occupancy-grid-like observation space__ is divided into `4` channels, each containing `3` lanes. A _ego-vehicle_ specific __feature vector__ is also considered. The authors use **policy-gradient** Proximal Policy Optimisation - `PPO` - method and decided not to share parameters between the _actor_ and the _critic_. [Source](https://arxiv.org/abs/1909.06710).")  |
|:--:|
| *The __occupancy-grid-like observation space__ is divided into `4` channels, each containing `3` lanes. A _ego-vehicle_ specific __feature vector__ is also considered. The authors use **policy-gradient** Proximal Policy Optimisation - `PPO` - method and decided not to share parameters between the _actor_ and the _critic_. [Source](https://arxiv.org/abs/1909.06710).* |

- Some related concepts:
  - `SISL`, `PPO`, `MPC`, `merging scenarios`
- One motivation: learn to perform comfortable merge into **dense traffic** using **model-free** RL.
  - _Dense traffic_ situations are difficult: traditional **rule-based** models **fail entirely**.
  - _Model-free_ means it **does not rely on driver models** of other vehicles, or even on **predictions about their motions**. No explicit model of **inter-vehicle interactions** is therefore needed.
  - _Model-free_ also means that natively, **safety cannot be guaranteed**. Some _masking_ mechanisms (called **_"overseer"_**) are contemplated for future work.
- One idea for _merging_ scenarios:
  - > Many other works "only accommodate a **fixed merge point** as opposed to the more realistic case of a **finite distance** for the task (lane change or merge) as in our work."
- One idea to adapt `IDM` to dense scenarios:
  - > "`IDM` is modified to include a **`stop-and-go`** behaviour that cycles between a *non-zero* and *zero* desired velocity in regular time intervals. This behaviour is intended to simulate real-world driving behaviours seen in heavy-traffic during rush-hour."
- One idea about **action space**:
  - > "Learning a policy over the acceleration and steering angle of a vehicle might lead to _jerky or oscillatory_ behaviour which is undesirable. Instead, we train our network to **predict the time derivatives** of these quantities, i.e. **jerk** `j` and **steering rate** `δ˙`. This helps us maintain a smooth signal over the true low-level control variables."
  - The policy for jerk and steering rate is parameterised as **Beta distributions** (allegedly _"this makes training more stable as the policy gradients are unbiased with respect to the finite support of `β`-distributions"_).
- One of their related works used as baseline:
  - ["Cooperation-Aware Lane Change Maneuver in Dense Traffic based on Model Predictive Control with Recurrent Neural Network"](https://arxiv.org/abs/1909.05665) from (Bae et al., 2019).
  - > "A `RNN` generates **predictions** for the motions of neighbouring vehicles based on a history of their observations. These predictions are then used to **create safety constraints** for an `MPC` optimisation that uses Monte Carlo rollouts to compute the (locally) optimal trajectory for the ego-vehicle."

---

</details>

Khurana, A. [2019].
**"Behavior Planning at Roundabouts"**.
[[pdf](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![Using recurrent units in a `DQN` and considering the action history. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).](media/2019_khurana_2.PNG "Using recurrent units in a `DQN` and considering the action history. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).")  |
|:--:|
| *Using recurrent units in a `DQN` and considering the action history. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).* |

| ![`Hidden modes`: decomposing the __non-stationary__ environment into __multiple stationary environments__, where each `mode` is an `MDP` with __distinct dynamics__. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).](media/2019_khurana_3.PNG "`Hidden modes`: decomposing the __non-stationary__ environment into __multiple stationary environments__, where each `mode` is an `MDP` with __distinct dynamics__. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).")  |
|:--:|
| *`Hidden modes` framework: decomposing the __non-stationary__ environment into __multiple stationary environments__, where each `mode` is an `MDP` with __distinct dynamics__. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).* |

- Some related concepts:
  - `POMDP`, `SUMO`, `generalization`
- One idea: using **reccurent nets** (hence `D`**`R`**`DQN`) to integrate **history** in order to cope with **partial observability**.
  - Two `LSTM` layers, (considering `15` past observations) was added after `DQN` layers.
  - They also try to include the **action history** of the agent, leading to **`A`**`DRQN`. But this does not modify the score.
- Another idea: **several** context-based **action spaces**:
  - Decision in **roundabouts** consists in:
    - **Merging** - `action_space` = {`go`, `no go`}
    - **Traversal** - `action_space` = {`acc`, `decelerate`, `change-left`, `change-right` , `cv`}
    - **Exit** - `action_space` = {`acc`, `decelerate`, `change-left`, `change-right` , `cv`}
  - Justification for using **discrete** action space: behavioral planning happens on a **slower time scale** than motion planning or trajectory control.
  - This reminds me some works on **hierarchical RL** (e.g. `Options` framework).
- Another idea: **Curriculum learning**
  - > "Each model is **first trained without any other interacting vehicle** so that it learns the most optimal policy and **later with other vehicles** with random initialization. In later stages, an **additional bonus reward** is given to merging and traversal if they lead to successful exit to **enable long-term** consistent behaviors."
- One note about the `POMDP` formulation:
  - > "This also enables us to **integrate planning** and **prediction** into a single problem, as the agent learns to reason about its future."
  - I am a little bit confused by their formulation of the **`PO`**`MDP`.
    - I would have expected some **hidden parameters** to be defined and some **belief** on them to be tracked, as often done for AD, _e.g. the intention of other participants_.
    - Instead, the `partial observability` refers here to the **uncertainty in perception**: _"there is a `0.2` probability that a car present in the agent’s perception field is **dropped** in the observation"_.
    - This **imperfect state estimation** encourages the **robustness**.
- One note about **model-free** RL:
  - Using RL seems relevant to offer **generalization** in **complex scenarios**.
  - But as noted by the authors: _"the rule-based planner outperforms others in the case of a_ **_single-lane roundabout_** as there’s no scope for lane change."_
- One addressed problem: **_"non-stationary"_** environments.
  - A **single policy** learned on a **specific traffic density** may perform badly on another density (the _dynamic_ of the world modelled by the `MDP` changes over time).
  - The goal is to **generalize across different traffic scenarios**, especially accross different **traffic densities**.
  - One idea is to decompose the **non-stationary** environment into **multiple stationary environments**, where each _mode_ is an MDP with **distinct dynamics**: this method is called `Hidden modes`.
    - _How to then switch between modes?_ The authors proposed to use **external information** (_Google Maps_ could for instance tell ahead if traffic jams occur on your planned route).
    - But as the number of **discrete modes** increases, the `hidden-mode` method suffers from oscillations at the **boundaries of the mode transitions**.
  - Thus the second idea is to consider one **single model**: this method is called `Traffic-Conditioned`.
    - Some continuously varying feature (ratio of `velocity of other vehicles` to `target speed`) is used. It should be representative of the non-stationary environment.
  - One quote about the relation of **hidden-mode formulation** to **hierarchical RL**:
    - > "For generalization, the hidden-mode formulation can also be viewed as a **hierarchical learning** problem where one `MDP`/`POMDP` framework **selects the mode** while the other learns the driving behavior **given the mode**".

---

</details>

Hu, B., Li, J., Yang, J., Bai, H., Li, S., & Sun, Y. [2019].
**"Reinforcement Learning Approach to Design Practical Adaptive Control for a Small-Scale Intelligent Vehicle"**
[[pdf](https://www.researchgate.net/publication/335693770_Reinforcement_Learning_Approach_to_Design_Practical_Adaptive_Control_for_a_Small-Scale_Intelligent_Vehicle)]

<details>
  <summary>Click to expand</summary>

- Some related concepts:
  - `non-deep RL`, `online learning`, `model-based RL`

- What I like in this paper:
  - The authors make sure they understand **tabular RL** methods, e.g. the difference between on-policy `SARSA(1)`, `SARSA(λ)` and off-policy `Q-learning`, before going to deep RL.
    - > "Compared with `Q-learning`, which can be described as _greedy_, _bold_, and _brave_, `SARSA(1)` is a _conservative_ algorithm that is sensitive to control errors."
  - They include a **model-based** algorithm (`Dyna-Q`) in their study. This seems promising when training directly in real world, where the **sampling efficiency** is crucial.
  - They claim RL methods bring advantages compared to **PID controllers** in term of **adaptability** (_generalization_ - i.e. some PID parameters appropriate for driving on a _straight road_ may cause issues in _sharp turns_) and burden of **parameter tuning**.
  - They consider the **sampling efficiency** (better for _model-based_) and **computational time per step** (better for _`1`-step `TD` methods_ than for _`SARSA(λ)`_).
    - > "`Q-learning` algorithm has a _poorer converging speed_ than the `SARSA(λ)` and `Dyna-Q` algorithm, it balances the performance between the _converging speed_, the _final control behavior_, and the _computational complexity_."
  - Obviously, this remains far from real and serious AD. But this paper gives a good example of application of basic RL methods.

---

</details>

Tram, T., Batkovic, I., Ali, M., & Sjöberg, J. [2019].
**"Learning When to Drive in Intersections by Combining Reinforcement Learning and Model Predictive Control"**
[[pdf](https://arxiv.org/abs/1908.00177)]

<details>
  <summary>Click to expand</summary>

Two figures:

| ![[Source Left](https://arxiv.org/abs/1908.00177) - [Source Right](https://pdfs.semanticscholar.org/fba5/6bac9a41b7baa2671355aa113462d2044fb7.pdf).](media/2019_tram_3.PNG "[Source Left](https://arxiv.org/abs/1908.00177) - [Source Right](https://pdfs.semanticscholar.org/fba5/6bac9a41b7baa2671355aa113462d2044fb7.pdf).")  |
|:--:|
| *[Source Left](https://arxiv.org/abs/1908.00177) - [Source Right](https://pdfs.semanticscholar.org/fba5/6bac9a41b7baa2671355aa113462d2044fb7.pdf).* |

| ![This work applies the **path-velocity decomposition** and focuses on the longitudinal control. **Three intentions** are considered: aggressive `take-way`, `cautious` (slows down without stopping), and passive `give-way`. [Source](https://arxiv.org/abs/1908.00177).](media/2019_tram_2.PNG "This work applies the **path-velocity decomposition** and focuses on the longitudinal control. **Three intentions** are considered: aggressive `take-way`, `cautious` (slows down without stopping), and passive `give-way`. [Source](https://arxiv.org/abs/1908.00177).")  |
|:--:|
| *This work applies the **path-velocity decomposition** and focuses on the longitudinal control. **Three intentions** are considered: aggressive `take-way`, `cautious` (slows down without stopping), and passive `give-way`. [Source](https://arxiv.org/abs/1908.00177).* |

- Some related concepts:
  - `model-free RL`, `MPC`, `Q-Masking`, `POMDP`
- Main idea: **hierarchy** in _learnt_/_optimized_ decision-making.
  - A **high-level decision module** based on RL uses the **feedback from the MPC controller** in the reward function.
  - The MPC controller is also responsible for **handling the comfort** of passengers in the car by **generating a smooth acceleration profile**.
- Previous works:
  - [_"Learning Negotiating Behavior Between Cars in Intersections using Deep Q-Learning"_](http://arxiv.org/abs/1810.10469) - (Tram, Batkovic, Ali, & Sjöberg, 2019)
  - [_"Autonomous Driving in Crossings using Reinforcement Learning"_](https://pdfs.semanticscholar.org/fba5/6bac9a41b7baa2671355aa113462d2044fb7.pdf) - (Jansson & Grönberg, 2017)
  - In particular they reused the concept of **_"actions as Short Term Goals (`STG`)"_**. e.g. _keep set speed or yield for crossing car_ instead of some numerical acceleration outputs.
    - This allows for **comfort** on actuation and **safety** to be **tuned separately**, reducing the policy selection to a classification problem.
    - The use of such abstracted / high-level decisions could be a first step toward **hierarchical RL** techniques (`macro-action` and `option` framework).
  - Another contribution consists in remplacing the **Sliding Mode** (**`SM`**) controller used previously by a `MPC`, allegedly to _"achieve safer actuation by using constraints"_.
    - The **intention of all agents** is implemented with a `SM` controller with **various target values**.
- I find it valuable to have details about the **training phase** (no all papers do that). In particular:
  - The **normalization** of the input features in [`-1`, `1`].
  - The **normalization** of the reward term in [`-2`, `1`].
  - The use of **equal weights** for inputs that describe the state of **interchangeable objects**.
  - Use of a **`LSTM`**, as an alternative to a **DQN with stacked observations**. (Findings from _(Jansson & Grönberg, 2017)_).
- Additional notes:
  - The main benefits of the combination seems to be about **avoiding over conservative behaviours** while improving the **"sampling-efficient"** of the model-free RL approach.
    - Such approachs looks to be particularly relevant (in term of _success rate_ and _collision-to-timeout ratio_ [`CTR`]) for complex scenarios, e.g. `2`-crossing scenarios.
    - For simple cases, the performance stays close to the baseline.
  - The reliance (_and the burden_) on an appropriate **parametrisation** inherent to rule-based has not disappeared and the **generalisation seems limited**:
    - _"Since MPC uses_ **_predefined models_** _, e.g. vehicle models and other obstacle prediction models, the_ **_performance relies on their accuracy and assumptions_** _."_
  - The problem is formulated as a `POMDP`.
    - _Honestly, from a first read, I did not find how `belief tracking` is performed. Maybe something related to the internal memory state of the LSTM cells?_
  - Sadly the **simulator** seems to be **home-made**, which makes reproducibility tricky.
- One quote about **`Q-masking`**, i.e. the technique of not exposing to the agent dangerous or non-applicable actions during the action selection.
  - > "**Q-masking helps the learning process** by **reducing the exploration** space by disabling actions the agent does not need to explore."
  - Hence the agent **does not have to explore** these options, while ensuring a certain level of safety (but this requires another **rule-based module** :blush: ).

---

</details>

Bouton, M., Nakhaei, A., Fujimura, K., & Kochenderfer, M. J. [2019].
**"Cooperation-Aware Reinforcement Learning for Merging in Dense Traffic"**
[[pdf](https://arxiv.org/abs/1906.11021)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://arxiv.org/abs/1906.11021).](media/2019_bouton.PNG "[Source](https://arxiv.org/abs/1906.11021).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1906.11021).* |

- Some related concepts:
  - `POMDP`, `offline RL`, `value-based RL`, `interaction-aware decision making`, `belief state planning`
- One idea: offline **belief state RL** to solve dense merging scenarios modelled as a POMDP.
  - A belief updater explicitly maintains a probability distribution over the **driver cooperation levels** of other cars.
- Another idea: projection of the merging vehicle on the main lane. This reduces the problem to **one dimension**, allowing for IDM. Similar to the [abstraction presented by R. Regele](https://ieeexplore.ieee.org/document/4488328/).
- One term: **"C-IDM"**: Cooperative Intelligent Driver Model.
  - It builds on IDM to control the longitudinal acceleration of the other vehicle and adds a **cooperation level** `c` `∈` [`0`, `1`].
  - Non-cooperative vehicles (`c=0`) are blind to the merging ego car and follow the vanilla IDM, while cooperative vehicles will yield (`c=1`).
- Another term: **"Burn-in time"**
  - When creating a new configuration, vehicles and their parameters are drawn from distributions and then move according to **driver models**.
  - The idea is between `10s` and `20s` before spawning the ego car, to allows the initial state to converge to a **more realistic situation**.
  - It should help for **generalization**.
- Another term: **"Curriculum Learning"**: The idea is to train the agent by **gradually increasing the difficulty** of the problem. In this case the traffic density.
- Two take-aways (similar to what [I identified at IV19](https://github.com/chauvinSimon/IV19))

> "Previous works has shown that only relying on deep RL is not sufficient to achieve safety. The deployment of those policies would require the addition of a safety mechanism."

> "Using deep reinforcement learning policies to guide the search of a classical planner (MCTS) may be a promising direction."

---

</details>

Hu, Y., Nakhaei, A., Tomizuka, M., & Fujimura, K. [2019].
**"Interaction-aware Decision Making with Adaptive Behaviors under Merging Scenarios"**
[[pdf](https://arxiv.org/abs/1904.06025)]
[[video](https://www.youtube.com/watch?v=2CTTFHDW1ec)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![Note: the visibility of each agent is assumed to be `100m` in front and back, with `0.5m`/`cell` resolution, for both its current lane (`obs_cl`) and the other lane (`obs_ol`). [Source](https://arxiv.org/abs/1904.06025).](media/2019_hu.PNG "Note: the visibility of each agent is assumed to be `100m` in front and back, with `0.5m`/`cell` resolution, for both its current lane (`obs_cl`) and the other lane (`obs_ol`). [Source](https://arxiv.org/abs/1904.06025).")  |
|:--:|
| *Note: the visibility of each agent is assumed to be `100m` in front and back, with `0.5m`/`cell` resolution, for both its current lane (`obs_cl`) and the other lane (`obs_ol`). [Source](https://arxiv.org/abs/1904.06025).* |

- Some related concepts:
  - `multi agent RL`, `interaction-aware decision making`, `curriculum learning`, `action masking`

- One term: **"IDAS"**: interaction-aware decision making with adaptive strategies.
  - The main goal is to generate manoeuvres which are **safe** but **less conservative** than rule-based approaches such as **IDM** and/or FSM.
  - The idea is to learn how to **negotiate** with other drivers, or at least consider **interactions** in the decision process.

- One idea: use multi-agent RL (**MARL**) to consider **interactions** between the multiple road entities.
  - In particular, the agent receives rewards for its **personal objective** as well as for **its contribution to the team’s "success"** (_multi-agent credit assignment_).

- One idea: a **masking mechanism** prevents the agent from **exploring states that violate common sense** of human judgment (c.f. [RSS](https://www.mobileye.com/responsibility-sensitive-safety/)) and increase the **learning efficiency**.
  - This idea of adding **rule-based constraints** to a RL policy has been applied in many works. Recently in [Wang, J. et al.](https://arxiv.org/abs/1904.00231) for instance where prediction is also considered.
  - Here, masking is not only based on **safety**, but also on considers **vehicle kinematics** and **traffic rules**.
  - A remaining question is **_where_ to apply the masking**: either _before_ the **action selection** (exposing only a subset of feasible actions), or _after_ (penalizing the agent if it takes a forbidden action).

- One quote (on the way to _transfer_ to the real world):

> "A motion control module will convert the discretized acceleration of the behaviour planner into continuous acceleration by applying algorithms like MPC at a higher frequency (100Hz)".

---

</details>

Bacchiani, G., Molinari, D., & Patander, M. [2019].
**"Microscopic Traffic Simulation by Cooperative Multi-agent Deep Reinforcement Learning"**
[[pdf](https://arxiv.org/abs/1903.01365)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://arxiv.org/abs/1903.01365).](media/2019_bacchiani.PNG "[Source](https://arxiv.org/abs/1903.01365).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1903.01365).* |

- Some related concepts:
  - `multi-agent A3C`, `off-policy learning`
- One idea: **"parallel actor-learners"**: to **decorrelate** the sequence of experiences used to update the policy network, a common approach is to sample <`s`, `a`, `r`, `s'`> tuples from a memory buffer (**_experience replay_**).
  - Here, a **multiple-agent** setting is used instead: each agent acts in a different **instance of the environment** (hence diverse experiences) and sends its updates asynchronously to a **central network**.
- Another idea: **"hybrid state representation"**: coupling some _grid-like_ representation (`path to follow`, `obstacles`, `navigable space`) with a vector of _explicit features_ (e.g. `target speed`, `distance to goal`, `elapsed time ratio`).
  - This combination offers a **generic scene representation** (i.e. independent of the number of vehicles) while allowing for tuning explicit goals (`target speeds`) in the state.
  - Such _hybrid_ representation seems popular, as [identified at IV19](https://github.com/chauvinSimon/IV19#generic-scene-representation)).
- Other ideas:
  - Stacking the `n=4` most recent views to _capture the evolution_ of the scene (e.g. relative speeds).
  - **_Action repeat_** technique for _temporal abstraction_ to stabilize the learning process (c.f. "_frame skip_").
- One concept: **"Aggressiveness tuning"**. Together with the `target speed`, the `elapsed time ratio` (ETR) feature is used to tune the aggressiveness of the car:

> "ETR Values close to `1` will induce the agent to drive faster, in order to avoid the predicted negative return for running out of time. Values close to `0` will tell the driver that it still has much time, and it is not a problem to yield to other vehicles."

---

</details>

Huegle, M., Kalweit, G., Mirchevska, B., Werling, M., & Boedecker, J. [2019].
**"Dynamic Input for Deep Reinforcement Learning in Autonomous Driving"**
[[pdf](https://arxiv.org/abs/1907.10994)]
[[video](https://www.youtube.com/watch?v=mRQgHeAGk2g)]

<details>
  <summary>Click to expand</summary>

- One diagram is better than 100 words:

| ![By summing all the dynamic terms (one per surrounding vehicle), the input keeps a constant size. [Source](https://arxiv.org/abs/1907.10994).](media/2019_huegle.PNG "By summing all the dynamic terms (one per surrounding vehicle), the input keeps a constant size. [Source](https://arxiv.org/abs/1907.10994).")  |
|:--:|
| *By summing all the dynamic terms (one per surrounding vehicle), the input keeps a constant size. [Source](https://arxiv.org/abs/1907.10994).* |

- Some related concepts:
  - `feature engineering`, `off-policy learning`, `DQN`, `SUMO`
- One goal: find a **flexible** and **permutation-invariant** representation to **deal with variable sized inputs** (_variable number of surrounding vehicles_) in high-level decision making in autonomous lane changes, using model-free RL.
  - Four representations are considered:
    - Relational Grid (fixed-size vector)
    - Occupancy grid (fixed-size grid processed by some `CNN`)
    - Set2Set-Q (some `RNN` combined with an _attention mechanism_ to create a set representation which is **permutation invariant** w.r.t. the input elements)
    - DeepSet-Q (proposed approach)
  - They are used as inputs to train and evaluate DQN agents.
- One finding:
  - "`Deep Sets` were able to outperform CNN and **recurrent attention approaches** and demonstrated **better generalization** to unseen scenarios".
- One quote: about the use of **policy-based** (as opposed to _value-based_), **on-policy** (as opposed to _off-policy_), model-free RL (here `PPO`).

> "Due to the higher demand for training and the non-trivial application to autonomous driving tasks of `on-policy` algorithms, we switched to `off-policy` Q-learning."

---

</details>

Jaâfra, Y., Laurent, J.-L., Deruyver, A., & Naceur, M. S. [2019].
**"Seeking for Robustness in Reinforcement Learning : Application on Carla Simulator"**
[[pdf](https://openreview.net/pdf?id=Bke03G85DN)]

<details>
  <summary>Click to expand</summary>

- Some background:

| ![`n`-step TD learning. [Source](http://www0.cs.ucl.ac.uk/staff/d.silver/web/Teaching.html).](media/2015_silver.PNG "`n`-step TD learning. [Source](http://www0.cs.ucl.ac.uk/staff/d.silver/web/Teaching.html).")  |
|:--:|
| *`n`-step TD learning. [Source](http://www0.cs.ucl.ac.uk/staff/d.silver/web/Teaching.html).* |

- Some related concepts:
  - `CARLA`, `A2C`
- One related work: reading this paper reminded me of one conclusion of the 2019 [CARLA AD Challenge](https://carlachallenge.org/):

> "**Robust** open-source AV stacks are not a commodity yet: **No public AV stack has solved the challenge yet**."

- One idea: use an actor-critic architecture with **multi-step returns** (`n`-step `A2C`) to _"achieve a better robustness"_.
  - The introduction of a **critic** aims at **reducing the variance** of the **gradient** of _policy-based_ methods.
  - As illustrated in the above figure, in _value-based_ methods, the **TD-target** of a critic can be computed in several ways:
    - With boostrapped, using the current estimate for the next state `s'`: `1`-step TD - **low variance** but biased estimate ...
    - ... Up to considering all the steps in the trajectory until termination: `Monte Carlo` - high variance but **unbiased estimate**.
    - In between are **multi-step returns critics** (`MSRC`). Obviously a trade-off between **bias**/**variance**.
- Some limitations:
  - The `MDP` is not very detailed, making **reproduction and comparison impossible**.
    - For instance, the `action` space allegedly contains `3` discrete driving instructions [`steering`, `throttle`, and `brake`], but not concrete number is given.
    - The same applies to the `state` and `reward` function: no quantitative description.
    - Based on their text, I can assume the authors were participating to the 2019 CARLA AD challenge. Maybe [track `1`](https://carlachallenge.org/challenge/). But again, information about the _town_/_scenario_ is missing.
  - No real comparison is performed: it should be feasible to use the built-it **rule-based agent** present in CARLA as a **baseline**.
  - _Why not also supplying a video of the resulting agent?_
- One additional work: **_Meta-Reinforcement Learning for Adaptive Autonomous Driving_**, (Jaafra, Luc, Aline, & Mohamed, 2019) [[pdf](https://openreview.net/pdf?id=S1eoN9rsnN)] [[poster](https://www.ds3-datascience-polytechnique.fr/wp-content/uploads/2019/06/DS3-552_2019.pdf)].
  - This idea is to use **multi-task learning** to improve **generalization** capabilities for a AD controller.
  - As detailed, _"Meta-learning refers to **learn-to-learn** approaches that aim at **training** a model on a set of different but linked tasks and subsequently **generalize** to new cases using **few additional examples**"_.
  - In other words, the goal is to find an optimal **initialization of parameters**, to then quickly adapt to a new task through a few standard gradient descents(**few-shot generalization**).
  - A **gradient-based meta-learner** inspired from **_Model-Agnostic Meta-Learning_** (`MAML` - Finn et al., 2017) is used.
  - RL performance in **non-stationary** environments and generalisation in AD are interesting topics. But no clear benefit is demonstrated, and the above limitations apply also here.

---

</details>

Isele, D., Nakhaei, A., Fujimura, K., [2018].
**"Safe Reinforcement Learning on Autonomous Vehicles."**
[[html](https://www.researchgate.net/publication/330595797_Safe_Reinforcement_Learning_on_Autonomous_Vehicles)]

<details>
  <summary>Click to expand</summary>

| ![ The `prediction` module **masks** undesired actions at each time step. [Source](https://www.researchgate.net/publication/330595797_Safe_Reinforcement_Learning_on_Autonomous_Vehicles).](media/2018_isele_1.PNG "The `prediction` module **masks** undesired actions at each time step. [Source](https://www.researchgate.net/publication/330595797_Safe_Reinforcement_Learning_on_Autonomous_Vehicles).")  |
|:--:|
| *The `prediction` module **masks** undesired actions at each time step. [Source](https://www.researchgate.net/publication/330595797_Safe_Reinforcement_Learning_on_Autonomous_Vehicles).* |

- Some related concepts:
  - `action masking`, `risk assessment`, `reachability set`, `SUMO`
- Previous works about **learning control with `DQN`s** at diverse intersections:
  - ["Navigating occluded intersections with autonomous vehicles using deep reinforcement learning"](https://arxiv.org/abs/1705.01196) - (Isele et al., 2017)
  - ["Transferring Autonomous Driving Knowledge on Simulated and Real Intersections"](https://arxiv.org/abs/1712.01106) - (Isele et al., 2017)
- _How to make model-free RL_ **_"safe"_**? Two options are mentioned (both required expert knowledge):
  - 1- Modifying the **reward function** (requires careful tuning).
  - 2- **Constraining exploration** (e.g. `action masking`, `action shielding`).
    - > "The methods can completely **forbid undesirable states** and are usually accompanied by **formal guarantees**".
    - In addtion, the **learning efficiency** can be increased (fewer states to explore).
  - For additional contents on _"Safe-RL in the context of autonomous vehicles"_, one could read [this github project](https://github.com/AliBaheri/Safe-Reinforcement-Learning) by @AliBaheri.
- Here: The `DQN` is augmented with some **action-masking mechanism**.
  - More precisely, a **prediction model** is used:
    - The **predicted position** of the **ego car** is compared against the predicted position of **all other traffic cars** (called `forward predictions`).
    - If an **overlap** of the regions is detected, the action is marked as **unsafe**.
    - Else, the agent is allowed to **freely explore the _safe_ state space**, using traditional model-free RL techniques.
  - Note: this illustrates the strong relation between `prediction` and `risk assessment`.
- One challenge: Ensuring "safety" not just for the next step, but for the **whole trajectory**.
  - > "To ensure that the agent never takes an unsafe action, we must check not only that a given action will not cause the agent to **transition to an unsafe state** in the **next time step**, but also that the action will not force the agent into an **unsafe state** **at some point in the future**."
  - > "Note that this is closely related to the **_credit assignment problem_**, but the **risk must be assigned prior to acting**".
  - _This made me think of tree search techniques, where a path is explored until its terminal node_.
  - To cope with the **exponential complexity**, the authors proposed some approximation to **restrict the exploration space**.
    - One of them being the `temporal abstraction` for actions (see [this video series](https://www.youtube.com/watch?v=jTcCyDJYK-Q) for a quick introduction).
    - The idea of this so-called **`option`** or `intentions` framework, is to distinguish between `low-level` and `high-level` actions
      - > "This can be thought of as selecting an **open-loop high-level** decision followed by subsequent **bounded** **closed-loop low-level** corrections."
      - For a given time horizon, the trajectories described with these options are way shorter, hence **reducing the size of the state set** that is to be checked.
    - This leads to the definition of a _functional local-state_ _(I did not understand all the details)_ including some **variance** term:
      - > "The variance acts as a bound that **encompasses the variety of low-level actions** that produce similar high-level actions. Additionally, we will use the variance to **create safety bounds**".
- One remark: similar to the **"collision checker"** for _path planning_, I can imagine that this **prediction module** becomes the **computational bottleneck** of the framework.

---

</details>

Plessen, M. G. [2017].
**"Automating Vehicles by Deep Reinforcement Learning Using Task Separation with Hill Climbing."**
[[pdf](https://arxiv.org/abs/1711.10785)]

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1711.10785).](media/2017_plessen_1.PNG "[Source](https://arxiv.org/abs/1711.10785).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1711.10785).* |

- Some related concepts:
  - `stochastic policy search`, `gradient-free RL`, `policy-gradient RL`, `reward shaping`
- One remark: to be honnest, I find this publication _not very easy_ to understand. But **it raises important questions**. Here are some take-aways.

- One term: `(TSHC)` = **_Task Separation with Hill Climbing_**
  - _Hill Climbing_ has nothing to do with the [gym _MountainCar_ env](https://github.com/openai/gym/wiki/MountainCar-v0).
    - It rather refers to as a **gradient-free** optimization method: the parameters are updated based on **greedy local search**.
    - For several reasons, the author claims derivative-free optimization methods are simpler and more appropriate for his problem, compared to **policy-gradient RL** optimization methods such as `PPO` and `DDPG` where tuned-parameters are numerous and sparse rewards are propagating very slowly.
  - The idea of _Task Separation_ concerns the main objective of the training phase: "encode many desired **motion primitives** (_training tasks_) in a neural network", hoping for **generalisation** when exposed to new tasks.
    - It is said to serve for **exploration** in optimization: each task leads to a possible region with locally optimal solution, and the best solution among all identified locally optimal solutions is selected.
- One concept: `sparse reward`.
  - **Reward shaping** is an important problem when formulation the decision-making problem for autonomous driving using a (PO)MDP.
  - The reward signal is the main signal used for the agent to update its policy. But if it **only receives positive reward** when **reaching the goal state** (i.e. **_sparse reward_**), two issues appear:
    - First, it will take random actions until, by chance, it gets some non-zero reward. Depending on how long it takes to get these non-zero rewards, it might **take the agent extremely long to learn anything**.
    - Secondly, because nonzero rewards are seen so rarely, the sequence of actions that resulted in the reward might be very long, and it is **not clear which of those actions were really useful** in getting the reward. This problem is known as **credit assignment** in RL. (Explanations are from [here](https://www.quora.com/Why-are-sparse-rewards-problematic-in-Reinforcement-Learning-RL-difficult)).
  - Two options are considered in this work:
    - **_"Rich reward signals"_**, where a feedback is provided at **every time step** (`r` becomes function of `t`).
    - **_"Curriculum learning"_**, where the leanring agent is first provided with **simpler examples** before gradually increasing complexity.
  - After trials, the author claims that no consistent improvement could be observed with these two techniques, adding that the **design** of both rich reward signals and "simple examples" for curriculum learning are problematic.
    - He rather kept working with sparse rewards (**_maximal sparse rewards_**), but introduced some _"virtual velocity constraints"_ to speed up the training.
- I like the points he made concerning **feature selection** for the state, i.e. how to design the state `s(t)` of the MDP.
  - He notes that `s(t)` must always relate the current vehicle state with **reference to a goal state**.
    - In other words, one should use **relative features** for the description of the position and velocity, relative to their targets.
  - In addition, `s(t)` should also **consider the past** and embed a collection of multiple past time measurements.
    - It seems sounds. But this would indicated that the **"Markov property"** in the MDP formulation does not hold.

---

</details>

## Model Based Reinforcement Learning

Baheri, A., Kolmanovsky, I., Girard, A., Tseng, E., & Filev, D. [2019].
**"Vision-Based Autonomous Driving : A Model Learning Approach"**
[[pdf](https://www.researchgate.net/publication/332912542_Vision-Based_Autonomous_Driving_A_Model_Learning_Approach)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![ The `perception` module, the `memory` or `prediction` module, and the `control` module. [Source](https://www.researchgate.net/publication/332912542_Vision-Based_Autonomous_Driving_A_Model_Learning_Approach).](media/2019_baheri_1.PNG "The `perception` module, the `memory` or `prediction` module, and the `control` module. [Source](https://www.researchgate.net/publication/332912542_Vision-Based_Autonomous_Driving_A_Model_Learning_Approach).")  |
|:--:|
| *The `perception` module, the `memory` or `prediction` module, and the `control` module. [Source](https://www.researchgate.net/publication/332912542_Vision-Based_Autonomous_Driving_A_Model_Learning_Approach).* |

- Some related concepts:
  - `VAE`, `stochastic policy search`, `CARLA`
- The idea is to first **learn a model** of the environment (the `transition function` of the `MDP`) and **subsequently derive a policy** based on it.
- Three modules are used:
  - 1- A `VAE` is trained to **encode** front camera views into an **abstract latent representation**.
  - 2- A `LSTM` is trained to **predict** the latent representation of the **one time-step ahead frame**, given the **action** taken and the current state representation. Based on this prediction (`mean` and `std`), a next state representation is sampled using the VAE.
  - 3- A `CMA-ES` is trained to **take actions** (`steering`, `acceleration`, and `brake`) based on the `LSTM` hidden state (capturing history information) and the current state representation (predicted). The problem is formulated as a `MDP`.
- One idea about the **continuous** action space:
  - > "We combine the acceleration and brake commands into a **single value** between `−1` to `+1`, where the values between `−1` and `0` correspond to the brake command and the values between `0` and `1` correspond to the acceleration command".
  - The authors use the term _"acceleration command"_ for one of the actions. CARLA works with `throttle`, as human use the gas-pedal.
  - I have realized that the mapping `acceleration` `->` `throttle` is very complex. Therefore I think the agent is leaning the `throttle` and considering the **single NN layer** used for the controller, this may be quite challenging.
- About the [`CMA-ES`](https://en.wikipedia.org/wiki/CMA-ES):
  - `ES` means "Evolution Strategy", i.e. an optimization technique based on ideas of evolution, iterating between of `variation` (via `recombination` and `mutation`) and `selection`.
    - `ES` is easy to **implement**, easy to **scale**, very fast if **parallelized** and extremely **simple**.
  - `CMA` means "Covariance Matrix Adaptation".
    - This means that in the `variation` phase, not only the `mean` but also the `covariance matrix` of the population is updated to increase the probability of previously successful steps.
    - Therefore, it can been seen as _Cross-Entropy Methods_ (`CEM`) with momentum.
- About **sampling efficiency**:
  - The authors note that `IL` and `model-free RL` baselines were taking resp. `14` hours and `12` days of driving for training and were both outperformed by the presented `model-based RL` approach which required `5` hours of human driving.
    - This only considers the **time to interact** with the environment, i.e. to record images.
    - It would be interesting to consider the time needed to **learn the policy** afterward.
  - `CMA-ES`, as a **derivative-free method**, is one of the least sample efficient approach.
    - I find interesting that an _evolutionary algorithm_ was chosen given the motivation of _increasing sampling efficiency_.
- About `model-based` RL:
  - The performance really depends on the **ability to learn a reliable model** of the environment.
    - The **low-level** representation of the `VAE` (size `128`) may not capture the most difficult situations.
    - The authors suggests looking at **mid-level** representations such as the [**affordance** representation](http://deepdriving.cs.princeton.edu/paper.pdf) of [DeepDriving](http://deepdriving.cs.princeton.edu/) instead.
  - Here, the authors **strictly split** the two tasks: First learn a model. Then do planning.
  - Why not ***keeping interacting from time to time with the `env`**, in order to vary the **sources of experience**?
    - This should still be more **sample efficient** than model-free approaches while making sure the agent keep seeing **"correct" transitions**.

---

</details>

Zhu, Y., & Zhao, D. [2019].
**"Vision ‑ based control in the open racing car simulator with deep and reinforcement learning"**
[[html](https://link.springer.com/article/10.1007/s12652-019-01503-y)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![ First __extract__ some variables - e.g. `curvature`, `desired speed`, `lateral offset`, `offset in heading` - from images using __supervised learning__ and then apply control learnt with __model-based `RL`__. [Source](https://link.springer.com/article/10.1007/s12652-019-01503-y).](media/2019_zhu_2.PNG "First __extract__ some variables - e.g. `curvature`, `desired speed`, `lateral offset`, `offset in heading` - from images using __supervised learning__ and then apply control learnt with __model-based `RL`__. [Source](https://link.springer.com/article/10.1007/s12652-019-01503-y).")  |
|:--:|
| *First __extract__ some variables - e.g. `curvature`, `desired speed`, `lateral offset`, `offset in heading` - from images using __supervised learning__ and then apply control learnt with __model-based `RL`__. [Source](https://link.springer.com/article/10.1007/s12652-019-01503-y).* |

| ![ The __model-based__ `PILCO` algorithm is used to quickly learn to predict the __desired speed__. [Source](https://link.springer.com/article/10.1007/s12652-019-01503-y).](media/2019_zhu_1.PNG "The __model-based__ `PILCO` algorithm is used to quickly learn to predict the __desired speed__. [Source](https://link.springer.com/article/10.1007/s12652-019-01503-y).")  |
|:--:|
| *The __model-based__ `PILCO` algorithm is used to quickly learn to predict the __desired speed__. [Source](https://link.springer.com/article/10.1007/s12652-019-01503-y).* |

- Some related concepts:
  - `PILCO`, `TORCS`
- Definitions:
  - **State** variables: `x` = [`lateral deviation`, `angle deviation`, `desired speed`].
  - **Dynamical** variables: `y` = [`x`, `curvature`].
  - **Cost** variables `z` = [`y`, `current speed`].
  - **Control** variables: `u` = [`steering`, `throttle or brake`].
  - The variable `current speed` is always known: either given by `TORCS` or read from `CAN bus`.
- One idea: contrary to `E2E`, the authors want to separate `perception` and `control`. Hence the training is divided into **two steps**:
  - 1- Extract _dynamical variables_ `y` from the simulator (assume _full observation_) and **learn a driving controller**. -> Using **model-based RL**.
  - 2- Try to extract `y` from images. -> Using **supervised learning**.
  - This step-by-step method brings advantages such as the possibility for **intermediate checks** and **uncertainty propagation**.
    - But both learning processes are **isolated**. And one defective block can cause the whole chain to fail.
    - In particular, the authors note that the `CNN` fails at predicting _`0`-lateral-offset_, i.e. when the car is **close to the centre**, causing the full system to **_"vibrate"_**.
    - This could be addressed on the **controller side** (_damping factor_ or adding _action consistency_ in the cost function), but it would be better to back-propagate these errors directly to the perception, as in _pixel-to-control_ approaches.
- _What is learnt by the controller?_
  - One option would be to learn the **transition function** leading to the new state: **`x[t+1]`** = `f`(`y`, `u`, `x`). This is what the simulator applies internally.
  - Instead, here, the _distribution_ of the **_change_** in **state** is learnt: **`delta`(`x`)** = `x[t+1]` - `x[t]` = `f`(`y`, `u`, `x`).
  - **Data is collected through interactions** and used to optimized the parameters of the controller:
    - Training **inputs** are formed by some recorded `Y` = [`y`, `u`].
    - Training **targets** are built with some recorded `ΔX` = [`delta`(`x`)].
- Another idea: the car is expected to **run at different velocities**.
  - Hence vary the **desired speed** depending on the **curvature**, the current velocity and the deviation in `heading`.
  - This is what the agent must learn to predict.
  - In the reward function of `PILCO`, the term about desired velocity play the **largest role** (_if you do not learn to decelerate before a turn, your experiences will always be limited since you will get off-road at each sharp turn_).
- One algorithm: `PILCO` = **`P`robabilistic `I`nference for `L`earning `CO`ntrol**.
  - In short, this is a **model-based** `RL` algorithm where the _system dynamics_ is modelled using a **Gaussian process** (`GP`).
  - The `GP` **predicts outcome distribution** of `delta`(`x`) with probabilities. _Hence first letter `P`_.
    - In particular, the job is to predict the **mean** and the **standard deviation** of this distribution which is **assumed to be Gaussian**.
    - This _probabilistic_ nature is important since **model-based** `RL` usually suffers from **_model bias_**.
  - The _cost variables_ are also predicted and based on this `z` distribution, the optimal control `u` is derived using **_policy gradient search_** (`PGS`).
    - More precisely, the _control variables_ `u` is assumed to be function of the expected cost `z` via an affine transformation followed by some saturation: `u` = `sat`(`w`*`z` + `b`).
    - Hence `PGS` aims at finding {`w`, `b`}: the predicted return and its derivatives are used to **optimize the controller parameters**.
    - The new controller is again used in `TORCS` to **generate data** and the learning process is repeated.
- _Why and how is the vanilla `PILCO` modified?_
  - The computational complexity of `PILCO` is linear in the **size of training set**.
    - Instead of using _sparse_ GP method (e.g. `FITC`), the authors decide to **prune the dataset** instead.
    - In particular, observation data are **sparsely collected** and **renewed at each iteration**.
  - Other modifications relate to the difference between input output/variable types. And the use of **different scenarios** to calculate the expected returns.
- One quote about the difference between `PILCO` and `MPC`:
  - > "The concept of PILCO is quite similar to explicit MPC algorithms, but MPC controllers are usually defined **piecewise affine**. For PILCO, control law can be represented in **any differentiable form**."

---

</details>

## Planning and Monte Carlo Tree Seach

Hoel, C.-J. [2019].
**"Tactical decision-making for autonomous driving: A reinforcement learning approach"**
[[pdf](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf)]

<details>
  <summary>Click to expand</summary>

Some figures:

| ![ `MCTS` is especially benefitial when it is necessary to plan relatively __far into the future__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).](media/2019_hoel_3.PNG "`MCTS` is especially benefitial when it is necessary to plan relatively __far into the future__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).")  |
|:--:|
| *`MCTS` is especially benefitial when it is necessary to plan relatively __far into the future__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).* |

| ![ The `RL`-learnt neural network predicts two values used to __guide the search__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).](media/2019_hoel_2.PNG "The `RL`-learnt neural network predicts two values used to __guide the search__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).")  |
|:--:|
| *The `RL`-learnt neural network predicts two values used to __guide the search__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).* |

| ![ Treate surrounding vehicles as __interchangeable objects__ using __`CNN`__ layers. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).](media/2019_hoel_1.PNG "Treate surrounding vehicles as __interchangeable objects__ using __`CNN`__ layers. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).")  |
|:--:|
| *Treate surrounding vehicles as __interchangeable objects__ using __`CNN`__ layers. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).* |

| ![ Comparison of __sampling efficiency__ - need for `domain knowledge` and `computational speed` should also be considered. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).](media/2019_hoel_4.PNG "Comparison of __sampling efficiency__ - need for `domain knowledge` and `computational speed` should also be considered. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).")  |
|:--:|
| *Comparison of __sampling efficiency__ - need for `domain knowledge` and `computational speed` should also be considered. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).* |

- Some related concepts:
  - `POMDP`, `MCTS`
- Three related works corresponding to three proposed approaches (all `RL`-related):
  - 1- **Genetic algorithm** (_policy based RL_) to train a **rule-based** driver model (each chromosome encodes a rule-based **driver model** via a set of instructions).
    - ["An evolutionary approach to general-purpose automated speed and lane change behavior"](https://arxiv.org/abs/1803.10056v2) - (Hoel et al. 2018).
  - 2- **DQN**.
    - ["Automated Speed and Lane Change Decision Making using Deep Reinforcement Learning"](https://arxiv.org/abs/1803.10056) - (Hoel et al. 2018).
  - 3- Combining **planning** (**`MCTS`**) and **learning** (**`model-free RL`**).
    - The neural network predicts two values used to **guide the search**:
      - The **value function** `V`(`state`).
      - The **policy** `p`(`state`, `action`) for each possible `action`.
    - This time, a **transition function** (or **_generative model_**) `G`(`s`, `a`) is considered and used during the **`SIMULATION`** phase of the search.
    - ["Combining Planning and Deep Reinforcement Learning in Tactical Decision Making for Autonomous Driving"](https://arxiv.org/abs/1905.02680) - (Hoel et al. 2019).
      - I have analysed this paper in [this section](https://github.com/chauvinSimon/IV19#combining-learning-and-planning) of my summary of `IV19`.
- One remark about the `POMDP` formulation:
  - Only the **physical parts** of the state (`position` and `speed`) are **observed**.
  - The **parameters** of the surrounding drivers, which are assumed to behave according to the `IDM`/`MOBIL` models, is not directly accessible by the ego-agent.
  - A **particle filter** is used to estimate them (**_belief state estimation_**).
- One idea: Treate surrounding vehicles as **interchangeable objects** using **`CNN`** layers.
  - Using CNN layers with `max-pooling` creates a **translational invariance** between the vehicles.
  - > "The output is independent on the ordering of the vehicles in the input vector, and it also removes the problem of specifying a fixed input vector size, which instead can be made larger than necessary and padded with dummy values for the extra slots"
- About **_"sampling efficiency_"**, **_"domain knowledge_"** and trade-off of `speed` vs. `generality`:
  - The **`GA`** agent requires much **domain knowledge** in the form of **handcrafted features** (form of the instructions).
  - The `DQN` agent requires between `2` and `3` **orders of magnitude** **less driving time** than the `GA` agent.
  - > "The `MCTS`/`NN` agent requires the **most domain knowledge**, since it needs a **generative model `G`** of the environment, a **belief state estimator**, and possibly knowledge on how to prune actions that lead to collisions."
- Results:
  - The baseline is a **rule-based** approach built with **`IDM`** and **`MOBIL`** driver models (also used in the _generative model_ and to _simulate other vehicles_).
  - > "All methods outperform the baseline `IDM`/`MOBIL` model by taking decisions that allows the vehicle to navigate through traffic between `5%` and `10%` faster."
  - `MCTS` is especially benefitial when it is necessary to **plan relatively far into the future** (e.g. _highway exit case_).

---

</details>

Lee, J., Balakrishnan, A., Gaurav, A., & Feb, L. G. [2019].
**"WiseMove: A Framework for Safe Deep Reinforcement Learning for Autonomous Driving"**
[[pdf](https://arxiv.org/abs/1902.04118)]
[[code](https://git.uwaterloo.ca/wise-lab/wise-move)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://arxiv.org/abs/1902.04118).](media/2019_lee.PNG "[Source](https://arxiv.org/abs/1902.04118).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1902.04118).* |

- Some related concepts:
  - `MCTS`, `options framework`, `LTL`, `hierarchical decision making`, `POMDP`
- One related work: The presented approach reminds me the work of [Paxton, C., Raman, V., Hager, G. D., & Kobilarov, M.](https://arxiv.org/abs/1703.07887).
- One term: **"WiseMove"**: the presented **options**-based **modular** safe DRL framework.
  - The _modular_, or _hierarchical_, aspect comes from the [**option framework**](http://www-anw.cs.umass.edu/~barto/courses/cs687/Sutton-Precup-Singh-AIJ99.pdf). Sometimes called **macro-actions**.
  - For more on **Hierarchical RL**, check out [this `thegradient.pub` post](https://thegradient.pub/the-promise-of-hierarchical-reinforcement-learning/).
  - The idea is to decompose the decision by working with **temporal abstracted actions** (e.g. _slow down_, _turn left_) on a high-level (like a **behaviour planner**).
  - Each of these so called _options_ rely on **low-level primitive policies** that implement their manoeuvres (similar to a **geometrical trajectory optimizer**).
- One idea: **LTL** formalism is used to check the validity of high-level decisions.
  - An option is defined by (1) a **underlying primitive policy**, but also by (2) a **initial condition** and (3) a **terminal condition**.
  - For instance, the option `take-over` is available only if a vehicle is on my lane and a second lane exists. The manoeuvre is finished when I arrive in front of the other vehicle.
  - _I like to think of it as another sort of_ **_masking mechanism_**.
  - Here, these conditions are expressed as **hand-crafted rules** in an **LTL**-like syntax.
- One remark: I think we are currently missing open-source simulators that offers OpenAI `gym`-like APIs for training and testing RL approaches for decision making.
  - Several interfaces to `SUMO` have been developed.
  - For instance [@LucasAlegre](https://github.com/LucasAlegre/sumo-rl), [@bstriner](https://github.com/bstriner/gym-traffic/), [@SaloniDash7](https://github.com/SaloniDash7/gym-sumo), [@sycdlcrain](https://github.com/sycdlcrain/gym_sumo) or [FLOW](https://flow-project.github.io/) which looks promising since it keeps being developed.
  - Here, the author of `WiseMove` release an `env` python module (together with `verifier`, `options` and `backends`) that should fulfil this function.
- Another remark: Combining learning [RL] and planning [(MC) tree search] is an idea [I find](https://github.com/chauvinSimon/IV19#combining-learning-and-planning) very promising.
  - Here, the **safest next option** is selected based on the stochastic look-aheads performed by the MCTS (_safety check_).
  - In return, the options effectively **reduce the number of decisions** needed to reach any depth in the tree (_sampling efficiency_).

---

</details>

Augustin, D., Schucker, J., Tschirner, J., Hofmann, M., & Konigorski, L. [2019].
**"A Simulation-Based Reinforcement Learning Approach for Long-Term Maneuver Planning in Highway Traffic Scenarios"**
[[pdf](https://www.researchgate.net/publication/334745733_A_Simulation-Based_Reinforcement_Learning_Approach_for_Long-Term_Maneuver_Planning_in_Highway_Traffic_Scenarios)]

<details>
  <summary>Click to expand</summary>

- One diagram is better than 100 words:

| ![The term `action` comprises a lateral maneuver decision and a set speed request. [Source](https://www.researchgate.net/publication/334745733_A_Simulation-Based_Reinforcement_Learning_Approach_for_Long-Term_Maneuver_Planning_in_Highway_Traffic_Scenarios).](media/2019_augustin.PNG "The term `action` comprises a lateral maneuver decision and a set speed request. [Source](https://www.researchgate.net/publication/334745733_A_Simulation-Based_Reinforcement_Learning_Approach_for_Long-Term_Maneuver_Planning_in_Highway_Traffic_Scenarios).")  |
|:--:|
| *The term `action` comprises a lateral maneuver decision and a set speed request. [Source](https://www.researchgate.net/publication/334745733_A_Simulation-Based_Reinforcement_Learning_Approach_for_Long-Term_Maneuver_Planning_in_Highway_Traffic_Scenarios).* |

- Some related concepts:
  - `combining learning/planning`, `hierarchical/modular decision making`, `POMDP`, `SUMO`
- One remark: I like the **hierarchy** and **modularity** of the approach.
  - Especially the fact that the `action` stays high-level (`speed desire` and `high-level manoeuvre`), as opposed to `steering angle` and `throttle` commands that are often used in RL.
- One promising tool: [FLOW](https://flow-project.github.io/)
  - `FLOW` is a Python library that interfaces the RL libraries [RLlib](https://ray.readthedocs.io/en/latest/rllib.html) and [rllab](https://github.com/rll/rllab) with `SUMO`. It has been developed and is supported by _UC Berkeley_.
  - It has not been used many times (_because of the lack of Windows support?_). Instead, many research using `SUMO` develop their own interface, which makes comparison and reproduction difficult.
  - A few recent `FLOW`-based works can be mentionned though:
    - "Simulation to scaled city: zero-shot policy transfer for traffic control via autonomous vehicles" by [(Jang et al., 2018)](https://arxiv.org/abs/1812.06120)
    - "Benchmarks for reinforcement learning in mixed-autonomy traffic" by [(Vinitsky et al., 2018)](https://arxiv.org/abs/1812.06120)

---

</details>

Sierra Gonzalez, D. [2019].
**"Towards Human-Like Prediction and Decision-Making for Automated Vehicles in Highway Scenarios"**
[[pdf](https://tel.archives-ouvertes.fr/tel-02184362/document)]
[[video - simulator](https://www.youtube.com/watch?v=Xx5OmV86CsM)]
[[code - simulator](https://github.com/marioney/hybrid_simulation/tree/decision-making)]

<details>
  <summary>Click to expand</summary>

Note: the planning part of this thesis takes advantage of the **prediction approaches** and the **driver models** referenced in previous sections.

- In particular, the derived predictive model is used for both the **belief update** (instead of often-used _particle filters_ and _IMM filters_) and as a **generative model** (for forward simulations) of the POMDP.
- The **value estimations** in the tree search are based on the learnt driver model and the long-term prediction methods previously referenced.

Some figures:

| ![Comparison of recent POMDP-based planning modules. [Source](https://tel.archives-ouvertes.fr/tel-02184362/document).](media/2019_sierra_gonzalez_1.PNG "Comparison of recent POMDP-based planning modules. [Source](https://tel.archives-ouvertes.fr/tel-02184362/document).")  |
|:--:|
| *Comparison of recent POMDP-based planning modules. [Source](https://tel.archives-ouvertes.fr/tel-02184362/document).* |

| ![Construction of the tree search with belief updates and model-based rollouts. [Source](https://tel.archives-ouvertes.fr/tel-02184362/document).](media/2019_sierra_gonzalez_2.PNG "Construction of the tree search with belief updates and model-based rollouts. [Source](https://tel.archives-ouvertes.fr/tel-02184362/document).")  |
|:--:|
| *Construction of the tree search with belief updates and model-based rollouts. [Source](https://tel.archives-ouvertes.fr/tel-02184362/document).* |

| ![[Source](https://tel.archives-ouvertes.fr/tel-02184362/document).](media/2019_sierra_gonzalez_4.PNG "[Source](https://tel.archives-ouvertes.fr/tel-02184362/document).")  |
|:--:|
| *[Source](https://tel.archives-ouvertes.fr/tel-02184362/document).* |

- Some related concepts:
  - `MCTS`, `online POMDP`, `POMCP`, `progressive widening`, `SUMO`

- The author targets some **_"human-like tactical planning"_**.
  - The **POMDP** formulation is ideal since it considers uncertainty in the `controls`, `states`, and the `intentions` of the traffic participants.
  - The idea is to include **estimation of intentions** for **long-term anticipation**.

- One idea: about the **rollout policy** used for the construction of the search tree.
  - One option is to use a **random** rollout policy.
  - Here, the previously-derived models are used to **predict approximately the long-term development** of traffic scenes.

- Another idea: adapt the combination of **_model-based_** and **_manoeuvre-estimation-based_** predictions, depending on **how far** the **rollout looks into the future**.

> "As we go **deeper into the history tree** (that is, into the future), the **observed dynamics** of the targets at the root node become **less relevant** and so we **rely increasingly** in the model to **predict the behavior** of the obstacles."

- Other related works:
  - The **Hierarchical architecture** of [(Sonu, E., Sunberg, Z., & Kochenderfer, M. J. (2018). _"Exploiting Hierarchy for Scalable Decision Making in Autonomous Driving"_)](https://www.researchgate.net/publication/328455111_Exploiting_Hierarchy_for_Scalable_Decision_Making_in_Autonomous_Driving).
  - The **`Double Progressive Widening (DPW)`** or `progressive unpruning` of [(Sunberg & Kochenderfer, 2017)](https://arxiv.org/abs/1709.06196) to deal with continuous observations.
  - The general approach of [(Bouton, M., Cosgun, A., & Kochenderfer, M. J. (2017). _"Belief state planning for autonomously navigating urban intersections"_)](https://arxiv.org/abs/1704.04322). The main difference is the substitution of the **Interacting Multiple Model** (`IMM`) with **DBN**-based model to **consider the interactions** between vehicles.

- One quote about the _(relative)_ benefits of POMDP formulations:

> "There have not been significative differences between the decisions taken by the **proposed POMDP planner** and the **reactive SUMO model**. This is due to the fact that neither of those scenes truly required to analyze the **long-term consequences** of a maneuver".

---

</details>

Weingertner, P., Autef, A., & Le Cleac’h, S. [2018].
**"Decision Making Under Uncertainty for Urban Driving"**
[[pdf](https://web.stanford.edu/class/aa228/reports/2018/final100.pdf)]
[[code](https://github.com/PhilippeW83440/ACT)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![Comparing the vanilla POMCP and proposed safe variant of it. [Source](https://web.stanford.edu/class/aa228/reports/2018/final100.pdf).](media/2018_weingertner.PNG "Comparing the vanilla POMCP and proposed safe variant of it. [Source](https://web.stanford.edu/class/aa228/reports/2018/final100.pdf).")  |
|:--:|
| *Comparing the vanilla POMCP and proposed safe variant of it. [Source](https://web.stanford.edu/class/aa228/reports/2018/final100.pdf).* |

- Some related concepts:
  - `POMDP`, `MCTS`, `julia`, `probabilistic risk assessment`, `value iteration`
- One algorithm: [`POMCP`](https://papers.nips.cc/paper/4031-monte-carlo-planning-in-large-pomdps).
  - Presented in 2010, `POMCP` is an extension of the traditional **MCTS algorithm to POMDP**.
  - Together with [`DESPOT`], `POMCP` is a often-used POMDP online solver.
- One term: **"observation class"**.
  - Different extensions of `POMCP` and `DESPOT` have been proposed. In the presented approach, the goal is to work with **continuous observations**, while ensuriung safety.
  - The idea is to **limit the number of observation nodes** in the tree by **grouping observations** based on some **utility function**.
  - This utility function should not to be confused with the **offline-learn _value function_** representing the probability of collision.
  - The safe **clusterization of observations** can be based on _smallest `TTC`_ or _smallest distance_ to other participants.
- One idea: **guide the online graph search** using a **offline methods** to improve safety.
  - This is based on the work of [(Bouton, Karlsson, et al., 2019)](https://arxiv.org/abs/1904.07189), where **offline `VI`** (value iteration) is used to compute `P_collision`(`s`, `a`).
  - This **safety criterion** is then used to limit the **set of safe available actions**.
  - In the presented work, the author reason over the `belief` instead of the `state`.
- Another idea: Use a **Kalman Filter** (instead of some particle filters) for **belief updater**.
- One quote:

> "Using an online method based on sparse sampling may lead to safety issues. Rare events with critical consequences may not be sampled leading to sub-optimal and potentially dangerous decisions."

- One promising tool: [POMDPs.jl](https://github.com/JuliaPOMDP/POMDPs.jl)
  - `POMDPs.jl` is an interface for defining, solving, and simulating `MDPs` and `POMDPs` on discrete and continuous spaces. It has been developed and is supported by a team from _Stanford_.
- Two ideas for future works:
  - In their [repo](https://github.com/PhilippeW83440/ACT), the authors suggest **combining learning** (e.g. model-free RL used as an heuristic and/or for rollout) with **planning** (MCTS), mentioning the success of AlphaGo Zero.
  - Another improvment concerns the **transition model** for the observed vehicles. Instead of `CV` (constant velocity) models, one could assume surrounding vehicles are **following a driver model** (e.g. `IDM`) and the task would be to **infer its hidden parameters**.

---

</details>

Vodopivec, T., Samothrakis, S., & Ster, B. [2017].
**"On Monte Carlo Tree Search and Reinforcement Learning"**
[[pdf](https://pdfs.semanticscholar.org/3d78/317f8aaccaeb7851507f5256fdbc5d7a6b91.pdf)]

<details>
  <summary>Click to expand</summary>

One figure:

| ![Four parameters introduced in a TD-Tree Search (`TDTS`) algorithm related to `forgetting`, `first-visit updating`, `discounting` and `initial bias`. [Source](https://pdfs.semanticscholar.org/3d78/317f8aaccaeb7851507f5256fdbc5d7a6b91.pdf).](media/2017_vodopivec_1.PNG "Four parameters introduced in a TD-Tree Search (`TDTS`) algorithm related to `forgetting`, `first-visit updating`, `discounting` and `initial bias`. [Source](https://pdfs.semanticscholar.org/3d78/317f8aaccaeb7851507f5256fdbc5d7a6b91.pdf).")  |
|:--:|
| *Four parameters introduced in a TD-Tree Search (`TDTS`) algorithm related to `forgetting`, `first-visit updating`, `discounting` and `initial bias`. [Source](https://pdfs.semanticscholar.org/3d78/317f8aaccaeb7851507f5256fdbc5d7a6b91.pdf).* |

- Some related concepts:
  - `RL`, `MCTS`, `learning`, `planning`
- Goal: The authors aim at promoting an _"unified view of `learning`, `planning`, and `search`"_.
  - First, the difference `planning` / `learning` is discussed.
    - It depends on the **source of experience**: _simulated_ / _real_ **interaction**.
  - Then, **sample-based** (RL and MCTS) **search** algorithms can all be described as a combination of:
    - 1- A **learning** algorithm, _= how the estimates get_ **_updated_** _from gathered_ **_experience_**.
    - 2- A **control** policy, _= how actions get_ **_selected_**.
    - 3- And a **representation policy** _= how is the underlying representation model adapted_.

- Some `RL`/`MCTS` similarities:
  - They are somehow connected to **MDP** formulations.
  - Both cope with the **_exploration-exploitation_** dilemma.
  - Both are **value-based** and share the concepts of:
    - **_Policy evaluation_**: `backpropagation` phase
    - **_Policy improvement_**: `selection` phase.
  - Both exhibit **anytime** behaviours.

- Two major `RL`/`MCTS` differences:
  - RL methods do not recognize the **playout phase**, i.e. a **separate policy for non-memorized** (i.e., non-represented) parts of space.
    - In MCTS it is often called the **_"default policy"_**.
    - It would be beneficial to have a **more complex** default policy (where **expert knowledge** could be included).
  - They present two different **memorization** approaches and two different **approximation accuracies**:
    - `RL` methods based on value function approximation (e.g. NN) can **weakly describe the whole state space**.
      - The `RL` theory should **acknowledge a non-represented** (i.e., non-memorized) part of the state space, i.e. the part that is **not described (estimated) by the representation model** at a given moment.
    - `MCTS` algorithms initially **approximate only a part of the state space** (with high accuracy).
      - Therefore `MCTS` maintains the distinction between a **_memorized_** and **_non-memorized_** part of the **state space**.
      - The state-space representation is changed online: it is a **_"adaptive (incremental) representation method"_**.
      - Indeed **“incomplete” representations** can be beneficial: it might be better to accurately approximate the **relevant part** of the space and less accurately (or not at all) the remaining part.

- Contribution: Based on this comparison, the authors introduce a framework called **_"temporal-difference tree search"_** (`TDTS`) which aims at **combining the advantages** of both RL and MCTS methods.
  - How it extends classic MCTS methods:
    - Replace MC backups with **bootstrapping backups**.
      - This is the idea of `TD search`: do not wait for the end of the trajectory to backup but instead update state value estimates **based on previous estimates**.
        - `TD` errors are decayed (`γ`) and accumulated (`λ`). It boils down to standard `UCT` if `λ` `=` `γ` `=` `1`.
      - The motivation is to **reduce the variance of the estimates**, I guess similarly to the introduction of a `TD`-based Critic in **Actor-Critic** methods.
      - This is done using **_"Eligibility Traces"_** (_"traces"_ because it tracks which states were previously visited and gives them _credit_, i.e. _"eligibility"_), as in `n`-step SARSA.
        - The authors note that the eligibility trace **decay rate** `λ` can be hard to tune.
  - How it extends classic RL methods:
    - Contrary to `TD search` methods, `TDTS` uses:
      - Some `playout` and `expansion` phases.
        - It has some representation policy for **incremental or adaptive** representation of the **non-memorized part** of the state space.
      - A **`playout` policy** and **`playout` values** (as opposed to the already existing `control` policy).
        - The idea is to **replace the missing value estimates** in the **non-memorized part** of the state space with **_"playout values"_**.
        - These values can be regarded as a **placeholder** (entry point) **for expert knowledge**.
  - All in all: recreate the four MCTS iteration phases:
    - (1) Selection – control in the **_memorized part_** of the search space.
    - (2) Expansion – changing the **representation** (_adaptive incremental_ model).
    - (3) Playout – control in the **_non-memorized_** part of the search space.
    - (4) Backpropagation – updating the value estimates with **bootstrap**.
  - `TDTS` is applied to `UCT` (`UCB` selection policy in Tree search), leading to `Sarsa`-`UCT`(`λ`).

- One takeaway: as identifies in previous summaries, one idea (e.g. in **_AlphaGo_**) is to **combine:**
  - **Prior knowledge** (value estimates from the RL-pre-trained network),
  - And **online feedback** (`MC` evaluations based on _playouts_).

- Two terms I learnt:
  - **_"Transposition tables"_**:
    - As I understand, it plays the role of **generalisation** in function approximation: _two similar states should have similar values_.
    - It originates from **search** of the **game tree**, where it is possible to reach a given position in more than one way. These are called `transpositions`.
    - The **transposition table** is a **kind of cache**: on encountering a new position, the program checks the table to see whether the **state has already been analyzed**. If yes, the value (stored) can be used instead of calculating it (which would require expending a subtree).
  - **_"afterstates"_**:
    - I understand it as a third metric to quantify the "value":
      - V(`s`): **state value** of **being** in state `s` and following policy `π`.
      - V(`s'`): **afterstate value** of **arriving** at state `s'` and thereafter following policy `π`.
      - Q(`s`, `a`): **action value** of **being** in state `s`, taking action `a` and thereafter following policy `π`.
    - This is used in the presented Sarsa-UCT(λ) algorithm.

</details>
