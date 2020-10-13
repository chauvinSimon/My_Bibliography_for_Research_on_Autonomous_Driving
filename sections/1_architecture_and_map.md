# `Architecture` and `Map`

---

**`"BARK : Open Behavior Benchmarking in Multi-Agent Environments"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.02604)]**
**[[:octocat:](https://bark-simulator.github.io/)]**
**[[:octocat:](https://github.com/bark-simulator/bark/)]**
**[** :mortar_board: `Technische Universität München` **]**
**[** :car: `Fortiss`, `AID` **]**

- **[** _`behavioural models`, `robustness`, `open-loop simulation`, `behavioural simulation`, `interactive human behaviors`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.02604).](../media/2020_bernhard_1.PNG "[Source](https://arxiv.org/abs/2003.02604).")  |
|:--:|
| *The **`ObservedWorld` model**, reflects the `world` that is perceived by an agent. **Occlusions** and **sensor noise** can be introduced in it. The **`simultaneous movement`** makes **simulator planning cycles** entirely **deterministic**. [Source](https://arxiv.org/abs/2003.02604).* |

| ![[Source](https://arxiv.org/abs/2003.02604).](../media/2020_bernhard_2.PNG "[Source](https://arxiv.org/abs/2003.02604).")  |
|:--:|
| *Two evaluations. Left: **Robustness** of the `planning` model against the `transition function`. The **scenario's density** is increased by reducing the **`time headway`** `IDM` parameters of interacting vehicles. **Inaccurate prediction model** impacts the performance of an `MCTS` (`2k`, `4k`, and `8k` search iterations) and `RL`-based (`SAC`) planner. Right: an **agent from the dataset is replaced** with various agent behaviour models. Four different parameter sets for the `IDM`. Agent sets `A0`, `A1`, `A2`, `A6` are not replaced with the `IDM` since this model **cannot change lane**. **Maintaining a specific order** is key for `merging`, but without fine-tuning model parameters, **most behaviour models fail to coexist next to replayed agents**. [Source](https://arxiv.org/abs/2003.02604).* |

Authors: Bernhard, J., Esterle, K., Hart, P., & Kessler, T.

- **`BARK`** is an acronym for **Behaviour BenchmARK** and is open-source under the `MIT` license.

- Motivations:
  - `1-` Focus on **driving behaviour models** for `planning`, `prediction`, and `simulation`.
    - > "`BARK` offers a **behavior model-centric** simulation framework that enables **fast-prototyping** and the **development** of behavior models. Behavior models can easily be integrated — either using `Python` or `C++`. Various behavior models are available ranging from machine learning to conventional approaches."
  - `2-` Benchmark **interactive** behaviours.
    - > "To model **interactivity**, planners must employ some kind of **`prediction` model** of other agents."

- Why existing **simulation frameworks** are limiting?
  - > "Most simulations rely on **datasets** and **simplistic behavior models** for traffic participants and do not cover the **full variety of real-world, interactive human behaviors**. However, existing frameworks for **simulating** and **benchmarking** behavior models rarely provide **sophisticated behavior models** for other agents."
  - [`CommonRoad`](https://commonroad.in.tum.de/): **only pre-recorded data** are used for the other agents, i.e. only enabling **non-interactive** behaviour planning.
  - [`CARLA`](http://carla.org/): A [CARLA-BARK interface](https://github.com/bark-simulator/carla-interface) is available.
    - > "Being based on the `Unreal Game Engine`, problems like **non-determinism** and **timing issues** are introduced, that we consider undesirable when developing and comparing behavior models."
  - [`SUMO`](https://sumo.dlr.de/docs/index.html): **Microscopic traffic simulators** can model **flow** but neglect **interactions** with other vehicles and does not track the **accurate motion** of each agent.

- Concept of **`simultaneous movement`**.
  - Motivation: Make **simulator planning cycles** entirely **deterministic**. This enables the simulation and experiments to be reproducible.
  - > "`BARK` models the world as a **multi-agent system** with agents performing **simultaneous movements** in the simulated world."
  - > "At **fixed, discrete world time-steps**, each agent plans using an agent-specific behavior model in a **cloned world** – the **_agent’s observed world_**."
  - Hence the other agents can **actively interact** with the ego vehicle.
- Implemented **behaviour models**:
  - `IDM` + `MOBIL`.
  - `RL` (`SAC`).
    - > "The **reward `r`** is calculated using `Evaluators`. These modules are available in **our [Machine Learning module](https://github.com/bark-simulator/bark-ml/)**. As it integrates the standard `OpenAi Gym`-interface, various popular `RL` libraries, such as [`TF-Agents`](https://github.com/tensorflow/agents) can be easily integrated used with `BARK`."
  - `MCTS`. Single-agent or multi-agent.
    - > [multi-agent] "Adapted to **interactive driving** by using `information sets` assuming **simultaneous, multi-agent movements** of traffic participants. They apply it to the context of **cooperative planning**, meaning that they introduce a **cooperative cost function**, which **minimizes the costs for all agents**."
  - **Dataset Tracking Model**.
    - The agent model tracks **recorded trajectories** as close as possible.

- Two **evaluations** (benchmark) of the behavioural models.
  - > "`Prediction` (a **discriminative** task) deals with **what will happen**, whereas `simulation` (often a **generative** task) deals with **what could happen**. Put another way, `prediction` is a tool for **forecasting the development of a given situation**, whereas `simulation` is a tool for **exploring a wide range of potential situations**, often with the goal of probing the robot’s planning and control stack for weaknesses that can be addressed by system developers." [`(Brown, Driggs-Campbell, & Kochenderfer, 2020)`](https://arxiv.org/abs/2006.08832).
  - `1-` Behaviour **`prediction`**:
    - _What is the effect of an inaccurate `prediction model` on the performance of an `MCTS` and `RL`-based `planner`?_
    - `MCTS` requires an **explicit generative model** for each `transition`. This prediction model used internally is evaluated here.
    - > [**Robustness** also tested for `RL`] "`RL` can be considered as an **offline planning** algorithm – not relying on a **prediction model** but requiring a **training environment** to learn an optimal policy **beforehand**. The **inaccuracy of prediction** relates to the amount of behavior model inaccuracy between `training` and `evaluation`."
  - `2-` Behaviour **`simulation`**.
    - _How planners perform when_ **_replacing human drivers_** _in recorded traffic scenarios?_
    - Motivation: combine **`data-driven` (recorded -> fixed trajectories)** and **`interactive`** _(longitudinally controlled)_ scenarios.
    - > "A planner is inserted into **recorded scenarios**. Others keep the behavior as specified in the dataset, yielding an **open-loop** simulation."
    - The [`INTERACTION Dataset​`](http://interaction-dataset.com/) is used since it **provides maps**, which are essential for most on-road planning approaches.
- Results and future works.
  - > [`RL`] "When the **other agent’s behavior** is different from that used in `training`, the **collision rate** rises more quickly."
  - > "We conclude that current **rule-based models** (`IDM`, `MOBIL`) perform poorly in **highly dense, interactive scenarios**, as they **do not model obstacle avoidance** based on `prediction` or future `interaction`. `MCTS` can be used, but without an **accurate model of the prediction**, it also leads to crashes."
  - > "A combination of **classical** and **learning-based** methods is computationally fast and achieves safe and comfortable motions."
  - The authors find **imitation learning** also promising.

</details>

---

**`"LGSVL Simulator: A High Fidelity Simulator for Autonomous Driving"`**

- **[** `2020` **]**
**[[:memo:](https://zelenkovsky.com/assets/files/LGSVL_Simulator_ITSC_2020.pdf)]**
**[[:memo:](https://www.lgsvlsimulator.com/)]**
**[[:octocat:](https://github.com/lgsvl/simulator)]**
**[[🎞️](https://www.youtube.com/channel/UChrPZIYAnKEKiQjmPmBwPKA)]**
**[** :car: `LG` **]**

- **[** _`simulator`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://zelenkovsky.com/assets/files/LGSVL_Simulator_ITSC_2020.pdf).](../media/2020_boise_1.PNG "[Source](https://zelenkovsky.com/assets/files/LGSVL_Simulator_ITSC_2020.pdf).")  |
|:--:|
| *A **bridge** is selected based on the user **AD stack’s runtime framework**: `Autoware.AI` and `Autoware.Auto`, which run on `ROS` and `ROS2`, can connect through **standard open source `ROS` and `ROS2` bridges**, while for `Baidu’s Apollo` platform, which uses a custom runtime framework called [`Cyber RT`](https://github.com/ApolloAuto/apollo/tree/master/cyber), a custom bridge is provided to the simulator. [Source](https://zelenkovsky.com/assets/files/LGSVL_Simulator_ITSC_2020.pdf).* |

Authors: Boise, E., Uhm, G., Gerow, M., Mehta, S., Agafonov, E., Kim, T. H., … Kim, S.

- Motivations (_Yet another simulator?_):
  - > "The LGSVL Simulator is a simulator that facilitates testing and development of autonomous driving software systems."
  - The main use case seems to be the **integration** to **AD stacks**: `Autoware.AI`, `Autoware.Auto`, `Apollo 5.0`, `Apollo 3.0`.
  - Compared to [`CARLA`](http://carla.org/) for instance, it seems more **focused** on _development_ rather than _research_.
- The simulation engine serves three functions:
  - **Environment** simulation
  - **Sensor** simulation
  - **Vehicle dynamics and control** simulation.
- Miscellaneous:
  - `LGSVL` = _LG Silicon Valley Lab_.
  - Based on `Unity` engine.
  - A `openAI-gym` environment is provided for **reinforcement learning**: [`gym-lgsvl`]().
    - Default `action` space: `steering` and `braking`/`throttle`.
    - Default `observation` space: single camera image from the front camera. _Can be enriched_.
  - For **perception** training, [`kitti_parser.py`](https://github.com/lgsvl/PythonAPI/blob/master/examples/kitti_parser.py) enables to **generate labelled data** in `KITTI` format.
  - A **custom [License](https://raw.githubusercontent.com/lgsvl/simulator/master/LICENSE)** is defined.
    - > "You may not **sell or otherwise transfer** or make available the Licensed Material, any copies of the Licensed Material, or any information derived from the Licensed Material in any form to any third parties for **commercial purposes**."
    - It makes it **hard to compare** to other simulators and AD software: for instance [`Carla`](http://carla.org/), [`AirSim`](https://github.com/microsoft/AirSim) and [`DeepDrive`](https://deepdrive.io/) are all under **`MIT License`** while code for `Autoware` and `Apollo` is protected by the **`Apache 2 License`**.

</details>

---

**`"Overview of Tools Supporting Planning for Automated Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.04081)]**
**[** :car: [`virtual vehicle research`](https://www.v2c2.at/) **]**

- **[** _`development tools`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.04081).](../media/2020_tong_3.PNG "[Source](https://arxiv.org/abs/2003.04081).")  |
|:--:|
| *The authors group **tools** that support **`planning`** in sections: `maps`, `communication`, `traffic rules`, `middleware`, `simulators` and `benchmarks`. [Source](https://arxiv.org/abs/2003.04081).* |

| ![[Source](https://arxiv.org/abs/2003.04081).](../media/2020_tong_1.PNG "[Source](https://arxiv.org/abs/2003.04081).")  |
|:--:|
| *About `simulators` and `dataset`. And how to **couple between tools**, either with co-simulation software or open interfaces. [Source](https://arxiv.org/abs/2003.04081).* |

| ![[Source](https://arxiv.org/abs/2003.04081).](../media/2020_tong_2.PNG "[Source](https://arxiv.org/abs/2003.04081).")  |
|:--:|
| *About maps. ''The planning tasks with different targets entail map models with different level of details. `HD map` provides the most sufficient information and can be generally categorized into **three layers**: **`road` model, `lane` model and `localization` model**''. [Source](https://arxiv.org/abs/2003.04081).* |

Authors: Tong, K., Ajanovic, Z., & Stettinger, G.

- Motivations:
  - `1-` Help **researchers** to make full use of open-source resources and **reduce effort of setting up a software platform** that suites their needs.
    - > [example] "It is a good option to choose open-source [`Autoware`](https://www.autoware.org/) as **software stack** along with **[`ROS`](https://www.ros.org/) middleware**, as `Autoware` can be further transferred to a **real vehicle**. During the development, he or she can use [`Carla`](http://carla.org/) as a **simulator**, to get its benefits of **graphic rendering and sensor simulation**. To make the simulation **more realistic**, he or she might adopt commercial software [`CarMaker`](https://ipg-automotive.com/products-services/simulation-software/carmaker/) for sophisticated **vehicle dynamics** and open-source [`SUMO`](https://sumo.dlr.de/docs/index.html) for large-scale traffic flow simulation. **[`OpenDRIVE`](http://www.opendrive.org/) map** can be used as a source and converted into the map format of `Autoware`, `Carla` and `SUMO`. Finally, [`CommonRoad`](https://commonroad.in.tum.de/) can be used to **evaluate** the developed algorithm and **benchmark** it against other approaches."
  - `2-` **Avoid reinventing the wheel**.
    - _Algorithms_ are available/adapted from **robotics**.
    - _Simulators_ are available/adapted from **gaming**.
- Mentioned **software libraries** for `motion planning`:
  - `ROS` (from _robotics_):
    - [Open Motion Planning Library](https://ompl.kavrakilab.org/) (`OMPL`)
    - [`MoveIt`](https://moveit.ros.org/)
    - [navigation package](http://wiki.ros.org/navigation)
    - [`teb`](http://wiki.ros.org/teb_local_planner) local planner.
  - `Python`: [`PythonRobotics`](https://github.com/AtsushiSakai/PythonRobotics)
  - `CPP`: [`CppRobotics`](https://github.com/onlytailei/CppRobotics)
- _How to express_ **_traffic rules_** _in a form understandable by an algorithm?_
  - `1-` Traffic rules can be formalized in **higher order logic** (e.g. using the [Isabelle theorem prover](https://isabelle.in.tum.de/)) to **check the compliance of traffic rules** unambiguously and formally for **trajectory validation**.
  - `2-` Another approach is to **represent traffic rules geometrically as obstacles** in a configuration space of motion planning problem.
  - >"In some occasions, it is necessary to **violate some rules** during driving for **achieving higher goals** (i.e. _avoiding collision_) [... e.g. with] a **rule book** with **hierarchical arrangement** of different rules."

- _About data visualization?_
  - **[`RViz`](http://wiki.ros.org/rviz/)** is a popular tool in `ROS` for visualizing data flow, as I also [`realized`](https://github.com/chauvinSimon/IV19) at `IV19`.
  - Apart from that, it seems each team is having its **own specific visualization tool**, sometimes released, as **[`AVS`](https://avs.auto/#/)** from `UBER` and `GM Cruise`.
- _What is missing for the research community?_
  - Evaluation tools for **quantitative comparison**.
  - Evaluation tools incorporating **human judgment**, not only from the vehicle occupants but also from other road users.
  - A **standard format** for **motion datasets**.

</details>

---

**`"Decision-making for automated vehicles using a hierarchical behavior-based arbitration scheme"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.01149)]**
**[** :mortar_board: `FZI`, `KIT` **]**

- **[** _`hierarchical behavioural planning`, `cost-based arbitration`, `behaviour components`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.01149).](../media/2020_orzechowski_1.PNG "[Source](https://arxiv.org/abs/2003.01149).")  |
|:--:|
| *Both **`urban`** and **`highway` behaviour options** are combined using a **`cost-based arbitrator`**. Together with `Parking` and `AvoidCollisionInLastResort`, these four **arbitrators** and the `SafeStop` fallback are **composed together** to the **top-most priority-based `AutomatedDriving` arbitrator**. [Source](https://arxiv.org/abs/2003.01149).* |

| ![[Source](https://arxiv.org/abs/2003.01149).](../media/2020_orzechowski_2.PNG "[Source](https://arxiv.org/abs/2003.01149).")  |
|:--:|
| *Top-right: two possible options. The arbitrator generally prefers the `follow lane behaviour` as long as it **matches the route**. Here, a lane change is necessary and selected by the **cost-based arbitration**: `ChangeLaneRight` has lower cost than `FollowEgoLane`, mainly due to the **`routing` term** in the cost expression. Bottom: the resulting **`behaviour` selection** over time. [Source](https://arxiv.org/abs/2003.01149).* |

Authors: Orzechowski, P. F., Burger, C., & Lauer, M.

- Motivation:
  - Propose an **alternative to `FSM`s (finite state machines)** and `behaviour`-based systems (e.g. _voting systems_) in **hierarchical architectures**.
  - In particular, `FSM`s can suffer from:
    - **poor interpretability**: _why is one `behaviour` executed?_
    - **maintainability**: effort to refine existing behaviour.
    - **scalability**: effort to achieve a high number of `behaviour`s and to combine a **large variety of scenarios**.
    - **options selection**: _"multiple behaviour options are applicable but have no clear and consistent priority against each other."_
      - > _"_**_How_** _and_ **_when_** _should an automated vehicle_ **_switch_** _from a regular `ACC` controller to a lane change, cooperative zip merge or parking planner?"_
    - **multiple planners**: Each `behaviour` component can compute its **manoeuvre command** with any preferred state-of-the-art method.
      - > _"How can we support `POMDPs`, `hybrid A*` and any other_ **_planning method_** _in our behaviour generation?"._

- Main idea:
  - **`cost-based arbitration`** between so-called **"`behaviour` components"**.
  - The **modularity** of these components brings several advantages:
    - Various scenarios can be handled within a single framework: `four-way intersections`, `T-junctions`, `roundabout`, `multilane bypass roads`, `parking`, etc.
    - Hierarchically combining behaviours, **complex `behaviour`** emerges from **simple components**.
    - Good **efficiency**: the **atomic structure** allows to **evaluate** `behaviour` options **in parallel**.

- About `arbitration`:
  - > "An `arbitrator` contains a list of behavior options to choose from. A specific **selection logic** determines which **option** is chosen based on **abstract** information, e.g., **expected utility** or **priority**."
  - > [about `cost`] "The **`cost`-based** `arbitrator` selects the **behavior option** with the **lowest expected cost**."
  - Each **behaviour option** is **evaluated** based on its expected **average travel velocity**, incorporating **routing costs** and penalizing **lane changes**.
    - The resulting behaviour can thus be **well explained**:
    - > "The **selection logic** of arbitrators is **comprehensive**."
  - About hierarchy:
    - > "To generate even more complex `behaviour`s, an `arbitrator` can also be a `behaviour` option of a **hierarchically higher `arbitrator`**."

- About **`behaviour` components**.
  - There are the **smallest building blocks**, representing **basic _tactical_ driving manoeuvres**.
  - Example of **atomic behaviour components** for **simple tasks** in _urban_ scenarios:
    - `FollowLead`
    - `CrossIntersection`
    - `ChangeLane`
  - They can be **specialized**:
    - **Dense** scenarios behaviours: `ApproachGap`, `IndicateIntention` and `MergeIntoGap` to refine `ChangeLane` _(multi-phase behaviour)_.
      - _Note:_ an alternative could be to use **one single** integrated **interaction-aware behaviour** such as `POMDP`.
    - **Highway** behaviours (_structured_ but _high speed_): `MergeOntoHighway`, `FollowHighwayLane`, `ChangeHighwayLane`, `ExitFromHighway`.
    - **Parking** behaviours: `LeaveGarage`, `ParkNearGoal`.
    - Fail-safe **emergency** behaviours: `EmergenyStop`, `EvadeObject`, `SafeStop`.
  - For a `behaviour` to be selected, it should be **_applicable_**. Hence a `behaviour` is defined together with:
    - **`invocation` condition**: when does it become _applicable_.
      - > "[example:] The `invocation` condition of `CrossIntersection` is true as long as the current ego lane intersects other lanes within its planning horizon."
    - **`commitment` condition**: when does it stay _applicable_.
  - This reminds me the concept of **`macro actions`**, sometimes defined by a tuple <**`applicability condition`**, `termination condition`, `primitive policy`>.
  - It also makes me think of [`MODIA`](http://rbr.cs.umass.edu/shlomo/papers/WWZijcai17.pdf) framework and other **scene-decomposition** approaches.
- A `mid-to-mid` approach:
  - > "[input] The **input** is an **abstract environment model** that contains a _fused_, _tracked_ and _filtered_ representation of the world."
  - [output] The selected **high-level decision** is passed to a **trajectory planner** or **controller**.
  - _What does the "decision" look like?_
    - **_One-size-fits-all_ is not an option**.
    - It is distinguished between maneuvers in a _structured_ or _unstructured_ environment:
    - `1- unstructured`: a trajectory, directly passed to a **trajectory planner**.
    - `2- structured`: a corridor-based driving command, i.e. a tuple <`maneuver corridor`, `reference line`, `predicted objects`, `maneuver variant`>. It requires both a _trajectory planner_ and a _controller_.

- One distinction:
  - `1-` **top-down `knowledge`-based** systems.
    - > "The `action selection` in a **centralized**, in a **_top-down_ manner** using a knowledge database."
    - > "The engineer designing the **action selection** module (in `FSM`s the **state transitions**) has to be **aware** of the conditions, _effects_ and possible interactions of all behaviors at hand."
  - `2-` **bottom-up `behaviour`-based** systems.
    - > "**Decouple** actions into **atomic simple `behaviour` components** that should be aware of their conditions and _effects_."
    - E.g. `voting systems`.
  - Here the authors combine **atomic behaviour components** (`bottom`/`down`) with **more complex behaviours** using **generic arbitrators** (`top`/`up`).

</details>

---

**`"A Review of Motion Planning for Highway Autonomous Driving"`**

- **[** `2019` **]**
**[[:memo:](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving)]**
**[** :mortar_board: `French Institute of Science and Technology for Transport` **]**
**[** :car: `VEDECOM Institute` **]**

<details>
  <summary>Click to expand</summary>

| ![The review divides **motion planning** into five unavoidable parts. The **`decision making`** part contains `risk evaluation`, `criteria minimization`, and `constraint submission`. In the last part, a **low-level reactive** planner **deforms** the generated motion from the high-level planner. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).](../media/2019_claussmann_1.PNG "The review divides **`motion planning`** into five  parts. The **`decision-making`** part contains `risk evaluation`, `criteria minimization`, and `constraint submission`. In the last part, a **low-level reactive** planner **deforms** the generated motion from the high-level planner. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).")  |
|:--:|
| *The review divides **motion-planning** into five parts. The **`decision-making`** part contains `risk evaluation`, `criteria minimization`, and `constraint submission`. In the last part, a **low-level reactive** planner **deforms** the generated motion from the high-level planner. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).* |

| ![The review offers two detailed tools for **comparing** methods for motion planning for highway scenarios. Criteria for the generated motion include: `feasible`, `safe`, `optimal`, `usable`, `adaptive`, `efficient`, `progressive` and `interactive`. The authors stressed the importance of **spatiotemporal consideration** and emphasize that **highway motion-planning** is highly structured. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).](../media/2019_claussmann_6.PNG "The review offers two detailed tools for **comparing** methods for motion planning for highway scenarios. Criteria for the generated motion include: `feasible`, `safe`, `optimal`, `usable`, `adaptive`, `efficient`, `progressive` and `interactive`. The authors stressed the importance of **spatiotemporal consideration** and emphasize that **highway motion-planning** is highly structured. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).")  |
|:--:|
| *The review offers two detailed tools for **comparing** methods for motion planning for highway scenarios. Criteria for the generated motion include: `feasible`, `safe`, `optimal`, `usable`, `adaptive`, `efficient`, `progressive` and `interactive`. The authors stressed the importance of **spatiotemporal consideration** and emphasize that **highway motion-planning** is highly structured. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).* |

| ![Contrary to `solve-algorithms` methods, `set-algorithm` methods require a **complementary algorithm** should be added to find the feasible motion. Depending on the importance of the `generation` (`iv`) and `deformation` (`v`) part, approaches are more or less **`reactive`** or **`predictive`**. Finally, based on their [work](https://www.researchgate.net/publication/321406071_A_study_on_al-based_approaches_for_high-level_decision_making_in_highway_autonomous_driving) on AI-based algorithms, the authors define four subfamilies to compare to human: `logic`, `heuristic`, `approximate reasoning`, and `human-like`. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).](../media/2019_claussmann_3.PNG "Contrary to `solve-algorithms` methods, `set-algorithm` methods require a **complementary algorithm** should be added to find the feasible motion. Depending on the importance of the `generation` (`iv`) and `deformation` (`v`) part, approaches are more or less **`reactive`** or **`predictive`**. Finally, based on their [work](https://www.researchgate.net/publication/321406071_A_study_on_al-based_approaches_for_high-level_decision_making_in_highway_autonomous_driving) on AI-based algorithms, the authors define four subfamilies to compare to human: `logic`, `heuristic`, `approximate reasoning`, and `human-like`. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).")  |
|:--:|
| *Contrary to `solve-algorithms` methods, `set-algorithm` methods require a **complementary algorithm** should be added to find the feasible motion. Depending on the importance of the `generation` (`iv`) and `deformation` (`v`) part, approaches are more or less **`reactive`** or **`predictive`**. Finally, based on their [work](https://www.researchgate.net/publication/321406071_A_study_on_al-based_approaches_for_high-level_decision_making_in_highway_autonomous_driving) on AI-based algorithms, the authors define four subfamilies to compare to human: `logic`, `heuristic`, `approximate reasoning`, and `human-like`. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).* |

| ![The review also offers overviews for possible space configurations, i.e. the choices for decomposition of the evolution space (Sampling-Based Decomposition, Connected Cells Decomposition and Lattice Representation) as well as Path-finding algorithms (e.g. `Dijkstra`, `A*`, and `RRT`). _Attractive and Repulsive Forces_, _Parametric and Semi-Parametric Curves_, _Numerical Optimization_ and _Artificial Intelligence_ are also developed. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).](../media/2019_claussmann_4.PNG "The review also offers overviews for possible space configurations, i.e. the choices for decomposition of the evolution space (Sampling-Based Decomposition, Connected Cells Decomposition and Lattice Representation) as well as Path-finding algorithms (e.g. `Dijkstra`, `A*`, and `RRT`). _Attractive and Repulsive Forces_, _Parametric and Semi-Parametric Curves_, _Numerical Optimization_ and _Artificial Intelligence_ are also developed. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).")  |
|:--:|
| *The review also offers overviews for possible **space configurations**, i.e. the choices for decomposition of the **evolution space** (`sampling-based decomposition`, `connected cells decomposition` and `lattice representation`) as well as **`path-finding algorithms`** (e.g. `Dijkstra`, `A*`, and `RRT`). `attractive and repulsive forces`, `parametric and semi-parametric curves`, `numerical optimization` and `artificial intelligence` are also developed. [Source](https://www.researchgate.net/publication/333124691_A_Review_of_Motion_Planning_for_Highway_Autonomous_Driving).* |

Authors: Claussmann, L., Revilloud, M., Gruyer, D., & Glaser, S.

</details>

---

**`"A Survey of Deep Learning Applications to Autonomous Vehicle Control"`**

- **[** `2019` **]**
**[[:memo:](http://arxiv.org/abs/1912.10773)]**
**[** :mortar_board: `University of Surrey` **]**
**[** :car: `Jaguar Land Rover` **]**

<details>
  <summary>Click to expand</summary>

| ![__Challenges__ for **learning-based** control methods. [Source](http://arxiv.org/abs/1912.10773).](../media/2019_kuutti_1.PNG "__Challenges__ for **learning-based** control methods. [Source](http://arxiv.org/abs/1912.10773).")  |
|:--:|
| *__Challenges__ for **learning-based** control methods. [Source](http://arxiv.org/abs/1912.10773).* |

Authors: Kuutti, S., Bowden, R., Jin, Y., Barber, P., & Fallah, S.

- Three categories are examined:
  - **`lateral`** control alone.
  - **`longitudinal`** control alone.
  - **`longitudinal`** and **`lateral`** control combined.
- Two quotes:
  - > "While **lateral control** is typically achieved through **`vision`**, the **longitudinal control** relies on measurements of relative velocity and distance to the preceding/following vehicles. This means that ranging sensors such as **`RADAR` or `LIDAR`** are more commonly used in **longitudinal control** systems.".
  - > "While **lateral control** techniques favour **`supervised learning`** techniques trained on labelled datasets, **longitudinal control** techniques favour **`reinforcement learning`** methods which learn through **interaction** with the environment."

</details>

---

**`"Longitudinal Motion Planning for Autonomous Vehicles and Its Impact on Congestion: A Survey"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1910.06070)]**
**[** :mortar_board: `Georgia Institute of Technology` **]**

<details>
  <summary>Click to expand</summary>

| ![`mMP` refers to machine learning methods for longitudinal motion planning. [Source](https://arxiv.org/abs/1910.06070).](../media/2019_zhou_1.PNG "`mMP` refers to machine learning methods for longitudinal motion planning. [Source](https://arxiv.org/abs/1910.06070).")  |
|:--:|
| *`mMP` refers to machine learning methods for longitudinal motion planning. [Source](https://arxiv.org/abs/1910.06070).* |

Authors: Zhou, H., & Laval, J.

- This review has been completed at a school of _"_**_civil_** and **_environmental_** _engineering"_.
  - It **does not have any scientific contribution**, but offers a quick overview about some current trends in `decision-making`.
  - The authors try to look at **industrial applications** (e.g. `Waymo`, `Uber`, `Tesla`), i.e. not just focussing on theoretical research. Since companies do no communicate explicitly about their approaches, most of their publications should be considered as **_research side-projects_**, rather than "actual state" of the industry.
- One focus of the review: the **machine learning** approaches for decision-making for **longitudinal motion**.
  - About the **architecture** and **representation** models. They mention the works of [**`DeepDriving`**](http://deepdriving.cs.princeton.edu/) and [(H. Xu, Gao, Yu, & Darrell, 2016)](https://arxiv.org/abs/1612.01079).
    - **`Mediated perception`** approaches **parse an entire scene** to make a driving decision.
    - **`Direct perception`** approaches first extract **affordance indicators** (i.e. only the **information** that are important for driving in a particular situation) and then map them to actions.
      - > "Only a small portion of detected objects are indeed related to the real driving reactions so that it would be meaningful to reduce the number of key perception indicators known as learning **`affordances`**".
    - **`Behavioural reflex`** approaches **directly map** an input image to a driving action by a regressor.
      - This `end-to-end` paradigm can be extended with **auxiliary tasks** such as learning semantic segmentation (_this "side task" should **_further improves_** the model_), leading to **`Privileged training`**.
  - About the learning methods:
    - `BC`, `RL`, `IRL` and `GAIL` are considered.
    - The authors argue that their `memory` and `prediction` abilities should make them stand out from the rule-based approaches.
    - > "Both `BC` and `IRL` algorithms implicitly assume that the **demonstrations are complete**, meaning that the action for each demonstrated state is fully observable and available."
    - > "We argue that adopting `RL` transforms the problem of learnt longitudinal motion planning from **imitating human demonstrations** to searching for a policy complying a **hand-crafted reward rule** [...] No studies have shown that a genuine **reward function for human driving** really exists."
- About **congestion**:
  - > "The AV industry has been mostly **focusing on the long tail** problem caused by `corner errors` related to **safety**, while the impact of AVs on **traffic efficiency** is almost ignored."
  - It reminds me the finding of (Kellett, J., Barreto, R., Van Den Hengel, A. & Vogiatzis. N., 2019) in ["How Might Autonomous Vehicles Impact the City? The Case of Commuting to Central Adelaide"](https://www.tandfonline.com/doi/full/10.1080/08111146.2019.1674646): **driverless cars could lead to more traffic congestion**.

</details>

---

**`"Design Space of Behaviour Planning for Autonomous Driving"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1908.07931)]**
**[** :mortar_board: `University of Waterloo` **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![The focus is on the `BP` module, together with its predecessor (`environment`) and its successor (`LP`) in a modular architecture. [Source](https://arxiv.org/abs/1908.07931).](../media/2019_ilievski_5.PNG "The focus is on the `BP` module, together with its predecessor (`environment`) and its successor (`LP`) in a modular architecture. [Source](https://arxiv.org/abs/1908.07931).")  |
|:--:|
| *The focus is on the `BP` module, together with its predecessor (`environment`) and its successor (`LP`) in a modular architecture. [Source](https://arxiv.org/abs/1908.07931).* |

| ![Classification for Question `1` - environment representation. A combination is possible. In black, my notes giving examples. [Source](https://arxiv.org/abs/1908.07931).](../media/2019_ilievski_6.PNG "Classification for Question `1` - environment representation. A combination is possible. In black, my notes giving examples. [Source](https://arxiv.org/abs/1908.07931).")  |
|:--:|
| *Classification for Question `1` - environment representation. A combination is possible. In black, my notes giving examples. [Source](https://arxiv.org/abs/1908.07931).* |

| ![Classification for Question `2` - on the architecture. [Source](https://arxiv.org/abs/1908.07931).](../media/2019_ilievski_2.PNG "Classification for Question `2` - on the architecture. [Source](https://arxiv.org/abs/1908.07931).")  |
|:--:|
| *Classification for Question `2` - on the architecture. [Source](https://arxiv.org/abs/1908.07931).* |

| ![Classification for Question `3` - on the decision logic representation. [Source](https://arxiv.org/abs/1908.07931).](../media/2019_ilievski_1.PNG "Classification for Question `3` - on the decision logic representation. [Source](https://arxiv.org/abs/1908.07931).")  |
|:--:|
| *Classification for Question `3` - on the decision logic representation. [Source](https://arxiv.org/abs/1908.07931).* |

Authors: Ilievski, M., Sedwards, S., Gaurav, A., Balakrishnan, A., Sarkar, A., Lee, J., Bouchard, F., De Iaco, R., & Czarnecki K.

The authors divide their review into three sections:

- Question `1`: **_How to represent the environment?_** (relation with _predecessor_ of `BP`)
  - Four representations are compared: `raw data`, `feature-based`, `grid-based` and `latent representation`.
- Question `2`: **_How to communicate with other modules, especially the local planner (`LP`)?_** (relation with _successor_ (`LP`) of `BP`)
  - A first sub-question is the relevance of **separation** `BP` / `LP`.
    - A complete separation (_top-down_) can lead to **computational redundancy** (both have a _collision checker_).
    - One idea, close to **sampling techniques**, could be to **invert the traditional architecture** for planning, i.e. **generate multiple possible local paths** (`~LP`) then selects the best manoeuvre according to a given cost function (`~BP`). But this exasperates the previous point.
  - A second sub-question concerns _prediction_: **_Should the `BP` module have its own dedicated prediction module?_**
    - First, three kind of prediction are listed, depending on what should be predicted (marked with `->`):
      - _Physics_-based (`->` trajectory).
      - _Manoeuvre_-based (`->` low-level motion primitives).
      - _Interaction_-aware (`->` intent).
    - Then, the authors distinguish between **_explicitly-defined_** and **_implicitly-defined_** prediction models:
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
      - The combination of both seems beneficial:
        - An initial behaviour is obtained through **imitation learning** (_learning from example_). Also possible with `IRL`.
        - But **improvements are made through interaction** with a simulated environment (_learning from interaction_).
          - By the way, the _learning from interaction_ techniques raise the question of the **origin of the experience** (e.g. realism of the simulator) and its **sampling efficiency**.
      - Another promising direction is **hierarchical RL** where the MDP is divided into sub-problems (the **lower for `LP`** and the **higher for `BP`**)
        - The _lowest level_ implementation of the hierarchy approximates a solution to the **control and LP problem** ...
        - ... while the _higher level_ **selects a manoeuvre** to be executed by the lower level implementations.
  - As mentioned in my the section on [Scenarios and Datasets](https://github.com/chauvinSimon/IV19#scenarios-and-datasets), the authors mention the **lack of benchmark** to compare and **evaluate** the performance of BP technologies.

One quote about the _representation of decision logic_:

- As identified in [my notes about IV19](https://github.com/chauvinSimon/IV19), the **combination** of _learnt_ and _non-learnt_ approaches looks the most promising.
- > "Without learning, traditional robotic solutions cannot adequately handle **complex, dynamic human environments**, but **ensuring the safety** of learned systems remains a significant challenge."
- > "Hence, we speculate that future high performance and safe behaviour planning solutions will be **_hybrid_** and **_heterogeneous_**, incorporating modules consisting of learned systems supervised by programmed logic."

</details>

---

**`"A Behavioral Planning Framework for Autonomous Driving"`**

- **[** `2014` **]**
**[[:memo:](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf)]**
**[** :mortar_board: `Carnegie Mellon University` **]**
**[** :car: `General Motor` **]**

- **[** _`behavioural planning`, `sampling-based planner`, `decision under uncertainty`, [`TORCS`](http://torcs.sourceforge.net/)_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![Comparison and fusion of the **hierarchical** and **parallel** architectures. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).](../media/2014_wei_1.PNG "Comparison and fusion of the **hierarchical** and **parallel** architectures. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).")  |
|:--:|
| *Comparison and fusion of the **hierarchical** and **parallel** architectures. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).* |

| ![The `PCB` algorithm implemented in the `BP` module. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).](../media/2014_wei_2.PNG "The `PCB` algorithm implemented in the `BP` module. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).")  |
|:--:|
| *The `PCB` algorithm implemented in the `BP` module. [Source](https://ri.cmu.edu/pub_files/2014/6/IV2014-Junqing-Final.pdf).* |

| ![Related work by (Xu, Pan, Wei, & Dolan, 2014) - Grey ellipses indicate the magnitude of the uncertainty of state. [Source](https://ri.cmu.edu/pub_files/2014/6/ICRA14_0863_Final.pdf).](../media/2014_xu_1.PNG "Related work by (Xu, Pan, Wei, & Dolan, 2014) - Grey ellipses indicate the magnitude of the uncertainty of state. [Source](https://ri.cmu.edu/pub_files/2014/6/ICRA14_0863_Final.pdf).")  |
|:--:|
| *Related work by (Xu, Pan, Wei, & Dolan, 2014) - Grey ellipses indicate the magnitude of the uncertainty of state. [Source](https://ri.cmu.edu/pub_files/2014/6/ICRA14_0863_Final.pdf).* |

Authors:  Wei, J., Snider, J. M., & Dolan, J. M.

Note: I find very valuable to get insights from the **CMU** (Carnegie Mellon University) Team, based on their **experience of the DARPA Urban Challenges**.

- Related works:
  - [_A prediction- and cost function-based algorithm for robust autonomous freeway driving_. 2010](https://ri.cmu.edu/pub_files/2010/6/2010_IV.pdf) by (Wei, Dolan, & Litkouhi, 2010).
    - They introduced the **_"Prediction- and Cost-function Based (`PCB`) algorithm"_** used.
    - The idea is to `generate`-`forward_simulate`-`evaluate` a set of manoeuvres.
    - The planner can therefore take **surrounding vehicles’ reactions** into account **in the cost function** when it searches for the best strategy.
    - At the time, the authors rejected the option of a `POMDP` formulation (_computing the control policy over the space of the belief state, which is a probability distribution over all the possible states_) deemed as computationally expensive. Improvements in hardware and algorithmic have been made since 2014.
  - [_Motion planning under uncertainty for on-road autonomous driving_. 2014](https://ri.cmu.edu/pub_files/2014/6/ICRA14_0863_Final.pdf) by (Xu, Pan, Wei, & Dolan, 2014).
    - An extension of the framework to **consider uncertainty** (both for _environment_ and the _others participants_) in the decision-making.
    - The prediction module is using a **Kalman Filter** (assuming constant velocity).
    - For each candidate trajectory, the **uncertainty** can be estimated using a **_Linear-Quadratic Gaussian_** (`LQG`) framework (based on the noise characteristics of the localization and control).
    - Their Gaussian-based method gives some **probabilistic safety guaranty** (e.g. likelihood `2%` of collision to occur).
- Proposed architecture for _decision-making_:
  - First ingredient: **Hierarchical** architecture.
    - The hierarchy `mission` `->` `manoeuvre` `->` `motion` [`3M` concept](https://github.com/chauvinSimon/Hierarchical-Decision-Making-for-Autonomous-Driving) makes it very modular but can raise limitations:
    - > "the higher-level decision making module usually **does not have enough detailed information**, and the lower-level layer **does not have authority** to **re-evaluate the decision**."
  - Second ingredient: **Parallel** architecture.
    - This is inspired from **ADAS** engineering.
    - The control modules (`ACC`, `Merge Assist`, `Lane Centreing`) are relatively **independent** and **work in parallel**.
    - In some **complicated cases** needing cooperation, this framework may not perform well.
      - _This probably shows that just_ **_extending the common ADAS architectures_** _cannot be enough to reach the_ **_level-`5` of autonomy_**.
  - Idea of the proposed framework: **combine the strengths** of the **_hierarchical_** and **_parallel_** architectures.
    - This relieves the path planner and the control module (the search space is reduced).
    - Hence the **computational cost** shrinks (by over 90% compared to a **_sample-based planner_** in the **spatio-temporal space**).
- One module worth mentioning: **_Traffic-free Reference Planner_**.
  - Its input: lane-level **sub-missions** from the _Mission Planning_.
  - Its output: kinematically and dynamically feasible **paths** and a **speed profile** for the **_Behavioural Planner_** (`BP`).
    - It assumes there is **no traffic** on the road, i.e. **ignores dynamic obstacles**.
    - It also applies **traffic rules** such as _speed limits_.
  - This **guides** the `BP` layer which considers both static and dynamic obstacles to generate so-called **_"controller directives"_** such as:
    - The **lateral driving bias**.
    - The **desired leading vehicle** to follow.
    - The **aggressiveness** of distance keeping.
    - The **maximum speed**.

</details>
