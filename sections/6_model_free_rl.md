# `Model-Free` `Reinforcement Learning`

---

**`"Deep Surrogate Q-Learning for Autonomous Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2010.11278)]**
**[** :mortar_board: `University of Freiburg` **]**
**[** :car: `BMW` **]**

- **[** _`offline RL`, `sampling efficiency`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2010.11278).](../media/2020_huegle_1.PNG "[Source](https://arxiv.org/abs/2010.11278).")  |
|:--:|
| *The goal is to **reduce the required driving `time`** to **collect** a certain number of **`lane-changes`**. The idea is to **estimate the `q-values`** not only for the ego car, but **also for the surrounding ones**. Assuming it is possible to infer which `action` each car was performing, this results in **many (`s`, `a`, `r`, `s'`) transitions** collected at each time step. The **replay buffer** hence can be filled way **faster** compared to considering only **ego-transitions**. [Source](https://arxiv.org/abs/2010.11278).* |

| ![[Source](https://arxiv.org/abs/2005.01643).](../media/2020_levine_1.PNG "[Source](https://arxiv.org/abs/2005.01643).")  |
|:--:|
| *From another paper: Illustration of the difference between `online` (`on-policy`), `off-policy` and **`offline`** `RL`. The later resembles **supervised learning** where **data collection** is easier. It therefore hold a huge potential for application were **`interactions` with the `environment`** are **expensive or dangerous**. [Source](https://arxiv.org/abs/2005.01643).* |

Authors: Huegle, M., Kalweit, G., Werling, M., & Boedecker, J.

- Main motivation:
  - **Interaction efficiency**: limit the **costly interactions** with the environment.
    - The goal is to **reduce the required `driving time`** to collect a certain number of **lane-changes**
  - Application: learn to make **high-level** decisions on the **highway**.
- Main idea:
  - **Leverage the `transitions` of surrounding observed cars**, instead of only considering the ego-agent's `transitions`.
    - Hence not only **`off-policy`**: transitions are collected with policies possibly different from the current one.
    - But also **`offline RL`**: forget about `exploration` and `interactions` and utilize previously **collected offline data**.

- Main hypotheses:
  - The agent can detect or **infer `actions`** of its surrounding cars.
  - All cars are assumed to have the **same action space** as the agent or `actions` that are **mappable to the agent's `action` space**.
  - No assumption about the `reward function` of other cars. Here, only the **ego `reward function`** is used to evaluate the (`s`, `a`, `s'`) transitions.
  - > "This is in contrast to the **multi-agent `RL`** setting, where the long-term `return` of all agents involved is **optimized together**."

- **`action` space** and **decision `period`**.
  - About `keep lane`, `left lane-change`, and `right lane-change`.
  - **Step size of `2s`**.
    - Hence not about **`safety`**. Rather about about efficiency and **long-term planning** capability.
    - > "**Collision avoidance** and maintaining safe distance in longitudinal direction are controlled by an **integrated `safety` module**."
    - > "**`acceleration`** is handled by a **low-level execution layer** with model-based control of acceleration to guarantee comfort and `safety`."
  - > "Additionally, we filter a **time span of `5s` before and after** all lane changes in the dataset with a **step size of `2s`**, leading to a consecutive chain of **`5` time-steps**."

- About **`offline` `RL`**:
  - **Recordings** are easier:
    - > [**static** sensors] "This **drastically simplifies data-collection**, since such a **`transition` set** could be collected by setting up a camera on top of a **bridge above a highway**."
    - > [sensors on a driving **test vehicle**] "In this setting, a policy can even be **learned without performing any lane-changes** with the **test vehicle** itself."
  - Promising for **`sim-to-real`**:
    - > "If it was possible to simply train policies with **previously collected data**, it would likely be **unnecessary in many cases to manually design high-fidelity simulators** for simulation-to-real-world transfer.
  - _What about `exploration`, `distributional shift` and `optimality`?_

- _Why "surrogate"?_
  - Because **other drivers** are used as surrogates, i.e. **they replace the ego-agent** to learn the ego-agent's `value function`.

- _How to deal with a_ **_variable-sized_** _input vehicles list and be_ **_permutation-independent_**_?_
  - Idea of `PointNet`, here called `DeepSet`: use an **aggregator**, for instance `pooling`.

- _How to_ **_estimate multiple `action-values` at once_**_?_
  - As opposed to multiple `**individual forward-passes** per sample.
  - In parallel _(if the hardware enables it!)_.
  - Extend the **`DeepSet`-like architecture** which already computes `Ψ`(`scene[t]`), the **representation of the scene**. This `Ψ` is needed here to estimate each `q-value`.

- _How to estimate the `q-value` of a (`s`, `a`) pair for a given (e.g. ego) agent?_
  - `1-` The **car-`state`** is concatenated with the **representation of the scene** `Ψ`.
  - `2-` This then goes through a **`Q` module**, here fully-connected net.

- _How to_ **_sample experiences_** from the replay buffer_?_
  - **Uniform** sampling is an option.
    - But as for **Prioritized Experience Replay** for `DQN`, one may prefer to **sample "interesting" cases**.
    - But one should be careful with the **induced distribution** (c.f. **importance sampling** in `PER`)
  - Here: the more **complex the scene**, the more `q-values` are estimated, the more **efficient the sampling**.
    - > "We propose a **sample distribution** dependent on the **_complexity of the scene_**, i.e. the **number of vehicles**."
    - Therefore called **Scene-centric Experience Replay (`SCER`)**.
  - About the resulting distribution.
    - > "`Scene-centric Experience Replay` via a **permutation-equivariant** architecture leads to a **more consistent gradient**, since the **`TD`-errors are normalized** w.r.t. all predictions for the **different positions** in the scene while keeping the **`i.i.d.` assumption of stochastic gradient descent** by sampling uniformly from the replay buffer."

</details>

---

**`"Deep Reinforcement Learning and Transportation Research: A Comprehensive Review"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2010.06187)]**
**[** :mortar_board: `University of Illinois` **]**

- **[** _`review`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2010.06187).](../media/2020_farazi_1.PNG "[Source](https://arxiv.org/abs/2010.06187).")  |
|:--:|
| *Most approaches are used for **low-level control** in specific tasks. Instead of predicting the **actuator commands such as `throttle`**, most are **predicting the `acceleration`** which should be easier to **transfer** from the simulator to a **real car** that has **different dynamics and capabilities**. [Source](https://arxiv.org/abs/2010.06187).* |

Authors: Farazi, N. P., Ahamed, T., Barua, L., & Zou, B.

- In short:
  - Review of some of the latest papers on **model-free `RL`** for several applications such as autonomous driving and traffic management e.g. `signal` and `route` control.
  - No mention of the **`safety` limitations**, which is one of the major problems of **model-free** methods.

</details>

---

**`"MIDAS: Multi-agent Interaction-aware Decision-making with Adaptive Strategies for Urban Autonomous Navigation"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2008.07081)]**
**[** :mortar_board: `University of Pennsylvania` **]**
**[** :car: `Nuro` **]**

- **[** _`attention`, `parametrized driver-type`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2008.07081).](../media/2020_chen_5.PNG "[Source](https://arxiv.org/abs/2008.07081).")  |
|:--:|
| *Top: scenarios are generated to require **interaction-aware** decisions from the ego agent. Bottom: A **`driver-type` parameter** is introduced to learn a **single policy** that works **across different planning objectives**. It represents the **driving style** such as the **level of `aggressiveness`**. It affects the terms in the **`reward` function** in an **affine way**. [Source](https://arxiv.org/abs/2008.07081).* |

| ![[Source](https://arxiv.org/abs/2008.07081).](../media/2020_chen_6.PNG "[Source](https://arxiv.org/abs/2008.07081).")  |
|:--:|
| *The conditional parameter (**ego’s `driver-type`**) should **only affect the encoding of its own `state`**, not the `state` of the other agents. Therefore it injected **after the `observation` encoder**. `MIDAS` is compared to **[`DeepSet`](https://arxiv.org/abs/1909.13582)** and **[“Social Attention”](https://arxiv.org/abs/1911.12250)**. `SAB` stands for ''set-attention block'', `ISAB` for ''induced `SAB`'' and `PMA` for ''pooling by multi-head attention''. [Source](https://arxiv.org/abs/2008.07081).* |

Authors: Chen, X., & Chaudhari, P.

- Motivations:
  - `1-` Derive an **adaptive** ego policy.
    - For instance, **conditioned** on a parameter that represents the **driving style** such as the **level of `aggressiveness`**.
    - > "`MIDAS` includes a **`driver-type` parameter** to learn a **single policy** that works **across different planning objectives**."
  - `2-` Handle an **arbitrary number** of other agents, with a **permutation-invariant** input representation.
    - > [`MIDAS` uses an `attention`-mechanism] "The ability to **pay attention** to only the part of the **`observation` vector** that matters for control **irrespective of the number of other agents** in the vicinity."
  - `3-` **Decision-making** should be **interaction-aware**.
    - > "A typical `planning` algorithm would **predict the forward motion** of the other cars and ego **would stop until it is deemed safe** and legal to proceed. While this is reasonable, it leads to **overly conservative plans** because it **does not explicitly model the mutual influence** of the actions of **interacting agents**."
    - The goal here is to obtain a policy **more optimistic than a worst-case assumption** via the **tuning of the `driver-type`**.

- _How to train a_ **_user-tuneable adaptive_** _policy?_
  - > Each agent possesses a **real-valued parameter `βk` ∈ [`−1`, `1`]** that models its **“driver-type”**. A large value of `βk` indicates an **aggressive agent** and a small value of `βk` indicates that the agent is **inclined to wait for others** around it before making progress."
  - `β` is **not observable** to others and is used to determine the **agent’s velocity** as `v = 2.7β + 8.3`.
  - This **affine form `wβ+b`** is also used in all sub-rewards of the **`reward` function**:
    - `1-` **Time-penalty** for every timestep.
    - `2-` Reward for **non-zero speed**.
    - `3-` **Timeout penalty** that discourages ego from stopping the traffic flow.
      - > "This includes a **stalement penalty** where all nearby agents including ego are **standstill waiting for one** of them to take initiative and break the tie."
    - `4-` **Collision** penalty.
    - `5-` A penalty for following **too close to the agent in front**. _- This one does not depend on `β`_.
  - The goal is **not to explicitly infer `β` from `observation`s**, as it is done by the `belief tracker` of some `POMDP` solvers.
  - _Where to inject `β`?_
    - > "We want **ego’s `driver-type`** information to **only affect the encoding of its own `state`**, not the state of the other agents."
    - > "We use a two-layer perceptron with ReLU nonlinearities to embed the scalar variable `β` and **add the output** to the encoding of ego’s `state`." [_Does that mean that the_ **_single scalar `β`_** _go alone through two FC layers?_]
  - To derive **adjustable drivers**, one could use `counterfactual reasoning` with `k-levels` for instance.
    - > "It however uses **self-play** to train the policy and while this approach is reasonable for **highway merging**, the competition is likely to result in **high collision rates in busy urban intersections** such as ours."

- About the **`attention`-based** architecture.
  - The **permutation-invariance** and **size independence** can be achieved by **combining a `sum`** followed by **some `aggregation` operator** such as `average pooling`.
    - The authors show the limitation of using a `sum`. Instead of **preferring a `max pooling`**, the authors suggest using `attention`:
    - > "Observe however that the **summation assigns the same weight** to all elements in the input. As we see in our experiments, a value function using this **[`DeepSet`](https://arxiv.org/abs/1909.13582) architecture** is likely to be **distracted by agents** that do not inform the optimal action."
  - > "An `attention` module is an elegant way for the `value function` to learn **`key`, `query`, `value` embeddings** that **pay more `attention`** to parts of the **input** that are more relevant to the output (for decision making)."
  - [**`Set transformer`**](https://arxiv.org/abs/1810.00825).
    - > "The **set-transformer** in `MIDAS` is an easy, automatic way to **encode variable-sized `observation` vectors**. In this sense, our work is closest to [“Social Attention”](https://arxiv.org/abs/1911.12250), (Leurent & Mercat, 2019), which learns to influence other agents based on **road priority** and demonstrates results on a limited set of road geometries."

- `observation` space (_not very clear_).
  - > "Their observation vector contains the **locations of all agents** within a Euclidean distance of `10m` and is created in an **ego-centric coordinate frame**."
  - The list of `waypoints` to reach **agents-specific goal locations** is allegedly also part of the `observation`.
  - _No orientation? No previous poses?_

- Binary `action`.
  - > "Control `actions` of all agents are `ukt` ∈ {`0`, `1`} which correspond to **`stop` and `go`** respectively."
  - No information about the `transition` / `dynamics` model.

- Tricks for **off-policy RL**, here `DQN`.
  - > "The **`TD2` objective** can be zero even if the value function is not accurate because the **Bellman operator** is only a **contraction in the `L∞` norm, not the `L2` norm.**"
  - Together with `double DQN`, and `duelling DQN`, the authors proposed **two variants**:
    - `1-` The `net 1` (**local**) selects the action with `argmax` for `net 2` (`target` or **`time-lagged`**, whose weights are **copied** from `net 1` at **fixed periods**) and vice-versa.
      - > "This forces the **first copy**, via its **time-lagged parameters** to be the **evaluator for the second copy** and vice-versa; it leads to further **variance reduction** of the target in the `TD` objective."
    - `2-` During `action selection`, `argmax` is applied on the **average of the `q`-values** of **both networks**.

- Evaluation.
  - **Non-ego agents** drive using an **Oracle policy** that has full access to trajectories of nearby agents. Here it is rule-based (`TTC`).
  - Scenarios are diverse for **training** (no `collision` scenario during **testing**. [_Why?_]):
    - `generic`: Initial and goal locations are **uniformly sampled**.
    - `collision`: The ego **will collide** with at least one other agent in the future if it does not stop at an appropriate timestep.
    - `interaction`: At least `2` other agents will **arrive at a location simultaneously** with the ego car.
      - > "Ego cannot do well in `interaction` episodes unless it **negotiates** with other agents."
      - > "We randomize over the **number of agents**, **`driver-types`**, agent IDs, **road geometries**, and add **small perturbations** to their **arrival time** to construct `1917` `interaction` episodes."
      - > "Curating the dataset in this fashion **aids the reproducibility of results** compared to using **random seeds** to initialize the environment."
  - > "[Robustness] At test time, we add **`Bernoulli` noise of probability `0.1`** to the `actions` of other agents to model the fact that **driving policies of other agents may be different** from each other."
  - Performance in the **simulator** is evaluated based on:
    - `1-` The **`time-to-finish`** which is the **average episode length**.
    - `2-` The **`collision-`, `timeout-` and `success rate`** which refer to the percentage of episodes that end with the corresponding status (the three add up to `1`).
  - > "To qualitatively compare performance, we prioritize `collision rate` (an indicator for `safety`) over the `timeout rate` and `time-to-finish` (which indicate `efficiency`). Performance of **the `Oracle planner` is reported over `4` trials**. Performance of the trained policy is reported **across `4` random seeds**."

</details>

---

**`"An end-to-end learning of driving strategies based on DDPG and imitation learning"`**

- **[** `2020` **]**
**[[:memo:](https://ieeexplore.ieee.org/document/9164410)]**
**[** :mortar_board: `Dalian University` **]**

- **[** _`imitation learning`, `off-policy`, `experience replay`, [`TORCS`](http://torcs.sourceforge.net/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://ieeexplore.ieee.org/document/9164410).](../media/2020_zou_1.PNG "[Source](https://ieeexplore.ieee.org/document/9164410).")  |
|:--:|
| *A **small set of collected expert demonstrations** is used to train an **`IL` agent** while **pre-training the `DDPG`** (`offline RL`). Then, the `IL` agent is used to **generate new experiences**, stored in the **`M1` buffer**. The `RL` agent is then **trained `online`** ('self-learning') and the **generated experiences** are stored in **`M2`**. During this training phase, the **sampling from `M1` is progressively reduced**. The **decay of the `sampling ratio`** is automated based on the `return`. [Source](https://ieeexplore.ieee.org/document/9164410).* |

Authors: Zou, Q., Xiong, K., & Hou, Y.

- _I must say the figures are low quality and some sections are not well-written. But I think the main idea is interesting to report here._
- Main inspiration: **[`Deep Q-learning from Demonstrations`](https://arxiv.org/pdf/1704.03732.pdf)** (Hester et al. 2017) at `DeepMind`.
  - > "We present an algorithm, **Deep Q-learning from Demonstrations (`DQfD`)**, that leverages **small sets of demonstration data** to massively **accelerate the learning process** even from relatively small amounts of demonstration data and is able to **automatically assess the necessary ratio of demonstration data** while learning thanks to a prioritized replay mechanism."
- Motivations:
  - Improve the **training efficiency** of model-free **off-policy** `RL` algorithms.
    - > [issue with `DDPG`] "The reason why `RL` converges slowly is that it requires **constant exploration** from the environment. The data in the **early experience pool (`s`, `a`, `r`, `s_`) are all low-reward data**, and the algorithm cannot always be trained with useful data, resulting in a **slow convergence rate**."
    - > [issue with `behavioural cloning`] "The performance of pure `IL` depends on the **quality and richness of expert data**, and **cannot be self-improved**."
  - > [main idea] "**Demonstration** data generated by `IL` are used to **accelerate** the learning speed of `DDPG`, and then the algorithm is further enhanced through **self-learning**."
- Using **`2` experience replay buffers**.
  - `1-` Collect a small amount of **expert demonstrations**.
    - > "We collect about **`2000` sets** of artificial data from the `TORCS` simulator as **expert data** and use them to **train a simple `IL` network**. Because there is less expert data, **`Dagger` (Dataset Aggregation)** is introduced in the training method, so that the network can get the best results with the least data."
  - `2-1.` **Generate demonstration data** using the `IL` agent. And **store** them in the **_`expert pool`_**.
  - `2-2.` Meanwhile, pre-train the `DDPG` algorithm on the demonstration data. It is **`offline`, i.e. the agent trains solely without any interaction with the environment**.
  - `3-` After the **pre-training** is completed, the `DDPG` algorithm starts **exploration**. It stores the exploration data in the **_`ordinary pool`_**.
    - **Experiences are sampled from the `2` pools**.
    - > "We record the maximum value of **total reward `E`** when `IL` produced demonstration data for each round, and use it as the **threshold to adjust the `sampling ratio`** [..] when the algorithm reaches an **approximate expert level**, the value of the **`expert/ordinary` ratio** will gradually decrease."
- Results
  - > [**training time**] "Experimental results show that the `DDPG-IL` algorithm is **`3` times faster** than the ordinary `DDPG` and **`2` times faster** than the **single experience pool `DDPG` based on demonstration data**."
  - In addition, `DDPG-IL` shows the best performance.

</details>

---

**`"High-Speed Autonomous Drifting with Deep Reinforcement Learning"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2001.01377)]**
**[[:octocat:](https://github.com/caipeide/drift_drl)]**
**[[🎞️](https://sites.google.com/view/autonomous-drifting-with-drl/)]**
**[** :mortar_board: `Hong Kong University` **]**

- **[** _`generalization`, `action smoothing`, `ablation`, `SAC`, [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2001.01377).](../media/2020_cai_1.PNG "[Source](https://arxiv.org/abs/2001.01377).")  |
|:--:|
| *The goal is to control the vehicle to **follow a trajectory** at **high speed** (`>80 km/h`) and drift through manifold corners with **large side `slip angles`** (`>20°`), like a **professional racing driver**. The **slip angle `β`** is the angle between the **direction of the heading** and the **direction of speed vector**. The **desired heading angle** is determined by the **vector field guidance (`VFG`)**: it is close to the direction of the **reference trajectory** when the **lateral error** is small. [Source](https://arxiv.org/abs/2001.01377).* |

| ![[Source](https://arxiv.org/abs/2001.01377).](../media/2020_cai_2.PNG "[Source](https://arxiv.org/abs/2001.01377).")  |
|:--:|
| *Top-left: The **generalization** capability is tested by using cars with **different kinematics and dynamics**. Top-right: An **`action` smoothing** strategy is adopted for **stable control outputs**. Bottom: the `reward` function **penalized deviations** to **references `states`** in term of `distance`, `direction`, and `slip angle`. The **speed factor `v`** is used to stimulate the vehicle to **drive fast**: If `v` is smaller than `6 m/s`, the total reward is **decreased by half** as a **punishment**. [Source](https://arxiv.org/abs/2001.01377).* |

| ![[Source](https://sites.google.com/view/autonomous-drifting-with-drl/).](../media/2020_cai_1.gif "[Source](https://sites.google.com/view/autonomous-drifting-with-drl/).")  |
|:--:|
| *The proposed `SAC`-based approach as well as the **three baselines** can **follow the reference trajectory**. However, `SAC` achieves a **much higher average velocity (`80 km/h`)** than the baselines. In addition, it is shown that the **`action smoothing` strategy** can improve the final performance by comparing `SAC-WOS` and `SAC`. Despite the `action smoothing`, the steering angles of `DDPG` is also shaky. Have a look at the **`steering` gauges**! [Source](https://sites.google.com/view/autonomous-drifting-with-drl/).* |

Authors: Cai, P., Mei, X., Tai, L., Sun, Y., & Liu, M.

- Motivations:
  - `1-` Learning-based.
    - The **car dynamics** during **transient drift** (high `speed` > `80km/h` and `slipe angle` > `20°` as opposed to _steady-state_ drift) is too hard to **model** precisely. The authors claim it should rather be addressed by **model-free learning** methods.
  - `2-` **Generalization**.
    - The drift controller should generalize well on various **`road structures`**, **`tire friction`** and **`vehicle types`**.

- `MDP` formulation:
  - `state` (**`42`**-dimensional):
    - [_Close to `imitation learning`_] It is called **"error-based `state`"** since it describes **deviations** to the **referenced drift trajectories** (performed by a **experienced driver** with a `Logitech G920`).
    - These **deviations** relate to the `location`, `heading angle`, `velocity` and `slip angle`.
      - > [Similar to `D` in `PID`] "**Time derivatives** of the **error variables**, such as `d(ey)/d(t)`, are included to **provide temporal information** to the controller."
    - The `state` also contains the **last `steering` and `throttle` commands**. Probably enabling **consistent action selection** in the `action smoothing` mechanism.
  - `action`:
    - The `steering` is limited to a smaller range of [`−0.8`, `0.8`] instead of [`−1`, `1`] to **prevent rollover**.
    - > "Since the vehicle is expected to **drive at high speed**, we further **limit** the range of the `throttle` to **[`0.6`, `1`]** to **prevent slow driving** and **improve training efficiency**.
  - **`action` smoothing** against **shaky control output**.
    - > "We impose **continuity in the `action`**, by constraining the **change of output** with the **deployed action in the previous step**: `a[t]` = `K1`.`a_net[t]` + `K2`.`a[t-1]`.

- Algorithms:
  - `1-` `DQN` can only handle the **discrete `action` space**: it selects among `5`*`10`=`50` combinations, without the `action smoothing` strategy.
  - `2-` `DDPG` is difficult to converge due to the limited exploration ability caused by its **deterministic** character.
  - `3-` [Proposed] **Soft actor-critic (`SAC`)** offers a **better convergence ability** while avoiding the **high sample complexity**: Instead of only seeking to maximize the lifetime rewards, `SAC` seeks to also **maximize the entropy of the policy** (as a regularizer). This **encourages exploration**: the policy should act **as randomly as possible** [encourage uniform action probability] while being able to succeed at the task.
  - `4-` The `SAC-WOS` baseline does not have any **`action smoothing` mechanism** (`a[t]` = `K1`.`a_net[t]` + `K2`.`a[t-1]`) and suffers from **shaky behaviours**.

- **Curriculum learning**:
  - > "Map (`a`) is **relatively simple** and is used for the **first-stage training**, in which the vehicle learns some **basic driving skills** such as speeding up by applying large values of throttle and drifting through **some simple corners**. Maps (`b`-`f`) have different levels of difficulty with diverse corner shapes, which are used for further **training with the pre-trained weights** from map (`a`). The vehicle can **use the knowledge learned from map (`a`)** and **quickly adapt to these tougher maps**, to learn a more advanced drift technique."
  - _Is the replay buffer build from training with map (`a`) reused? What about the weighting parameter `α` of the entropy term `H` in the objective function?_

- **Robust `training`** for **generalization**:
  - The idea is to expose different `road structures` and `car models` during training, i.e. make the **`MDP` environment `stochastic`**.
  - > "At the start of each episode, the **`tire friction`** and `vehicle mass` are sampled from the range of [`3.0`, `4.0`] and [`1.7t`, `1.9t`] respectively."
  - > [Evaluation.] "Note that for each kind of vehicle, the **referenced drift trajectories** are different in order to **meet the respective physical dynamics**." [_How are they adjusted?_]
  - Benefit of `SAC`:
    - > [from [`BAIR` blog](https://bair.berkeley.edu/blog/2018/12/14/sac/)] "Due to **entropy maximization** at training time, the policy can **readily generalize** to these **perturbations** without any additional learning."

- **Ablation** study for the `state` space.
  - > "Can we also provide **less information** during the training and achieve **no degradation** in the final performance?"
  - Findings:
    - `1-` Necessity of **`slip angle`** information (in `state` and `reward`) during `training`.
      - > [Need for **`supervision` / `expert demonstration`**] "Generally, **accurate `slip angles`** from **expert drift trajectories** are indeed necessary in the training stage, which can improve the final performance and the training efficiency."
    - `2-` **Non-degraded performance** with a rough and easy-to-access **reference trajectory** during `testing`. Making it **less dependant** on **expert demonstraions**.

</details>

---

**`"Trajectory based lateral control: A Reinforcement Learning case study"`**

- **[** `2020` **]**
**[[:memo:](https://www.sciencedirect.com/science/article/pii/S0952197620301858)]**
**[[🎞️](https://storage.googleapis.com/jlrie-aia-rl-vmc/agent_training_porgression.mp4)]**
**[[🎞️](https://storage.googleapis.com/jlrie-aia-rl-vmc/agent_velocities.mp4)]**
**[[🎞️](https://storage.googleapis.com/jlrie-aia-rl-vmc/validation_videos.mp4)]**
**[** :car: `Jaguar Land Rover` **]**

- **[** _`generalization`, `sim2real`, `ablation`, `DDPG`, [`IPG CarMaker`](https://ipg-automotive.com/products-services/simulation-software/carmaker/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://www.sciencedirect.com/science/article/pii/S0952197620301858).](../media/2020_wasala_1.PNG "[Source](https://www.sciencedirect.com/science/article/pii/S0952197620301858).")  |
|:--:|
| *The task is to predict the `steering` commands to **follow the given trajectory** on a **race track**. Instead of **a `sum`**, a **`product` of three deviation terms** is proposed for the **multi-objective `reward`**. Not clear to me: which `WP` is considered for the `deviation` computation in the `reward`? [Source](https://www.sciencedirect.com/science/article/pii/S0952197620301858).* |

| ![[Source](https://www.sciencedirect.com/science/article/pii/S0952197620301858).](../media/2020_wasala_1.gif "[Source](https://www.sciencedirect.com/science/article/pii/S0952197620301858).")  |
|:--:|
| *Testing a vehicle with **different drive dynamics** on an **unseen track**. [Source](https://www.sciencedirect.com/science/article/pii/S0952197620301858).* |

Authors: Wasala, A., Byrne, D., Miesbauer, P., O’Hanlon, J., Heraty, P., & Barry, P.

- Motivations:
  - `1-` **Generalization** in simulation.
    - The learnt agent should be able to complete at `test`-time **unseen and more complex tracks** using different **unseen vehicle** models.
    - > "How can RL agents be trained to **handle diverse scenarios** and adapt to **different car models**?"
    - The **learnt policy** is tested on vehicles with different `dimensions`, `weights`, `power-trains`, `chassis`, `transmission` and `aerodynamics`.
  - `2-` **`sim-to-real`** transfer.
    - > "Is **training solely in a simulated environment** usable in a **real car without any additional training / fine-tuning** on the car itself?"
    - > "[Other works] Those that did implement **live vehicle testing**, required **further tuning/training** in order to make the jump from simulation and **were constrained to low speeds** between `10km∕h` and `40km∕h`."
  - `3-` **Ablation** study.
    - > "What are the **most useful (essential) `state`–space parameters**?"

- Task: predict the `steering` commands to **follow the given trajectory** on a **race track**.
  - > "[Need for learning-based approach.] These **traditional controllers [`MPC`, `PID`]** require **large amounts of tuning** in order to perform at an acceptable level for safe autonomous driving."
  - **Path-velocity decomposition**:
    - The longitudinal control (`acceleration` and `braking`) is handled by some **"baseline driver"**.
    - > "Our experiments show that trying to control **all three actuation signals** with a single network makes it **very difficult** for the agent to converge on an optimal policy."
    - > "[The **decomposition**] helped to improve **`credit assignment`**, a common problem in the field involving the agent trying to **assign what `action` is responsible for the `reward`** the agent is receiving."

- `MDP` formulation:
  - `state`:
    - **`10` waypoints** spaced evenly `5m` apart.
    - Several **ego vehicle parameters** such as the `wheel speed`, and `engine rpm`.
  - `discount factor` set to `1`.
  - Exploration and termination.
    - When the car **drives `off-road`** or when a high `yaw acceleration` is measured, the episode is **terminated**.
  - `reward`
    - Multi-objective: `efficiency`, `safety` and `comfort`.
    - Instead of **a `sum`**, here a **`product` of three deviation terms**: `speed` (ratio), `angle` (normalized ratio) and `distance` (normalized ratio).
  - Algorithm:
    - `DDPG` was preferred over `PPO` since it is `off-policy`: better **sampling efficiency**.
  - Simulator:
    - The **commercial vehicle dynamics simulation** platform [`IPG CarMaker`](https://ipg-automotive.com/products-services/simulation-software/carmaker/).

- **Ablation** study in the `state` space.
  - `1-` Parameters can be **overwritten** at **test time**.
    - During **inference**, `wheel speed` and `engine RPM` are replaced with `0.5` (the median of the normalized range [`0`, `1`]).
    - > "Surprisingly, the agent was still able to **drive successfully** with no evident side effects being displayed."
  - `2-` Parameters can be **removed** during **training**.
    - > "However, once these features [`wheel speed` and `engine RPM`] were removed, the agent was **no longer able to converge**."
    - > [Intuition - _I think there must be something else_] "Similar to **_training wheels_** on a bicycle, these features helped to **stabilize the training** and allow the agent to **see patterns**, that are otherwise more **difficult to detect**."

- Generalization.
  - `1-` Driving on **unseen and more complex `tracks`**.
    - The **stability** of the agent’s policy began to deteriorate as it was exposed to **increasingly complex `states` outside of its `training` scope**.
  - `2-` Driving at **higher speeds** (`120` − `200 km∕h`), while being trained on a top speed of `80 km∕h`.
    - > "Once we **re-scaled the normalization thresholds** of the `state`–space to match the **increased range of velocity**, the agent was able to handle even greater speeds."
    - > "**Re-normalizing** the `state` space to match the range of the **initial training `state` space** played a key role in the agent' ability to adapt to new and unseen challenges."
  - `3-` Driving with **unseen** and **varying car models**.
    - > "The generalizability across **multiple vehicle models** could be improved by adding a **vehicle _descriptor_** to the `state`–space and **training with a wider range** of vehicle models."

- Transfer to a **live test vehicle**.
  - Challenges:
    - `1-` Port the trained `Keras` model to `C++`, with **[frugally-deep](https://github.com/Dobiasd/frugally-deep)**.
    - `2-` Build the `state` from the AD stack.
      - > "`Engine rpm` was the only **element of the `state`–space we could not reproduce in the AD stack**, since we wanted our controller to only utilize information that any other controller had available. We replaced the `engine rpm` with the **median value** of the normalized range ([`0`, `1`]), i.e. `0.5`."
    - `3-` Deal with **latency** between `commands` and `actuation`.
      - _Actually a big topic!_
      - Addressed with **`action` holding**.
      - > [Also, about the **decision frequency**:] "The created trajectory in our **AD stack** was updated every **100 ms**, as opposed to the **`CarMaker` simulation** where it was updated each simulation step, i.e. **continuously**."
  - Findings:
    - > "[Unseen `state`s and different `transition` function] We found that moving from simulation to the real-world without **additional training** poses several problems. Firstly, our training simulation did not contain any **sloped roads**, weather disturbances (e.g. `wind`), or **inaccuracies of sensor measurements** used to represent the `state`–space."
  - > "While the **ride comfort** was not ready for a **production vehicle** yet, it achieved **speeds of over `60 km∕h`** while **staying within lane** and **taking turns**."

- I really like the **_`lessons learned`_** section.
  - `1-` Generalization.
    - > "We believe that the cause of the agents high `generalization` capabilities stems from **_how_ the agent was trained** [`state`, `reward`, `scenarios`], **as opposed to _what algorithm_** was used for learning."
    - > "**The `scenario` used for `training` the agent is _paramount_ to its `generalization` potential."**
  - `2-` Problem of **oscillation**.
    - > "The authors combat this by using an **`action` smoothing** method that **constrains maximum change allowed between `actions`**. This constraint helps reduce the oscillation of the agent and promotes a smoother control policy."
  - `3-` **Catastrophic forgetting**.
    - > "Seemingly **without explanation**, the agent would forget everything it had learned and **degraded into a policy** that would immediately drive the agent off the track."
    - Solution: greatly increase the **size of the replay buffer**.
      - This meant the agent was able to **sample** from a **much wider distribution of experiences** and still see examples of driving in poor conditions such as being off-centre.
      - A similar idea could be to **maintain two buffers**: a `safe` and a `non-safe`, as in [`(Baheri et al., 2019)`](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving).
  - `4-` **Buffer initialization**.
    - > "During our experiments, we noticed that **pre-populating the replay buffer**, before training **accelerated the convergence**."
  - `5-` **`Loss` in the `Q`-net** is **not converging**.
    - As noted in [this post](https://stats.stackexchange.com/questions/313876/loss-not-decreasing-but-performance-is-improving):
      - This is not unusual for `RL` and **does not indicate anything is wrong**.
      - `1)` As the agent **gets better** at playing, **estimating the reward** does get **more difficult** (because it's no longer always `0`).
      - `2)` As the **`reward` gets higher**, and the average **episode length** gets **longer**, the amount of **variance in the `reward`** can also get larger, so it's challenging even to prevent the loss from increasing.
      - `3)` A third factor is that the **constantly changing** d poses a **"moving-target"** problem for the `Q-network`.
    - > "While the `DNN`’s **`loss` plot** gives an indication of whether or not the agent **is learning**, it does not reflect the **quality of the policy** the agent has learned, i.e. an agent may **learn a bad policy well**."
  - `6-` **`action` repeats**.
    - The agent repeats the **same `action`** for a given number of consecutive steps **without revaluating the `state`–space**.
    - > "We could **accelerate the training** and address the **latency issues**."
    - _Is the agent aware of that? For instance by_ **_penalizing `action` changes_**_?_
  - `7-` **Poor reproducibility**.
  - Other **references for `RL debugging`**:
    - **[Brilliant notes!](https://github.com/williamFalcon/DeepRLHacks)** on [Nuts and Bolts of Deep RL](https://www.youtube.com/watch?v=8EcdaCk9KaQ).
    - [10 Steps and some tricks to set up neural reinforcement controllers](https://ml.informatik.uni-freiburg.de/former/_../media/publications/12riedmillertricks.pdf).
    - [RL Debugging and Diagnostics | Stanford CS229](https://www.youtube.com/watch?v=pLhPQynL0tY) - mostly the first `20min`.
    - [Deep Reinforcement Learning Doesn't Work Yet](https://www.alexirpan.com/2018/02/14/rl-hard.html).

</details>

---

**`"Reinforcement Learning based Control of Imitative Policies for Near-Accident Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2007.00178)]**
**[[🎞️](https://www.youtube.com/watch?v=envT7b5YRts)]**
**[[🎞️](https://www.youtube.com/watch?v=CY24zlC_HdI)]**
**[[🎞️](https://www.youtube.com/watch?v=IPa-cxcdT8U&t=1106s)]**
**[** :mortar_board: `Stanford` **]**
**[** :car: `Toyota Research Institute` **]**

- **[** _`hierarchical learning`, `RL+IL`, [`CARLO`](https://github.com/Stanford-ILIAD/CARLO), [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2007.00178).](../media/2020_cao_1.gif "[Source](https://arxiv.org/abs/2007.00178).")  |
|:--:|
| *Illustration of **rapid phase transitions**: when **small changes in the critical `states`** – the ones we see in near-accident scenarios – **require dramatically different `actions`** of the autonomous car to stay safe.  [Source](https://arxiv.org/abs/2007.00178).* |

| ![[Source](https://arxiv.org/abs/2007.00178).](../media/2020_cao_1.PNG "[Source](https://arxiv.org/abs/2007.00178).")  |
|:--:|
| *I must say I am a bit disappointed by the `collision rate` results. The authors mention **`safety`** a lot, but their approaches **crash every third trial** on the `unprotected turn`. The too-conservative agent `TIMID` gets zero collision. Is it then fair to claim **''Almost as Safe as Timid''**? In such cases, what could be needed is an **uncertainty-aware** agent, e.g. `POMDP` with **information gathering behaviours**.  [Source](https://arxiv.org/abs/2007.00178).* |

| ![[Source](https://arxiv.org/abs/2007.00178).](../media/2020_cao_2.PNG "[Source](https://arxiv.org/abs/2007.00178).")  |
|:--:|
| *One of the five scenarios: In this `unprotected left turn`, a truck **occludes** the oncoming ado car. Bottom left: The **two primitive policies (`aggressive` and `timid`)** are first learnt by `imitation`. Then they are used to train a **high-level** policies with `RL`, to select **at each timestep `ts`** (larger that primitive timestep) **which primitive to follow**. Bottom right: While `AGGRESSIVE` achieves higher `completion rates` for the **low time limits**, it cannot improve further with the increasing limit with collisions. [Source](https://arxiv.org/abs/2007.00178).* |

| ![[Source](https://arxiv.org/abs/2007.00178).](../media/2020_cao_3.PNG "[Source](https://arxiv.org/abs/2007.00178).")  |
|:--:|
| *About **phase transition**: `H-REIL` usually chooses the `timid` policy at the areas that have a **collision risk** while staying `aggressive` at other locations when it is safe to do so. Baselines: `π-agg`, resp. `π-agg`, has been trained only on **`aggressive`**, resp. `timid`, **rule-based demonstrations** with `IL`. `π-IL` was trained on the **mixture** of both. A pity that **no pure `RL` baseline** is presented. [Source](https://arxiv.org/abs/2007.00178).* |

Authors: Cao, Z., Bıyık, E., Wang, W. Z., Raventos, A., Gaidon, A., Rosman, G., & Sadigh, D.

- Motivation:
  - **Learn** (no `rule-based`) driving policies in **_near-accident_** scenarios, i.e. where quick reactions are required, being `efficient`, while `safe`.
  - Idea: **decompose** this complicated task into **two levels**.

- Motivations for a **hierarchical** structure:
  - `1-` The presence of **rapid phase transitions** makes it hard for `RL` and `IL` to capture the policy because **they learn a _smooth_ policy across `states`**.
    - > "**Phase transitions** in autonomous driving occur when **small changes in the critical `states`** – the ones we see in near-accident scenarios – **require dramatically different `actions`** of the autonomous car to stay safe."
    - Due to the **non-smooth `value` function**, an `action` taken in one state may not **generalize to nearby `states`**.
    - During training, the algorithms must be able to visit and **handle all the critical states individually**, which can be **computationally inefficient**.
    - _How to model the rapid phase transition?_
      - By **switching from one `driving mode` to another**.
      - > [The main idea here] "Our key insight is to **model phase transitions as _optimal switches_**, learned by `RL`, between different **`modes` of driving styles**, each learned through `IL`."
  - `2-` To achieve **full coverage**, `RL` needs to **explore the full environment** while `IL` requires a **large amount of expert demonstrations covering all `states`**.
    - Both are prohibitive since the **`state`-`action` space** in **driving** is continuous and **extremely large**.
      - One solution to **improve data-efficiency**: **Conditional** imitation learning (`CoIL`). it extends `IL` with **high-level commands** and learns a **separate `IL` model** for each command. High-level **commands are required at test time**, e.g., the direction at an intersection. Instead of depending on drivers to provide commands, the authors would like to **learn these optimal mode-switching policy**.
    - > "The **mode switching** can model **rapid phase transitions**. With the **reduced action space** and **fewer time steps**, the **high-level `RL`** can explore all **the `states`** efficiently to address **`state` coverage**."
    - > "`H-REIL` framework usually outperforms `IL` with a large margin, supporting the claim that in near-accident scenarios, training a **generalizable `IL`** policy **requires a lot of demonstrations**."
  - > `Hierarchical RL` enables **efficient exploration** for the higher level with a **reduced `action` space**, i.e. goal space, while **making `RL` in the lower level easier** with an **explicit** and **short-horizon** goal.

- Motivations for **combining `IL`+`RL`**, instead of single `HRL` or `CoIL`:
  - `1-` For the low-level policy:
    - Specifying `reward` functions for `RL` is hard.
    - > "We emphasize that `RL` would **not be a reasonable fit** for learning the low-level policies as it is **difficult to define the `reward` function**."
    - > "We employ `IL` to **learn low-level** policies `πi`, because each low-level policy **sticks to one driving style**, which behaves relatively **consistently across `states`** and requires **little rapid phase transitions**."
  - `2-` For the high-level policy:
    - > "`IL` does not fit to the **high-level policy**, because it is not natural for human drivers to accurately **demonstrate** **_how to switch driving modes_**."
    - > "`RL` is a better fit since we need to learn to maximize the `return` based on a `reward` that contains a **trade-off** between various terms, such as **efficiency and safety**. Furthermore, the action space is now reduced from a **continuous `space`** to a **finite discrete `space`** [the **conditional branches**]."
    - Denser `reward` signals: setting `ts > 1` **reduces the number of time steps in an episode** and makes the **collision penalty**, which appears at most once per episode, **less sparse**.

- **Hierarchical** `reinforcement` and `imitation` learning (`H-REIL`).
  - `1-` One **high-level** (meta-) policy, learned by `RL` that **switches between different `driving modes`**.
    - Decision: _which low-level policy to use?_
    - Goal: learn a **mode switching policy** that maximizes the `return` based on a **simpler pre-defined `reward` function**.
  - `2-` Multiple **low-level** policies `πi`, learned by `IL`: **one per `driving mode`**.
    - Imitate drivers with different characteristics, such as different `aggressiveness` levels.
    - They are "basic" and realize **relatively easier goals**.
    - > "The **low-level policy** for each `mode` can be efficiently learned with `IL` even with **only a few expert demonstrations**, since `IL` is now learning a **much simpler** and specific policy by sticking to one driving style with **little phase transition**."
- _How often are decisions taken?_
  - `500ms` = timestep of HL.
  - `100ms` = timestep of LL.
  - There is clearly a **trade-off**:
    - `1-` The high level should run at **low frequency** (**`action` abstraction**).
    - `2-` Not too _low_ since it should be able **to react and switch quickly**.
  - Maybe the `timid` `IL` primitive could **take over** before the end of the `500ms` if needed.
  - Or add some **`reward` regularization** to **discourage changing modes** as long as it is not very crucial.

- **Two `driving modes`**:
  - `1-` **`timid`**: drives in a **safe** way to **avoid all potential accidents**. It **slows down** whenever there is even a **slight risk** of an accident.
  - `2-` **`aggressive`**: favours `efficiency` over `safety`. _"It drives fast and frequently collides with the ado car."_
  - > "Since humans often do not optimize for other nuanced metrics, such as `comfort`, in a near-accident scenario and **the planning horizon of our high-level controller is extremely short**, there is a **limited amount of diversity** that different modes of driving would provide, which makes having **extra modes unrealistic and unnecessary** in our setting."
  - > "This intelligent **mode switching** enables `H-REIL` to drive reasonably under different situations: **slowly and cautiously under uncertainty**, and **fast when there is no potential risk.**"

- About the **low-level `IL`** task.
  - **`Conditional imitation`**:
    - All the policies **share the same feature extractor**.
    - Different **branches** split in later layers for `action` prediction, where each corresponds to one `mode`.
    - The **branch** is selected by **external input** from high-level `RL`.
  - Each scenario is run with the ego car following a **hand-coded** with two settings: `difficult` and `easy`.
    - _Are there demonstrations of collisions?_ _Are `IL` agents supposed to imitate that?_ _How can they learn to recover from near-accident situations?_
    - > "The `difficult` setting is described above where the ado car **acts carelessly or aggressively**, and is likely to collide with the ego car."
    - These demonstrations are used to learn the `aggressive` and `timid` primitive policies.
  - Trained with [`COiLTRAiNE`](https://github.com/felipecode/coiltraine).
  - Number of **episodes collected** per `mode`, for **imitation**:
    - `CARLO`: `80.000` (computationally lighter).
    - `CARLA`: `100` (it includes perception data).

- About the **high-level `RL`** task: **`POMDP`** formulation.
  - `reward`
    - > "It is now much easier to define a `reward` function because the ego car **already satisfies some properties** by following the policies learned from expert demonstrations. We do not need to worry about `jerk` [if `ts` large], because the **experts naturally give low-jerk demonstrations**."
    - `1-` **Efficiency** term: `Re` is **negative in every time step**, so that the agent will try to **reach its destination** as quickly as possible.
      - _Ok, but how can it scale to_ **_continuous_** _(non-episodic) scenarios, such as real-world driving?_
    - `2-` **Safety** term: `Rs` gets an **extremely large negative value** if a **collision** occurs.
  - `training` environment: `CARLO`.
  - _No detail about the `transition` model._

- About `observation` spaces, for **both tasks**:
  - In `CARLO`: `positions` and `speeds` of the **ego car** and the **ado car**, if **not occluded**, perturbed with **Gaussian noise**.
  - In `CARLA`: Same but with **front-view `image`**.
    - _How to process the image?_
      - Generate a **binary image** using an object detection model.
      - Only the **bounding boxes are coloured white**. It provides information of the **ado car** more **clearly** and **alleviates the environmental noise**.
  - _How can the agents be trained if the `state` space varies?_
  - _Are frames stacked, as represented on the figure?_ Yes, one of the authors told me `5` are used.

- About [`CARLO`](https://github.com/Stanford-ILIAD/CARLO) simulator to **train faster**.
  - `CARLO` stands for **_`CARLA` - Low Budget_**. It is less realistic but **computationally much lighter** than `CARLA`.
  - > "While `CARLO` does not provide realistic visualizations other than **two-dimensional diagrams**, it is useful for **developing control models** and **collecting large amounts of data**. Therefore, we use `CARLO` as a **simpler environment** where we assume perception is handled, and so we can **directly use the noisy measurements** of other vehicles’ `speeds` and `positions` (if not occluded) in addition to the `state` of the ego vehicle."

- Some concerns:
  - _What about the_ **_car dynamics_** _in `CARLO`?_
    - > [same `action` space] "For both `CARLO` and `CARLA`, the **control inputs** for the vehicles are `throttle`/`brake` and `steering`."
    - `CARLO` assumes **point-mass dynamics** models, while the model of physics engine of `CARLA` is much more complex with **non-linear dynamics**!
    - First, I thought agents were trained in `CARLO` and tested in `CARLA`. But the transfer is not possible because of **mismatch in `dynamics` and `state` spaces.
      - But apparently training and testing are performed **individually and separately** in both simulators. _Ok, but only one set of results is presented. I am confused._
  - **Distribution shift and overfitting**.
    - **Evaluation** is performed on scenarios used during `training`.
    - _Can the system address situations it has not been trained on?_
  - **Safety!**

</details>

---

**`"Safe Reinforcement Learning for Autonomous Lane Changing Using Set-Based Prediction"`**

- **[** `2020` **]**
**[[:memo:](https://mediatum.ub.tum.de/doc/1548735/256213.pdf)]**
**[** :mortar_board: `TU Munich` **]**

- **[** _`risk estimation`, `reachability analysis`, `action-masking`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://mediatum.ub.tum.de/doc/1548735/256213.pdf).](../media/2020_krasowski_1.PNG "[Source](https://mediatum.ub.tum.de/doc/1548735/256213.pdf).")  |
|:--:|
| *__`action` masking__ for **safety verification** using **set-based prediction** and sampling-based trajectory planning. Top-left: A **braking trajectory** with **maximum deceleration** is appended to the sampled trajectory. The ego vehicle never follows this `braking trajectory`, but it is utilized to **check** if the vehicle is in an **invariably safe `state`** at the end of its driving trajectory. [Source](https://mediatum.ub.tum.de/doc/1548735/256213.pdf).* |

Authors: Krasowski, H., Wang, X., & Althoff

- Previous work: [`"High-level Decision Making for Safe and Reasonable Autonomous Lane Changing using Reinforcement Learning"`](https://mediatum.ub.tum.de/doc/1454224/712763187208.pdf), (Mirchevska, Pek, Werling, Althoff, & Boedecker, 2018).

- Motivations:
  - Let a **safety layer** **guide the `exploration` process** to forbid (`mask`) **high-level `actions`** that might result in a collision and to **speed up training**.

- Why is it called "**_set_**-based prediction"?
  - Using **reachability analysis**, the **`set` of future occupancies** of each surrounding traffic participant and the ego car is computed.
  - > "If both **`occupancy sets` do not intersect** for all consecutive time intervals within a predefined time horizon and if the ego vehicle reaches an **invariably safe set**, a collision is impossible."
  - Two-step prediction:
    - `1-` The occupancies of the surrounding traffic participants are obtained by using **TUM's tool [`SPOT`](https://spot.in.tum.de)** = **_"Set-Based Prediction Of Traffic Participants"_**.
      - > [`action` space] "**High-level `actions`** for lane-changing decisions: `changing to the left lane`, `changing to the right lane`, `continuing in the current lane`, and staying in the current lane by activating a safe adaptive cruise control (`ACC`)."
      - `SPOT` considers the **physical limits** of surrounding traffic participants and **constraints** implied by traffic rules.
    - `2-` The **precise movement** is obtained by a **sampling-based** trajectory planner.

- Other approaches:
  - > "One approach is to execute the planned trajectories if they do not collide with a traffic participant
according to **its prediction**. The limitation is that collisions still happen if other traffic participants’ behaviour **deviates from their prediction**."
  - **Reachability analysis** verifies the safety of planned trajectories by computing **all possible future motions of obstacles** and checking whether they **intersect** with the occupancy of the ego vehicle.
    - > "Since computing the _exact_ reachable sets of **nonlinear systems** is impossible, reachable sets are **over-approximated** to ensure safety."

- _What if, after `masking`, all actions are verified as unsafe? Can safety be guaranteed?_
  - > "To guarantee safety, we added a **verified fail-safe planner**, which holds available a safe action that is activated when the agent fails to identify a safe action." [not clear to me]
  - Apparently, the lane is kept and an **`ACC` module** is used.

- `MDP` formulation: **Episodic** task.
  - > "We terminate an episode if the time horizon of the current traffic scenario is reached, the **goal area is reached**, or the ego vehicle collides with another vehicle."
  - Furthermore, the `distance to goal` is contained inside the `state`.
  - Not clear to me: _How can this work in long highway journeys? By setting successive goals? How can `state` values be consistent? Should not the task be `continuous` rather than `episodic`?_

- About safe `RL`.
  - Safe `RL` approaches are distinguished by approaches that:
    - `1-` Modify the **`optimization criterion`.**
    - `2-` Modify the **`exploration process`**."
  - > "By **modifying the `optimality objective`**, agents behave **more cautious** than those trained without a risk measure included in the objective; however, the **absence of unsafe behaviors cannot be proven**. In contrast, by verifying the safety of the `action` and **excluding possible unsafe `actions`**, we can ensure that the **exploration process is safe**."

- Using **real-world** highway dataset (`highD`).
  - > "We generated tasks by **removing a vehicle from the recorded data** and using its start and the final `state` as the initial `state` and the center of the goal region, respectively."
  - > [Evaluation] "We have to **differentiate** between collisions for which the ego vehicle is **responsible** and collisions that occur because **no interaction between traffic participants was considered** due to **prerecorded** data."

- Limitations.
  - `1-` Here, _safe_ `actions` are determined by **set-based prediction**, which considers **all possible motions** of traffic participants.
    - > "Due to the **computational overhead** for determining safe actions, the computation time for training safe agents is **`16` times higher** than for the non-safe agents. The average training step for safe agents takes `5.46s` and `0.112s` for non-safe agents."
    - > "This significant increase in the training time is mainly because instead of one trajectory for the selected action, **all possible trajectories are generated** and compared to the predicted occupancies of traffic participants."
  - `2-` Distribution shift.
    > "The agents **trained in `safe` mode** did not experience **dangerous situations** with high penalties during training and cannot solve them in the **`non-safe` test** setting. Thus, the **safety layer** is necessary during deployment to ensure safety."
  - `3-` The **interaction** between traffic participants is essential.
    > "Although the proposed approach **guarantees safety** in all scenarios, the agent drives **more cautiously** than normal drivers, especially in dense traffic."

</details>

---

**`"Safe Reinforcement Learning with Mixture Density Network: A Case Study in Autonomous Highway Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2007.01698)]**
**[** :mortar_board: `West Virginia University` **]**

- **[** _`risk estimation`, `collision buffer`, `multi-modal prediction`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2007.01698).](../media/2020_baheri_1.PNG "[Source](https://arxiv.org/abs/2007.01698).")  |
|:--:|
| *Multimodal future trajectory predictions are incorporated into the **`learning` phase** of `RL` algorithm as a **model lookahead**: If **one of the future states of one of the possible trajectories** leads to a collision, then a penalty will be assigned to the reward function to prevent collision and to reinforce to **remember unsafe states**. Otherwise, the `reward` term **penalizes deviations** from the `desired speed`, `lane position`, and `safe longitudinal distance` to the lead traffic vehicle. [Source](https://arxiv.org/abs/2007.01698).* |

Author: Baheri, A.

- Previous work: [`"Deep Q-Learning with Dynamically-Learned Safety Module: A Case Study in Autonomous Driving"`](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving) (Baheri et al., 2019), detailed on this page too.

- Motivations:
  - Same ideas as the previous work.
  - `1-` **Speed up** the learning phase.
  - `2-` Reduce collisions (**no safety guarantees** though).
- Ingredients:
  - "Safety" is _improved_ via **`reward` shaping**, i.e. modification of the **optimization criterion**, instead of **constraining exploration** (e.g. `action masking`, `action shielding`).
  - Two ways to classify a `state` as risky:
    - `1-` Heuristic (**rule-based**). From a minimum relative `gap` to a traffic vehicle **based on its relative `velocity`**.
    - `2-` Prediction (**learning-based**). Model lookaheads (prediction / rollouts) are performed to **assess the risk of a given `state`**.
    - > [_Why learnt?_] "Because **heuristic safety rules** are susceptible to deal with **unexpected behaviors** particularly in a highly changing environment". [_Well, here the scenarios are generated in a simulator, that is ... heuristic-based_]
  - Contrary to **`action masking`**, _"bad behaviours"_ are not discarded: they are stored in a **`collision`-buffer**, which is sampled during the `update` phase.

- How to predict **a set of possible trajectories**?
  - **Mixture density `RNN`** (`MD-RNN`).
  - It has been **offline** trained (supervised learning).
  - > "The central idea of a `MDN` is to predict an **entire probability distribution** of the output(s) instead of generating a **single prediction**. The `MD-RNN` outputs a `GMM` for multimodal future trajectory predictions that each mixture component describes a certain driving behavior."

</details>

---

**`"Reinforcement Learning with Uncertainty Estimation for Tactical Decision-Making in Intersections"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2006.09786)]**
**[** :mortar_board: `Chalmers University` **]**
**[** :car: `Volvo`, `Zenuity` **]**

- **[** _`uncertainty-aware`, `continuous episodes`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2006.09786).](../media/2020_hoel_3.PNG "[Source](https://arxiv.org/abs/2006.09786).")  |
|:--:|
| *__Confidence__ of the recommended `actions` in a `DQN` are estimated using an **`ensemble` method**. Situation that are **outside of the training distribution** and **events rarely seen** while training can be detected. Middle: The `reward` associated to **non-terminated `state`s** is function of the **`jerk`** (comfort) and **scaled** so that its **accumulation until `timeout`** reaches `-1` if no `jerk` is applied. [Source](https://arxiv.org/abs/2006.09786).* |

Authors: Hoel, C.-J., Tram, T., & Sjöberg, J.

- Previous work: ["Tactical Decision-Making in Autonomous Driving by Reinforcement Learning with Uncertainty Estimation"](https://arxiv.org/abs/2004.10439) (Hoel, Wolff, & Laine, 2020), detailed on this page too.

- Motivation:
  - Same idea as the previous work (ensemble `RPF` method used to **estimate the confidence** of the recommended `actions`) but addressing **intersections** instead of **highways** scenarios.

- Similar conclusions:
  - **Uncertainty** for situations that are **outside** of the training distribution can be detected, e.g. with the approaching vehicle driving **faster than during training**.
  - > "Importantly, the method also indicates high uncertainty for **rare events within the training distribution.**"

- Miscellaneous:
  - The threshold **`c-safe`** is still **manually selected**. Automatically updating its value during training would be beneficial.
  - `state` similar to ["Learning When to Drive in Intersections by Combining Reinforcement Learning and Model Predictive Control"](https://arxiv.org/abs/1908.00177), (Tram, Batkovic, Ali, & Sjöberg, 2019), detailed on this page too.
  - `action` space more **abstract**: {`take way`, `give way`, `follow car {1, ... , 4}`}, **relying on an `IDM`** for the implementation.
  - The **timestep** has been reduced **from `1s` to `0.04s`**. _Better to react!_

- Learning **interactions**:
  - A **`1d-CNN` structure** is used for to input about the surrounding (and **interchangeable**) vehicles.
  - > "Applying the **same weights** to the input that describes the **surrounding vehicles** results in a better performance."

- How to **learn `continuous` tasks** while being **trained on `episodic` ones** (with `timeout`)?
  - > "If the current `policy` of the agent decides to stop the ego vehicle, **an `episode` could continue forever**. Therefore, a `timeout` time is set to `τmax = 20 s`, at which the **episode terminates**. The **last experience** of such an episode is **not added to the replay memory**. This trick prevents the agent to learn that an **episode can end due to a `timeout`**, and makes it seem like an episode can **continue forever**, which is important, since the **terminating `state` due to the time limit** is not part of the `MDP`."
  - > "The `state` space, described above, did not provide any information on **_where in an episode_** the agent was at a given time step, e.g. if it was in the _beginning_ or _close to the end_. The reason for this choice was that the goal was to train an agent that performed well **in `highway` driving of _infinite_ length**. Therefore, the **`longitudinal position` was irrelevant**. However, at the end of a **successful episode**, the future discounted `return` was `0`. To **avoid that the agent learned this**, the **last experience was not stored in the experience replay memory**. Thereby, the agent was **tricked to believe that the episode continued forever**. [_(C. Hoel, Wolff, & Laine, 2018)_]"

</details>

---

**`"Development of A Stochastic Traffic Environment with Generative Time-Series Models for Improving Generalization Capabilities of Autonomous Driving Agents"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/ftp/arxiv/papers/2006/2006.05821.pdf)]**
**[** :mortar_board: `Istanbul Technical University` **]**

- **[** _`generalisation`, `driver model`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/ftp/arxiv/papers/2006/2006.05821.pdf).](../media/2020_ozturk_1.PNG "[Source](https://arxiv.org/ftp/arxiv/papers/2006/2006.05821.pdf).")  |
|:--:|
| *The **trajectory generator** uses the `Social-GAN` architecture with a **`0.8s` horizon**. Bottom-right: `state` vector of the agent trained with **`Rainbow-DQN`**. Bottom-right: `reward` function: **`hard crash`** refers to **direct collisions** with other vehicles whereas the **`soft crash`** represents **dangerous approaches** (no precise detail). [Source](https://arxiv.org/ftp/arxiv/papers/2006/2006.05821.pdf).* |

Authors: Ozturk, A., Gunel, M. B., Dal, M., Yavas, U., & Ure, N. K.

- Previous work: _"Automated Lane Change Decision Making using Deep Reinforcement Learning in Dynamic and Uncertain Highway Environment"_ [(Alizadeh et al., 2019)](https://arxiv.org/abs/1909.11538)
- Motivation:
  - Increase **generalization capabilities** of a `RL` agent by training it in a **non-deterministic** and **data-driven** traffic simulator.
    - > "Most existing work assume that **surrounding vehicles employ rule-based decision-making** algorithms such as `MOBIL` and `IDM`. Hence the traffic surrounding the ego vehicle always follow **smooth and meaningful trajectories**, which does not reflect the **real-world traffic** where surrounding vehicles driven by humans mostly execute **erroneous manoeuvres** and **hesitate during lane changes**."
    - > "In this work, we develop a **data driven traffic simulator** by training a **generative adversarial network (`GAN`)** on real life **trajectory** data. The simulator generates **randomized trajectories** that resembles real life traffic **interactions** between vehicles, which enables training the `RL` agent on much **richer and realistic** scenarios."
- About `GAN` for **trajectory generations**:
  - > "The **`generator`** takes the real vehicle trajectories and generate **new trajectories**. The **`discriminator`** classifies the generated trajectories as real or fake."
  - Two ways to adapt `GAN` architecture to **time-series** data:
    - `1-` **Convert the time series** data into a **2D array** and then perform **convolution** on it.
    - `2-` [done here] Develop a **sequence to sequence** encoder and decoder `LSTM` network.
  - `NGSIM` seems to have been used for training. Not detailed.
- To generate **interaction**-aware trajectories:
  - Based on [`Social-GAN`](https://arxiv.org/abs/1803.10892).
    - > "A **social pooling** is introduced where a **`LSTM` Encoder** encodes all the vehicles' position in a **relative** manner to the rest of the vehicles then a **max pooling** operation is performed at the hidden states of the encoder; arriving at a **socially aware module**."
  - Alternatives could be to include **graph-based** or **convolution-based** models to extract **interaction** models.

</details>

---

**`"From Simulation to Real World Maneuver Execution using Deep Reinforcement Learning"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2005.07023)]**
**[[🎞️](https://drive.google.com/file/d/1X9Sy2Mru_SfWyid3-Va8Ftm1pXfk5w3o/view)]**
**[** :mortar_board: `University of Parma` **]**
**[** :car: `VisLab` **]**

- **[** _`sim2real`, `noise injection`, `train-validation-test`, `D-A3C`, `conditional RL`, `action repeat`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2005.07023).](../media/2020_capasso_2.PNG "[Source](https://arxiv.org/abs/2005.07023).")  |
|:--:|
| *Environments for `training` (top) differ from those used for `validation` and `testing` (bottom): **Multi-environment System** consists in **four `training` roundabouts** in which vehicles are **trained simultaneously**, and a **`validation` environment** used to **select the best network parameters** based on the results obtained on such scenario. The **generalization performance** is eventually measured on a separate `test` environment. [Source](https://arxiv.org/abs/2005.07023).* |

| ![[Source](https://arxiv.org/abs/2005.07023).](../media/2020_capasso_1.PNG "[Source](https://arxiv.org/abs/2005.07023).")  |
|:--:|
| *A sort of **conditional-learning** methods: an **`aggressiveness` level** is set as an input in the `state` and considered during the `reward` computation. More precisely: `α = (1−aggressiveness)`. During the `training` phase, `aggressiveness` assumes a random value from `0` to `1` kept fixed for the **whole episode**. Higher values of `aggressiveness` should encourage the actor to **increase the impatience**; consequently, **dangerous actions will be less penalized**. The authors note that values of `aggressiveness` sampled outside the training interval `[0, 1]` produce consistent consequences to the agent conduct, intensifying its behaviour even further. The **non-visual channel** can also be used to specify the **desired cruising speed**. [Source1](https://arxiv.org/abs/2005.07023) [Source2]((https://arxiv.org/abs/1903.01365)).* |

| ![[Source]((https://drive.google.com/file/d/1iGc820O_qeBSrWHbwhPTiTyby_HupM-_/view)).](../media/2020_capasso_2.gif "[Source]((https://drive.google.com/file/d/1iGc820O_qeBSrWHbwhPTiTyby_HupM-_/view)).")  |
|:--:|
| *The `active` agent decides **high-level actions**, i.e. when to enter the roundabout. It should interact with `passive` agents, that have been collectively trained in a **multi-agent fashion** and decide lower-level `speed`/`acceleration` actions. [Source]((https://drive.google.com/file/d/1iGc820O_qeBSrWHbwhPTiTyby_HupM-_/view)).* |

| ![[Source]((https://drive.google.com/file/d/1X9Sy2Mru_SfWyid3-Va8Ftm1pXfk5w3o/view)).](../media/2020_capasso_1.gif "[Source]((https://drive.google.com/file/d/1X9Sy2Mru_SfWyid3-Va8Ftm1pXfk5w3o/view)).")  |
|:--:|
| *[Source]((https://drive.google.com/file/d/1X9Sy2Mru_SfWyid3-Va8Ftm1pXfk5w3o/view)).* |

Authors: Capasso, A. P., Bacchiani, G., & Broggi, A.

- Motivations:
  - `1-` Increase **robustness**. In particular **generalize** to **unseen scenarios**.
    - This relates to the problem of **overfitting** for `RL` agents.
    - > "Techniques like **_random starts_** and **_sticky actions_** are often **un-useful to avoid overfitting**."
  - `2-` **Reduce the gap** between **synthetic** and **real data**.
    - Ambition is to deploy a system in the **real world** even if it was **fully trained in simulation**.
  - Ingredients:
    - `1-` **Multiple `training` environments**.
    - `2-` Use of separated **`validation`** environment to select the best hyper-parameters.
    - `3-` **Noise injection** to increase robustness.
- Previous works:
  - `1` [Intelligent Roundabout Insertion using Deep Reinforcement Learning](https://arxiv.org/abs/2001.00786) [[🎞️](https://drive.google.com/file/d/1iGc820O_qeBSrWHbwhPTiTyby_HupM-_/view)] about the need for a **learning-based** approach.
    - > "The results show that a **rule-based** (`TTC-`based) approach could lead to **long waits** since its **lack of negotiation and interaction capabilities** brings the agent to perform the entry only when the roundabout is **completely free**".
  - `2` [Microscopic Traffic Simulation by Cooperative Multi-agent Deep Reinforcement Learning](https://arxiv.org/abs/1903.01365) about **cooperative multi-agent**:
    - > "Agents are **collectively trained** in a **multi-agent fashion** so that **cooperative behaviors** can emerge, gradually inducing agents to learn to **interact** with each other".
    - As opposed to **rule-based** behaviours for `passive` actors **hard coded** in the simulator (`SUMO`, `CARLA`).
    - > "We think that this **multi-agent** learning setting is captivating for many applications that require a **simulation environment with intelligent agents**, because it **learns the joint interactive behavior** eliminating the need for hand-designed behavioral rules."
- Two types of `agent`, `action`, and `reward`.
  - `1-` **Active** agents learn to **enter** a roundabout.
    - The **length of the episode does not change** since it ends once the insertion in the roundabout is completed.
    - **High-level `action`** in {`Permitted`, `Not Permitted`, `Caution`}
  - `2-` **Passive** agents learn to **drive** in the roundabout.
    - **Low-level `action`** in {`accelerate` (`1m/s2`), `keep the same speed`, `brake` (`−2m/s2`).}
    - The **length of the episode changes!**
      - > "We multiply the `reward` by a **normalization factor**, that is the ratio between the **path length** in which the rewards are computed, and the **longest path** measured among all the training scenarios"
  - They interact with each other.
    - The **density** of passive agents on the roundabout can be set in {`low`, `medium`, `high`}.
  - The abstraction levels differ: _What are the timesteps used?_ _What about action holding?_
- About the **hybrid** `state`:
  - **Visual** channel:
    - `4` images `84`x`84`, corresponding to a square area of **`50`x`50` meters around the agent**:
      - **Obstacles**.
      - **Ego-Path** to follow.
      - **Navigable space**.
      - **Stop line**: the position where the agent should stop if the entry cannot be made safely.
  - **Non-visual** channel:
    - Measurements:
      - **Agent speed**.
      - **Last action** performed by the vehicle.
    - **Goal specification**:
      - > "We coupled the visual input with some parameters whose values **influence the agent policy**, inducing different and **tuneable behaviors**: used **to define the goal** for the agent."
      - **Target speed**.
      - **`Aggressiveness`** in the manoeuvre execution:
        - A parameter fed to the net, which is taken into account in the **reward computation**.
        - I previous works, the authors tuned this `aggressiveness level` as the `elapsed time ratio`: _time and distance left_.
      - _I see that as a_ **_conditional_**_-learning technique: one can_ **_set the level of aggressiveness of the agent_** _during testing_.
  - _In previous works,_ _**frames were stacked**_, _so that speed can be inferred._
- About **robustness** via **noise injection**:
  - **Perception noise**: `position`, `size` and `heading` as well as errors in the detection of passive vehicles _(probably `false positive` and `false negative`)_.
  - **Localisation noise**: generate a **new path** of the active vehicle **perturbing the original** one using Cubic Bézier curves.
- **Multi-env** and `train-validate-test`:
  - > "**Multi-environment System** consists in **four `training` roundabouts** in which vehicles are **trained simultaneously**, and a **`validation` environment** used to **select the best network parameters** based on the results obtained on such scenario."
  - The **generalization performance** is eventually measured on a further `test` environment, which does not have any active role during the training phase.
- About the simulator.
  - **Synthetic representations** of real scenarios are built with the **[Cairo](https://en.wikipedia.org/wiki/Cairo_(graphics))** graphic library.
- About the `RL` algorithm.
  - **Asynchronous** Advantage Actor Critic (`A3C`):
    - Reducing the **correlation between experiences**, by collecting experiences from agents running in **parallel environment instances**. An alternative to the commonly used **experience replay**.
    - This parallelization also increases the **time and memory efficiency**.
  - Delayed A3C (`D-A3C`):
    - Keep **parallelism**: different environment instances.
    - Make **interaction** possible: **several agents** in each environment, so that they can **sense each other**.
    - Reduce the **synchronization burden** of `A3C`:
    - > "In `D-A3C`, the system collects the contributions of each **asynchronous learner** during the **whole episode**, sending the updates to the global network **only at the end of the episode**, while in the classic `A3C` this exchange is performed **at fixed time intervals**".
- About **frame-skipping technique** and **action repeat** (previous work).
  - > "We tested both with and without **action repeat of `4`**, that is **repeating the last action** for the **following `4` frames** (repeated actions are not used for computing the updates). It has been proved that in some cases **action repeat** improves learning by increasing the capability to **learn associations between temporally distant** (`state`, `action`) pairs, giving to `action`s **more time to affect** the `state`."
  - However, the use of **repeated actions** brings a drawback, that is to **diminish the rate** at which agents interact with the environment.

</details>

---

**`"Delay-Aware Multi-Agent Reinforcement Learning"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2005.05441)]**
**[[:octocat:](https://github.com/baimingc/damarl)]**
**[** :mortar_board: `Carnegie Mellon` **]**

- **[** _`delay aware MDP`, [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2005.05441).](../media/2020_chen_3.PNG "[Source](https://arxiv.org/abs/2005.05441).")  |
|:--:|
| *Real cars exhibit `action` delay. Standard delay-free `MDP`s can be augmented to **Delay-Aware `MDP` (`DA-MDP`)** by enriching the `observation` vector with the **sequence of previous `action`**. [Source1](https://arxiv.org/abs/2005.05441) [Source2](https://arxiv.org/abs/2005.05440).* |

| ![[Source](https://arxiv.org/abs/2005.05441).](../media/2020_chen_4.PNG "[Source](https://arxiv.org/abs/2005.05441).")  |
|:--:|
| *Combining **multi-agent** and **delay-awareness**. In the left-turn scenario, `agent`s decide the longitudinal **acceleration** based on the `observation` the **position** and **velocity** of other vehicles. They are positively rewarded if **all of them successfully finish** the left turn and penalized if any collision happens. Another cooperative task is tested: driving out of the parking lot. [Source](https://arxiv.org/abs/2005.05441).* |

Authors: Chen, B., Xu, M., Liu, Z., Li, L., & Zhao, D.

- Motivations:
  - `1-` Deal with `observation` **delays** and `action` **delays** in the `environment`-`agent` interactions of **`MDP`s**.
    - It is hard to **transfer** a policy learnt with **standard delay-free MDPs** to real-world cars because of **actuator** delays.
      - > "Most `DRL` algorithms are evaluated in turn-based simulators like `Gym` and `MuJoCo`, where the `observation`, `action` selection and actuation of the agent are **assumed to be instantaneous**."
    - Ignoring the delay of agents violates the **Markov property** and results in `POMDP`s, with **historical actions as hidden states**.
      - To retrieve the **Markov property**, a **Delay-Aware `MDP` (`DA-MDP`)** formulation is proposed here.
  - `2-` One option would be to go `model-based`, i.e. learning the **transition dynamic model**. Here the authors prefer to stay `model-free`.
    - > "The **control** community has proposed several methods to address the delay problem, such as using **Smith predictor** [24], [25], **Artstein reduction** [26], [27], **finite spectrum assignment** [28], [29], and **H∞ controller**."
  - `3-` Apply to **multi-agent** problems, using the **Markov game (`MG`)** formulation.
    - > "**Markov game** is a **multi-agent** extension of `MDP` with **partially observable** environments.
    - Application: vehicles trying to **cooperate at an unsignalized intersection**.
- About delays:
  - There exists **two** kinds:
    - > "For simplicity, we will focus on the **`action` delay** in this paper, and the algorithm and conclusions should be able to generalize to systems with **`observation` delays**."
  - > "The **delay** of a vehicle mainly includes:"
    - **Actuator delay**.
      - > "The actuator delay for vehicle **powertrain system** and **hydraulic brake** system is usually between **`0.3` and `0.6` seconds**."
    - **Sensor delay**.
      - > "The delay of sensors (cameras, LIDARs, radars, GPS, etc) is usually between `0.1` and `0.3` seconds."
    - **Time for decision making**.
    - **Communication delay**, in `V2V`.
- > "Consider a velocity of `10 m/s`, the `0.8` seconds delay could cause a **position error of `8 m`**, which injects **huge uncertainty and bias** to the state-understanding of the agents."
- Main idea: **`observation` augmentation with `action` sequence buffer** to retrieve the **Markov property**.
  - The authors show that a Markov game (`MG`) with **multi-step action delays** can be converted to a **regular** `MG` by **state augmentation** (delay aware `MG` = `DA-MG`).
    - Proof by comparing their corresponding `Markov Reward Processes` (`MRP`s).
    - Consequence: instead of solving `MG`s with delays, we can alternatively **solve the corresponding `DA-MGs` directly with `DRL`**.
  - The **input** of the policy now consists of:
    - The current information of the environment. E.g. `speeds` and `positions`.
    - The **planned action sequence** of **length `k`** that will be executed from.
  - The agents **interact with the environment not directly** but through an **action buffer**.
    - The **state vector** is augmented with an **action sequence** being executed in the next `k` steps where `k` `∈` `N` is the **delay duration**.
    - The dimension of `state` space increases, since `X` becomes `S` × `Ak`.
    - `a(t)` is the action **taken** at time `t` but **executed** at time `t + k` due to the `k`-step action delay.
      - Difference between _select_ an action (done by the `agent`) and _execute_ an `action` (done by the `environment`).
- About **delay sensitivity**:
  - Trade-off between **_delay-unawareness_** and **_complexity_** for the learning algorithm.
  - > "When the **delay is small** (here less than `0.2s`), the effect of **expanding state-space** on training is more severe than the **model error** introduced by **delay-unawareness**".
- Papers about action delay in `MDP`s:
  - [Learning and planning in environments with delayed feedback](https://www.researchgate.net/publication/227301866_Learning_and_planning_in_environments_with_delayed_feedback) (Walsh, Nouri, Li & Littman, 2008) -> `model-based` predictions.
  - [At human speed: Deep reinforcement learning with action delay](https://arxiv.org/pdf/1810.07286.pdf) (Firoiu, Ju, & Tenenbaum, 2018) -> `model-based` predictions. The delayed system was reformulated as an **augmented MDP problem**.
  - [Real-time reinforcement learning](https://arxiv.org/pdf/1911.04448.pdf) (Ramstedt & Pal, 2019) -> solve the **`1`-step** delayed problem.
- About multi-agent `RL`:
  - > "The simplest way is to directly train each agent with **single-agent `RL`** algorithms. However, this approach will introduce the **non-stationary** issue."
  - **Centralized training** and **decentralized execution**.
    - Centralized `Q-function`s: The **centralized critic** conditions on **global information** (global `state` representation and the `actions` of all agents).
      - > "The non-stationary problem is alleviated by **centralized training** since the transition distribution of the environment is **stationary when knowing all agent actions**."
    - Decentralized `policy` for each agent: The **decentralized actor** conditions only on **private observation** to avoid the need for a **centralized controller** during execution.

</details>

---

**`"Offline Reinforcement Learning: Tutorial, Review, and Perspectives on Open Problems"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2005.01643)]**
**[** :mortar_board: `UC Berkeley` **]**

- **[** _`offline RL`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2005.01643).](../media/2020_levine_1.PNG "[Source](https://arxiv.org/abs/2005.01643).")  |
|:--:|
| *__Offline__ reinforcement learning algorithms are reinforcement learning algorithms that **utilize previously collected data**, without additional **online** data collection. The term ''`fully off-policy`'' is sometimes used to indicate that **no additional online data collection** is performed. [Source](https://arxiv.org/abs/2005.01643).* |

Authors: Levine, S., Kumar, A., Tucker, G., & Fu, J.

Note: This **theoretical tutorial** could also have been part of sections on `model-based RL` and `imitation learning`.

- Motivations:
  - Make `RL` **data-driven**, i.e. forget about _exploration_ and _interactions_ and utilize **only previously collected offline** data.
    - > "**Fully offline** `RL` framework is enormous: in the same way that **supervised machine learning** methods have enabled data to be turned into generalizable and powerful pattern recognizers (e.g., image classifiers, speech recognition engines, etc.), offline `RL` methods equipped with powerful function approximation may enable data to be turned into generalizable and powerful decision making engines, effectively allowing anyone **with a large enough dataset** to turn this dataset into a policy that can optimize a desired utility criterion."
  - Enable the use of `RL` for **safety** critical applications.
    - > "In many settings, the **online interaction** is impractical, either because **data collection is expensive** (e.g., in robotics, educational agents, or healthcare) and **dangerous** (e.g., in **_autonomous driving_**, or healthcare)."
    - This could also remove the need for **"simulation to real-world transfer" (`sim-2-real`)**, which is difficult:
    - > "If it was possible to **simply train policies with previously collected data**, it would likely be unnecessary in many cases to manually **design high-fidelity simulators** for simulation-to-real-world transfer."
- About the "**data**-driven" aspect.
  - Because formulation resembles the standard **_supervised_ learning** problem statement, technics / **lesson-learnt** from supervised learning could be considered.
  - > "Much of the amazing practical progress that we have witnessed over the past decade has arguably been driven just as much **by advances in datasets as by advances in methods**. In real-world applications, **collecting large, diverse, representative, and well-labeled datasets** is often **far more important than utilizing the most advanced methods.**" _I agree!_
  - Both [`RobotCar`](https://robotcar-dataset.robots.ox.ac.uk/) and [`BDD-100K`](https://bair.berkeley.edu/blog/2018/05/30/bdd/) are cited as **large video datasets** containing thousands of hours of real-life driving activity.
- One major **challenge** for `offline RL` formulation:
  - > "The fundamental challenge with making such **counterfactual queries** _[given data that resulted from a given set of decisions, infer the consequence of a different set of decisions]_ is **distributional shift**: while our function approximator (policy, value function, or model) might be trained under one distribution, it will be **evaluated on a different distribution**, due both to the change in visited states for the new policy and, more subtly, by the act of maximizing the expected return."
  - This makes **`offline RL`** differ from Supervised learning methods which are designed around the **assumption that the data is independent and identically distributed** (`i.i.d.`).
- About applications to `AD`:
  - **`offline RL`** is potentially a promising tool for enabling **safe and effective** learning in autonomous driving.
    - > "**Model-based `RL`** methods that employ constraints to keep the agent close to the training data for the model, so as to **avoid out-of-distribution inputs** as discussed in Section `5`, can effectively provide elements of **imitation learning** when training on driving demonstration data.
  - The work of (Rhinehart, McAllister & Levine, 2018) [`"Deep Imitative Models for Flexible Inference, Planning, and Control"`](https://arxiv.org/abs/1810.06544v4), is mentioned:
    - It tries to combine the benefits of **`imitation learning`** (**`IL`**) and **`goal-directed planning`** such as `model-based RL` (**`MBRL`**).
    - > "Indeed, with the widespread **availability of high-quality demonstration data**, it is likely that effective methods for **offline `RL`** in the field of autonomous driving will, explicitly or implicitly, **combine elements of `imitation learning` and `RL`**.
  
</details>

---

**`"Deep Reinforcement Learning for Intelligent Transportation Systems: A Survey"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2005.00935)]**
**[** :mortar_board: `University of South Florida` **]**

- **[** _`literature review`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2005.00935).](../media/2020_haydari_1.PNG "[Source](https://arxiv.org/abs/2005.00935).")  |
|:--:|
| *The review focuses on **traffic signal control** (`TSC`) use-cases. Some `AD` applications are nonetheless shortly mentioned. [Source](https://arxiv.org/abs/2005.00935).* |

Authors: Haydari, A., & Yilmaz, Y.

</details>

---

**`"Tactical Decision-Making in Autonomous Driving by Reinforcement Learning with Uncertainty Estimation"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2004.10439)]**
**[[:octocat:](https://github.com/carljohanhoel/BayesianRLForAutonomousDriving)]**
**[[🎞️](https://github.com/carljohanhoel/BayesianRLForAutonomousDriving)]**
**[** :mortar_board: `Chalmers University` **]**
**[** :car: `Volvo` **]**

- **[** _`ensemble`, `bayesian RL`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2004.10439).](../media/2020_hoel_1.PNG "[Source](https://arxiv.org/abs/2004.10439).")  |
|:--:|
| *The main idea is to use an **ensemble of neural networks with additional randomized prior functions** (`RPF`) to **estimate the uncertainty of decisions**. Each member estimates `Q(s, a)` in a **sum `f + βp`**. Note that the **prior `p` nets** are initialized with random parameters `θˆk` that are **kept fixed**. [Source](https://arxiv.org/abs/2004.10439).* |

| ![[Source](https://arxiv.org/abs/2004.10439).](../media/2020_hoel_2.PNG "[Source](https://arxiv.org/abs/2004.10439).")  |
|:--:|
| *Bottom: example of **situation outside of the training distribution**. Before seeing the other, the policy chooses to **maintain its current speed**. As soon as the stopped vehicle is seen, the **uncertainty `cv` becomes higher that the safety threshold**. The agent chooses then the **fallback action `brake hard`** early enough and manages to avoid a collision. The baseline DQN agent also brakes when it approaches the stopped vehicle, but **too late**. [Source](https://arxiv.org/abs/2004.10439).* |

Authors: Hoel, C.-J., Wolff, K., & Laine, L.

- Motivations:
  - `1-` Estimate an **uncertainty** for each **(`s`, `a`) pair** when computing the `Q(s,a)`, i.e. express some **confidence** about the decision in `Q`-based algorithms.
  - `2-` Use this metric together with some **safety criterion** to **detect situations** are **outside of the training distribution**.
    - Simple `DQN` can **cause collisions** if the confidence of the agent is not considered:
    - > "A fundamental problem with these methods is that no matter what situation the agents are facing, they **will always output a decision**, with **no information on the uncertainty** of the decision or if the agent has **experienced anything similar** during its training. If, for example, an agent that was trained for a one-way highway scenario would be deployed in a scenario with **oncoming traffic**, it would still output a decision, **without any warning**."
    - > "The DQN algorithm returns a **`maximum-likelihood` estimate** of the `Q-values`. But gives no information about the uncertainty of the estimation. Therefore **collisions occur in unknown situations**".
  - `3-` And also leverage this **uncertainty estimation** to better train and **transfer** to real-world.
    - > "The uncertainty information could be used to **guide the training** to situations that the agent is **currently not confident about**, which could improve the **sample efficiency** and broaden the distribution of situations that the agent can handle."
    - > If an agent is trained in a **simulated environment** and **later deployed in real traffic**, the uncertainty information could be used to detect situations that need to be **added to the simulated world**.

- Previous works:
  - [Automated Speed and Lane Change Decision Making using Deep Reinforcement Learning](https://arxiv.org/abs/1803.10056) (Hoel, Wolff, & Laine, 2018).
  - [Combining Planning and Deep Reinforcement Learning in Tactical Decision Making for Autonomous Driving](https://arxiv.org/abs/1905.02680) (Hoel, Driggs-Campbell, Wolff, Laine, & Kochenderfer, 2019) analysed in [my `IV19` report](https://github.com/chauvinSimon/IV19).
  - [Tactical decision-making for autonomous driving: A reinforcement learning approach](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf). Hoel's 2019 thesis.
- How to estimate uncertainty:
  - `1-` **Statistical bootstrapping (sampling)**.
    - The **risk** of an `action` is represented as the **variance of the return** (`Q(s, a)`) when taking that action. The **variance** is estimated using an **ensemble of models**:
    - > "An **ensemble of models** is trained on **different subsets** of the available data and the **distribution** that is given by the ensemble is used to **approximate the uncertainty**".
      - Issue: "No mechanism for uncertainty that **does not come from the observed data**".
  - `2-` **Ensemble-`RPF`**.
    - Same idea, but a **randomized untrainable prior function** (`RPF`) is added to each **ensemble member**.
      - **_"untrainable"_**: The `p` nets are initialized with random parameters `θˆk` that are kept fixed.
    - This is inspired by the work of DeepMind: [Randomized prior functions for deep reinforcement learning](https://arxiv.org/pdf/1806.03335.pdf) (Osband, Aslanides, & Cassirer, 2018).
    - Efficient parallelization is required since `K` nets need to be trained instead of one.
- About the `RPF`:
  - Each of the **`K` ensemble members** estimates `Q(s, a)` in a **sum `f + βp`**, where `β` balances the importance of the **prior function**.
  - Training (_generate_ + _explore_): **One member is sampled**. It is used to generate an experience `(s, a, r, s')` that is **added to each ensemble buffer** with probability `p-add`.
  - Training (_evaluate_ + _improve_): A **minibatch `M`** of experiences is sampled from **each ensemble buffer** and the trainable network parameters of the corresponding ensemble member are updated.
- About the **_"coefficient of variation"_ `cv`(`s`, `a`)**:
  - It is used to estimate the **agent's uncertainty** of taking different actions from a given state.
  - It represents the **relative standard deviations**, which is defined as the **ratio** of the **`standard deviation`** to the **`mean`**.
  - It indicates **_how far `(s, a)` is from the training distribution_**.
- At inference (during testing):
  - `action`s with a **level of uncertainty `cv(s, a)`** that exceeds a **pre-defined threshold** are **prohibited**.
  - If no `action` fulfils the criteria, a **fallback action `a-safe`** is used.
  - Otherwise, the selection is done **maximizing the mean `Q-value`**.
  - The `DQN`-baseline, `SUMO`-baseline and proposed **`DQN`-`ensemble-RPF`** are tested on **scenarios outside** of the **training distributions**.
    - > "The `ensemble RPF` method both indicates a **high uncertainty** and **chooses safe actions**, whereas the `DQN` agent causes collisions."
- About the `MDP`:
  - `state`: relative positions and speeds.
  - `action`:
    - longitudinal: {`-4`, `−1`, `0`, `1`} `m/s2`.
    - lateral: {`stay in lane`, `change left`, `change right`}.
    - The **fallback action `a-safe`** is set to `stay in lane` and apply `−4 m/s2`
  - `time`:
    - Simulation timestep `∆t` = `1s`. _Ok, they want high-level decision. Many things can happen within `1s` though! How can it react properly?_
    - A lane change takes `4s` to complete. Once initiated, it cannot be aborted. _I see that as a_ **_de-bouncing_** _method_.
  - `reward`:
    - `v/v-max`, in `[0, 1]`, encouraging the agent to overtake slow vehicles.
    - `-10` for **collision / off-road** (when changing lane when already on the side). `done`=`True`.
    - `-10` if the behaviour of the ego vehicle causes another vehicle to **emergency brake**, or if the ego vehicle **drives closer to another vehicle** than a minimum time gap. `done`=`False`.
      - This reminds me the idea of [(Kamran, Lopez, Lauer, & Stiller, 2020)](https://arxiv.org/abs/2004.04450) to **add risk estimate to the reward function** to offer **more frequent signals**.
    - `-1` for each lane change. To discourage unnecessary lane changes.
- One quote about the `Q`-neural net:
  - > "By applying `CNN` layers and a `max pooling` layer to the part of the input that describes the **surrounding vehicles**, the output of the network becomes **independent of the ordering** of the surrounding vehicles in the **input vector**, and the architecture allows a **varying input vector size**."
- One quote about **hierarchy in decision-making**:
  - > "The **decision-making** task of an autonomous vehicle is commonly divided into **`strategic`, `tactical`, and `operational`** decision-making, also called **`navigation`, `guidance` and `stabilization`**. In short, **`tactical` decisions** refer to **high level, often discrete**, decisions, such as when to _change lanes_ on a highway."
- One quote about the **`k-Markov` approximation**:
  - > "Technically, the problem is a `POMDP`, since the ego vehicle **cannot observe the internal state** of the driver models of the **surrounding vehicles**. However, the `POMDP` can be approximated as an `MDP` with a **`k-Markov` approximation**, where the state consists of the **last `k` observations**."
  - Here the authors define **full observability** within `200 m`.
- Why is it called _Bayesian_ `RL`?
  - Originally used in `RL` to creating efficient **`exploration` methods**.
  - Working with an **ensemble** gives **probability distribution** for the **`Q` function**.
  - The `prior` is introduced, here `p`.
  - _What is the posterior?_ The resulting `f+βp`.
  - _How could the likelihood be interpreted?_

</details>

---

**`"Risk-Aware High-level Decisions for Automated Driving at Occluded Intersections with Reinforcement Learning"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2004.04450)]**
**[[:memo:](https://www.uni-das.de/images/pdf/fas-workshop/2020/FAS_2020_KAMRAN.pdf)]**
**[[🎞️](https://www.dropbox.com/s/vnrjl0pro1uqw8w/rl_occlusion.avi?dl=0)]**
**[[🎞️](https://youtu.be/IPa-cxcdT8U?t=6196)]**
**[** :mortar_board: `KIT` **]**

- **[** _`risk-aware`, `occlusion`, [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2004.04450).](../media/2020_kamran_1.PNG "[Source](https://arxiv.org/abs/2004.04450).")  |
|:--:|
| *The `state` considers the **topology** and include information about **occlusions**. A **`risk` estimate** is computed for each `state` using **manually engineered rules**: is it possible for the ego car to **safely stop/leave** the intersection? [Source](https://arxiv.org/abs/2004.04450).* |

| ![[Source](https://arxiv.org/abs/2004.04450).](../media/2020_kamran_2.PNG "[Source](https://arxiv.org/abs/2004.04450).")  |
|:--:|
| *Centre-up: The main idea is to **punish risky situations** instead of **only collision failures**. Left: Other **interesting tricks** are detailed: To deal with a **variable number of vehicles**, to relax the **Markov assumption**, and to focus on the **most important** (**closest**) parts of the scene. Centre-down: Also, the intersection is described as a **region** (`start` and `end`), as opposed to just a **single crossing point**. [Source](https://arxiv.org/abs/2004.04450).* |

| ![[Source](https://www.dropbox.com/s/vnrjl0pro1uqw8w/rl_occlusion.avi?dl=0).](../media/2020_kamran_1.gif "[Source](https://www.dropbox.com/s/vnrjl0pro1uqw8w/rl_occlusion.avi?dl=0).")  |
|:--:|
| *[Source](https://www.dropbox.com/s/vnrjl0pro1uqw8w/rl_occlusion.avi?dl=0).* |

Authors: Kamran, D., Lopez, C. F., Lauer, M., & Stiller, C.

- Motivations:
  - `1-` **Scenario**: deal with **occluded** areas.
  - `2-` **Behaviour**: Look more **human-like**, especially for **_creeping_**, paying attention to the risk.
    - > "This [_`RL` with sparse rewards_] policy usually **drives fast** at occluded intersections and **suddenly stops** instead of having **creeping behavior** similar to humans at risky situations."
  - `3-` **Policy**: Find a trade-off between `risky` and `over-cautious`.
    - > "Not as **overcautious** as the rule-based policy and also not as **risky** as the collision-based `DQN` approach"

- `reward`: The main idea is to **compromise between `risk`** and **`utility`**.
  - **Sparse `reward` functions** ignoring `risk` have been applied in multiple previous works:
    - `1-` **Penalize collisions**.
    - `2-` Reward **goal reaching**.
    - `3-` Give **tiny penalties** at each step to favour motion.
  - > "We believe that such **sparse reward** can be improved by **explicitly providing risk measurements** for each (`state`, `action`) pair during training. The total `reward` value for the current state will then be calculated as the **weighted average** of **`risk`** and **`utility`** rewards."
  - **`utility` reward**: about the speed of ego vehicle.
  - **`risk` reward**: Assuming a worst-case scenario. **Two safety conditions** between the ego vehicle and each related vehicle are defined.
    - `1-` _Can the ego-car_ **_safely leave_** _the intersection before the other vehicle can reach it?_
    - `2-` _Can the ego-car_ **_safely stop_** _to yield before the stop line?_
    - They are not just binary. Rather continuous, based on time computations.
- `state`.
  - A **vector of measurements** that can be used by a rule-based or a learning-based policy.
  - The **map topology** is strongly considered.
    - As opposed to _grid-based_ representations which, in addition, require more complex nets.
  - About **occlusion**:
    - > "For each **intersecting lane `L.i`**, the **closest point** to the conflict zone which is visible by perception sensors is identified and **its distance along the lane** to the **conflict point** will be represented as `do.i`. The maximum **allowed velocity** for each lane is also mapped and will be represented as `vo.i`."
  - About **varying number of vehicles**:
    - The **`5` most-important interacting vehicles** are considered, based on some distance metric.
  - About the **discretization**.
    - Distances are binned in a non-linear way. The **resolution is higher** in close regions: Using `sqrt`(`x`/`d-max`) instead of `x`/`d-max`.
  - About the temporal information:
    - **`5` frames are stacked**. With **`delta-t` = `0.5s`**.
    - Probably to **relax the `Markov` assumption**. In particular, the "real" car dynamic **suffers from delay** (`acceleration` is not immediate).
- `action`.
  - **High-level actions** define **target speeds**:
    - `stop`: **full stop** with maximum deceleration.
    - `drive-fast`: reach `5m/s`.
    - `drive-slow`: reach `1m/s`.
  - **Hierarchical** pipeline:
    - > "Such **high-level** `action` space helps to **learn decision-making** instead of **speed control** for the automated vehicle. The trajectory planner and **`control` module** will take care of **low-level control commands** for the ego vehicle in order to follow these high-level actions. Therefore, **high-level decisions** can be updated with **smaller rate (`2Hz`)** that improves the quality of learning and makes the policy to **only focus on finding optimal behaviors** instead of low level-control."
- About **robustness**:
  - Challenging scenarios are implemented to test the **robustness** of the different approaches:
    - **Dense traffic**.
    - **Severe occlusion**.
    - **Sensor noise**.
    - **Short sensor range**.
  - Decisions must be **interaction-aware** (first case) and **uncertainty-aware** (partial observability).
  - > "The **rule-based** policy is **very conservative** and **fails** for test scenarios with short sensor range (`40m`)."

</details>

---

**`"Learning Robust Control Policies for End-to-End Autonomous Driving from Data-Driven Simulation"`**

- **[** `2020` **]**
**[[:memo:](https://www.mit.edu/~amini/pubs/pdf/learning-in-simulation-vista.pdf)]**
**[[:memo:](https://www.mit.edu/~amini/vista/)]**
**[[🎞️](https://www.youtube.com/watch?v=YgFlMnQmASw)]**
**[[:octocat:](https://forms.gle/FiKEQjddFPDRd4pz9)]**
**[** :mortar_board: `MIT` **]**
**[** :car: `Toyota Research Institute`, `NVIDIA` **]**

- **[** _`sim2real`, `data-driven simulation`, `robustness`, [`VISTA`](https://www.mit.edu/~amini/vista/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://www.mit.edu/~amini/pubs/pdf/learning-in-simulation-vista.pdf).](../media/2020_amini_2.PNG "[Source](https://www.mit.edu/~amini/pubs/pdf/learning-in-simulation-vista.pdf).")  |
|:--:|
| *Top left: one figure is better than many words. **New possible trajectories** are synthesized to learn virtual agent control policies. Bottom and right: robustness analysis compared to two baselines using a **model-based simulator** (`CARLA`): [**domain randomization**](https://arxiv.org/abs/1703.06907) and [**domain adaptation**](https://arxiv.org/abs/1812.03823) and one real-world [imitation learning](https://arxiv.org/abs/1604.07316) approach. **Interventions** marked as red dots. [Source](https://www.mit.edu/~amini/pubs/pdf/learning-in-simulation-vista.pdf).* |

| ![[Source](https://www.mit.edu/~amini/pubs/pdf/learning-in-simulation-vista.pdf).](../media/2020_amini_1.PNG "[Source](https://www.mit.edu/~amini/pubs/pdf/learning-in-simulation-vista.pdf).")  |
|:--:|
| *`VISTA` stands for **'Virtual Image Synthesis and Transformation for Autonomy'**. One main idea is to **synthesize perturbations** of the **ego-agent's position**, to learn to navigate a some worse-case scenarios before cruising down real streets. [Source](https://www.mit.edu/~amini/pubs/pdf/learning-in-simulation-vista.pdf).* |

| ![[Source](https://www.youtube.com/watch?v=YgFlMnQmASw).](../media/2020_amini_1.gif "[Source](https://www.youtube.com/watch?v=YgFlMnQmASw).")  |
|:--:|
| *[Source](https://www.youtube.com/watch?v=YgFlMnQmASw).* |

Authors: Amini, A., Gilitschenski, I., Phillips, J., Moseyko, J., Banerjee, R., Karaman, S., & Rus, D.

- Motivations:
  - `1-` Learn a policy **`end-to-end`** in a **simulator** that can **successfully transfer** into the **real-world**.
    - It is one of the rare works that **go out of the lab to actually test on a road**.
  - `2-` Achieve **robustness** in the transfer, **recovering** from complex, **near crash** driving scenarios.
    - As stated, training AD requires `robustness`, `safety` and `realism`.
  - `3-` Avoid **explicit supervision** where **ground truth human control labels** are used.
    - Supervised learning methods such as **_behavioural cloning_** is not an option.
    - Instead, a single **human collected trajectory** is used.
    - > "**Preserving photorealism** of the real world allows the virtual agent to **move beyond imitation learning** and instead explore the space using reinforcement learning."

- About the data-driven **simulator**:
  - > "[for `end2end`] Simulator must address the challenges of **photorealism**, real-world **semantic complexities**, and **scalable exploration of control options**, while avoiding the fragility of imitation learning and preventing unsafe conditions during data collection, evaluation, and deployment."
  - `Option 0`: **no simulator**, i.e. **`open-loop` replay**.
    - One can use e.g. **supervised** behavioural cloning.
    - One problem: real-world datasets **do not include many hazardous edge cases**.
    - To **generate realistic images** for lateral control failure cases, `viewpoint augmentation` technique can be used, taking advantage of the large field of view.
    > "Augmenting learning with **views from synthetic side cameras** is the standard approach to **increase robustness** and teach the model to **recover from off-center** positions on the roads.
  - `Option 1`: **model-based** simulator.
    - For instance **`CARLA`**.
    - > "While **tremendous effort** has been placed into making the `CARLA` environment as photorealistic as possible, a simulation gap still exists. We found that end-to-end models trained solely in `CARLA` were **unable to transfer to the real-world**."
    - The authors implement two techniques with additional **`data viewpoint augmentation`** to **increase its robustness** in the real-world:
      - `Domain Randomization`.
      - `Domain Adaptation`: training a the policy with **both _simulated_ and _real_** images, i.e. **sharing the latent space**.
  - `Option 2`: **data-driven** simulator.
    - `VISTA` synthesizes **photorealistic and semantically accurate** local viewpoints using a **small dataset** of **human collected driving trajectories**.
    - Motion is simulated in `VISTA` and compared to the human’s estimated motion in the real world:
      - From a `steering curvature` and `velocity` executed during `dt`, `VISTA` can generate the next observation, used for **`action` selection** by the agent.
      - The process repeats.
    - For more details about scene reconstruction, [this post](https://www.greencarcongress.com/2020/03/20200326-mit.html) give good reformulation of the paper.

- About `end-to-end` policy and the `RL` training:
  - `input`: **raw image pixels**.
  - No **temporal stacking** is performed.
    - > "We considered controllers that act based on their current perception **without memory or recurrence** built in."
  - `output`: probability distribution for the desired **curvature** of motion (**lateral control** only), hence treated as a continuous variable.
    - > "Note that curvature is equal to the **inverse turning radius** [`m−1`] and can be converted to **steering angle** at inference time using a **bike model**."
  - `reward`: only the **feedback from interventions** in simulation. i.e. **very sparse** reward signals.
    - This is as **opposed to behaviour cloning** where the agent has **knowledge of how the human drove** in that situation.
  - `training`: Policy Gradient.
    - _Happy to see that one can achieve such impressive results with an algorithm very far from the latest policy-gradient complex methods_.

- About robustness:
  - The idea is to **synthesize perturbations** of the **ego-agent's position**.
    - I.e. during training, **generate new observations** by applying _translations_ and _rotations_ to the views of the recorded trajectory.
  - **Robustness** analysis is perform for **recovery** from **near-crash scenarios**.
    - > "All models showed greater robustness to **recovering from translations** than **rotations** since rotations required significantly more **aggressive control** to recover with a much smaller room of error."
    - `VISTA` successfully recovered over `2×` more frequently than the next best: [NVIDIA's `IMIT-AUG`](https://arxiv.org/abs/1604.07316) which is based on imitation learning with augmentation.
  - Future work: `VISTA` could **synthesize other dynamic obstacles** in the environment such as _cars_ and _pedestrians_.

</details>

---

**`"Interpretable Multi Time-scale Constraints in Model-free Deep Reinforcement Learning for Autonomous Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.09398)]**
**[** :mortar_board: `University of Freiburg` **]**
**[** :car: `BMW` **]**

- **[** _`constrained MDP`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.09398).](../media/2020_kalweit_1.PNG "[Source](https://arxiv.org/abs/2003.09398).")  |
|:--:|
| *Bottom-left: While learning the `Q function`, another quantity is estimated: `JπH` represents the **amount of constraint violations** within **horizon `H`** when following the current **policy `πk`**. It is used to **defined safe action sets** for action selection. This is one of the ideas of the **Multi Time-scale Constrained `DQN`** proposed to solve the **`constrained MDP`**. Bottom-right: example illustrating the **need of long-term predictions/considerations** for in **constrained MDP**. Here state `s6` is marked as unsafe and has to be avoided. The **one-step `action masking`** cannot guarantee **optimality**: at the point of decision it can only choose the path leading to `s10` with a non-optimal return of `+0.5`. [Source](https://arxiv.org/abs/2003.09398).* |

Authors: Kalweit, G., Huegle, M., Werling, M., & Boedecker, J.

- Motivations: _how to perform_ **_model-free `RL`_** _while considering_ **_constraints_**_?_
  - Previous works are limiting:
    - `1-` **Reward shaping**: creating a weighted combination of **different objectives** in the **`reward` signal**.
      - > "We add **weighted penalties** for `lane changes` and for not driving on the `right lane`."
    - `2-` **Lagrangian optimization**: including **constraints** in the **loss function**.
      - > "The **constrained `MDP`** is converted into an **equivalent unconstrained problem** by making infeasible solutions sub-optimal."
    - `3-` **Safe Policy Extraction**: filter out unsafe actions (`action masking`).
      - > "Restrict the **`action` space** during **policy extraction**, masking out all actions leading to constraint violations."
    - The first two require **parameter tuning**, which is difficult in multi-objective tasks (trade-off `speed` / `constraint` violations).
    - The last one does not deal with **multi-step** constraints.

- Here, I noted **three main ideas**:
  - `1-` Separate **`constraints` considerations** from the **`return` optimization**.
    - > "Instead of a **weighted combination of the different objectives**, agents are optimizing **one objective** while satisfying constraints on expectations of auxiliary costs."
  - `2-` Define the `constraints` with **different time scales**.
    - > "Our approach combines the intuitive formulation of **constraints on the short-term horizon** as in `model-based` approaches with the robustness of a `model-free RL` method for the **long-term optimization**."
  - `3-` Do not wait for **policy extraction** to mask actions.
    - Modify the `DQN` output to **jointly estimate `Q`-values** and the **constraint related `comfort`-values**.
    - Instead, **apply masking** directly during the **`Q`-update** to learn the optimal `Q`-function.
  
- About the task:
  - **Highway lane changing**.
    - > "The discrete `action` space includes three actions: `keep lane`, `perform left lane change` and `perform right lane change`."
    - > "**Acceleration** and maintaining safe-distance to the preceding vehicle are controlled by a **low-level `SUMO` controller**."
  - There is a **primary objective**: drive as close as possible to a **desired velocity**.
  - And there are **multiple constraints**: `safety` (change lane only if lane available and free), `keep-right` and `comfort` (within the defined time span, a maximum of lane changes is allowed).

- About the **constrained** Markov Decision Process (**`CMDP`**) framework:
  - The `MDP` is extended with **two `action` sets**:
  - `1-` Set of **safe `action`s** for a **single-step** constraint: **fully known**.
  - `2-` Set of **_expected_ safe** `action`s of a **multi-step** constraint `Jπ`: function of policy.
    - For instance, limit the **amount of lane changes** over `Horizon=5` (`10s`).
    - They are modelled as **expectations** of **auxiliary costs**:
      - The idea is to introduce **`constraint` signals** for each transition (similar to `reward` signals).
      - And **estimate** the **cumulative sum** when following a policy (similar to the `Return`).
        - > "Typically, long-term dependencies on constraints are represented by the **expected sum of discounted or average `constraint` signals**."
        - Here only the **next `H` steps** are considered (**_truncated_ estimation**) and **no discounting** is applied (improving the interpretability of constraints).
    - This quantity is estimated from experience.
      - > "we jointly fit `Q-function` and **multi-step constraint-values `Jh`** in one **function approximator**."
    - The set of expected safe actions for the constraint can then be defined as {`a ∈ A`| **`JH(st, a)` `≤` `threshold`** }
      - `JπH` represents the **amount of constraint violations** within **horizon `H`** when following the current **policy `πk`**.

- About **`off-policy`** learning.
  - **Transitions** are either:
  - `1-` Recorded from a simulator [`SUMO`](https://sumo.dlr.de/docs/index.html)
  - `2-` Or extracted from [`HighD`](https://www.highd-dataset.com) dataset.

</details>

---

**`"Review, Analyse, and Design a Comprehensive Deep Reinforcement Learning Framework"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2002.11883)]**
**[** :mortar_board: `Deakin University`, `Notheastern University` **]**
**[[:octocat:](https://github.com/garlicdevs/Fruit-API)]**

- **[** _`RL software framework`, [`FruitAPI`](https://github.com/garlicdevs/Fruit-API)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2002.11883).](../media/2020_nguyen_3.PNG "[Source](https://arxiv.org/abs/2002.11883).")  |
|:--:|
| *The `API` is based on three **core concepts**: `policy network`, `network configuration`, and `learner`. Besides, the `monitor` is used to manage **multiple learners** (if multi-threading is used) and collect any data from the learners during training while `factory` components can offer **higher abstraction**. No need to reinvent the wheel: **`Plugins` are gateways that extract learners** from **other libraries** such as `PyTorch`, `Tensorflow`, `Keras` and plug them into our proposed framework. [Source](https://arxiv.org/abs/2002.11883).* |

| ![[Source](https://arxiv.org/abs/2002.11883).](../media/2020_nguyen_1.PNG "[Source](https://arxiv.org/abs/2002.11883).")  |
|:--:|
| *[Source](https://arxiv.org/abs/2002.11883).* |

| ![[Source](https://arxiv.org/abs/2002.11883).](../media/2020_nguyen_2.PNG "[Source](https://arxiv.org/abs/2002.11883).")  |
|:--:|
| *[Source](https://arxiv.org/abs/2002.11883).* |

Authors: Nguyen, N. D., Nguyen, T. T., Nguyen, H., & Nahavandi, S.

- **_`FruitAPI`: Yet another `RL` framework?_**
  - _Whatever! Some useful reminders about `RL` concepts and good insights about how_ **_software can be structured_** _for `RL` applications._
- Motivations:
  - > "Our ultimate goal is to build an **educational software platform** for deep `RL`."
    - For instance, they plan to develop a **GUI application** that can configure the neural network, modify the learner, and visualize the `RL` workflow.
  - `1-` The API should work with any deep learning libraries such as `PyTorch`, `Tensorflow`, `Keras`, etc.
    - > "Instead of implementing a lot of deep `RL` algorithms, we provide a flexible way to integrate existing deep RL libraries by introducing **`plugins`**. Plugins **extract learners** from other deep RL libraries and plug into **`FruitAPI`**."
  - `2-` Support different disciplines in `RL` such as **multiple objectives**, **multiple agents**, and **human-agent interaction**.
- Mentioned frameworks:
  - [`TensorForce`](https://github.com/tensorforce/tensorforce).
    - > "It is an ambitious project that targets both **industrial applications** and **academic research**. The library has the **best modular architecture** we have reviewed so far. However, the framework has a deep **software stack** (“pyramid” model) that includes many **abstraction layers**. This hinders **novice readers** from prototyping a new deep RL method."
  - [`OpenAI Baselines`](https://github.com/openai/baselines).
  - [`RLLib`](https://ray.readthedocs.io/en/latest/rllib.html). For scalable `RL`. Built on top of [`ray`](https://github.com/ray-project/ray).
  - [`RLLab`](https://github.com/rll/rllab). Now [`garage`](https://github.com/rlworkgroup/garage). A toolkit for reproducible reinforcement learning research.
  - [`Keras-RL`](https://github.com/keras-rl/keras-rl).
  - [`Chainer`](https://chainer.org).

</details>

---

**`"Learning hierarchical behavior and motion planning for autonomous driving"`**

- **[** `2020` **]**
**[[:memo:](https://jingk.wang/uploads/wjk3.pdf)]**
**[** :mortar_board: `Zhejiang University`, `Arizona State University` **]**

- **[** _`hierarchical RL`, `transfer`, `PPO`, `DAgger`, [`CARLA`](http://carla.org), [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://jingk.wang/uploads/wjk3.pdf).](../media/2020_wang_1.PNG "[Source](https://jingk.wang/uploads/wjk3.pdf).")  |
|:--:|
| *To achieve **transfer**, the authors develop a **sharable representation** for input sensory data **across simulation platforms** and **real-world environment**. Left: **static**. Right: **dynamic** information. [Source](https://jingk.wang/uploads/wjk3.pdf).* |

| ![[Source](https://jingk.wang/uploads/wjk3.pdf).](../media/2020_wang_2.PNG "[Source](https://jingk.wang/uploads/wjk3.pdf).")  |
|:--:|
| *Left - The `behaviour` layer makes a decision based on the current observation, e.g. `lane change`, while the **`motion planning`** layer yields the **trajectory** to complete this decision. More precisely, the output of the `PPO` actor-network is the **distribution over the `behaviour`s**. The `motion planner` is **conditioned over this high-level decision**. If you are also wondering: `traffic light state` and `speed limit` are represented probably since they are part of the **`road profile`** (static information) - surprising to limit a `3`-lanes highway at `30mps`? [Source](https://jingk.wang/uploads/wjk3.pdf).* |

Authors: Wang, J., Wang, Y., Zhang, D., Yang, Y., & Xiong, R.

- Motivations:
  - `1-` **Jointly optimize** both the `behaviour` and `motion` planning modules. I.e. **do not treat them independently**.
    - Methods that use `RL` to decide low-level commands such as `throttle` and `steering` often lead to weak **_tactical_ decision-making**.
    - > "To demonstrate a similar level of driving as the **conventional modular pipeline**, a **SINGLE `RL` network** has to model the `perception`, `behavior` and `motion planner` altogether. Given the **low dimensional control commands** as supervision in `IL` or handcrafted rewards in `RL`, it is likely that the network **only learns the route tracking with limited _tactical_ decision-making ability**."
  - `2-` Apply **learning-based** methods (model-free `RL` and `IL`) to derive the _decision_ policies.
  - `3-` Address the issues of **sampling complexity** and **sparse reward** in **model free `RL`**.
    - > "**Reward engineering** for `RL` is challenging, especially for **long-horizon tasks**."
  - `4-` Do not use any **"_explicit handcrafted_** _representation"_ for traffic situation.
    - > "[not clear to me] We follow the idea of `HBMP` in order to **learn a representation** of traffic situation **_implicitly_** and improve the tactical driving in learning-based solution."
    - > "[is it what they mean by _'not explicit handcrafted representation'_?] Note that **no cyclist recognition** is utilized in the whole pipeline, but **only occupied map** directly extracted from lidar data."

- One concept: **hierarchical behaviour and motion planning** (`HBMP`).
  - The **`behaviour` layer** makes a decision based on the current observation, e.g. `lane change`, while the **`motion planning` layer` yields the **trajectory to complete this decision**.
  - > "our idea is to introduce the **hierarchical modeling** of `behavior` and `motion` in the **conventional modular pipeline** to the **learning-based** solution."
- About the size of search spaces in `hierarchical RL`:
  - When **coupling the two `action` spaces** (`behaviour` and `motion`), the **search complexity** becomes a challenge, especially in the **long-horizon driving task**. The `HRL` is therefore re-formulated:
    - > "[solution:] `RL` is applied to solve an **equivalent `behavior` learning problem** whose **`reward`s** are assigned by the `cost` from the `motion planner`."
    - > "The key insight is **assigning the optimal costs** from **low-level motion planner**, i.e. a classical **sampling-based planner**, as `reward`s for learning behavior."
    - The **ablation study** shows the importance of `hierarchical` modelling to **reduce the `action` space**.
- Improving the **_sampling complexity_** and **_sparse reward_** issues:
  - `1-` The above-mentionned **Hierarchy**: Implement a **temporal abstraction for `action`s**.
    - > "To **reduce the complexity**, behaviour planning is introduced to **restrict the search space**."
    - > "As the **high-level** `behaviour` `bk` is a **tactical decision for a longer horizon** than `control` command, each `bk` is **fixed for a time interval**".
    - The **low-level** policy is **conditioned** on `bk`: `ut` = `πbu`(`xt`,`bk`).
    - The behaviour planner choses a **`behaviour`** a `bk` in {`speed up`, `speed down`, `change left`, `change right`, `keep lane`}.
    - A **_Behaviour-Conditioned_ motion planner** produces a trajectory for this behaviour, implemented by a subsequent `PID` controller.
      - Here it is **sampling-based**, i.e. **optimal without training**.
      - The `cost` includes terms about `speed deviation`, `lateral deviation` to some reference path as well as `distances to obstacles`.
  - `2-` Good **policy inititialization**.
    - > "We **initialize** the `RL` policy network trained on `CARLA` using **`IL`-based** policy (`DAgger`) trained on `SUMO` by presenting a **sharable sensory data representation**, further accelerating `RL` training".
    - The key ingredient for the **transfer** between the two simulators is the **`state` representation**, among other the **deformed _grid map_ `Mt`**.
- _How to deal with sparse rewards?_
  - > "Obviously, such `reward` design [_sparse rewards_] gives **extremely sparse guidance** only in the final stage, thus **lots of trials** may have almost the **same rewards**, causing the **inefficiency** of `RL` solver."
  - The above-mentioned **Hierarchy**: add **`dense reward`s**.
  - The low-level motion planner offers **denser rewards** than relying on the far-away `reward` of `termination state`s.
- **Generalization** to real-world.
  - Again, the **sharable representation** for input sensory data **across simulation platforms** and **real-world environment** is key for **transfer**.
  - It consist of
    - `1-` the `vehicle-centric grid map` for _dynamic_ information
      - > "Note that **no detection and recognition** is utilized to parse the sensory data but only geometric transform calculation."
    - `2-` the **`road profile`** that contains _static_ information:
    - The **existence** and **direction** of left and right lanes.
    - The next **traffic lights** and the distance to the **stop line** of intersection.
    - The current **speed limit**.

</details>

---

**`"Safe Reinforcement Learning for Autonomous Vehicles through Parallel Constrained Policy Optimization"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.01303)]**
**[** :mortar_board: `Tsinghua University`, `University of Michigan` **]**
**[** :car: `Toyota` **]**

- **[** _`safe RL`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.01303).](../media/2020_wen_1.PNG "[Source](https://arxiv.org/abs/2003.01303).")  |
|:--:|
| *Risk-Actor-Critic: Similar to the **`critic`** which estimates the **expectation** of the `value function` (cumulated `reward`s), the **`risk` network** estimates the sum of coming **`risk` signals**, i.e. signals received when **moving to unsafe states**. In this case  `PCPO` considers this **`risk` function** and **bounds the expected risk** within **predefined hard constraints**. [Source](https://arxiv.org/abs/2003.01303).* |

| ![[Source](https://arxiv.org/abs/2003.01303).](../media/2020_wen_2.PNG "[Source](https://arxiv.org/abs/2003.01303).")  |
|:--:|
| *Note that what is wanted is **`approximate safety`** during the **TRAINING phase** - not just the **testing** one. `PCPO` is compared to `CPO` and `PPO`. All three methods can eventually learn a **safe lane-keeping policy**, however, the vehicle deviates from the lane multiple times **during the learning process** of the `PPO`. [Source](https://arxiv.org/abs/2003.01303).* |

Authors: Wen, L., Duan, J., Eben, Li, S., Xu, S., & Peng, H.

- Motivations:
  - `1-` Ensure the policy is **_safe_** in the **learning process**.
    - > "Existing RL algorithms are **rarely applied to real vehicles** for two predominant problems: (1) behaviors are **unexplainable**, (2) they **cannot guarantee safety** under **new scenarios**."
    - > "**Safety** is the **most basic requirement** for autonomous vehicles, so a training process that **only looks at `reward`**, and not **potential risk**, is **not acceptable**."
  - `2-` Improve the convergence speed.

- About `safe RL`:
  - > "Observing **safety constraints** and getting **high rewards** are **adversarial**."
  - `safe RL` could be divided into being **_strictly_ safe** and **_approximately_ safe**.
  - Two approaches:
    - `1-` Modifying the **optimization criterion**.
      - `1-a.` **`maxi-min`**: consider a policy to be optimal if it has the **maximum worst-case `return`**.
      - `1-b.` **`risk-sensitive`**: includes the notion of risk and return variance in the long term `reward` **maximization objective**, e.g. probability of entering an error state.
      - `1-c.` **`constrained`**: ensure that the expectation of return is subject to one or more constraints, e.g. the **variance of the `return` must not exceed a given threshold**.
    - `2-` Modifying the **exploration process**.
      - `2-a.` Incorporating **external knowledge**.
        - E.g. **post-posed shielding**: the shield monitors the agent's action and **corrects them** if the chosen action causes a **violation**.
      - `2-b.` Risk directed exploration.
        - Encourage the agent to explore **controllable regions** of environment by introducing **`risk` metric as an exploration bonus**.
- About **_Parallel Constrained Policy Optimization_** (**`PCPO`**). Three ingredients:
  - `1-` **Constrained** policy optimization (`CPO`).
    - > "Besides the **`reward` signal `r`**, the vehicle will also observe a **scalar risk signal `˜r`** at each step."
    - It is designed by human experts and is usually assigned a large value when the vehicle moves into an **unsafe state**.
    - Similar to the **value function `Vπ`** that estimate the discounted cumulated rewards, the **risk function J˜(π)** of policy `π` estimates the **sum of coming risks**.
      - Similar to the **`critic`** net, a **`risk` net** is introduced.
    - The idea of `CPO`:
      - > "**Bound this expected risk** within **predefined hard constraints**."
  - `2-` **Trust-region** constraint.
    - **Monotonic improvement** condition can be guaranteed only when the policy changes are not very large.
    - The author go through the derivation of the **surrogate objective function** with **importance sampling**.
  - `3-` Synchronous **parallel** agents learning.
    - Multiple agents (`4` cars in the experiment) are **exploring different state spaces** in parallel.
    - It helps to **reduce the correlation** and **increase the coverage** of all collected samples, which increases the possibility of **finding feasible states** - i.e. better **exploration**.
- About the `MDP` formulation for the **_intersection-crossing_ problem**:
  - `S` = {`l1`, `v1`, `l2`, `v2`, `l3`, `v3`}, where `l` denotes the **distance of the vehicle to the middle point of its track**, and `v ∈ [6, 14] (m/s)` denotes the `velocity`."
  - `A` = {`a1`, `a2`, `a3`}, where `a ∈ [−3, 3] (m/s2)`.
    - > "We adopt a **stochastic policy**, the output of which is the `mean` and `standard deviation` of the Gaussian distribution."
  - `reward` model:
    - The agents receive a reward of `10` for every passing vehicle, as well as a reward of `-1` for every time step and an additional reward of `10` for **terminal success**.
    - The agents are given a **risk of `50`** when a **collision occurs**.
  - That means **"collision"** is **not part of the `reward`**. Rather in the **`risk`**.
    - _Ok, but how can it be_ **ensured to be _safe_ during training** _if it has never encountered any collision and does not event know what it means (no "model")?_ This is not clear to me.

</details>

---

**`"Deep Reinforcement Learning for Autonomous Driving: A Survey"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2002.00444)]**
**[** :mortar_board: `ENSTA ParisTech`, `National University of Ireland` **]**
**[** :car: `Navya`, `Valeo` **]**

- **[** _`review`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2002.00444).](../media/2020_kiran_1.PNG "[Source](https://arxiv.org/abs/2002.00444).")  |
|:--:|
| *[Source](https://arxiv.org/abs/2002.00444).* |

Authors: Ravi Kiran, B., Sobh, I., Talpaert, V., Mannion, P., Sallab, A. A. Al, Yogamani, S., & Pérez, P.

- _Not too much to report. A rich literature overview and some useful reminders about general `RL` concepts._
  - _Considering the support of serious industrial companies (`Navya`, `Valeo`), I was surprised not to see any section about_ **_"`RL` that works in reality"_**.
- Miscellaneous notes about **decision-making** methods for `AD`:
  - One term: **_"holonomic"_** _(I often met this term without any definition)_
    - > "A robotic agent capable of controlling `6`-degrees of freedom (`DOF`) is said to be **_holonomic_**, while an agent with **fewer controllable `DOF`s** than its total `DOF` is said to be **non-holonomic**."
    - Cars are **_non-holonomic_** (addressed by `RRT` variants but not by `A*` or `Djisktra` ones).
  - About `optimal control` (I would have mentioned **`planning`** rather than `model-based RL`):
    - > "`Optimal control` and `RL` are intimately related, where `optimal control` can be viewed as a **model-based `RL`** problem where the dynamics of the vehicle/environment are modeled by well defined differential equations."
  - About **`GAIL`** to generate a policy, as an alternative to `apprenticeship learning` (`IRL`+ `RL`):
    - > "The theory behind `GAIL` is an **equation simplification**: qualitatively, if `IRL` is going from **demonstrations** to a **cost function** and `RL` from a **cost function** to a **policy**, then we should altogether be able to go from **demonstration to policy** in a single equation while **avoiding the cost function estimation**."
  - Some mentioned challenges:
    - **Validation** and **safety**.
    - **Sample efficiency** and **training stability** in general.
    - **`MDP` definition**, e.g. space _discretization_ or not, level of _abstraction_.
    - **Simulation-reality gap** (_domain transfer_), mentioning **domain adaptation** solutions:
      - `feature`-level
      - `pixel`-level
      - `real-to-sim`: **adapting the real camera streams to the synthetic modality**, so as to map the unfamiliar or unseen features of real images **back into the simulated style**.
    - **Reward shaping**. Mentioning **intrinsic** rewards:
      - > "In the absence of an _explicit_ reward shaping and expert demonstrations, agents can use **intrinsic rewards** or **intrinsic motivation**."
      - `curiosity`: the **error** in an agent’s ability to **predict the consequence** of its own actions in a visual feature space learned by a self-supervised **inverse dynamics model**.
    - **Reproducibility**. Multiple frameworks are mentioned:
      - [OpenAI Baselines](https://github.com/openai/baselines).
      - [TF-Agents](https://github.com/tensorflow/agents).
      - [Coach](https://github.com/NervanaSystems/coach) by [Intel AI Lab](https://www.intel.ai/rl-coach-new-release/).
      - [rlpyt](https://github.com/astooke/rlpyt) by [bair.berkeley](https://bair.berkeley.edu/blog/2019/09/24/rlpyt/).
      - [bsuite](https://github.com/deepmind/bsuite) by [DeepMind](https://deepmind.com/research/open-source/bsuite).

</details>

---

**`"Interpretable End-to-end Urban Autonomous Driving with Latent Deep Reinforcement Learning"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2001.08726)]**
**[** :mortar_board: `UC Berkeley`, `Tsinghua University` **]**

- **[** _`probabilistic graphical models`, `MaxEnt RL`, `mid-to-end`, [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![The `RL` problem is formulated with a **probabilistic graphical model** (**`PGM`**). `zt` represents for the **latent state**, i.e. the hidden state. `xt` is the **observation** sensor inputs. `at` is the action. `Ot` denotes the **optimality variable**. The authors introduce a **`mask`** variable `mt` and learn its **probability of emission** conditioned on `z`. [Source](https://arxiv.org/abs/2001.08726).](../media/2020_chen_1.PNG "The `RL` problem is formulated with a **probabilistic graphical model** (**`PGM`**). `zt` represents for the **latent state**, i.e. the hidden state. `xt` is the **observation** sensor inputs. `at` is the action. `Ot` denotes the **optimality variable**. The authors introduce a **`mask`** variable `mt` and learn its **probability of emission** conditioned on `z`. [Source](https://arxiv.org/abs/2001.08726).")  |
|:--:|
| *The `RL` problem is formulated with a **probabilistic graphical model** (**`PGM`**). `zt` represents for the **latent state**, i.e. the hidden state. `xt` is the **observation** sensor inputs. `at` is the action. `Ot` denotes the **optimality variable**. The authors introduce a **`mask`** variable `mt` and learn its **probability of emission** conditioned on `z`. [Source](https://arxiv.org/abs/2001.08726).* |

| ![The `mask` is a _semantic_ representation of the scene, helpful for **interpretation of the `perception`** part (not for `decision`). The learnt **transition** function (from `z` to `z'` conditioned on `a`) can be used for **prediction**. [Source](https://arxiv.org/abs/2001.08726).](../media/2020_chen_2.PNG "The `mask` is a _semantic_ representation of the scene, helpful for **interpretation of the `perception`** part (not for `decision`). The learnt **transition** function (from `z` to `z'` conditioned on `a`) can be used for **prediction**. [Source](https://arxiv.org/abs/2001.08726).")  |
|:--:|
| *The `mask` is a _semantic_ representation of the scene, helpful for **interpretation of the `perception`** part (not directly applicable to understand the `decision`). The learnt **transition** function (from `z` to `z'` conditioned on `a`) can be used for **prediction**. [Source](https://arxiv.org/abs/2001.08726).* |

Authors: Chen, J., Li, S. E., & Tomizuka, M.

- Motivations:
  - `1-` Offer more **interpretability** to end-to-end model-free `RL` methods.
  - `2-` Reduce the **sample complexity** of model-free `RL` (_actually not really addressed_).

- Properties of the **environment**:
  - `1-` High-dimensional **observations**: `camera` frame and `lidar` point cloud.
  - `2-` **Time-sequence** probabilistic dynamics.
  - `3-` **Partial observability**.

- One first idea: formulate the `RL` problem as a **probabilistic graphical model** (**`PGM`**):
  - A binary random variable (`Ot`), called **_optimality variable_**, is introduced to indicate whether the agent is **acting optimally** at time step `t`.
    - It turn out that its conditional probability is **exponentially proportional** to the **one-step reward**: `p`(`Ot=1` | `zt`, `at`) = `exp`(`r`(`zt`, `at`))
    - The stochastic exploration strategy has therefore the form of a **`Boltzmann`-like distribution**, with the `Q`-function acting as the **negative energy**.
  - Then, the task is to make sure that the **most probable trajectory** corresponds to the trajectory from the **optimal policy**.
  - > "`MaxEnt RL` can be interpreted as learning a `PGM`."
    - `MaxEnt RL` is then used to **maximize the likelihood** of **optimality variables** in the `PGM`.
    - More precisely, the original log-likelihood is maximized by maximizing the `ELBO` (c.f. `variational methods`).
- About **`MaxEnt RL`**:
  - The standard `RL` is modified by adding an **entropy regularization** term `H`(`π`) = `−log`(`π`) to the **reward**.
  - `MaxEnt RL` has a **_stochastic_** policy by default, thus the policy itself includes the **exploration** strategy.
  - Intuitively, the goal is to learn a policy that acts **as randomly as possible** (encouraging _uniform_ action probability) while is still aiming at succeeding at the task.
  - About **soft actor critic** ([`SAC`](https://arxiv.org/abs/1801.01290)):
    - This is an example of `MaxEnt RL` algorithm: it incorporates the **entropy measure** of the policy into the **reward to encourage exploration**.
    - _Why "soft"?_
      - For small values of `Q`, the approximation `V`=`log`(`exp`(`Q`)) ~= `max`(`Q`) is loose and the **maximum is said _soft_**, leading to an **optimistic `Q`-function**.
- One reference about the **duality `RL`/`PGM`** and **`MaxEnt RL`**:
  - ["Reinforcement learning and control as probabilistic inference: Tutorial and review"](https://arxiv.org/abs/1805.00909) by (Levine, 2018).
  > "In this article, we will discuss how a generalization of the `RL` or optimal control problem, which is sometimes termed **`MaxEnt RL`**, is equivalent to exact **probabilistic inference** in the case of deterministic dynamics, and **variational inference** in the case of stochastic dynamics."

- Another idea: add a **second generated / emitted variable**.
  - So far, the `observation` variable is generated/emitted from the `hidden state`. Here observations are `camera` and `lidar` scans.
  - Here, a **second variable is emitted** (_"decoded"_ is the term used by the authors): a **semantic bird-eye _mask_**, noted `mt`.
  - It contains **semantic meanings** of the environment in a _human_ understandable way:
    - Drivable areas and lane markings.
    - `Waypoints` on the desired route.
    - Detected objects (**bounding boxes**) and their history.
    - Ego bounding box.
  - It makes me think of the _image-like_ scene representations used in **`mid-to-end` approaches**. E.g. [`chaufferNet`](https://arxiv.org/abs/1812.03079).
  - > "At training time we need to provide the **ground truth labels of the mask**, but at test time, the mask can be **decoded from the latent state**, showing how the system is understanding the environment semantically."
- One note:
  - The author call **"`latent` variable"** what is sometimes referred to as `hidden state` in `PGM` (e.g. in `HMM`).
- _Why_ **_"joint"_** _model learning?_
  - The policy is learned jointly with the **other `PGM` models**.
    - `1-` **`policy`**: `π`(`at`|`zt`) - _I would have conditioned it on the `observation` variable since the `hidden` state is by definition not observable_.
    - `2-` **`inference`**: of the current latent state: `p`(`zt+1`|`x1:t+1`, `a1:t`).
    - `3-` **`latent dynamics`**: `p`(`zt+1`|`zt`, `at`). This is then used for **prediction**.
    - `4-` **`generative model`** for `observation`: `p`(`xt`|`zt`), i.e. the **emission** of probability from the latent space to the **`observation`** space.
    - `5-` **`generative model`** for `mask`: `p`(`mt`|`zt`), i.e. the generation of the **semantic mask** from the `hidden state`, to provide **interpretability**.
- _Why calling it "sequential"?_
  - > "Historical high-dimensional raw observations [`camera` and `lidar` frames] are **compressed** into this **low-dimensional latent space** with a **"sequential"** latent environment model."
  - Actually I am not sure. Because they learn the transition function and are therefore able to estimate how the hidden state evolves.
- One limitation:
  - This intermediate representation shows how the model **understands** the scene.
  - > "[but] it **does not provide any intuition about how it makes the decisions**, because the driving policy is obtained in a `model-free` way."
  - In this context, `model-based` `RL` is deemed as a promising direction.
  - It reminds me the distinction between **`learn to see`** (_controlled by the presented `mask`_) and **`learn to decide`**.

</details>

---

**`"Hierarchical Reinforcement Learning for Decision Making of Self-Driving Cars without Reliance on Labeled Driving Data"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2001.09816)]**
**[** :mortar_board: `Tsinghua University` **]**

- **[** _`hierarchical RL`, `async parallel training`, `actuator command increments`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2001.09816).](../media/2020_duan_1.PNG "[Source](https://arxiv.org/abs/2001.09816).")  |
|:--:|
| *__Hierarchical-`RL`__ with **async parallel** training is applied to improve the **training efficiency** and the **final performance**. It can be seen as a **`temporal` / `action` abstraction**, although here both **`high` and `low` levels** work at the **same frequency**. The **`master` policy** is trained once all the `sub-policies` have been trained. Note that only **`4` surrounding cars** are considered. [Source](https://arxiv.org/abs/2001.09816).* |

| ![[Source](https://arxiv.org/abs/2001.09816).](../media/2020_duan_2.PNG "[Source](https://arxiv.org/abs/2001.09816).")  |
|:--:|
| *Each sub-policy (`manoeuvre` policies) has its **specific `state` representation** since not all information is relevant for each sub-task. The **`lateral` and `longitudinal` actions are decoupled** only for the `drive-in-lane` sub-policy. **Noise** is added to the `state` to form the `observation` used by the agent, to improve **robustness**. [Source](https://arxiv.org/abs/2001.09816).* |

Authors: Duan, J., Li, S. E., Cheng, B., Luo, Y., & Li, K.

- Motivations:
  - `1-` Perform the **full `vertical` + `horizontal` decision-making stack** with `RL`.
    - `RL` as opposed to `behavioural cloning`. Hence the title **_"without reliance on labeled driving data_**.
    - > [`vertical`] "Current works of `RL` only focus on **`low-level` motion control** lacer. It is hard to solve driving tasks that require many **sequential decisions or complex solutions** without considering the `high-level` maneuver."
    - [`horizontal`] The control should be done in both the `lateral` and `longitudinal` directions.
    - One idea could be to use a **single policy**, but that may be **hard to train**, since it should learn both **`low-level` drive controls** together with **`high level` strategic decisions**.
      - > "The **single-learner** `RL` fails to learn suitable policies in these two cases because the **`state` dimension is relatively high**. Therefore, `lane-change` maneuver requires **more exploration** and is **more complicated** than `driving-in-lane` maneuver, which is **too hard for single car-learner**."
  - `2-` Improve the **training efficiency** and **final performance** of a `RL` agent on **a complex task**.
    - Here the complexity comes, among others, from the **long horizon** of the sequential problem.
  - `3-` Focus on two-lane highway driving.
    - _It would be interesting to extend the idea to urban intersections._
  - Main ingredients:
    - `action` abstraction: **hierarchical `RL`**.
    - Training **multiple agents** in **parallel**: **asynchronous parallel `RL` (`APRL`)**

- Architecture:
  - `1-` `master` policy.
    - The **high-level** manoeuvre selection.
  - `2-` `manoeuvre` policies.
    - The **low-level** motion control.
    - Input: The `state` representation **differs for each sub-policy**.
    - Output: Actuator commands.
      - Each policy contains a `steer` policy network (`SP-Net`) and an `accelerate` policy network (`AP-Net`). They can be coupled or not.

- _What manoeuvres?_
  - `1-` `driving-in-lane`.
    - > "It is a combination of many behaviors including `lane keeping`, `car following` and `free driving`."
    - In this case, the **two `state`, `rewards` and `actions` are independent**: `path-speed` decomposition.
  - `2-` `left lane change`.
  - `3-` `right lane change`.

- `action masking` in the `master` policy.
  - Not **all manoeuvres are available** in every `state` throughout driving.
    - E.g., the `left-lane-change` manoeuvre is **only available in the right lane**.
  - > "Thus, we **predesignate a list of available maneuvers** via the observations at each step. Taking an **unavailable maneuver** is considered an **error**. The self-driving car **should filter its maneuver choices** to make sure that only legal maneuvers can be selected."

- `action` space for the `manoeuvre` policies.
  - > "Previous `RL` studies on autonomous vehicle decision making usually take the `front wheel angle` and `acceleration` as the policy outputs. This is **unreasonable because the physical limits** of the **actuators** are not considered."
  - > "To prevent **large discontinuity of the control commands**, the outputs of maneuver policy `SP-Net` and `AP-Net` are `front wheel angle` **increment** and `acceleration` **increment** respectively."

- _Which level is trained first?_
  - `1-` Each **sub-policy** is first trained. Each having their **specific `state` representation**.
  - `2-` Then, the **`master` policy** is learned to choose the maneuver policy to be executed in the current `state`.
    - > "The results of **asynchronous parallel reinforcement learners** (`APRL`) show that the **`master` policy converges much more quickly** than the **`maneuver` policies**."

- Robustness.
  - **Gaussian noise** is added to some **`state` variables** before it is observed by the car-learner.
  - > "We assume that the **noise** of all `states` in **practical application** is **`M` times the noise used in the training**, so the real sensing noise obeys `U`(`−M ∗ Nmax`, `M ∗ Nmax`). To assess the sensitivity of the `H-RL` algorithm to **`state` noise**, we fix the parameters of previously trained `master` policy and `maneuver` policies and assess these policies for different `M`. [...] The policy is less affected by noise when `M ≤ 7`."

- Nets.
  - **`32` asynchronous parallel** car-learners.
  - `n-step` return with `n=10`.
    - "Back-propagation after **`n=10` forward steps** by explicitly computing `10-step` returns."
  - `ELU` activation for hidden layers.
  - Only `FC` layers.

- _Do the two level operate at same frequency?_
  - Apparently yes: "the **system frequency is `40Hz`**".
  - _How to deal then with_ **_changing objectives_** _coming from the `master` policy?_

</details>

---

**`"Worst Cases Policy Gradients"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1911.03618)]**
**[[🎞️](https://youtu.be/IPa-cxcdT8U?t=11609)]**
**[** :car: `Apple` **]**

- **[** _`safe RL`, [`distributional RL`](https://arxiv.org/pdf/1707.06887.pdf), `risk-sensitive`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1911.03618).](../media/2019_tang_1.PNG "[Source](https://arxiv.org/abs/1911.03618).")  |
|:--:|
| *The **critic** predicts a **distribution of the expected `value function`** instead of a single scalar. Both the actor and critic take **risk-tolerance** `α` as input during **training**, which allows the learned policy to operate with **varying levels of risk** after training. Lower `α` values **improved robustness** dramatically. Top-right: the **uncertainty** can be quantified from the **variance** of the **`return` distribution**. [Source](https://arxiv.org/abs/1911.03618).* |

| ![[Source](https://ieeexplore.ieee.org/abstract/document/8813791).](../media/2019_bernhard_1.PNG "[Source](https://ieeexplore.ieee.org/abstract/document/8813791).")  |
|:--:|
| *A related work (Bernhard and Knoll 2019): Working with **distributions** instead of **mean expectation** can offer **uncertainty-aware** `action` selection. [Source](https://ieeexplore.ieee.org/abstract/document/8813791).* |

Authors: Tang, Y. C., Zhang, J., & Salakhutdinov, R.

- One sentence:
  - > "Our policies can be **adjusted dynamically after deployment** to select **risk-sensitive `actions`**."

- Motivations:
  - `1-` Models the **uncertainty** of the future.
  - `2-` Learn policies with **risk-tolerance** parameters.
    - > "The **learned policy** can map the same state to different `actions` depending on the **propensity for risk**."
    - `α`=0.0 for **risk-aware**. `α`=1.0 for **risk-neural**.

- Going beyond standard `RL`.
  - > "Standard `RL` maximizes for the **expected (possibly discounted) future `return`**. However, maximizing for **average `return`** is not sensitive to the **possible risks** when the **future `return` is stochastic**, due to the **inherent randomness** of the environment not captured by the **observable `state`**."
  - > "When the `return` distribution has **high variance** or is **heavy-tailed**, finding a `policy` which maximizes the **expectation of the distribution** might not be ideal: a high variance `policy` (and therefore higher risk) that has **higher `return` in expectation** is preferred over **low variance `policies`** with **lower expected `returns`**. Instead, we want to learn more robust policies by **minimizing long-tail risks**, reducing the likelihoods of bad outcomes."

- _Why "_**_Worst Cases_** _Policy Gradients" (`WCPG`)?_
  - > "However, for **risk-averse** learning, it is desirable to maximize the expected **worst cases** performance instead of the **average-case** performance."
  - You do not care about the average expected return. You care about what the best return will be under the worst possible scenario.

- About safe RL. Two categories:
  - > "The first type is based on the **modification of the exploration process** to avoid **unsafe exploratory actions**. Strategies consist of **incorporating external knowledge** or using **risk-directed explorations**."
  - The second type modifies the **optimality criterion** used during **training**.
    - > "Our work falls under this latter category, where we try to optimize our policy to **strike a balance between pay-off and avoiding catastrophic events.**"

- About **conditional [`Value-at-Risk`](https://www.ise.ufl.edu/uryasev/files/2011/11/CVaR1_JOR.pdf)** (`CVaR`)
  - > "Working under the assumption that the **future `return`** is inherently stochastic, we first start by **modeling its distribution**. The **risk** of various actions can then be computed from this distribution. Specifically, we use the **conditional `Value-at-Risk`** as the criterion to maximize."
  - > "`WCPG` optimizes for `CVaR` indirectly by first using **distributional RL** techniques to **estimate the distribution of `return`** and then compute `CVaR` from this distribution."
    - Intuitively, `CVaR` represents the expected `return` should we experience the bottom `α`-percentile of the possible outcomes.
  - How to compute the **loss** for the critic?
    - The **scalar version with `L2`** is replaced by the **Wasserstein distance** between the predicted and the expected distribution.
  - How to model the distribution?
    - The authors assume a **Normal distribution**. Therefore only the `mean` and `variance` are to be predicted by the critic.

</details>

---

**`"Driving Reinforcement Learning with Models"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1911.04400)]**
**[[:octocat:](https://github.com/GIOVRUSSO/Control-Group-Code/tree/master/MPRL)]**
**[** :mortar_board: `University College Dublin`, `Imperial College London`, `Salerno University` **]**

- **[** _`planning`, `MPC`, `environment model`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1911.04400).](../media/2020_rathi_1.PNG "[Source](https://arxiv.org/abs/1911.04400).")  |
|:--:|
| *When a **model of the environment is available** (e.g. when the ball comes toward the ego paddle), `planning` is performed. Otherwise, a **model-free `RL` agent** is used. The `RL` agent maximizes the expected `return`, while **imitating the `MPC`** when it is used. This improves the **sampling efficiency**, while bringing some **guarantees**. [Source](https://arxiv.org/abs/1911.04400).* |

Authors: Rathi, M., Ferraro, P., & Russo, G.

- Motivations:
  - `1-` **Model-free `RL`** is **sample inefficient** and **lacks (e.g. `safety`) guarantees** while learning.
  - `2-` **`planning`** can efficiently solve a `MDP`. But the `transition` and `reward` models must be known.
  - `3-` In many real-world systems, **tasks can be broken down** into a set of **functionalities** and, for some of these, **a mathematical model might be available**.
    - > [Observation] "In many applications, such as applications requiring **physical interactions** between the agent and its environment, while a **full model** of the environment might not be available, **at least parts of the model**, for e.g. a **subset of the `state` space**, might be known/identifiable."

- Main idea:
  - Use a **`planning`** (here a `MPC`) when a **mathematical model is available** and `Q-learning` otherwise.
  - `MPC` is **not only** used for **`action` selection**. Even if `MPC` is used, `RL` makes a prediction and is **penalized if predictions differ**. This accelerates the learning phase of the `RL` part by:
    - `1-` Driving the **`state`-space exploration** of `Q-learning`.
    - `2-` Tuning its rewards.
- At first sight, it seems that the learning-agent **simultaneously maintains two goals**:
  - `1-` Maximize the expected sum of discounted `rewards`: **`RL`** part.
  - `2-` **Imitate** the `MPC`: **`behavioural cloning`**.
  - If the **supervised imitation** is too strict, the performances degrade:
    - > "Simulations show that, while the `MPC` component is important for **enhancing the agent’s defence**, too much influence of this component on the `Q-L` can **reduce the attack performance** of the agent (this, in turn, is essential in order to score higher points)."

- About `MPC`:
  - > "At each time-step, the algorithm computes a **control `action`** by **solving an optimization** problem having as **constraint the dynamics** of the system being controlled. In addition to the dynamics, other **system requirements** (e.g. `safety` or `feasibility` requirements) can also be **formalized as constraints** of the optimization problem."

- Example: `pong`.
  - During the **defence** phase, `MPRL` used its `MPC` component since this phase is completely governed by the **physics** of the game and by the **moves** of the ego-agent.
  - During the **attack** phase,  `MPRL` used its `Q-L` component.
    - > "Indeed, even if a mathematical model describing the evolution of the position of the ball could be devised, there is **no difference equation** that **could predict what our opponent would do in response** (as we have no control over it)."
  - The **`MPC` takes over** when the paddle is not correctly posed to receive the ball, **ensuring that the ego-agent does not lose**.
    - > "Intuitively, the paddle was moved by the `MPC` component when the ball was **coming towards the `MPRL` paddle** and, at the same time, the **future vertical position** of the ball (predicted via the model) was **far from the actual position** of the agent’s paddle."
  - [`MPC` < `RL` for attack] "While `MPC` allows the agent to **defend**, it does not allow for the learning of an **attack strategy** to consistently obtain points."

- _How can it be applied to decision-making for AD?_
  - The task of driving is a **partially observable** and **partially controllable** problem:
    - `1-` When surrounded by vehicles, the **ego-`action` has full control on the `state-transition`**. Hence a `RL` agent can be used to cope with uncertainty.
    - `2-` But when the car is **driving alone**, a rule-based controller such `PID` / `MPC` could be used, since the **ego-car dynamic is fully known** (_deterministic_ transitions).
    - The `planning` part would **alleviates the `RL` exploration**, accelerating the learning process, but this does not provide any **safety guarantees**.

</details>

---

**`"End-to-end Reinforcement Learning for Autonomous Longitudinal Control Using Advantage Actor Critic with Temporal Context"`**

- **[** `2019` **]**
**[[:memo:](http://personal.ee.surrey.ac.uk/Personal/R.Bowden/publications/2019/Kuutti_ITSC2019pp.pdf)]**
**[** :mortar_board: `University of Surrey` **]**
**[** :car: `Jaguar Land Rover` **]**

- **[** _`sampling efficiency`, `switching actions`_  **]**

<details>
  <summary>Click to expand</summary>

| ![In the reward function, the `time headway` term encourages the agent to **maintain a `headway` close to `2s`**, while the `headway-derivative` term rewards the agent for taking actions which **bring it closer** to the ideal headway. [Source](http://personal.ee.surrey.ac.uk/Personal/R.Bowden/publications/2019/Kuutti_ITSC2019pp.pdf).](../media/2019_kuutti_2.PNG "In the reward function, the `time headway` term encourages the agent to **maintain a `headway` close to `2s`**, while the `headway-derivative` term rewards the agent for taking actions which **bring it closer** to the ideal headway. [Source](http://personal.ee.surrey.ac.uk/Personal/R.Bowden/publications/2019/Kuutti_ITSC2019pp.pdf).")  |
|:--:|
| *In the reward function, the `time headway` term encourages the agent to **maintain a `headway` close to `2s`**, while the `headway-derivative` term rewards the agent for taking actions which **bring it closer** to the ideal headway. [Source](http://personal.ee.surrey.ac.uk/Personal/R.Bowden/publications/2019/Kuutti_ITSC2019pp.pdf).* |

| ![Using **recurrent units** in the actor net leads to a **smoother driving style** and maintains a closer headway to the `2s` target. [Source](http://personal.ee.surrey.ac.uk/Personal/R.Bowden/publications/2019/Kuutti_ITSC2019pp.pdf).](../media/2019_kuutti_3.PNG "Using **recurrent units** in the actor net leads to a **smoother driving style** and maintains a closer headway to the `2s` target. [Source](http://personal.ee.surrey.ac.uk/Personal/R.Bowden/publications/2019/Kuutti_ITSC2019pp.pdf).")  |
|:--:|
| *Using **recurrent units** in the actor net leads to a **smoother driving style** and maintains a closer headway to the `2s` target. [Source](http://personal.ee.surrey.ac.uk/Personal/R.Bowden/publications/2019/Kuutti_ITSC2019pp.pdf).* |

Authors: Kuutti, S., Bowden, R., Joshi, H., Temple, R. De, & Fallah, S.

- Motivations for **"headway-keeping"**, i.e. _longitudinal_ control, using model-free `RL`:
  - `1-` Address inherent **sampling inefficiency**.
  - `2-` Address common **jerky driving behaviours**, i.e. aim at _smoother_ longitudinal trajectories.
    - > "Without any **temporal context** given, the agent has to decide the current action without any **consideration for the previous actions**, sometimes leading to **rapid switching** between the `throttle` and `brake` pedals."
- The task: keep a **`2s` time headway** from the lead vehicle in [`IPG CarMaker`](https://ipg-automotive.com/products-services/simulation-software/carmaker/).
  - The `state` consists in:
    - `ego-speed`
    - `ego-acc`
    - `delta speed to leader`
    - `time headway to leader`
  - _Personal note_: Since the `longitudinal` control of the ego-car has no influence on the speed of the leading vehicle, the `transition` function of this `MDP` should be **stochastic**.
- One idea for sampling efficiency: **_"Proxy Network for Vehicle Dynamics"_**.
  - For training, the simulator was **constrained** to running at **real-time** (timestep = `40ms`). At the same time, model-free methods require many samples.
  - One idea is to learn the **"ego-dynamics"**, i.e. one part of the `transition` function.
    - `input` = [`ego-speed`, `ego-acc`, `previous pedal action`, `current pedal action`, `road coefficient of friction`]
    - `output` = `ego-speed` at the next time-step.
  - The authors claim that this model (derived with _supervised learning_), can be used to replace the simulator when training the `RL` agent:
    - > "The training was completed using the proxy network in under `19` hours."
    - _As noted above, this `proxy net`_ **_does not capture the full transition_**. _Therefore, I do not understand how it can substitute the simulator and_ **_"self-generate"_** _samples, except if assuming_ **_constant speed_** _of the leading vehicle - which would boil down to some `target-speed-tracking` task instead._"
      - In addition, one could apply `planning` techniques instead of `learning`.
- One idea again **jerky action switch**:
  - The authors add a **`16` `LSTM` units** to the **actor** network.
  - Having a **recurrent cell** provides a **temporal context** and leads to **smoother predictions**.
- _How to deal with_ **_continuous actions_**_?_
  - `1-` The actor network estimates the action value **mean** `µ` and the estimated **variance** `σ`.
  - `2-` This transformed into a **Gaussian probability distribution**, from which the control action is then **sampled**.
- About the **"Sequenced"** **experience replay**:
  - The `RL` agent is train `off-policy`, i.e. the experience tuples used for updating the policy were not collected from the currently-updated policy, but rather **drawn from a replay buffer**.
  - Because the `LSTM` has an internal state, experience tuples <`s`, `a`, `r`, `s'`> cannot be sampled individually.
  - > "Since the network uses `LSTMs` for **temporal context**, these minibatches are sampled as **sequenced trajectories** of experiences."
- About the **"Trauma memory"**:
  - A **second set of experiences** is maintained and used during training.
  - This **"Trauma memory"** stores trajectories which **lead to collisions**.
  - The ratio of `trauma memory` samples to experience `replay samples` is set to `1/64`.

</details>

---

**`"Multi-lane Cruising Using Hierarchical Planning and Reinforcement Learning"`**

- **[** `2019` **]**
**[[:memo:](https://ieeexplore.ieee.org/document/8916928/)]**
**[** :car: `Huawei` **]**

- **[** _`hierarchical planning`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

| ![The high-level behavioural planner (`BP`) selects a `lane` while the underlying motion planner (`MoP`) select a `corridor`. [Source](https://ieeexplore.ieee.org/document/8916928/).](../media/2019_rezaee_1.PNG "The high-level behavioural planner (`BP`) selects a `lane` while the underlying motion planner (`MoP`) select a `corridor`. [Source](https://ieeexplore.ieee.org/document/8916928/).")  |
|:--:|
| *The high-level behavioural planner (`BP`) selects a `lane` while the underlying motion planner (`MoP`) select a `corridor`. [Source](https://ieeexplore.ieee.org/document/8916928/).* |

| ![The decision-making is **hierarchically divided** into three levels. The first two (`BP` and `MoP`) are **learning-based** while the last module that decides of low-level commands such as `throttle` and `steering` is left **rule-based** since it is car-specific. [Source](https://ieeexplore.ieee.org/document/8916928/).](../media/2019_rezaee_2.PNG "The decision-making is **hierarchically divided** into three levels. The first two (`BP` and `MoP`) are **learning-based** while the last module that decides of low-level commands such as `throttle` and `steering` is left **rule-based** since it is car-specific. [Source](https://ieeexplore.ieee.org/document/8916928/).")  |
|:--:|
| *The decision-making is **hierarchically divided** into three levels. The first two (`BP` and `MoP`) are **learning-based**. The `keep-lane` and `switch-lane` tasks are achieved using a **shared `MoP` agent**. The last module that decides of low-level commands such as `throttle` and `steering` is left **rule-based** since it is car-specific. [Source](https://ieeexplore.ieee.org/document/8916928/).* |

Authors: Rezaee, K., Yadmellat, P., Nosrati, M. S., Abolfathi, E. A., Elmahgiubi, M., & Luo, J.

- Motivation:
  - **Split** the decision-making process based on **different levels of abstraction**. I.e. **hierarchical** planning.
- About the **_hierarchical_** nature of **"driving"** and the concept of **_"symbolic punctuation"_**:
  - > "Different from regular robotic problems, _driving_ is heavily **symbolically punctuated** by _signs_ and _rules_ (e.g. _`lane markings`_, _`speed limit signs`_, _`fire truck sirens`_, _`turning signals`_, _`traffic lights`_) on top of what is largely a **continuous control task**."
  - In other words, `higher level` decisions on **discrete state transitions** should be coordinated with `lower level` motion planning and control in continuous state space.
- About the use of **_learning-based_** methods for **low-level** control:
  - The authors mentioned, among other, the [work](https://www.mobileye.com/wp-content/uploads/2016/01/Long-term-Planning-by-Short-term-Prediction.pdf) of `Mobileye`, where `RL` is used in the **planning phase** to model the **vehicle’s `acceleration`** given results from the prediction module.
    - > "Given well-established controllers such as `PID` and `MPC`, we believe that **learning-based methods** are more effective in the **high** and **mid-level** decision making (e.g. `BP` and `Motion Planning`) rather than **low-level controllers**."
- Another concept: **_"`skill`-based"_** planning:
  - It means that planning submodules are **specialized** for a driving **sub-task** (e.g. `lane keeping`, `lane switching`).
  - The authors introduce a **road abstraction**: a **`lane`** (selected by the `BP`) is divided into (`5`) **`corridors`** (selected by the `MoP`).
    - Corridor selection is equivalent to **selecting a path** among a set of **predefined paths**.
- Proposed structure:
  - `1-` The **Behavioural planner** (**`BP`**) outputs a **high-level decisions** in {`keep lane`, `switch to the left lane`, `switch to the right lane`}.
    - _Also, some target speed??? No, apparently a separated module sets some `target speed set-point` based on the `BP` desire and the physical boundaries, such as heading cars, or any interfering objects on the road - Not clear to me._
  - `2-` The **Motion planners** (**`MoP`**) outputs a `target corridor` and a `target speed`.
  - `3-` A **separated** and _non-learnt_ **trajectory controller** converts that into **low-level** commands (`throttle` and `acceleration`)
- About the high-level **_`Option`_**: _How to define `termination`?_
  - > "The typical solution is to assign a **fixed expiration time** to each option and penalize the agent if execution time is expired."
  - According the authors, `BP` **should not wait** until its command gets executed (since any **fixed lifetime for `BP` commands is dangerous**).
    - > "BP should be able to **update its earlier decisions**, at every time step, according to the new states."
- _How to train the two modules?_
  - > "We design a **coarse-grained reward** function and avoid any **fine-grained rules** in our reward feedback."
  - The reward function of the `MoP` depends on the `BP` objective:
    - Reward `+1` is given to the `MoP` agent if being in the middle corridor (_ok, but is the target lane considered?? Not clear to me_), `AND`:
      - `EITHER` the speed of the ego vehicle is within a threshold of the **`BP` target speed**,
      - `OR` the **minimum front gap** is within a threshold of the **safe distance** `d` (computed based on `TTC`).
  - The `MoP` is first trained with random `BP` commands (_target lanes are sampled every `20s`_). Then the `BP` agent is trained: its gets positive rewards for driving above a threshold while being penalized at each lane change.
  - Main limitation:
    - **Training** is done **separately** (both `BP` and `MoP` agents cannot adapt to each other, hence the final decision might be sub-optimal).
- Another advantage: _alleged_ easy **`sim-to-real` transfer**.
  - > "In practice, **low-level policies** may result in oscillatory or undesirable behaviours when deployed on **real-world** vehicles due to imperfect sensory inputs or **unmodeled kinematic** and **dynamic** effects."
    - > "Our state-action **space abstraction** allows **transferring** of the trained models from a simulated environment with virtually no dynamics to the one with **significantly more realistic dynamics** without a need for retraining."

</details>

---

**`"Learning to Drive using Waypoints"`**

- **[** `2019` **]**
**[[:memo:](https://ml4ad.github.io/files/papers/Learning%20to%20Drive%20using%20Waypoints.pdf)]**
**[** :mortar_board: `Carnegie Mellon University` **]**

- **[** _`PPO`, `waypoint-based navigation`, [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![The communication of **navigation goals** at intersection is not done using **high-level commands** (c.f. `conditional RL`), but rather by giving the `PPO` agent a **list of predefined waypoints** to follow. [Source](https://ml4ad.github.io/files/papers/Learning%20to%20Drive%20using%20Waypoints.pdf).](../media/2019_agarwal_1.PNG "The communication of **navigation goals** at intersection is not done using **high-level commands** (c.f. `conditional RL`), but rather by giving the `PPO` agent a **list of predefined waypoints** to follow. [Source](https://ml4ad.github.io/files/papers/Learning%20to%20Drive%20using%20Waypoints.pdf).")  |
|:--:|
| *The communication of **navigation goals** at intersection is not done using **high-level commands** (c.f. `conditional RL`), but rather by giving the `PPO` agent a **list of predefined waypoints** to follow. [Source](https://ml4ad.github.io/files/papers/Learning%20to%20Drive%20using%20Waypoints.pdf).* |

Authors: Agarwal, T., Arora, H., Parhar, T., Deshpande, S., & Schneider, J.

- One idea (**alternative to `conditional learning`**):
  - > "Past approaches have used a **higher-level planner** that directs the agent using **high-level commands** on turning. Instead of this, we propose to use **trajectory waypoints to guide navigation**, which are readily available in real world autonomous vehicles."
  - Using such a **predefined path** probably constrains applications to **single-lane scenarios**, and relies on an **up-to-date HD map**.
  - > "We acknowledge that both the baseline methods use **higher level navigation features** and **RGB images** in contrast to richer **low-level waypoint** features and simpler **semantically segmented** images used in our approach."
- The agent policy network (`PPO`) takes two inputs:
  - `1-` The bottleneck embedding of an **auto-encoder** applied on **semantically segmented images**.
  - `2-` The **coming `n` waypoints** to follow. They are `2m`-spaced and are extracted from an offline map.
  - Both are independant of the realistism of the simulator it has trained on. One could therefore expect the approach to **transfer well to real-world**.

</details>

---

**`"Social Attention for Autonomous Decision-Making in Dense Traffic"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1911.12250)]**
**[[🎞️](https://eleurent.github.io/social-attention/)]**
**[** :mortar_board: `Inria` **]**
**[** :car: `Renault` **]**

- **[** _`attention mechanism`_ **]**

<details>
  <summary>Click to expand</summary>

| ![ Weights in the **encoding linear** are shared between all vehicles. Each **encoding** contains **individual features** and has size `dx`. For each **head in the stack**, different **linear projections** (`Lq`, `Lk`, `Lv`) are applied on them. Results of projections are `key` and `values` (plus a `query` for the ego-agent). Based on the **similarity** between the ego-query `q0` and the `key`s, an **attention matrix** is built. This matrix should **select a subset of vehicles** that are important, depending on the context. It is multiplied with the concatenation of the individual `values features`, and then passed to a **decoder** where results from all heads are combined. The output are the estimated `q-values`.. [Source](https://arxiv.org/abs/1911.12250).](../media/2019_leurent_1.PNG " Weights in the **encoding linear** are shared between all vehicles. Each **encoding** contains **individual features** and has size `dx`. For each **head in the stack**, different **linear projections** (`Lq`, `Lk`, `Lv`) are applied on them. Results of projections are `key` and `values` (plus a `query` for the ego-agent). Based on the **similarity** between the ego-query `q0` and the `key`s, an **attention matrix** is built. This matrix should **select a subset of vehicles** that are important, depending on the context. It is multiplied with the concatenation of the individual `values features`, and then passed to a **decoder** where results from all heads are combined. The output are the estimated `q-values`. [Source](https://arxiv.org/abs/1911.12250).")  |
|:--:|
| *Weights in the **encoding linear** are shared between all vehicles. Each **encoding** contains **individual features** and has size `dx`. For each **head in the stack**, different **linear projections** (`Lq`, `Lk`, `Lv`) are applied on them. Results of projections are `key` and `values` (plus a `query` for the ego-agent). Based on the **similarity** between the ego-query `q0` and the `key`s, an **attention matrix** is built. This matrix should **select a subset of vehicles** that are important, depending on the context. It is multiplied with the concatenation of the individual `values features`, and then passed to a **decoder** where results from all heads are combined. The output are the estimated `q-values`.. [Source](https://arxiv.org/abs/1911.12250).* |

| ![ Example with a stack of **two heads**. Both direct their attention to **incoming vehicles** that are **likely to collide** with the ego-vehicle. Visualization of the **`attention matrix`**: The ego-vehicle is connected to every vehicle by a line whose width is proportional to the corresponding **attention weight**. The green head is only watching the vehicles coming **from the left**, while the blue head restricts itself to vehicles in the **front and right directions**.. [Source](https://arxiv.org/abs/1911.12250).](../media/2019_leurent_2.PNG " Example with a stack of **two heads**. Both direct their attention to **incoming vehicles** that are **likely to collide** with the ego-vehicle. Visualization of the **`attention matrix`**: The ego-vehicle is connected to every vehicle by a line whose width is proportional to the corresponding **attention weight**. The green head is only watching the vehicles coming **from the left**, while the blue head restricts itself to vehicles in the **front and right directions**. [Source](https://arxiv.org/abs/1911.12250).")  |
|:--:|
| *Example with a stack of **two heads**. Both direct their attention to **incoming vehicles** that are **likely to collide** with the ego-vehicle. Visualization of the **`attention matrix`**: The ego-vehicle is connected to every vehicle by a line whose width is proportional to the corresponding **attention weight**. The green head is only watching the vehicles coming **from the left**, while the blue head restricts itself to vehicles in the **front and right directions**.. [Source](https://arxiv.org/abs/1911.12250).* |

Authors: Leurent, E., & Mercat, J.

- Question: About the **`MDP`** **`state`** (or **representation of driving scene**): **_how surrounding vehicles can be represented?_**
- Motivations / requirements:
  - `1-` Deal with a **varying number** of surrounding vehicles (problematic with **function approximation** which often expects **constant-sized** inputs).
  - `2-` The driving policy should be **permutation-invariant** (invariant to the **ordering** chosen to describe them).
  - `3-` Stay **accurate** and **compact**.
- Current approaches:
  - **List of features** representation (fails at `1` and `2`).
    - `Zero-padding` can help for varying-size inputs.
  - **Spatial grid** representation (suffers from accuracy-size trade-off).
- One concept: **"Multi-head social attention mechanism"**.
  - The `state` may contain many types of information. But the agent should only **pay attention** to vehicles that are **close or conflict** with the planned route.
    - > "Out of a **complex scene description**, the model should be able to **filter information** and **consider only what is relevant for decision**."
  - About **_"attention mechanism"_**:
    - > "The **attention** architecture was introduced to enable neural networks to **discover interdependencies** within a **variable number of inputs**".
    - For each head, a **stochastic matrix** called the `attention matrix` is derived.
    - The visualisation of this **attention matrix** brings **interpretability**.

</details>

---

**`"End-to-End Model-Free Reinforcement Learning for Urban Driving using Implicit Affordances"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1911.10868)]**
**[[🎞️](https://www.youtube.com/watch?v=YlCJ84VO3cU)]**
**[** :mortar_board: `Mines ParisTech` **]**
**[** :car: `Valeo` **]**

- **[** _`affordance learning`, [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![ An **encoder** is trained to predict **high-level information** (called **`affordances`**). The `RL` agent does not use directly them as input `state` but rather one layer before (hence **`implicit`** affordances). This **compact representation** offers benefits for **interpretability**, and for **training efficiency** (lighter to save in the replay buffer). A **command** {`follow lane`, turn `left`/`right`/`straight`, change lane `left`/`right`} for **direction at intersection** and **lane changes** is passed to the agent via a **conditional branch**. [Source](https://arxiv.org/abs/1911.10868).](../media/2019_toromanoff_1.PNG " An **encoder** is trained to predict **high-level information** (called **`affordances`**). The `RL` agent does not use directly them as input `state` but rather one layer before (hence **`implicit`** affordances). This **compact representation** offers benefits for **interpretability**, and for **training efficiency** (lighter to save in the replay buffer). A **command** {`follow lane`, turn `left`/`right`/`straight`, change lane `left`/`right`} for **direction at intersection** and **lane changes** is passed to the agent via a **conditional branch**. [Source](https://arxiv.org/abs/1911.10868).")  |
|:--:|
| *An **encoder** is trained to predict **high-level information** (called **`affordances`**). The `RL` agent does not use directly them as input `state` but rather one layer before (hence **`implicit`** affordances). This **compact representation** offers benefits for **interpretability**, and for **training efficiency** (lighter to save in the replay buffer). A **command** {`follow lane`, turn `left`/`right`/`straight`, change lane `left`/`right`} for **direction at intersection** and **lane changes** is passed to the agent via a __conditional branch__. [Source](https://arxiv.org/abs/1911.10868).* |

| ![ **Augmentation** is needed for robustness and generalization (to address the **distribution mismatch** - also in `IL`) (_left_). Here, the **camera is moved around** the autopilot. One main finding is the benefit of using **adaptive target speed** in the reward function (_right_). [Source](https://arxiv.org/abs/1911.10868).](../media/2019_toromanoff_2.PNG " **Augmentation** is needed for robustness and generalization (to address the **distribution mismatch** - also in `IL`) (_left_). Here, the **camera is moved around** the autopilot. One main finding is the benefit of using **adaptive target speed** in the reward function (_right_). [Source](https://arxiv.org/abs/1911.10868).")  |
|:--:|
| *__Augmentation__ is needed for robustness and generalization (to address the **distribution mismatch** - also in `IL`) (_left_). Here, the **camera is moved around** the autopilot. One main finding is the benefit of using **adaptive target speed** in the reward function (_right_). [Source](https://arxiv.org/abs/1911.10868).* |

Authors: Toromanoff, M., Wirbel, E., & Moutarde, F.

- One quote:
  - > "A promising way to solve both the **data efficiency** (particularly for DRL) and the **black box problem** is to use **privileged information** as **auxiliary losses**, also coined **"affordances"** in some recent papers."
- One idea: The **`state`** of the `RL` agent are **affordance predictions** produced by a separated network.
  - `1-` An **encoder** in trained in a **supervised way** to predict **high-level information**, using a **stack of images** as input.
    - Two main losses for this supervised phase:
      - `1-` **Traffic light state** (binary classification).
      - `2-` **semantic segmentation**.
    - Infordances inclues:
      - Semantic segmentation maps _(ground-truth available directly in CARLA)_.
      - Distance and state of the incoming traffic light.
      - Whether the agent is at an intersection or not.
      - Distance from the middle of the lane.
      - Relative rotation to the road.
    - Why **_"implicit"_** affordance?
      - > "We coined this scheme as **"implicit affordances"** because the RL agent **implicit affordances** because the RL agent do not use the **_explicit_** predictions but have only access to the **_implicit_** features (i.e the features from which our initial supervised network predicts the explicit affordances)."
      - Hence multiple affordances are predicted in order to **help the supervised training** (similar to `auxiliary learning`).
    - This feature extractor is **frozen** while training the `RL` agent.
  - `2-` These predicted affordances serve as **input** to a **model-free** `RL` agent.
    - The dueling network of the `Rainbow- IQN Ape-X` architecture was removed (very big in size and no clear improvement).
    - Training is **distributed**:
      - > "CARLA is too slow for RL and cannot generate enough data if only one instance is used."
    - Despite being limited to **discrete actions** (`4` for `throttle`/`brake` and `9`-`24` for `steering`), it shows very good results.
      - > "We also use a really simple yet effective trick: we can reach **more fine-grained discrete actions** by using a **bagging of multiple** [_here `3` consecutive_] **predictions** and average them."
    - **Frequency** of decision is not mentioned.
  - Benefits of this architecture:
    - Affordances features are way **lighter** compared to images, which enables the use of a **replay buffer** (off-policy).
      - All the more, since the images are bigger than usual (`4` **`288`x`288`** frames are concatenated, compared to "classical" single `84`x`84` frames) in order to **capture the state of traffic lights**.
    - This **decomposition** also brings some **interpretable feedback** on how the decision was taken (affordances could also be used as input of a _rule-based_ controller).
  - This `2`-stage approach reminds me the concept **`"learn to see"` / `"learn to act"`** concept of (Chen et al. 2019) in `"Learning by Cheating"`.
    - > "As expected, these experiments prove that **training a large network using only `RL` signal is hard**".

- About **"model-free"** `RL`.
  - As opposed to "model-based", e.g. [(Pan et al. 2019)](https://ieeexplore.ieee.org/document/8794437) where a network is also trained to predict high-level information.
  - But these affordances rather relate to the **transition model**, such as **probability of collision** or **being off-road** in the near futures from a **sequence of observations and actions**.
- About the **reward** function:
  - It relies on `3` components:
    - `1-` Desired **`lateral position`**.
      - To stay in the **middle of the lane**.
    - `2-` Desired **`rotation`**.
      - To prevent **oscillations** near the center of lane.
    - `3-` Desired **`speed`**, which is **adaptive**.
      - > "When the agent arrives near a red traffic light, the **desired speed goes linearly to `0`** (the closest the agent is from the traffic light), and **goes back to maximum allowed speed when it turns green**. The same principle is used when arriving behind an **obstacle**, **pedestrian**, **bicycle** or **vehicle**."
      - The authors find that **without this adaptation**, the agent **fails totally** at braking for both cases of red traffic light or pedestrian crossing.

</details>

---

**`"Hierarchical Reinforcement Learning Method for Autonomous Vehicle Behavior Planning"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1911.03799)]**
**[** :mortar_board: `Carnegie Mellon` **]**
**[** :car: `General Motors` **]**

- **[** _`Hierarchical RL`_ **]**

<details>
  <summary>Click to expand</summary>

| ![ Driving is treated as a _task_ with **multiple sub-goals**. A _meta controller_ choose an **option** among **`STOP AT STOP-LINE`**, **`FOLLOW FRONT VEHICLE`**. Based on this **sub-goal**, a _low-level controller_ decides on the `THROTTLE` and `BRAKE` **action**. [Source](https://arxiv.org/abs/1911.03799).](../media/2019_qiao_1.PNG "Driving is treated as a _task_ with **multiple sub-goals**. A _meta controller_ choose an **option** among **`STOP AT STOP-LINE`**, **`FOLLOW FRONT VEHICLE`**. Based on this **sub-goal**, a _low-level controller_ decides on the `THROTTLE` and `BRAKE` **action**: [Source](https://arxiv.org/abs/1911.03799).")  |
|:--:|
| *`Driving` is treated as a _task_ with **multiple sub-goals**. A `meta` controller choose an **option** among **`STOP AT STOP-LINE`**, **`FOLLOW FRONT VEHICLE`**. Based on this **sub-goal**, a `low-level` controller decides the `THROTTLE` and `BRAKE` **action**. [Source](https://arxiv.org/abs/1911.03799).* |

| ![ The **`state`** is **shared** by the different hierarchical layers (`option` and `action`). An **attention model** extracts the relevant information for the current **`sub-goal`**. [Source](https://arxiv.org/abs/1911.03799).](../media/2019_qiao_2.PNG "The **`state`** is **shared** by the different hierarchical layers (`option` and `action`). An **attention model** extracts the relevant information for the current **`sub-goal`**. [Source](https://arxiv.org/abs/1911.03799).")  |
|:--:|
| *The **`state`** is **shared** across different hierarchical layers (`option` and `action`). An **attention model** extracts the relevant information for the current **`sub-goal`**. [Source](https://arxiv.org/abs/1911.03799).* |

Authors: Qiao, Z., Tyree, Z., Mudalige, P., Schneider, J., & Dolan, J. M.

- Motivations:
  - `1-` Have an **unique learning-based method** (`RL`) for behavioural decision-making in an environment where the agent must **pursue multiple sub-goals**.
  - `2-` Improve **convergence speed** and **sampling efficiency** of traditional `RL` approaches.
  - These are achieved with **hierarchical `RL`**, by **reusing** the different learnt policies across **similar tasks**.
- About the **hierarchical** structure: concepts of **_"Actions_"** and **_"Options"_**.
  - `1-` First, a high-level **option**, seen as a **`sub-goal`**, is selected by some **"meta-controller"**, depending on the scenario:
    - `Option_1` = **`STOP AT STOP-LINE`**.
    - `Option_2` = **`FOLLOW FRONT VEHICLE`**.
  - `2-` Based on the selected `sub-goal`, an **action network** generates **low-level longitudinal** **_"actions"_** about **`throttle`** and **`brake`**.
    - The continues until a next **sub-goal** is generated by the **meta controller**.
- About the **`state`**:
  - The authors decide to **share one state set** for the **whole hierarchical** structure.
  - For each sub-goal, only **relevant information is extracted** using an **attention mechanism**:
  - > "An **attention model** is applied to define the **importance** of each state element **`I`(`state`, `option`)** with respect to each hierarchical level and **sub-goal**".
  - For instance:
    - > "When the ego car is **approaching the front vehicle**, the **attention** is mainly **focused on `dfc`/`dfs`**." (`1` `-` `distance_to_the_front_vehicle` `/` `safety_distance`)
    - > "When the front vehicle leaves without stopping at the stop-line, **the ego car transfers more and more attentions** to **`ddc`/`dds`** during the process of **approaching the stop-line**." (`1` `-` `distance_to_stop_sign` `/` `safety_distance`)
- About the **reward function**:
  - The authors call that **"hybrid reward mechanism"**:
    - An **"`extrinsic` meta reward"** for the **`option`-level**.
    - An **"`intrinsic` reward"** for the **`action`-level**, **conditioned** on the selected `option`.
  - For instance, the terms related to the `target_speed` in the **intrinsic reward** will be **adapted** depending if the meta controller defines a `highway driving` or an `urban driving` sub-goal.
  - At first sight, this reward function embeds **a lot of parameters**.
    - One could say that the effort **parameter engineering** in **rule-based** approaches has been transferred to the **reward-shaping**.
- One idea: **"Hierarchical Prioritized Experience Replay"** (**`HPER`**).
  - When updating the weights of the two networks, transitions {`s`, `o`, `a`, `ro`, `ra`, `s'`} are considered.
  - > But "If the output of the **option-value network `o` is chosen wrongly**, then the success or failure of the corresponding action-value network is **inconsequential to the current transition**."
  - In other words, for a **fair evaluation** of the `action`, it would be nice to have the **correct option**.
  - One solution consists in **setting priorities** when sampling from the **replay buffer**:
    - In the **option-level**, priorities are based on the error directly (the `TD-error` for the `optionNet`, similar to the traditional `PER`).
    - In the **lower level**, priorities are based on the **difference between errors** coming from two levels, i.e. the difference between (`TD-error` for the `actionNet`) and (`TD-error` for the `optionNet`).
- About the simulator: [`VIRES VTD`](https://vires.com/vtd-vires-virtual-test-drive/).

</details>

---

**`"DeepRacer: Educational Autonomous Racing Platform for Experimentation with Sim2Real Reinforcement Learning"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1911.01562)]**
**[[:octocat:](https://github.com/awslabs/amazon-sagemaker-examples/tree/master/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo)]**
**[[🎞️](https://github.com/awslabs/amazon-sagemaker-examples/tree/master/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/videos)]**
**[** :car: `Amazon` **]**

- **[** _`sim2real`, `end-to-end`, `sampling efficiency`, `distributed RL`_ **]**

<details>
  <summary>Click to expand</summary>

| ![ `real2sim` experiment for `model-free RL` `end-to-end` (from **monocular camera** to low-level controls) **track following** using a **`1/18th` scale** car. [Source](https://arxiv.org/abs/1911.01562).](../media/2019_balaji_2.PNG "`real2sim` experiment for `model-free RL` `end-to-end` (from **monocular camera** to low-level controls) **track following** using a **`1/18th` scale** car. [Source](https://arxiv.org/abs/1911.01562).")  |
|:--:|
| *`real2sim` experiment for `model-free RL` `end-to-end` (from **monocular camera** to low-level controls) **track following** using a **`1/18th` scale** car. [Source](https://arxiv.org/abs/1911.01562).* |

| ![ Comparison of tools to perform **`RL` `sim2real`** applications. [Source](https://arxiv.org/abs/1911.01562).](../media/2019_balaji_1.PNG "Comparison of tools to perform **`RL` `sim2real`** applications. [Source](https://arxiv.org/abs/1911.01562).")  |
|:--:|
| *Comparison of tools to perform **`RL` `sim2real`** applications. [Source](https://arxiv.org/abs/1911.01562).* |

Authors: Balaji, B., Mallya, S., Genc, S., Gupta, S., Dirac, L., Khare, V., Roy, G., Sun, T., Tao, Y., Townsend, B., Calleja, E., Muralidhara, S. & Karuppasamy, D.

- _What?_
  - > "`DeepRacer` is an experimentation and **educational platform** for **`sim2real`** `RL`."
- About the `POMDP` and the algorithm:
  - **Observation**: `160x120` **grayscale** front view.
  - **Action**: `10` discretized values: **`2`** levels for **throttle** and **`5`** for **steering**.
  - **Timestep**: One action per observation. At **`15 fps`** (average speed `1.6 m/s`).
  - `model-free` (_that is important_) `PPO` to learn the policy.
- Tools:
  - [`Amazon RoboMaker`](https://aws.amazon.com/de/robomaker/): An extension of the `ROS` framework with **cloud services**, to develop and test deploy this robot software.
  - [`Amazon SageMaker`](https://aws.amazon.com/de/sagemaker/): The Amazon cloud **platform** to train and deploy machine learning models at scale using the `Jupyter` Notebook.
  - [`Amazon S3`](https://aws.amazon.com/s3/) (Simple **Storage** Service) to **save** and store the neural network **models**.
  - [`OpenAI Gym`] **interface** between the `agent` and the `env`.
  - [`Intel Coach`](https://github.com/NervanaSystems/coach) **`RL` framework** for easy experimentation.
  - [`Intel OpenVINO`](https://docs.openvinotoolkit.org/), a toolkit for quick development of vision-based applications, to convert the `Tensorflow` models to an **optimized binary**.
  - [`Gazebo`](http://gazebosim.org/) robotics **simulator**.
  - [`ROS`](https://www.ros.org/) for the **communication** between the agent and the simulation.
  - [`ODE`](https://www.ode.org/) (Open **Dynamics** Engine) to simulate the laws of physics using the robot model.
  - [`Ogre`](https://www.ogre3d.org/) as graphics **rendering** engine.
  - [`Redis`](https://redis.io/), an in-memory **database**, as a buffer to store the **experience tuples** <`obs`, `action`, `reward`, `next_obs`>.
- About **`sim2real`**:
  - The policy is **learnt** (_+tested_) from **simulations** ("replica track") and then **transferred** (_+tested_) to real world.
  - > "The entire process from training a policy to testing in the real car takes `< 30` minutes."
  - The authors mention several approaches for `sim2real`:
    - **Mixing** `sim` and `real` experiences during training:
      - E.g. learn **features** from a combination of simulation and real data.
      - Mix **expert demonstrations** with simulations.
    - **Model-based** dynamics transfer:
      - Assess simulation bias.
      - Learn **model ensembles**.
      - Perform **calibration**: here, they have to **match the robot model** to the measured dimensions of the car.
    - **Domain randomization**: simulation **parameters are perturbed** during training to **gain robustness**.
      - Add **adversarial** noise.
      - **Observation noise**, such as _random colouring_.
      - **Action noise** (up to `10%` uniform random noise to `steering` and `throttle`).
      - Add "reverse direction of travel" each episode (_I did not understand_).
    - **`Privileged learning`**:
      - E.g. learn **semantic segmentation** at the same time, as an **auxiliary task**.
  - They note that (_I thought they were using grayscale?_):
    - > "**Random colour** was the **most effective** method for **`sim2real` transfer**."
- About the **distributed rollout** mechanism, to improve the **sampling efficiency**:
  - > "We introduce a training mechanism that **decouples `RL` policy updates** with the **rollouts**, which enables independent scaling of the simulation cluster."
  - As I understand, **multiple simulations** are run in parallel, with the **same dynamics model**.
  - Each worker sends its **experience tuples** (_not the gradient_) to the central `Redis` buffer where **updates of the shared model** are performed.

</details>

---

**`"Autonomous Driving using Safe Reinforcement Learning by Incorporating a Regret-based Human Lane-Changing Decision Model"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1910.04803)]**
**[** :mortar_board: `Michigan State University, Clemson University` **]**

- **[** _`safe RL`, `action masking`, `motivational decision model`, `regret theory`, [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![ Every time after the `RL` agent chooses an action, the **supervisor checks** whether this action is `safe` or not. It does that by **predicting** the trajectories of other cars using a **regret decision model**. [Source](https://arxiv.org/abs/1910.04803).](../media/2019_chen_1.PNG "Every time after the `RL` agent chooses an action, the **supervisor checks** whether this action is `safe` or not. It does that by **predicting** the trajectories of other cars using a **regret decision model**. [Source](https://arxiv.org/abs/1910.04803).")  |
|:--:|
| *Every time after the `RL` agent chooses an action, the **supervisor checks** whether this action is `safe` or not. It does that by **predicting** the trajectories of other cars using a **regret decision model**. [Source](https://arxiv.org/abs/1910.04803).* |

| ![ The two actions (`keep lane` and `change lane`) have different **probabilities of occurrence**, and different **harms** (**costs**) (`I`) that can be formulated as utilities (`II`) expressed with physical variables (`III`). **Affordance indicators** are used in the **world representation**. [Source](https://arxiv.org/abs/1910.04803).](../media/2019_chen_2.PNG "The two actions (`keep lane` and `change lane`) have different **probabilities of occurrence**, and different **harms** (**costs**) (`I`) that can be formulated as utilities (`II`) expressed with physical variables (`III`). **Affordance indicators** are used in the **world representation**. [Source](https://arxiv.org/abs/1910.04803).")  |
|:--:|
| *The two actions (`keep lane` and `change lane`) have different **probabilities of occurrence**, and different **harms** (**costs**) (`I`) that can be formulated as **utilities** (`II`), expressed with **physical variables** (`III`). **Affordance indicators** are used in the **world representation**. [Source](https://arxiv.org/abs/1910.04803).* |

Authors: Chen, D., Jiang, L., Wang, Y., & Li, Z.

- Main motivation for `safeRL` for two-lane highway scenarios:
  - > "We found that when using **conventional `RL`**, about `14.5%` training epochs ended with collisions. [...] Collisions cause **training unstable** as the RL algorithm needs to **reset the simulation** whenever collisions happen."
- One idea: A **"safety supervisor"** evaluates the consequences of an ego-action and decides **whether an it is _"safe"_** using a **prediction model**.
  - _"What is predicted?"_
    - The **`lane-changing` decisions** of other cars.
  - _"How?"_
    - The approach belongs to the **_motivational_** methods, hence **explainable** (as opposed to the **_data-driven_** methods).
    - A **"regret decision model"** is developed:
      - > "**`Risks`** in driving have two dimensions: **`harm`** (=`costs`) and **`probabilities`**."
      - A `probability` (relating to the `time-to-collision`), a `weighting effect` function (non-linear function of the probability), a `cost` function, a `utility` (cost normalized by `collision cost`) and a `regret` (non-linear function of the utility) are defined.
      - The (_deterministic_) decision is based on the resulting the **`net advantage`** (**~weighted regret**) of **option `C`** (_change_) over **option `K`** (_keep lane_).
  - _"Can we draw a parallel with how human drivers reason?"_
    - > "Not all the drivers have experienced collisions, but all of them can **perceive the threat** of a potential collision. It is plausible to assume that the surrounding driver perceives the threat of the approaching vehicle to be **proportional to its kinematic energy**." Hence somehow proportional to `speed²`.
  - _"What does `safe` means here?"_
    - Short-horizon **rollouts** are performed.
      - `1-` The ego trajectory is anticipated based on the `action` candidate.
      - `2-` Trajectories are obtained for other all cars using the **predictor** and their current speeds.
    - The action is **considered `safe`** if the ego agent **stays on the road** and if his **trajectories does not intersect** (_in spatial-temporal domain_) with any other trajectory.
  - _What if the action is deemed `unsafe`?_
    - `1-` The supervisor **replaces** the identified risky action with a safe option (_probably based on the model_).
      - This **avoids terminating the episode** and starting a new one during _training_ and obviously it improves safety during _testing_.
    - `2-` An experience tuple <`state`, `action`, `r_collision`, `*` (_no next-action_)> is created and added to the **experience replay buffer** (together with _safe_ tuples). This will be **sampled** during the update phase.
      - It reminds me the idea of another work by the authors (["Deep Q-Learning with Dynamically-Learned Safety Module : A Case Study in Autonomous Driving"](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving) - detailed below) where **two distinct buffers** (`safe` and `collision`) are maintained and **equally sampled**.
  - _"What values for the model parameters?"_
    - Ideally, these values should be **inferred for each observed car** (they are parts of the **hidden state**). For instance, within a `POMDP` formulation, using a **belief tracker**.
    - Here, a single set of parameters is derived via `max-likelihood` based on **human demonstration**.
  - _"What is the impact on training?"_
    - It is **faster** and **more stable** (almost **constant improving rate**) compared to conventional `RL`.
- Another idea: **_hierarchical_ decision-making**:
  - The `RL` agent first selects _"manoeuvre-like"_ decisions among {`decelerating`, `cruising`, `accelerating`} and {`lane change`, `lane keeping`}.
  - Its decision is then **implemented by a low-level controller** (`PID`).
    - > "This hierarchical design greatly **reduces the training time** compared to methods using agents to **output control signals directly**."
- One related previous work (about **_regret decision model_**):
  - ["A human-computer interface design for quantitative measure of regret theory"](https://arxiv.org/abs/1810.00462) - (Jiang L. & Wang Y., 2019).

</details>

---

**`"Learning Resilient Behaviors for Navigation Under Uncertainty Environments"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1910.09998)]**
**[[🎞️](https://www.youtube.com/watch?v=KxRJp_Aanpo)]**
**[[🎞️](https://sites.google.com/view/resilient-nav/)]**
**[** :mortar_board: `Fuzhou University, University of Maryland, University of Hong Kong` **]**
**[** :car: `Baidu` **]**

- **[** _`uncertainty estimation`, `uncertainty-aware policy`, `SAC`_ **]**

<details>
  <summary>Click to expand</summary>

| ![ The **confidence in the prediction** (left) is used as an **`uncertainty` estimate**. This estimate impacts the decision (`μ` = **`mean` of the steering action distribution**) of the agent. [Source](https://arxiv.org/abs/1910.09998).](../media/2019_fan_2.PNG "The **confidence in the prediction** (left) is used as an **`uncertainty` estimate**. This estimate impacts the decision (`μ` = **`mean` of the steering action distribution**) of the agent. [Source](https://arxiv.org/abs/1910.09998).")  |
|:--:|
| *The **confidence in the prediction** (left) is used as an **`uncertainty` estimate**. This estimate impacts the decision (`μ` = **`mean` of the steering action distribution**) of the agent. [Source](https://arxiv.org/abs/1910.09998).* |

| ![ The `variance` of the steering action distribution (**`behavioural uncertainty`**) is not estimated by the agent itself, but rather built by a simple **mapping-function** from the **`environmental uncertainty`** estimated by the **prediction module**. [Source](https://arxiv.org/abs/1910.09998).](../media/2019_fan_1.PNG "The `variance` of the steering action distribution (**`behavioural uncertainty`**) is not estimated by the agent itself, but rather built by a simple **mapping-function** from the **`environmental uncertainty`** estimated by the **prediction module**. [Source](https://arxiv.org/abs/1910.09998).")  |
|:--:|
| *The `variance` of the steering action distribution (**`behavioural uncertainty`**) is not estimated by the agent itself, but rather built by a simple **mapping-function** from the **`environmental uncertainty`** estimated by the **prediction module**. [Source](https://arxiv.org/abs/1910.09998).* |

Authors: Fan, T., Long, P., Liu, W., Pan, J., Yang, R., & Manocha, D.

- One motivation: derive an **uncertainty-aware** + **`model-free`** + **`RL`** policy.
  - **_"Uncertainty-aware"_**:
    - The core idea is to forward the **observation uncertainty** to the **behaviour uncertainty** (i.e. **uncertainty of the action distribution**), in order to **boost exploration** during training.
  - `Model-free` as opposed to `model-based` `RL` methods.
    - In `model-based` methods, the **collision probability** and **uncertainty prediction** can [**explicitly**] be **formulated** into the **risk term** for an `MPC` to minimize.
    - Here, the **action selection** is directly output by a net, based on **raw laser data** (i.e. not from some `MPC` that would require challenging parameter tuning).
  - **`RL`** as opposed to `IL` (where _uncertainty estimation_ has already been applied, by recycling techniques of supervised learning).
    - Besides, in `IL`, it is difficult to learn policies to **actively avoid** uncertain regions.
- The proposed framework consists in `3` parts:
  - `1-` A **prediction** net:
    - Inputs: A **sequence of laser scans** and velocity data.
    - Outputs:
      - `1-1.` The **motion prediction** of surrounding environments (_this seems to be trajectories_).
      - `1-2.` Some associated **uncertainty**: _"we expect to measure the_ _**environmental uncertainty**_ _by computing the confidence of the prediction results."_
    - It is trained with **supervised learning** with recorded trajectories. The loss function therefore **discourages large uncertainty** while **minimizing prediction errors**.
  - `2-` A **policy net**:
    - Inputs:
      - `2-1.` The two estimates (`uncertainty` and `motion information`) of the predictor.
      - `2-2.` The current `laser scan`, the current `velocity` and the relative `goal position`.
    - Outputs: The **`mean`*** of the **action distribution** (`steering`).
    - It is trained with model-free `RL`, using the `ROS`-based [**`Stage`**](http://wiki.ros.org/stage) simulator.
  - `3-` A parallel **NON-LEARNABLE** **`mapping function`** estimates the **`variance`** of that action distribution directly from the environmental uncertainty.
    - Input: The "predicted" environmental uncertainty. (_I would have called it `observation uncertainty`_).
    - Output: The **`variance`** of the action distribution (`steering`).
    - About the _mapping_: The uncertainty predictions are **weighted** according to the distance of each laser point:
      - > "We consider that **the closer the laser point**, the **higher the impact on the action**."
    - Again, the **variance** of the action distribution **is not learnt!**
      - It reminds me the work of [(Galias, Jakubowski, Michalewski, Osinski, & Ziecina, 2019)](https://deepsense.ai/wp-content/uploads/2019/06/Simulation-based-reinforcement-learning-for-autonomous-driving.pdf) where best results are achieved when the policy outputs **both the `mean` and `variance`**.
      - Here, the policy network should learn to **adaptively generate the `mean` value** according to the `variance` value (capturing the environment uncertainty), e.g. exhibit more **conservative behaviours** in the case of **high environmental uncertainty**.
- About the `RL` method: `SAC` = **Soft Actor-Critic**.
  - The above defined **mapping forwards environmental uncertainties** to the **action variance**.
    - The idea is then to **encourage the agent to reduce this action variance** (`distribution entropy`) in order to obtain some **"uncertainty-averse"** behaviour.
    - **`SAC`** is appropriate for that:
  - > "The key idea of `SAC` is to maximize the `expected return` and **`action entropy`** together instead of the expected return itself to balance the exploration and exploitation."
    - `SAC` trains a **stochastic policy** with **entropy regularization**, and explores in an **on-policy** way.
    - My interpretation:
      - `1-` The agent is **given** (non-learnable) an `action variance` from the uncertainty mapping.
      - `2-` This **impacts its objective function**.
      - `3-` It will therefore try to **decrease this uncertainty of action distribution** and by doing so will try to **minimize the environmental uncertainty**.
      - `4-` Hence more exploration during the training phase.
  - Similar to the **`ε`-greedy annealing process** in `DQNs`, the **`temperature` parameter** is decayed during training to weight between the two objectives (`entropy` of policy distribution and `expected return`).
    - `SAC` incorporates the `entropy` measure of the policy into the reward to encourage exploration, i.e. the agent should act **as randomly as possible** [encourage uniform action probability] while it is still able to **succeed** at the task.
- Bonus (not directly connected to their contributions): _How to model uncertainties in `DNN`s?_
  - > "The **`aleatoric`** uncertainty (_data_ uncertainty) can be modelled by a specific **loss function** for the **uncertainty term** in the network output".
  - > "The **`epistemic`** uncertainty (i.e. _model_ uncertainty) can be captured by the **Monte-Carlo Dropout** (`MC`-Dropout) technique" - _dropout can be seen as a Bayesian approximation_.

</details>

---

**`"Deep Q-Learning with Dynamically-Learned Safety Module: A Case Study in Autonomous Driving"`**

- **[** `2019` **]**
**[[:memo:](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving)]**
**[** :mortar_board: `University of Michigan`, `West Virginia University` **]**
**[** :car: `Ford` **]**

- **[** _`DQN`_ **]**

<details>
  <summary>Click to expand</summary>

| ![ The **`affordance indicator`** refers to the `MDP` `state`. It has length `20` and contains and represents the **spatio-temporal information** of the nearest traffic vehicles. The agent controls the **discrete acceleration** (`maintain`, `accelerate`, `brake`, and `hard brake`) and selects its **lane** (`keep lane`, `change to right`, and `change to left`). [Source](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving).](../media/2019_baheri_3.PNG "The **`affordance indicator`** refers to the `MDP` `state`. It has length `20` and contains and represents the **spatio-temporal information** of the nearest traffic vehicles. The agent controls the **discrete acceleration** (`maintain`, `accelerate`, `brake`, and `hard brake`) and selects its **lane** (`keep lane`, `change to right`, and `change to left`). [Source](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving).")  |
|:--:|
| *The **`affordance indicator`** refers to the `MDP` `state`. It has length `20` and contains and represents the **spatio-temporal information** of the nearest traffic vehicles. The agent controls the **discrete acceleration** (`maintain`, `accelerate`, `brake`, and `hard brake`) and selects its **lane** (`keep lane`, `change to right`, and `change to left`). [Source](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving).* |

| ![ Two purposes: `1-` **Accelerates the learning process** without inhibiting meaningful exploration. `2-` Learn to **avoid accident-prone states**. Note that **collisions still occur**, but less often. [Source](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving).](../media/2019_baheri_2.PNG "Two purposes: `1-` **Accelerates the learning process** without inhibiting meaningful exploration. `2-` Learn to **avoid accident-prone states**. Note that **collisions still occur**, but less often. [Source](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving).")  |
|:--:|
| *Two purposes: `1-` **Accelerates the learning process** without inhibiting meaningful exploration. `2-` Learn to **avoid accident-prone states**. Note that **collisions still occur**, but less often. [Source](https://www.researchgate.net/publication/336591335_Deep_Q-Learning_with_Dynamically-Learned_Safety_Module_A_Case_Study_in_Autonomous_Driving).* |

Authors: Baheri, A., Nageshrao, S., Kolmanovsky, I., Girard, A., Tseng, E., & Filev, D.

- Main motivation: **guide the exploration** process when learning a **`DQN`**-based driving policy by **combining two safety modules** (`rule-based` and `learning-based`):
  - A **handcrafted** safety module.
    - A **heuristic** rule ensures a minimum relative gap to a traffic vehicle.
  - A **dynamically-learned** safety module.
    - `1-` It first **predicts future states** within a finite horizon.
    - `2-` And then determines if one of the **future sates violates the handcrafted safety rule**.
- One idea for `DQN`: Two **buffers** are used to store experience (_off-policy_ learning). They are **equally sampled** in the update stage.
  - `1-` **Safe buffer**: As in classic DQN.
  - `2-` **Collision buffer**: The TD-target only consists in the observed `reward` (i.e. the `s` is a **terminal state**).
    - An experience tuple ends in this collision buffer:
      - `1-` If the `handcrafted` safety module considers the **action is not safe** or if the action leads to a _static_ collision in the next observed state.
      - `2-` If the `dynamically-learned` safety module detects a _dynamic_ collision for **any future predicted states**. In that case, an additional **negative term is assigned to the reward**.
  - Although the sampling is uniform in each buffer, the use of two buffer can been seen as some variant of **prioritized experience replay** (`PER`) where the **sampling is biased** to expose the agent to critical situations.
  - Contrary to **`action masking`**, the _"bad behaviours"_ are not discarded (**half the batch is sampled from the collision buffer**).
    - The agent must therefore **generalize the states that could lead to undesirable states**.
- Relation to **`model-based RL`**:
  - The **transition function** of the `MDP` is **explicitly learnt** by the `RNN` (mapping <`a`, `s`> to `s'`).
  - The dynamically-learned safety module incorporates a **model lookahead**. But **prediction** is not used for explicit planning.
  - Instead, it determines whether the future states lead to undesirable behaviours and consequently **adapts _on-the-fly_ the reward function**.
- Related works:
  - [_"Autonomous Highway Driving using Deep Reinforcement Learning"_](https://arxiv.org/abs/1904.00035) - (Nageshrao, Tseng, & Filev, 2019).
    - _Basically, the same approach is presented._
    - > "Training a standard `DDQN` agent without explicit safety check could not learn a decent policy and **always resulted in collision**. [...] Even with `continuous adaptation` the mean number safety trigger never converges to zero."
    - Since the `NN` function approximation can potentially chose a non-safe action, the agent should be **augmented with some checker** that detects safety violation.
    - The authors introduce the idea of a **"short horizon safety checker"**.
    - If the original `DDQN` action choice is deemed unsafe, then the **safety check replaces it** with an alternate "safe" action, in this case relying on an **`IDM` controller**.
    - This technic enhances learning efficiency and without inhibiting meaningful exploration
  - For additional contents on **_"Safe-RL in the context of autonomous vehicles"_**, the first author @AliBaheri has written [this github project](https://github.com/AliBaheri/Safe-Reinforcement-Learning).

</details>

---

**`"Simulation-based reinforcement learning for autonomous driving"`**

- **[** `2019` **]**
**[[:memo:](https://deepsense.ai/wp-content/uploads/2019/06/Simulation-based-reinforcement-learning-for-autonomous-driving.pdf)]**
**[[🎞️](https://sites.google.com/view/sim2real-autonomous-driving)]**
**[[🎞️](https://sites.google.com/deepsense.ai/sim2real-carla)]**
**[** :mortar_board: `Universities of Warsaw and Jagiellonian` **]**
**[** :car: `deepsense.ai` **]**

- **[** _`sim-to-real`, `end-to-end`, [`CARLA`](http://carla.org)_ **]**

<details>
  <summary>Click to expand</summary>

| ![Neural architecture of the policy function trained with `PPO`: the `RGB` image is concatenated with its **semantic segmentation**. Randomisation is performed to **prevent over-fitting** and **increase sampling-efficiency**. It is also worth mentioning the high-level navigation command that is provided to guide the agent when approaching intersections. [Source](https://deepsense.ai/wp-content/uploads/2019/06/Simulation-based-reinforcement-learning-for-autonomous-driving.pdf).](../media/2019_galias_1.PNG "Neural architecture of the policy function trained with `PPO`: the `RGB` image is concatenated with its **semantic segmentation**. Randomisation is performed to **prevent over-fitting** and **increase sampling-efficiency**. It is also worth mentioning the high-level navigation command that is provided to guide the agent when approaching intersections. [Source](https://deepsense.ai/wp-content/uploads/2019/06/Simulation-based-reinforcement-learning-for-autonomous-driving.pdf).")  |
|:--:|
| *Neural architecture of the policy function trained with `PPO`: the `RGB` image is concatenated with its **semantic segmentation**. Randomisation is performed to **prevent over-fitting** and **increase sampling-efficiency**. It is also worth mentioning the high-level navigation command that is provided to guide the agent when approaching intersections. [Source](https://deepsense.ai/wp-content/uploads/2019/06/Simulation-based-reinforcement-learning-for-autonomous-driving.pdf).* |

| ![Several option for producing the **`std`** of the **steering distribution**. Best results are achieved when the **policy outputs both `mean` and `std`**. The left screenshot illustrates that **`shaped rewards`** (as opposed to `sparse rewards` where rewards are only five at the goal state) can bias learning and lead to **un-intended behaviours**: to make the agent stay close to the centre line, the authors originally penalized the gap in `X`, `Y` but also `Z` coordinates. ''Due to technical reasons our list of lane-centre positions was actually placed **above the road in the `Z` axis**. This resulted in a policy that drives with **two right side wheels placed on a high curb**, so its **elevation is increased** and distance to the centre-line point above the ground is decreased''. [Source-1](https://deepsense.ai/wp-content/uploads/2019/06/Simulation-based-reinforcement-learning-for-autonomous-driving.pdf) [Source-2](https://www.youtube.com/watch?v=YCpFQuAAhqI).](../media/2019_galias_2.PNG "Several option for producing the **`std`** of the **steering distribution**. Best results are achieved when the **policy outputs both `mean` and `std`**. The left screenshot illustrates that **`shaped rewards`** (as opposed to `sparse rewards` where rewards are only five at the goal state) can bias learning and lead to **un-intended behaviours**: to make the agent stay close to the centre line, the authors originally penalized the gap in `X`, `Y` but also `Z` coordinates. ''Due to technical reasons our list of lane-centre positions was actually placed **above the road in the `Z` axis**. This resulted in a policy that drives with **two right side wheels placed on a high curb**, so its **elevation is increased** and distance to the centre-line point above the ground is decreased''. [Source-1](https://deepsense.ai/wp-content/uploads/2019/06/Simulation-based-reinforcement-learning-for-autonomous-driving.pdf) [Source-2](https://www.youtube.com/watch?v=YCpFQuAAhqI).")  |
|:--:|
| *Several option for producing the **`std`** of the **steering distribution**. Best results are achieved when the **policy outputs both `mean` and `std`**. The left screenshot illustrates that **`shaped rewards`** (as opposed to `sparse rewards` where rewards are only five at the goal state) can bias learning and lead to **un-intended behaviours**: to make the agent stay close to the centre line, the authors originally penalized the gap in `X`, `Y` but also `Z` coordinates. ''Due to technical reasons our list of lane-centre positions was actually placed **above the road in the `Z` axis**. This resulted in a policy that drives with **two right side wheels placed on a high curb**, so its **elevation is increased** and distance to the centre-line point above the ground is decreased''. [Source-1](https://deepsense.ai/wp-content/uploads/2019/06/Simulation-based-reinforcement-learning-for-autonomous-driving.pdf) [Source-2](https://www.youtube.com/watch?v=YCpFQuAAhqI).* |

Authors: Galias, C., Jakubowski, A., Michalewski, H., Osinski, B., & Ziecina, P.

- One goal: learn the **continuous `steering` control** of the car to **stay on its lane** (no consideration of traffic rules) in an `end-to-end` fashion using **`RL`**.
  - The **`throttle`** is controlled by a `PID` controller with constant speed target.
  - In addition, the simulated environment is **static**, without any moving cars or pedestrians.
  - The authors want to test how good a **policy learnt in simulation** can **transfer to real-world**. This is sometimes called **`sim-to-real`**.
    - For this second motivation, this reminds me the impressive [`Wayve`](https://wayve.ai)'s work ["Simulation Training, Real Driving"](https://wayve.ai/blog/sim2real).
- _How to model the_ **_standard deviation_** _parameter of the_ **_continuous `steering` action distribution_**_?_
  - It could be set it to a **constant value** or treated as an **external learnable variable** (detached from the policy).
  - But the authors found that **letting the policy control it**, as for the `mean`, gave the best results.
    - It allows the policy to **adjust the degree of exploration** on a per-observation basis.
    - > "An important implementation detail was to **enforce an upper boundary** for the **standard deviation**. Without such a boundary the standard deviation would **sometime explode** and never go down below a certain point (the **entropy of the policy climbs up**), performing poorly when deployed on real-world cars."
- An interesting variant: **`end-to-mid`**, i.e. do not directly predict **raw control commands**.
  - In [another work](https://www.mimuw.edu.pl/~henrykm/pubs_2019/sim2real_outdoor.pdf), the task was not to directly predict the `CARLA` `steering` command, but rather **some target waypoint** on the road, and **_"outsource"_ the steering control task** to some external controller.
  - > "Given a **target waypoint**, **low-level steering** of the driving wheel is executed in order to reach this point. In simulation, it is realized by a **`PID` controller** while in the case of the real car, we use a proprietary **control system**. The action space is discrete - potential waypoints are located every `5` degrees between `−30` and `30`, where `0` is the current orientation of the vehicle."
- Two ideas: The use of **`3` sources of observations**. And the inclusion of **segmentation mask** to the `state` (input) of the `RL` net:
  - `1-` A `RGB` image.
    - It is **concatenated by its semantic segmentation**: it passes through a **previous-learnt segmentation network**.
    - This can be thought as an **intermediate human-designed** or **learned representations** of the real world.
    - As said, the `seg-net` has been learnt before with supervised learning.
      - But it could also be (_further_) **trained online**, at the same time as the policy, leading to the promising concept of **`auxiliary learning`**.
      - This has been done for instance by [(Tan, Xu, & Kong, 2018)](http://arxiv.org/abs/1801.05299), where a framework of **`RL` with image semantic segmentation** network is developped to make the **whole model adaptable to reality**.
  - `2-` A **high-level navigation command** to guide the agent when approaching an intersection.
    - In {`LANE FOLLOW`, `GO STRAIGHT`, `TURN RIGHT`, `TURN LEFT`}.
    - This is inspired by [(Codevilla et al., 2017)](https://arxiv.org/abs/1710.02410).
  - `3-` The current `speed` and `acceleration`.
  - The authors tried to also **include the information about the last action**.
    - Without success: the car was rapidly switching between `extreme left` and `extreme right`.
    - In other words, the **steering** was controlling in a _pulse width modulation-like manner_.
- One idea to **promote generalisation** and **robustness**: **_Randomisation_**.
  - It should also **prevent overfitting** and improve **sample efficiency**.
  - Some randomizations are applied to **visual camera input**.
    - An **additional loss term** is introduce to check if the policy outputs a **similar distribution** for the **_perturbed_** and **_unperturbed_ images**.
  - Some **perturbations** are also applied to the **car dynamics**.
- One idea for debug: generate **saliency map**.
  - The idea is to find which **region of the image** has the most **impact in the prediction** of the steering command.
  - This can be done by **blurring different patches** of the input image, i.e. removing information from that patch, and **measuring the output difference**.
- Some good ideas mentioned for future works:
  - To improve the driving stability, try to focus the training on fragments of scenarios with the **highest uncertainty**. c.f. concepts of **Bayesian `RL`**.
  - Go to **`mid-to-end`**, using an **intermediate representation layer**, for example a `2D`-map or a **bird's-eye view**. e.g. `ChauffeurNet` - also detailed on this page.
  - To further improve the **sampling efficiency**, **model-based methods** could be integrated. c.f. ["Wayve Simulation Training, Real Driving"](https://wayve.ai/blog/sim2real)

</details>

---

**`"Dynamic Interaction-Aware Scene Understanding for Reinforcement Learning in Autonomous Driving"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1909.13582)]**
**[** :mortar_board: `University of Freiburg` **]**
**[** :car: `BMW` **]**

- **[** _`feature engineering`, `graph neural networks`, `interaction-aware networks`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![In the proposed `DeepSet` approach, **embeddings** are first created depending on the object type (using `φ1` for vehicles and `φ2` for lanes), forming the `encoded scene`. They are _'merged'_ only in a **second stage** to create a **fixed vector representation**. `Deep Set` can be extended with `Graph Convolutional Networks` when combining the set of node features to **capture the relations** - _interaction_ - between vehicles. [Source](https://arxiv.org/abs/1909.13582).](../media/2019_huegle_1.PNG "In the proposed `DeepSet` approach, **embeddings** are first created depending on the object type (using `φ1` for vehicles and `φ2` for lanes), forming the `encoded scene`. They are _'merged'_ only in a **second stage** to create a **fixed vector representation**. `Deep Set` can be extended with `Graph Convolutional Networks` when combining the set of node features to **capture the relations** - _interaction_ - between vehicles. [Source](https://arxiv.org/abs/1909.13582).")  |
|:--:|
| *In the proposed `DeepSet` approach, **embeddings** are first created depending on the object type (using `φ1` for vehicles and `φ2` for lanes), forming the `encoded scene`. They are _'merged'_ only in a **second stage** to create a **fixed vector representation**. `Deep Set` can be extended with `Graph Convolutional Networks` when combining the set of node features to **capture the relations** - _interaction_ - between vehicles. [Source](https://arxiv.org/abs/1909.13582).* |

Authors: Huegle, M., Kalweit, B., Werling, M., & Boedecker, J.

- Two motivations:
  - `1-` Deal with an **arbitrary number of objects or lanes**.
    - The authors acknowledge that a **fix-size state** will be **enough for scenarios** like **highways driving** where interactions with the **direct neighbours** of the agent are most important.
      - But they also note that a **variable-length list** can be very important in certain situations such as **urban driving**.
    - To deal with the variable-length **dynamic input set** `X-dyn`, there encodings are just `summed`.
      - This makes the `Q-function` **permutation invariant** w.r.t. the order of the dynamic input and **independent of its size**.
  - `2-` Model **interactions between objects** in the **scene representations**.
    - The structure of **Graph Convolutional Networks** (`GCN`) is used for that. All **_node features_** are combined by the `sum`.
    - > "`Graph Networks` are a class of neural networks that can **learn functions on graphs as input** and can reason about how objects in complex systems **interact**."
- Baselines:
  - > "Graph-Q is compared to two other **interaction-aware `Q`-learning** algorithms, that use input modules originally proposed for supervised vehicle **trajectory prediction**."
    - [**Convolutional Social Pooling**](https://arxiv.org/abs/1805.06771) (`SocialCNN`) is using a **grid-map**: "a social tensor is created by learning latent vectors of all cars by an encoder network and **projecting them to a grid map** in order to **learn spatial dependencies**".
    - [**Vehicle Behaviour Interaction Networks**](https://arxiv.org/abs/1903.00848) (`VBIN`) imposes working with a **fixed number of cars** since the embedding vectors are just **concatenated**, i.e. **not summarizing** as in the Deep Sets approach.
  - A built-in `SUMO` **rule-based controller** is also used for comparison.
- Previous works:
  - `Dynamic input for deep reinforcement learning in autonomous driving` - detailed below.
    - Introducing the idea of `Deep Set`.
  - `High-level Decision Making for Safe and Reasonable Autonomous Lane Changing using Reinforcement Learning` - detailed below.
    - _How to ensure safety when working with a `DQN`_?
    - The concept of **`action masking`** is applied, i.e. the technique of **not exposing** to the agent **dangerous or non-applicable actions** during the action selection.

</details>

---

**`"Driving in Dense Traffic with Model-Free Reinforcement Learning"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1909.06710)]**
**[[:octocat:](https://github.com/sisl/AutomotiveDrivingModels.jl)** `simulator` **]**
**[[:octocat:](https://github.com/honda-research-institute/NNMPC.jl)** `MPC` **]**
**[[:octocat:](https://github.com/honda-research-institute/DRLDriving)** `ML` **]**
**[[🎞️](https://www.youtube.com/watch?v=zY4taTNRM1k)]**
**[** :mortar_board: `UC Berkeley`, `Carnegie Mellon` **]**
**[** :car: `Honda` **]**

- **[** _`SISL`, `PPO`, `MPC`, `merging scenarios`_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![[Source](https://arxiv.org/abs/1909.06710).](../media/2019_saxena_5.PNG "[Source](https://arxiv.org/abs/1909.06710).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1909.06710).* |

| ![The __occupancy-grid-like observation space__ is divided into `4` channels, each containing `3` lanes. An _ego-vehicle_ specific __feature vector__ is also considered. The authors use **policy-gradient** Proximal Policy Optimisation - `PPO` - method and decided not to share parameters between the _actor_ and the _critic_. [Source](https://arxiv.org/abs/1909.06710).](../media/2019_saxena_1.PNG "The __occupancy-grid-like observation space__ is divided into `4` channels, each containing `3` lanes. An _ego-vehicle_ specific __feature vector__ is also considered. The authors use **policy-gradient** Proximal Policy Optimisation - `PPO` - method and decided not to share parameters between the _actor_ and the _critic_. [Source](https://arxiv.org/abs/1909.06710).")  |
|:--:|
| *The __occupancy-grid-like observation space__ is divided into `4` channels, each containing `3` lanes. An _ego-vehicle_ specific __feature vector__ is also considered. The authors use **policy-gradient** Proximal Policy Optimisation - `PPO` - method and decided not to share parameters between the _actor_ and the _critic_. [Source](https://arxiv.org/abs/1909.06710).* |

| ![In [another work](https://arxiv.org/abs/1909.05665), the authors try to incorporate an __`RNN` as a prediction model__ into an __`MPC` controller__, leading to a _reliable_, _interpretable_, and _tunable_ framework which also contains a __data-driven model__ that captures __interactive motions__ between drivers. [Source](https://arxiv.org/abs/1909.05665).](../media/2019_bae_1.PNG "In [another work](https://arxiv.org/abs/1909.05665), the authors try to incorporate an __`RNN` as a prediction model__ into an __`MPC` controller__, leading to a _reliable_, _interpretable_, and _tunable_ framework which also contains a __data-driven model__ that captures __interactive motions__ between drivers. [Source](https://arxiv.org/abs/1909.05665).")  |
|:--:|
| *In [another work](https://arxiv.org/abs/1909.05665), the authors try to incorporate an __`RNN` as a prediction model__ into an __`MPC` controller__, leading to a _reliable_, _interpretable_, and _tunable_ framework which also contains a __data-driven model__ that captures __interactive motions__ between drivers. [Source](https://arxiv.org/abs/1909.05665).* |

Authors: Saxena, D. M., Bae, S., Nakhaei, A., Fujimura, K., & Likhachev, M.

- One motivation: learn to perform comfortable merge into **dense traffic** using **model-free** RL.
  - _Dense traffic_ situations are difficult: traditional **rule-based** models **fail entirely**.
    - One reason is that in heavy traffic situations vehicles cannot merge into a lane without **cooperating** with other drivers.
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
  - > "A `RNN` generates **predictions** for the motions of neighbouring vehicles based on a history of their observations. These predictions are then used to **create safety constraints** for an `MPC` optimisation."
    - `1`- For the prediction part:
      - A [`Social-GAN`](https://arxiv.org/abs/1803.10892) (**_socially acceptable trajectories_**) is used to predict the other vehicles’ **interactive motions** that are reactive to the ego vehicle’s actions.
      - Other prediction modules are tested, such as a **_constant velocity model_**.
      - > "When all drivers are **cooperative**, all three prediction models can lead to successful lane change. That is, the **imprecision of predictions** on drivers’ interactive motions is **not critical** when the drivers are very cooperative, since the drivers **easily submit space** to other vehicles, even with **rough control** inputs resulting from **inaccurate motion predictions**. This, however, is **no longer valid if the drivers are aggressive**."
    - `2`- For the MPC part:
      - A heuristic algorithm based on **Monte Carlo simulation** along with a roll-out is used to deal with the **non-convexity** of the problem (some constraints are non-linear).
      - To reduce the search complexity during the sampling phase, the **action space** is adapted. For instance only _steering angles to the left_ are considered when turning _left_.
  - > "This `SGAN`-enabled controller **out-performs the learning-based controller** in the success rate, (arguably) **safety** as measured by minimum distances, and **reliability** as measured by variances of performance metrics, while **taking more time to merge**".

</details>

---

**`"Behavior Planning at Roundabouts"`**

- **[** `2019` **]**
**[[:memo:](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf)]**
**[** :mortar_board: `Carnegie Mellon` **]**

- **[** _`POMDP`, [`SUMO`](https://sumo.dlr.de/docs/index.html), `generalization`_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![Using recurrent units in a `DQN` and considering the action history. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).](../media/2019_khurana_2.PNG "Using recurrent units in a `DQN` and considering the action history. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).")  |
|:--:|
| *Using recurrent units in a `DQN` and considering the action history. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).* |

| ![`Hidden modes`: decomposing the __non-stationary__ environment into __multiple stationary environments__, where each `mode` is an `MDP` with __distinct dynamics__. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).](../media/2019_khurana_3.PNG "`Hidden modes`: decomposing the __non-stationary__ environment into __multiple stationary environments__, where each `mode` is an `MDP` with __distinct dynamics__. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).")  |
|:--:|
| *`Hidden modes` framework: decomposing the __non-stationary__ environment into __multiple stationary environments__, where each `mode` is an `MDP` with __distinct dynamics__. [Source](https://www.ri.cmu.edu/wp-content/uploads/2019/08/ri_report_20190810.pdf).* |

Author: Khurana, A.

- One idea: using **recurrent nets** (hence `D`**`R`**`DQN`) to integrate **history** in order to cope with **partial observability**.
  - Two `LSTM` layers, (considering `15` past observations) was added after `DQN` layers.
  - They also try to include the **action history** of the agent, leading to **`A`**`DRQN`. But this does not modify the score.
- Another idea: **several** context-based **action spaces**:
  - Decision in **roundabouts** consists in:
    - **Merging** - `action_space` = {`go`, `no go`}
    - **Traversal** - `action_space` = {`acc`, `decelerate`, `change-left`, `change-right` , `cv`}
    - **Exit** - `action_space` = {`acc`, `decelerate`, `change-left`, `change-right` , `cv`}
  - Justification for using **discrete** action space: behavioural planning happens on a **slower time scale** than motion planning or trajectory control.
  - This reminds me some works on **hierarchical RL** (e.g. `Options` framework).
- Another idea: **Curriculum learning**
  - > "Each model is **first trained without any other interacting vehicle** so that it learns the most optimal policy and **later with other vehicles** with random initialization. In later stages, an **additional bonus reward** is given to merging and traversal if they lead to successful exit to **enable long-term** consistent behaviours."
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
  - The goal is to **generalize across different traffic scenarios**, especially across different **traffic densities**.
  - One idea is to decompose the **non-stationary** environment into **multiple stationary environments**, where each _mode_ is an MDP with **distinct dynamics**: this method is called `Hidden modes`.
    - _How to then switch between modes?_ The authors proposed to use **external information** (_Google Maps_ could for instance tell ahead if traffic jams occur on your planned route).
    - But as the number of **discrete modes** increases, the `hidden-mode` method suffers from oscillations at the **boundaries of the mode transitions**.
  - Thus the second idea is to consider one **single model**: this method is called `Traffic-Conditioned`.
    - Some continuously varying feature (ratio of `velocity of other vehicles` to `target speed`) is used. It should be representative of the non-stationary environment.
  - One quote about the relation of **hidden-mode formulation** to **hierarchical RL**:
    - > "For generalization, the hidden-mode formulation can also be viewed as a **hierarchical learning** problem where one `MDP`/`POMDP` framework **selects the mode** while the other learns the driving behaviour **given the mode**".

</details>

---

**`"Reinforcement Learning Approach to Design Practical Adaptive Control for a Small-Scale Intelligent Vehicle"`**

- **[** `2019` **]**
**[[:memo:](https://www.researchgate.net/publication/335693770_Reinforcement_Learning_Approach_to_Design_Practical_Adaptive_Control_for_a_Small-Scale_Intelligent_Vehicle)]**
**[** :mortar_board: `Chongqing University` **]**

- **[** _`non-deep RL`, `online learning`, `model-based RL`_ **]**

<details>
  <summary>Click to expand</summary>

Authors: Hu, B., Li, J., Yang, J., Bai, H., Li, S., & Sun, Y.

- What I like in this paper:
  - The authors make sure they understand **tabular RL** methods, e.g. the difference between on-policy `SARSA(1)`, `SARSA(λ)` and off-policy `Q-learning`, before going to deep RL.
    - > "Compared with `Q-learning`, which can be described as _greedy_, _bold_, and _brave_, `SARSA(1)` is a _conservative_ algorithm that is sensitive to control errors."
  - They include a **model-based** algorithm (`Dyna-Q`) in their study. This seems promising when training directly in real world, where the **sampling efficiency** is crucial.
  - They claim RL methods bring advantages compared to **PID controllers** in term of **adaptability** (_generalization_ - i.e. some PID parameters appropriate for driving on a _straight road_ may cause issues in _sharp turns_) and burden of **parameter tuning**.
  - They consider the **sampling efficiency** (better for _model-based_) and **computational time per step** (better for _`1`-step `TD` methods_ than for _`SARSA(λ)`_).
    - > "`Q-learning` algorithm has a _poorer converging speed_ than the `SARSA(λ)` and `Dyna-Q` algorithm, it balances the performance between the _converging speed_, the _final control behaviour_, and the _computational complexity_."
  - Obviously, this remains far from real and serious AD. But this paper gives a good example of application of basic RL methods.

</details>

---

**`"Learning When to Drive in Intersections by Combining Reinforcement Learning and Model Predictive Control"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1908.00177)]**
**[** :mortar_board: `Chalmers University` **]**
**[** :car: `Zenuity` **]**

- **[** _`model-free RL`, `MPC`, `Q-Masking`, `POMDP`_ **]**

<details>
  <summary>Click to expand</summary>

Two figures:

| ![[Source Left](https://arxiv.org/abs/1908.00177) - [Source Right](https://pdfs.semanticscholar.org/fba5/6bac9a41b7baa2671355aa113462d2044fb7.pdf).](../media/2019_tram_3.PNG "[Source Left](https://arxiv.org/abs/1908.00177) - [Source Right](https://pdfs.semanticscholar.org/fba5/6bac9a41b7baa2671355aa113462d2044fb7.pdf).")  |
|:--:|
| *Sort of feedback loop in the **hierarchical structure**: the **`MPC` notifies** via the `reward` signal if the decision is **feasible, safe and comfortable**. [Source Left](https://arxiv.org/abs/1908.00177) - [Source Right](https://pdfs.semanticscholar.org/fba5/6bac9a41b7baa2671355aa113462d2044fb7.pdf).* |

| ![This work applies the **path-velocity decomposition** and focuses on the longitudinal control. **Three intentions** are considered: aggressive `take-way`, `cautious` (slows down without stopping), and passive `give-way`. [Source](https://arxiv.org/abs/1908.00177).](../media/2019_tram_2.PNG "This work applies the **path-velocity decomposition** and focuses on the longitudinal control. **Three intentions** are considered: aggressive `take-way`, `cautious` (slows down without stopping), and passive `give-way`. [Source](https://arxiv.org/abs/1908.00177).")  |
|:--:|
| *This work applies the **path-velocity decomposition** and focuses on the longitudinal control. **Three intentions** are considered: aggressive `take-way`, `cautious` (slows down without stopping), and passive `give-way`. [Source](https://arxiv.org/abs/1908.00177).* |

Authors: Tram, T., Batkovic, I., Ali, M., & Sjöberg, J.

- Main idea: **hierarchy** in _learnt_/_optimized_ decision-making.
  - A **high-level decision module** based on RL uses the **feedback from the MPC controller** in the reward function.
  - The MPC controller is also responsible for **handling the comfort** of passengers in the car by **generating a smooth acceleration profile**.
- Previous works:
  - [_"Learning Negotiating Behavior Between Cars in Intersections using Deep Q-Learning"_](http://arxiv.org/abs/1810.10469) - (Tram, Batkovic, Ali, & Sjöberg, 2019)
  - [_"Autonomous Driving in Crossings using Reinforcement Learning"_](https://pdfs.semanticscholar.org/fba5/6bac9a41b7baa2671355aa113462d2044fb7.pdf) - (Jansson & Grönberg, 2017)
  - In particular they reused the concept of **_"actions as Short Term Goals (`STG`)"_**. e.g. _keep set speed_ or _yield for crossing car_ instead of some numerical acceleration outputs.
    - This allows for **comfort** on actuation and **safety** to be **tuned separately**, reducing the policy selection to a classification problem.
    - The use of such abstracted / high-level decisions could be a first step toward **hierarchical RL** techniques (`macro-action` and `option` framework).
  - Another contribution consists in replacing the **Sliding Mode** (**`SM`**) controller used previously by an `MPC`, allegedly to _"achieve safer actuation by using constraints"_.
    - The **intention of all agents** is implemented with a `SM` controller with **various target values**.
- I find it valuable to have details about the **training phase** (no all papers do that). In particular:
  - The **normalization** of the input features in [`-1`, `1`].
  - The **normalization** of the reward term in [`-2`, `1`].
  - The use of **equal weights** for inputs that describe the state of **interchangeable objects**.
  - Use of a **`LSTM`**, as an alternative to a **DQN with stacked observations**. (Findings from _(Jansson & Grönberg, 2017)_).
- Additional notes:
  - The main benefits of the combination seem to be about **avoiding over conservative behaviours** while improving the **"sampling-efficient"** of the model-free RL approach.
    - Such approach looks to be particularly relevant (in term of _success rate_ and _collision-to-timeout ratio_ [`CTR`]) for complex scenarios, e.g. `2`-crossing scenarios.
    - For simple cases, the performance stays close to the baseline.
  - The reliance (_and the burden_) on an appropriate **parametrisation** inherent to rule-based has not disappeared and the **generalisation seems limited**:
    - _"Since MPC uses_ **_predefined models_** _, e.g. vehicle models and other obstacle prediction models, the_ **_performance relies on their accuracy and assumptions_** _."_
  - The problem is formulated as a `POMDP`.
    - _Honestly, from a first read, I did not find how `belief tracking` is performed. Maybe something related to the internal memory state of the LSTM cells?_
  - Sadly the **simulator** seems to be **home-made**, which makes reproducibility tricky.
- One quote about **`Q-masking`**, i.e. the technique of not exposing to the agent dangerous or non-applicable actions during the action selection.
  - > "**Q-masking helps the learning process** by **reducing the exploration** space by disabling actions the agent does not need to explore."
  - Hence the agent **does not have to explore** these options, while ensuring a certain level of safety (but this requires another **rule-based module** :blush: ).

</details>

---

**`"Cooperation-Aware Reinforcement Learning for Merging in Dense Traffic"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1906.11021)]**
**[[:octocat:](https://github.com/sisl/AutonomousMerging.jl)]**
**[** :mortar_board: `Stanford` **]**
**[** :car: `Honda` **]**

- **[** _`POMDP`, `offline RL`, `value-based RL`, `interaction-aware decision making`, `belief state planning`_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://arxiv.org/abs/1906.11021).](../media/2019_bouton.PNG "[Source](https://arxiv.org/abs/1906.11021).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1906.11021).* |

Authors: Bouton, M., Nakhaei, A., Fujimura, K., & Kochenderfer, M. J.

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

</details>

---

**`"Interaction-aware Decision Making with Adaptive Behaviors under Merging Scenarios"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1904.06025)]**
**[[🎞️](https://www.youtube.com/watch?v=2CTTFHDW1ec)]**
**[** :mortar_board: `UC Berkeley` **]**
**[** :car: `Honda` **]**

- **[** _`multi agent RL`, `interaction-aware decision making`, `curriculum learning`, `action masking`_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![Note: the visibility of each agent is assumed to be `100m` in front and back, with `0.5m`/`cell` resolution, for both its current lane (`obs_cl`) and the other lane (`obs_ol`). [Source](https://arxiv.org/abs/1904.06025).](../media/2019_hu.PNG "Note: the visibility of each agent is assumed to be `100m` in front and back, with `0.5m`/`cell` resolution, for both its current lane (`obs_cl`) and the other lane (`obs_ol`). [Source](https://arxiv.org/abs/1904.06025).")  |
|:--:|
| *Note: the visibility of each agent is assumed to be `100m` in front and back, with `0.5m`/`cell` resolution, for both its current lane (`obs_cl`) and the other lane (`obs_ol`). [Source](https://arxiv.org/abs/1904.06025).* |

Authors: Hu, Y., Nakhaei, A., Tomizuka, M., & Fujimura, K.

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

</details>

---

**`"Microscopic Traffic Simulation by Cooperative Multi-agent Deep Reinforcement Learning"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1903.01365)]**
**[** :mortar_board: `University of Parma` **]**
**[** :car: `VisLab` **]**

- **[** _`multi-agent A3C`, `off-policy learning`_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://arxiv.org/abs/1903.01365).](../media/2019_bacchiani.PNG "[Source](https://arxiv.org/abs/1903.01365).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1903.01365).* |

Authors: Bacchiani, G., Molinari, D., & Patander, M.

- One idea: **"parallel actor-learners"**: to **decorrelate** the sequence of experiences used to update the policy network, a common approach is to sample <`s`, `a`, `r`, `s'`> tuples from a memory buffer (**_experience replay_**).
  - Here, a **multiple-agent** setting is used instead: each agent acts in a different **instance of the environment** (hence diverse experiences) and sends its updates asynchronously to a **central network**.
- Another idea: **"hybrid state representation"**: coupling some _grid-like_ representation (`path to follow`, `obstacles`, `navigable space`) with a vector of _explicit features_ (e.g. `target speed`, `distance to goal`, `elapsed time ratio`).
  - This combination offers a **generic scene representation** (i.e. independent of the number of vehicles) while allowing for tuning explicit goals (`target speeds`) in the state.
  - Such _hybrid_ representation seems popular, as [identified at IV19](https://github.com/chauvinSimon/IV19#generic-scene-representation)).
- Other ideas:
  - Stacking the `n=4` most recent views to _capture the evolution_ of the scene (e.g. relative speeds).
  - **_Action repeat_** technique for _temporal abstraction_ to stabilize the learning process (c.f. "_frame skip_").
- One concept: **"Aggressiveness tuning"**. Together with the `target speed`, the `elapsed time ratio` (`ETR`) feature is used to tune the **aggressiveness** of the car:
  - > "`ETR` Values close to `1` will induce the agent to **drive faster**, in order to avoid the predicted negative return for running out of time. Values close to `0` will tell the driver that it still **has much time**, and it is not a problem to yield to other vehicles."

</details>

---

**`"Dynamic Input for Deep Reinforcement Learning in Autonomous Driving"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1907.10994)]**
**[[🎞️](https://www.youtube.com/watch?v=mRQgHeAGk2g)]**
**[** :mortar_board: `University of Freiburg` **]**
**[** :car: `BMW` **]**

- **[** _`feature engineering`, `off-policy learning`, `DQN`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

- One diagram is better than 100 words:

| ![By summing all the dynamic terms (one per surrounding vehicle), the input keeps a constant size. [Source](https://arxiv.org/abs/1907.10994).](../media/2019_huegle.PNG "By summing all the dynamic terms (one per surrounding vehicle), the input keeps a constant size. [Source](https://arxiv.org/abs/1907.10994).")  |
|:--:|
| *By summing all the dynamic terms (one per surrounding vehicle), the input keeps a constant size. [Source](https://arxiv.org/abs/1907.10994).* |

Authors: Huegle, M., Kalweit, G., Mirchevska, B., Werling, M., & Boedecker, J.

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

</details>

---

**`"Seeking for Robustness in Reinforcement Learning : Application on Carla Simulator"`**

- **[** `2019` **]**
**[[:memo:](https://openreview.net/pdf?id=Bke03G85DN)]**
**[** :mortar_board: `Université de Strasbourg & ENIT Tunis` **]**
**[** :car: `Segula` **]**

- **[** _[`CARLA`](http://carla.org), `A2C`_ **]**

<details>
  <summary>Click to expand</summary>

- Some background:

| ![`n`-step TD learning. [Source](http://www0.cs.ucl.ac.uk/staff/d.silver/web/Teaching.html).](../media/2015_silver.PNG "`n`-step TD learning. [Source](http://www0.cs.ucl.ac.uk/staff/d.silver/web/Teaching.html).")  |
|:--:|
| *`n`-step TD learning. [Source](http://www0.cs.ucl.ac.uk/staff/d.silver/web/Teaching.html).* |

Authors: Jaâfra, Y., Laurent, J.-L., Deruyver, A., & Naceur, M. S.

- One related work: reading this paper reminded me of one conclusion of the 2019 [CARLA AD Challenge](https://carlachallenge.org/):

> "**Robust** open-source AV stacks are not a commodity yet: **No public AV stack has solved the challenge yet**."

- One idea: use an actor-critic architecture with **multi-step returns** (`n`-step `A2C`) to _"achieve a better robustness"_.
  - The introduction of a **critic** aims at **reducing the variance** of the **gradient** of _policy-based_ methods.
  - As illustrated in the above figure, in _value-based_ methods, the **TD-target** of a critic can be computed in several ways:
    - With bootsrapped, using the current estimate for the next state `s'`: `1`-step TD - **low variance** but biased estimate ...
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
  - This idea is to use **multi-task learning** to improve **generalization** capabilities for an AD controller.
  - As detailed, _"Meta-learning refers to **learn-to-learn** approaches that aim at **training** a model on a set of different but linked tasks and subsequently **generalize** to new cases using **few additional examples**"_.
  - In other words, the goal is to find an optimal **initialization of parameters**, to then quickly adapt to a new task through a few standard gradient descents(**few-shot generalization**).
  - A **gradient-based meta-learner** inspired from **_Model-Agnostic Meta-Learning_** (`MAML` - Finn et al., 2017) is used.
  - RL performance in **non-stationary** environments and generalisation in AD are interesting topics. But no clear benefit is demonstrated, and the above limitations apply also here.

</details>

---

**`"High-level Decision Making for Safe and Reasonable Autonomous Lane Changing using Reinforcement Learning"`**

- **[** `2018` **]**
**[[:memo:](http://mediatum.ub.tum.de/doc/1454224/712763187208.pdf)]**
**[** :mortar_board: `University of Freiburg` **]**
**[** :car: `BMW` **]**

- **[** _`Q-Masking`, `RSS`_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](http://mediatum.ub.tum.de/doc/1454224/712763187208.pdf).](../media/2018_mirchevska_1.PNG "[Source](http://mediatum.ub.tum.de/doc/1454224/712763187208.pdf).")  |
|:--:|
| *[Source](http://mediatum.ub.tum.de/doc/1454224/712763187208.pdf).* |

Authors: Mirchevska, B., Pek, C., Werling, M., Althoff, M., & Boedecker, J.

- It relates to the **`RSS`** and **`Q-masking`** principles.
  - The **learning-based** algorithm (`DQN`) is combined with a `rule-based` checker to ensure that **only safe actions are chosen at any time**.
  - A `Safe Free Space` is introduced.
    - For instance, the agent must **keep a safe distance** from other vehicles so that it **can stop without colliding**.
- _What if the `Safe Free Space` is empty?_
  - > "If the action is considered safe, it is executed; if not, we take the **second-best action**. If that one is also unsafe, we **stay in the current lane**."
- About the [**`PELOPS`**](https://de.wikipedia.org/wiki/PELOPS_(Verkehrsflusssimulationsprogramm)) simulator:
  - It has been developed between **[** :car: `fka` (`ZF`) **]** and **[** :car: `BMW` **]**.
  - In future works (see above), they switch to an **open source** simulator: [`SUMO`](https://sumo.dlr.de/docs/index.html).

</details>

---

**`"Safe Reinforcement Learning on Autonomous Vehicles"`**

- **[** `2018` **]**
**[[:memo:](https://arxiv.org/abs/1910.00399)]**
**[** :car: `Honda` **]**

- **[** _`action masking`, `risk assessment`, `reachability set`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

| ![ The `prediction` module **masks** undesired actions at each time step. [Source](https://arxiv.org/abs/1910.00399).](../media/2018_isele_1.PNG "The `prediction` module **masks** undesired actions at each time step. [Source](https://arxiv.org/abs/1910.00399).")  |
|:--:|
| *The `prediction` module **masks** undesired actions at each time step. [Source](https://arxiv.org/abs/1910.00399).* |

| ![ Here a related patent from the authors. [Source](https://patents.google.com/patent/US20190332110A1/en).](../media/2018_isele_2.PNG "Here a related patent from the authors. [Source](https://patents.google.com/patent/US20190332110A1/en).")  |
|:--:|
| *Here a related patent from the authors :lock: :wink:. [Source](https://patents.google.com/patent/US20190332110A1/en).* |

Authors: Isele, D., Nakhaei, A., Fujimura, K.

- Previous works about **learning control with `DQN`s** at diverse intersections:
  - ["Navigating occluded intersections with autonomous vehicles using deep reinforcement learning"](https://arxiv.org/abs/1705.01196) - (Isele et al., 2017)
  - ["Transferring Autonomous Driving Knowledge on Simulated and Real Intersections"](https://arxiv.org/abs/1712.01106) - (Isele et al., 2017)
- _How to make model-free RL_ **_"safe"_**? Two options are mentioned (both required expert knowledge):
  - 1- Modifying the **reward function** (requires careful tuning).
  - 2- **Constraining exploration** (e.g. `action masking`, `action shielding`).
    - > "The methods can completely **forbid undesirable states** and are usually accompanied by **formal guarantees**".
    - In addition, the **learning efficiency** can be increased (fewer states to explore).
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

</details>

---

**`"Automating Vehicles by Deep Reinforcement Learning Using Task Separation with Hill Climbing"`**

- **[** `2017` **]**
**[[:memo:](https://arxiv.org/abs/1711.10785)]**
**[** :mortar_board: `IMT Lucca` **]**

- **[** _`stochastic policy search`, `gradient-free RL`, `policy-gradient RL`, `reward shaping`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1711.10785).](../media/2017_plessen_1.PNG "[Source](https://arxiv.org/abs/1711.10785).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1711.10785).* |

Author: Plessen, M. G.

- One remark: to be honest, I find this publication _not very easy_ to understand. But **it raises important questions**. Here are some take-aways.

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
    - **_"Curriculum learning"_**, where the learning agent is first provided with **simpler examples** before gradually increasing complexity.
  - After trials, the author claims that no consistent improvement could be observed with these two techniques, adding that the **design** of both rich reward signals and "simple examples" for curriculum learning are problematic.
    - He rather kept working with sparse rewards (**_maximal sparse rewards_**), but introduced some _"virtual velocity constraints"_ to speed up the training.
- I like the points he made concerning **feature selection** for the state, i.e. how to design the state `s(t)` of the MDP.
  - He notes that `s(t)` must always relate the current vehicle state with **reference to a goal state**.
    - In other words, one should use **relative features** for the description of the position and velocity, relative to their targets.
  - In addition, `s(t)` should also **consider the past** and embed a collection of multiple past time measurements.
    - It seems sounds. But this would indicate that the **"Markov property"** in the MDP formulation does not hold.

</details>

---

**`"A Comprehensive Survey on Safe Reinforcement Learning"`**

- **[** `2015` **]**
**[[:memo:](https://jmlr.org/papers/v16/garcia15a.html)]**
**[** :mortar_board: `University Carlos III of Madrid` **]**

- **[** _`optimization criteria`, `safe exploration/exploitation`, `risk sensitivity`, `teacher advice`, `model uncertainty`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://jmlr.org/papers/v16/garcia15a.html).](../media/2015_garcia_1.png "[Source](https://jmlr.org/papers/v16/garcia15a.html).")  |
|:--:|
| *Note: `S/a` means that the method is applied to domains with **continuous or large `state` space** and **discrete and small `action` space**. [Source](https://jmlr.org/papers/v16/garcia15a.html).* |

| ![[Source](https://jmlr.org/papers/v16/garcia15a.html).](../media/2015_garcia_2.png "[Source](https://jmlr.org/papers/v16/garcia15a.html).")  |
|:--:|
| *[Source](https://jmlr.org/papers/v16/garcia15a.html).* |

Authors: Garcia, J., & Fernandez, F.

- Personal notes:
  - While reading, try to classify these two commonly-used approaches in **`AD`**:
    - Add a **high penalty** for **unsafe `states`**.
    - Apply **rule-based logics** to **mask** **unsafe `actions`**.
  - When trying to design a `RL` agent, ask the questions:
    - What **`objective`** should be **optimized**?
    - How to **quantify / detect `risk`**?

- Remaining open questions for me:
  - Are there **safety guarantees**?
  - How to avoid **overly conservative** behaviours?
  - What about `POMDP`s?
  - How to deal with large continuous `states` such as **images**?
  - Can **"teach advice"** also be used during **deployment**? E.g. `action-masking`.
  - How **"solvers"** differ from those used for _unconstrained_ problems?

- Definition:
  - > "`Safe RL` can be defined as the process of learning policies that **maximize the expectation of the return** in problems in which it is important to **ensure reasonable system performance** and/or **respect `safety constraints`** during the **`learning` and/or `deployment`** processes."
  - For `AD`, `RL` agents are probably trained in **a simulator**. Therefore:
    - `training`: Crashes can happen.
    - `deployment`: **No crash** should happen!
      - > "If the goal is to obtain a **`safe` `policy` at the end**, without worrying about the number of **dangerous or undesirable situations** that occur during the `learning` process, then the **`exploratory` strategy** used to learn this `policy` is not so relevant from a `safety` point of view."

- Although most approaches are **model-free** `RL`, ideas and challenges for **model-based `RL`** are also mentioned.
  - > "Having such a [learnt] model is a useful entity for `Safe RL`: it allows the agent to **predict the consequences** of `actions` before they are taken, allowing the agent to **generate virtual experience**." [_but how to make sure the learnt model is correct?_]
  - How to **_safely_ explore** the relevant parts of the `state`/`action` spaces to build up a **sufficiently accurate dynamics** model from which derive a good policy?
    - For instance by learning the **`dynamics` model** from **teacher `safe` demonstrations**.
    - But what when the agent sees a `state` it as never encountered? I.e. `covariate shift`.

- Two big classes:
  - `1-` Modify the **optimality criterion** with some **_`safety` term_**
    - For instance by including the **probability of visiting _error_ `states`**.
    - > [Issue] "The transformation of the **optimization criterion** produces a **distortion in the `action values`** and the **true long term utility of the `actions` are lost**."
    - > [Issue] "It is difficult to find an optimization objective which **correctly models our intuition of `risk` awareness**. In `Section 3.1`, most of the approaches are updated based on a **conservative criterion** and the resulting policy tends to be **overly pessimistic**."
  - `2-` Modify the **exploration process** to **avoid risk situations** through:
    - ... Either the incorporation of **external knowledge**,
    - ... or the guidance of a **risk metric**.
    - This is especially important when `safety` is required during `training`.
      - > "The use of **random exploration** would require an **undesirable `state`** to be visited before it can be labelled as **undesirable**."
    - **Rule-based approaches** can be **used in parallel**. E.g. `action-masking`.
  - Interesting idea: **combine both families!**

- About the concept of **`risk`**:
  - A `policy` optimal for the _vanilla_ `MDP`, i.e. **maximizing the `expected return`**, is said **"`risk`-neutral"**.
  - The authors associate `risk` to the **uncertainty** of the environment, i.e. its **stochastic nature**.
    - > "A **robust `MDP`** deals with **uncertainties in parameters**; that is, some of the parameters, namely, **`transition` probabilities**, of the `MDP` are not known exactly."
    - E.g. based on the **variance of the return** or its **worst possible outcome**.
    - E.g. based on the **level of knowledge** of a particular state, or on the difference between the highest and lowest `Q-values`.
    - For me, they more express the **_confidence_ of an agent**.
      - But an agent can be wrong, being 100% sure that a bad action is good.
    - I would have rather expected the `risk` to be related to the **_probability for something bad to happen in the future_** if `a` is taken in `s`.
      - This can for instance rely on some notion of **`reachibility`** and some **partition of `state` space** into:
      - A `safe` sub-space.
      - A `unsafe` sub-space.
  - Distinction for **`risk` detection**:
    - `1-` **Immediate** risk: if based of the **current `state`**, it may be **too late to react**.
    - `2-` **Long-term** risk: some **risk function**, `ρπ`(`s`), ensures the selection of `safe` `actions`, preventing **long-term risk situations** once the `risk` function is correctly approximated.
  - How to estimate `risk` when **starting from scratch**?
    - > "**Teacher advice** techniques can be used to **incorporate prior knowledge**, thus mitigating the effects of **immediate `risk` situations** until the **`risk function`** is correctly approximated."
  - Not clear to me, is how the agent can learn this `risk function`.
    - > "The mechanism for **`risk` detection** should be **automatic** and **not based on the intuition of a human teacher**."

- Class `1`: what for **extended optimization criteria**?
  - `1-` Maximize the **worst-case `return`**.
    - Here, the goal is to deal with **`uncertainties`**:
      - ... Either the **inherent uncertainty** related to the **stochastic nature** of the system,
      - ... And/or the **parameter uncertainty** related to some of the parameters of the `MDP` are **not known exactly** (c.f. model-based `RL`).
    - It can lead to **over-conservative** behaviours.

  - `2-` Maximize an **objective function** that includes a **`risk` term** `ω`.
    - For instance the linear combination `Eπ(R) − β*ω`, or the **exponential utility function**: `log[Eπ(exp[βR])] / β`.
      - Where `ω` could also reflect the **probability** of entering into an **error `state`**.
    - The **"risk sensitivity"** scalar parameter `β` allows the **desired level of `risk`** to be controlled.
      - `β=0` implies **`risk` neutrality**.
    - The `risk` `ω` is often associated with the `variance` of the `return`.
      - > "Higher `variance` implies **more instability** and, hence, more `risk`."
      - But there are **several limitations** when using the `return variance` as a **measure of `risk`**.
    - But for `AD` it does not make much sense to describe `risk` by the `variance` of the `return`. Think of the **tails of the distribution**.

  - `3-` Maximize the `expected return`, **under some constraints**.
    - It puts constraints on the `policy` space.
    - > "We want to **maximize the expectation of the `return`** while keeping other types of **`expected utilities` lower than some given bounds**."
    - **_Soft_** or **_hard_ constraints**?
      - > "The previous constraint is a **hard constraint** that cannot be violated, but other approaches allow a **certain admissible `chance` of constraint violation**. This **`chance`-constraint** metric, `P(E(R) ≥ α) ≥ (1 − ε)`)." [_Here the `constraint` is on the `return`, but could be on the `probability of collision`_]
      - Leading to **`chance`-constrained** `MDP`s.

- Class `2.1`: How the **`exploration` process** can be modified by **including prior knowledge** of the task?
  - `1-` Provide **initial** knowledge, used to **_bootstrap_ the learning algorithm**.
    - To obtain for instance an **initial `value function`**.
    - Or perform **transfer learning**, i.e. using a **`policy` trained on a similar task**.
    - > "The learning algorithm is **exposed to the most relevant regions** of the `state` and `action` spaces from the earliest steps of the learning process, thereby **eliminating the time needed in random `exploration`** for the discovery of these regions."
    - > [limitation] "The `exploration` process following the **initial training phase** can result in visiting new `states` for which the agent has no information on **how to act**."

  - `2-` Derive a **policy** from a finite **set of demonstrations**.
    - > "All (`state`-`action`) trajectories seen so far are used to **learn a model from the system’s `dynamics`**. For this model, a (near-)optimal `policy` is to be found using any `RL` algorithm."
    - It relates to
      - **off-line `RL`**
      - **model-based `RL`**.
      - **Learning from Demonstration (`LfD`)**, with **inverse `RL`** and **behavioural cloning**.
    - _How the agent should act when it encounters a `state` for which no demonstration exists?_
      - See "teacher advice" techniques below. Basically it needs some **life-long mentoring**.

  - `3-` Provide **teach advice**, e.g., **`safe` `actions`**.
    - > [Motivation] "However, while furnishing the agent with **initial knowledge** helps to mitigate the problems associated with **random exploration**, this initialization alone is not sufficient to prevent the **undesirable situations** that arise in the **subsequent explorations** undertaken to improve learner ability. An additional mechanism is necessary to **guide this subsequent exploration process** in such a way that the agent may be kept far away from **catastrophic `states`**."
    - It generally includes four main steps:
      - `(i)` **Request** or **receive** the "advise".
        - Either the agent can **explicitly ask** for help. As a "joker" / **"ask for help"** approach. Using some **confidence parameter** to detect risky situations, e.g. when all `actions` have similar `Q-values`.
        - Or the teacher **can interrupt** the agent learning execution at any point, e.g. to **prevent catastrophic situations**. Close to `action masking`.
      - `(ii)` Convert advice into a **usable form**.
        - The teacher does not necessary use the **same input** as the agent.
        - For instance for simple `action masking`, knowing about the **distance to the leading vehicle** can be enough to impose `braking` when driving too close.
        - In [(Bouton et al., 2019)](https://arxiv.org/abs/1904.07189) a **probabilistic model checker** is  derived **prior** to learning a `policy`.
          - It computes the **probability of reaching the goal safely** for each (`state`, `action`) pair.
          - And is subsequently **queried to mask actions**.
          - Issue: require **discrete states** and **full knowledge of the `dynamics`**.
      - `(iii)` Integrate the reformulated _advice_ into the **agent’s knowledge base**.
      - `(iv)` Judge the **value of advice**.
        - > "The teacher is giving advice on `policies` rather than `Q-values`, which is a more natural way for **humans to present advice** _(e.g., when the nearest taker is at least `8` meters, holding the ball is preferred to passing it)_."
    
    - It assumes the **availability of a teacher** for the learning agent, which is **not necessary an expert** in the task.
      - Following the advice of the teacher can lead the learner to **promising parts of the `state` space**, which also helps for **safe exploration**.
    - It does not modify the `objective`.
      - > "The `Safe RL` algorithm should incorporate a **mechanism to detect and react to immediate `risk`** by manifesting different risk attitudes, while **leaving the optimization criterion untouched**."

- Class `2.2`: How to do **risk-directed exploration**?
  - For instance the agent can be encouraged to **seek controllable regions** of the environment.

</details>
