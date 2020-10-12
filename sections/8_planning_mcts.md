# `Planning` and `Monte Carlo Tree Search`

---

**`"Vehicle Control with Prediction Model Based Monte-Carlo Tree Search"`**

- **[** `2020` **]**
**[[:memo:](http://cpslab.snu.ac.kr/publications/papers/2020_ur_mcts.pdf)]**
**[** :mortar_board: `Seoul National University`**]**

- **[** _`highway`, `combine learning + planning`, `learnt transition model`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](http://cpslab.snu.ac.kr/publications/papers/2020_ur_mcts.pdf).](media/2020_ha_1.PNG "[Source](http://cpslab.snu.ac.kr/publications/papers/2020_ur_mcts.pdf).")  |
|:--:|
| *A **`prediction` model** is learnt from the `NGSIM` dataset. And used as **`transition` model to estimate the `next states`** of the nodes and the corresponding `rewards`. This contrasts with conventional `MCTS` formulations where the **`transition` model** is **rule-based**. [Source](http://cpslab.snu.ac.kr/publications/papers/2020_ur_mcts.pdf).* |

| ![[Source](http://cpslab.snu.ac.kr/publications/papers/2020_ur_mcts.pdf).](media/2020_ha_2.PNG "[Source](http://cpslab.snu.ac.kr/publications/papers/2020_ur_mcts.pdf).")  |
|:--:|
| *__`Learning` and `planning` are combined__. A `RL` agent is first trained offline. Then, its `value net` helps for the **`evaluation` stage** while its `policy net` guides the **`selection` phase** of the `MCTS`. And the **trajectories** sampled during the **`MCTS` online rollouts** are used to fine-tune the `value net`. [Source](http://cpslab.snu.ac.kr/publications/papers/2020_ur_mcts.pdf).* |

Authors: Ha, T., Cho, K., Cha, G., Lee, K., & Oh, S.

- Motivations:
  - `1-` Improve the **`transition` model** that predicts the `next state` from `current state` and `current action`.
    - > "In conventional `MCTS` methods, they assume that the `next state` of the agent can be calculated from a **pre-defined deterministic model**, or sampled by a **self-play method**."
    - Here the model is **learnt** offline.
  - `2-` **Combine `learning` and `planning`**.
    - By using a **pre-trained `RL` agent**.

- `state`
  - `1-` An image. It represents the **bird-eye view** of the scene.
  - `2-` A vector. It represents the `position` and `speed` of the ego-car.
  - `3-` A set of vectors. It represents the `position` and `speed` of neighbouring vehicles.

- `action`
  - `35` combinations of predefined (`acceleration`, `steer`) pairs.

- _How to predict the `transition`s?_
  - A **prediction network** is trained on the `NGSIM` dataset.
    - Called **"Multi-Agent Joint Trajectory Prediction Model".**
    - **[`"Deep Predictive Autonomous Driving Using Multi-Agent Joint Trajectory Prediction and Traffic Rules"`](http://cpslab.snu.ac.kr/publications/papers/2019_iros_predstl.pdf)** - also detailed on this page.
  - Max number of surrounding vehicles = `6`.

- _How can a_ **_pre-trained `RL` agent_** _can be used to improve the `MCTS`?_
  - It offers a `value` and a `policy` function.
  - `1-` The `value` function **evaluates the expanded nodes** during the **`evaluation` step**.
  - `2-` The `policy` function **guides the `selection` phase** of `MCTS`.
    - A **modified `UCT`**: `argmax` is applied on `Q + c*π*U`.
    - `π` is the `policy` function.
    - The **upper bound `U`** is a function of the **(`node`, `action`) visitation** counter: a larger **number of trials** should give us a **smaller bound**.
    - Here `c=200`. But the **weighting** depends on the **magnitude of `Q`**, which depends on the range of `r`, with here `γ=0.8`.

- _How can online `MCTS` improve the pre-trained `RL` agent?_
  - It is said above that the `policy` net **guides the `selection`**.
    - **Trajectories** are collected (from one node up to a `termination` node).
  - The `value` net can be **fine-tuned using these sampled trajectories**.
    - I.e. **Monte Carlo methods** which is **unbiased**.
    - And this is **`on-policy`** since the experiences come from the `policy net`.

- _Once the tree is constructed (here after `200` loops), how is the_ **_final decision_** _made?_
  - > "In most `MCTS` algorithms, the `final action` is selected with the **highest value of the `Q-value`**, the **visit count `N`**, or a **combination of them `Q−U`**. In this paper, we adopt the last method."

- Other detail for the `section` phase.
  - **Breath-first-search** to the **root node**: the `selection` method first selects **not-visited `action`** before all the `action` from the root node are searched at least once.

- Benchmark:
  - It would have been interesting to use **a version of `IDM`** to model the **reaction of the other vehicles**.

</details>

---

**`"Driving Maneuvers Prediction Based Autonomous Driving Control by Deep Monte Carlo Tree Search"`**

- **[** `2020` **]**
**[[:memo:](https://ieeexplore.ieee.org/document/9082903/)]**
**[** :mortar_board: `University of Chengdu`, `University of Texas`, `San Diego State University`**]**

- **[** _`AlphaGo`, `end-to-end`, `combining planning + learning`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://ieeexplore.ieee.org/document/9082903/).](media/2020_chen_7.PNG "[Source](https://ieeexplore.ieee.org/document/9082903/).")  |
|:--:|
| *The **`MCTS` search** is assisted by two networks: A **`value` net** is learnt from the **collected experiences** and used for the **`expansion` and `evaluation` steps**. A second net learns to **predict the `transitions`**, enabling rollouts **independently of the simulator**. [Source](https://ieeexplore.ieee.org/document/9082903/).* |

| ![[Source](https://ieeexplore.ieee.org/document/9082903/).](media/2020_chen_8.PNG "[Source](https://ieeexplore.ieee.org/document/9082903/).")  |
|:--:|
| *The **`expansion` and `simulation` steps** of the vanilla `MCTS` are merged, relying on a network that predicts the **`action` probabilities** and **associated `values`** based on the current `state`. [Source](https://ieeexplore.ieee.org/document/9082903/).* |

| ![[Source](https://www.nature.com/articles/nature16961).](media/2016_silver_1.PNG "[Source](https://www.nature.com/articles/nature16961).")  |
|:--:|
| *Compared to `AlphaGo`, the **evaluation of an `expanded` node** is done using the **`value net` only**. **No rollout** with some `fast rollout policy` `pπ` is performed. [Source](https://www.nature.com/articles/nature16961).* |

| ![[Source](https://ieeexplore.ieee.org/document/9082903/).](media/2020_chen_9.PNG "[Source](https://ieeexplore.ieee.org/document/9082903/).")  |
|:--:|
| *How the **prior `action` probability** is used during the `selection` phase? Left: proposed approach. Right: `AlphaGo`. [Source1](https://ieeexplore.ieee.org/document/9082903/) [Source2](https://www.nature.com/articles/nature16961).* |

Authors: Chen, J., Zhang, C., Luo, J., Xie, J., & Wan, Y.

- Close to **model-based `RL`** / **learning-based `planning`**, it uses the idea of `AlphaGo`.

- Motivations:
  - `1-` **`end-to-end`**: predict `steering` from front view image on a racing track.
    - Using the Udacity Self-driving Simulator (`USS`) and `Torcs`.
  - `2-` Improve **training efficiency**. Compared to **model-free `RL`** and behavioural planning.
    - > "The `training losses` of [_the proposed_] `AVP` network, `DQN`, `DDPG` and `IL` converge at the training step `40,000`, `80,000`, `70,000` and `30,000`, respectively. Hence, the `AVP` network improves `50.0%` compared to `DQN`, `42.9%` compared to `DDPG` in **training efficiency** and only loses `33.3%` in training efficiency compared to `IL`. [_Probably because `AVP` should predict the `next state` for_ **_all `actions`_**, not only the optimal ones_.]"
  - `3-` Interpretability: **predict manoeuvres** by reconstructing **multiple possible trajectories**.

- Main idea: `MCTS` with some **learnt modules**.
  - > "We employ `deep-MCTS` based on **asynchronous policy** and the **value MCTS** algorithm ([`APV-MCTS`](https://www.nature.com/articles/nature16961) _from `AlphaGo`_) which contains **three steps** by **merging the `expansion` and `simulation` steps**."
  - No info about the `timestep`.

- _What_ **_transition model?_** _How to predict `s'`?_
  - A first net predicts the `next state` of the vehicle based on a given control `action` and the current `state`.
    - I.e. `grey-scale image` to `grey-scale image` prediction.
  - Loss: distance between images. E.g. `L2`.

- _How to_ **_guide_** _the tree `expansion` and make the `evaluation` step more efficient?_
  - `1-` By estimating a **`value` function `V(s)`** to complement/avoid **rollouts**.
  - `2-` By estimating the **`action` selection probabilities** (i.e. a **prior policy**), used for both the **initialization of new nodes** and during the `selection` phase.
  - A second net predicts the **`action` selection probabilities `pt`** and the `value` `vt` of current `state` `ςt`.
    - `P(n)` is the **prior selection probability** of node `n`.

- Metrics:
  - Stability of driving trajectory: the mean of the **deviation** to track centre (`MDC`).
  - Stability of driving **control**: the mean **continuity** error (`MCE`).
  - No consideration of the **efficiency**? E.g. `speed`.

- About `AlphaGo`:
  - Neural networks are used to reduce the **effective `depth` and `breadth`** of the search tree:
    - `1-` **Evaluating positions** using a `value` network.
    - `2-` **Sampling actions** using a `policy` network.
    - > "First, the **`depth` of the search** is reduced by **`state evaluation`**: truncating the search tree at `state` `s` and replacing the subtree below `s` by an **approximate `value` function `v(s)`** that **predicts the outcome from `state` `s`**."
    - > "Second, the **`breadth` of the search** is reduced by **sampling `actions` from a `policy` `p(a|s)`** that is a probability distribution over possible moves `a` in position `s`."
  - Three learning steps:
    - > [`policy` learnt from `BC`] "**Supervised learning (`SL`)** `policy` network directly from expert human moves."
    - > [`policy` improved by `self-play`] "Next, we train a reinforcement learning (`RL`) `policy` network that **improves the `SL` policy network** by optimizing the final outcome of games of **self-play**."
    - > "Finally, we train a **`value` network** that predicts the winner of games played by the `RL` `policy` network against itself."
  - **`expansion` phase**:
    - The **leaf state `sL`** is processed just once by the **`SL` policy network**. The **output probabilities** are stored as **prior probabilities `P`** for each legal `action`.
      - This impacts the **action selection**: `argmax`[`Q(s)`+`u(s)`] where the **bonus term `u`** is proportional to that **prior probability** but decays with repeated visits to **encourage exploration**.
    - The **leaf node** is **evaluated** in two very different ways. Evaluations are combined by a **weighted sum**.
      - `1-` By the **value network `vθ(sL)`**.
      - `2-` By the outcome of a **random rollout played out** until terminal step `T` using the **fast rollout policy `pπ`**.

</details>

---

**`"Autonomous Driving at Intersections: A Critical-Turning-Point Approach for Planning and Decision Making"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.02409)]**
**[** :mortar_board: [`Cogdrive lab`](https://www.cogdrive.ai/aboutus/) at `University of Waterloo`**]**

- **[** _`CTP`, `ABT`, `unprotected left turn`, `information gathering`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.02409).](media/2020_shu_4.PNG "[Source](https://arxiv.org/abs/2003.02409).")  |
|:--:|
| *__Left-turning__ behaviours at **unsignalized intersection** can be modelled using the **Critical Turning Points Model"** (`CTP`). Before the `CTP`, the ego-vehicle is **creeping forward** to collect information and assess the situation. After the `CTP`, it turns and accelerates. The action space is therefore made `2`-dimensional (`acceleration` and `CTP`). This improves the **`driving` efficiency** compared to just having control of the `acceleration` on a **fixed pre-defined path**, where the ego vehicle **stops and waits** for the critical zone to clear. Here, if **one of the paths is blocked**, it can **keep moving forward** and take the sharp turn at the **next `CTP`**. [Source](https://arxiv.org/abs/2003.02409).* |

| ![[Source](https://arxiv.org/abs/2003.02409).](media/2020_shu_3.PNG "[Source](https://arxiv.org/abs/2003.02409).")  |
|:--:|
| *The **nodes** in the **tree** represent the `beliefs`. Each node is formed by a **set of particles**, where every **particle** is associated with a quadruple (`s`, `a`, `o`, `r`) obtained from **previous episodes**. The edge that connects the nodes `b` and `b'` in the tree is formed by a (`action`-`observation`) pair, which means that a particle in the belief `b` performs an action `a` and reached to belief `b'` (i.e. `b'` = `transition(b,a,o)`) which perceives an observation `o`. The **size of the tree** and thus the **complexity of the search** depends on the **discretization of the `observation` space**. Here, it is parametrized with an **accuracy variable `κa`**. [Source](https://arxiv.org/abs/2003.02409).* |

Author: Shu, K.

- A **master thesis**, with very good explanations about **`POMDP` solvers**!
- Related work: [Autonomous Driving at Intersections: A Critical-Turning-Point Approach for Left Turns](https://arxiv.org/abs/2003.02409), (Shu et al., 2020). See below.

- Motivations:
  - `1-` Address **left-turning** at **unsignalized intersection**.
    - > "**Left turning** when the oncoming vehicles are **not using turn signals** is one of the most common and challenging tasks for planning at **unsignalized intersections**."
  - `2-` Consider the **unknown intentions** of the oncoming vehicle to **make less conservative decisions**.
  - `3-` Ensure real-time performance.
    - > "An **observation accuracy** as `1` and a **tree depth** of `4` or `5` [_`2` or `2.5s`_] would be an **ideal tree size** for good performance in this test scenario."
    - > "Complexity is influenced by the **numbers of `CTPs`**, the more `CTPs` that are used, the **more candidate paths** would be generated to be taken into account to find an appropriate behavior. [...] Here, **`4` CTPs** would be appropriate."

- About the architecture:
  - `1-` **Offline**: select proper **candidate paths** (yes, multiple) for the particular intersection.
  - `2-` **Online**: select one `path` and decide the `speed` on it.
    - > "With the **generated candidate left-turning paths**, the left-turn problem is **formulated as a `POMDP` problem.**"
    - > "`POMDP` provide a way to make decisions on the **best sequences of actions** to perform when the agent **can not fully observe the `states`** of the environment, in our case, it is the **intention** of the surrounding [_opposing_] vehicles."

- How to **model behaviours** at intersection?
  - With a **_"Critical Turning Point Model"_** (`CTP`).
  - From dataset, the author finds that behaviours at left turns at intersections can be divided into two stages:
    - `1-` A **`creeping forward` phase** when the driver **assesses the safety** of the environment.
    - `2-` An **intense `steering` and `acceleration` phase** when the driver **feels confident** to drive through the potential collision area.
  - The **physical point** where the vehicle transition from `1-` to `2-` is called a **critical turning points (`CTP`)**.
    - **Multiple `CTP`s** exist for one intersection.
    - They can be extracted from or validated by **naturalistic driving data**, using `yaw rate`, rather that `longitudinal speed`, as a major criterion.

- How to avoid **over-conservatist** decisions?
  - Enable **`information gathering`** by allowing **slight modifications around the "turning" path**, instead of just controlling the `longitudinal acceleration` on one **fixed pre-defined path**.
  - > "Without the uses of `CTPs`, the path of the ego vehicle is **fixed**, thus, when the **Critical Zone is blocked by the oncoming vehicle**, the ego vehicle **stops and waits** for the critical zone to clear until any acceleration can be executed."
  - > "However, with the uses of `CTPs`, if **one of the paths is blocked**, the vehicle has the option to **keep moving forward** and take the sharp turn at the **next `CTP`**."

- How to **estimate `intention`**?
  - With **`belief` tracking**. Using particle filtering in the **`belief` tree**.

- About the `POMDP` formulation:
  - `state`:
    - `1-` **Travelled `distance`** on the pre-defined path, in the **Frenet–Serret frame**.
    - `2-` **`speed`** along that path.
    - `3-` **`intention`** of the vehicles, which includes `going straight`, `making right` or `left turns`.
  - `observation`:
    - Only the `position` and `speed` are observable.
    - The `intention` (route to follow) is unknown to the ego vehicle and must be **estimated**.
    - > [**Discretization** of the space] "An **accuracy variable `κa`** for the observation model is introduced. The `states` are multiplied by the **accuracy variable** first, then **rounded to the closest integer** and finally **divided by `κa`**. By tuning `κa`, the **width of the search tree** could be adjusted."
  - `action`:
    - `1-` **`acceleration`** along the pre-defined path.
      - Discrete in [`-4`, `4m/s²`] with a interval of `1m/s`.
    - `2-` **Left-turning `instruction`**
      - Boolean.
      - > "When this parameter is set to `1`, the ego vehicle **will shift from the creeping forward phase** to the **sharp left-turning phase** at the next `CTP` ahead."
  - `reward`:
    - `-` **Crashing** and **going backward**.
    - `+` Reaching the **goal**.
    - `+` Moving as the **referenced `speed`**.
    - `+` Selecting the **proper path**.
      - > "Finally, the key `reward` that enables the vehicle to **select the appropriate path** is set as `Rm = 10y − 50x²`, which encourages the vehicle to **move closer to the goal** on both `x` and `y`-axis."
  - `transition` model:
    - All the surrounding vehicles are moving on their **pre-defined paths** with an **uncertain `acceleration` / `deceleration`** which follows a **normal distribution**.

- About the decision **frequency**:
  - > "[Due to] the **rapidly changing** nature of the intersection scenarios [...] the **frequency of decision-making** needed to be realized **as high as possible**."
  - Here, working at `2Hz`.

- About the `POMDP` online solver.
  - > "Among all the `MCTS` methods, **`Adaptive Belief Tree`** ([`ABT`](https://link.springer.com/chapter/10.1007/978-3-319-28872-7_35)) is implemented. This solver uses `MCTS` to generate a **`policy` tree**, then the `action` with the highest `reward` is executed and `belief` is updated using a **particle filter**. After that, instead of **discarding the entire policy tree** as the classical [`POMCP`](https://papers.nips.cc/paper/4031-monte-carlo-planning-in-large-pomdps.pdf) online solver, `ABT` only identifies and **trims the parts** of the policy tree that are influenced by the **_change of the model_** (e.g. change of `action` space, `transition` model, `observation` model, etc.), then revises the influence part of the **`policy` tree** and improves it if there is still time left in that time cycle. Since most part of the **`policy` tree** is **saved after each time step**, the **search efficiency** is boosted."

</details>

---

**`"POMDP Autonomous Vehicle Visibility Reasoning"`**

- **[** `2020` **]**
**[[🎞️](https://www.youtube.com/watch?v=NojRqX3cIH4)]**
**[[:memo:](https://kylewray.com/s/WWrssra20.pdf)]**
**[** :car: `Renault-Nissan` **]**

- **[** _`occlusion`, `MODIA`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://kylewray.com/s/WWrssra20.pdf).](media/2020_wray_1.PNG "[Source](https://kylewray.com/s/WWrssra20.pdf).")  |
|:--:|
| *Left: the `MODIA` framework consists of two **small POMDP decision problems (`DP`s)** that are **solved offline**. One to deal **with another car** at an intersection. Another to deal with an **obstacle**. When vehicles are **perceived online**, `DPs` are **instantiated** as **decision components (`DC`s)**. `DC`s **recommend** an `action` at specific arbitration points along the route, with **conflicts resolved by an `executor arbitration` function** (e.g., **_take the safest action_**). Virtual vehicles, imagined just outside of the field-of-view, are also created and instantiate `DC`s to allow for **reasoning about possible imperceptible vehicles**. Right (from [`MODIA`](https://rbr.cs.umass.edu/shlomo/papers/WWZijcai17.pdf)): the policy seems to be derived iteratively, with **`α`-vectors**. This is possible since only **`128` states** are defined. [Source](https://kylewray.com/s/WWrssra20.pdf).* |

Authors: Wray, K. H., Lange, B., Jamgochian, A., Witwicki, S. J., Kobashi, A., Hagaribommanahalli, S., & Ilstrup, D.

- Motivations:
  - `1-` Decision making under **limited visibility**.
    - > "To maximize safety, AVs must reason about these **“known unknowns”** [_they are capable of_ **_detecting their own limited visibility_**] and intelligently make decisions for when to `go`, `stop`, or `edge forward slowly` for visibility when entering an **occluded T-intersection** or **passing an obstacle** in the road."
    - > "`POMDP` provides a powerful model for **sequential decision-making under limited visibility, sensor noise**, and other forms of **uncertainty** known **sensor limitations** through its **probabilistic model** of observing other vehicles. They enable **`belief`-based reasoning** about a **vehicle’s existence** even if it has never actually been perceived by perception."
  - `2-` Scalability: **multiple vehicles** and different scenarios.
    - > [issue] "**Single monolithic `POMDP`s** for AV decision-making define a `state` space for **up to some maximum number of possible vehicles**."
  - `3-` Real world. _Unfortunately, no video is provided._

- **`MODIA`**: for scalability.
  - From (Wray, Witwicki, & Zilberstein, 2017): **[`"Online Decision-Making for Scalable Autonomous Systems"`](https://rbr.cs.umass.edu/shlomo/papers/WWZijcai17.pdf)**. [Slides](https://pdfs.semanticscholar.org/1df1/8e66c9117852468f8e327a67622b5738558a.pdf).
  - It is a framework for **multiple online decision-components with interacting actions**.
  - The main idea is the **scene-decomposition**, i.e. to divide the problem into **canonical decision-making subproblems** and to **solve separately** each of them.
    - This idea has already been applied in multiple papers, including patents: [`RL with scene decomposition for navigating complex environments`](http://www.freepatentsonline.com/20200247402.pdf) from Honda and Stanford ([related paper](https://arxiv.org/abs/1904.11483)).
    - > "In `MODIA`, a collection of possible **decision-problems (`DP`s)**, known a priori, are **instantiated online** and executed as **decision-components (`DC`s)**, unknown a priori.
  - Two decision-making problem (`DP`) are considered:
    - `DP.1`: negotiating with a **single vehicle** at a **T-intersection**.
    - `DP.2`: **passing a single obstacle** [_not detailed here??_].
  - Multiple instances of each `DP` are created:
    - For each **perceived** (or **virtual**) vehicle, **a `DC` is instantiated** as a copy of one of the two `DP`s original, including its `policy` that has been derived offline.
    - Each created `DC` makes decision proposals. An **executor arbitration** function **aggregates these proposals** to produce one action to be performed.
      - > "Here we consider an **executor arbitration** function that selects the **most conservative recommendation**. That is, `stop` is preferred to `edge` which is preferred to `go`."
    - At each time step, each `DC`[`j`] (_instantiated from `DP`[`i`]_) obtains its **`belief`** and **`arbitration point`** from the **monitor `M`[`i`]**.

- The two `DP` are formulated as `POMDP`.
  - They are **solved offline**.
    - _How is belief tracking done?_
      - `b` is updated to `b'` with: `b'`(`s'`) = `η` . `O(a,s',ω)` . SUM-over-s[`T(s,a,s')`.`b(s)`]
    - _How is the policy derived?_ No detail here. But from `MODIA` paper:
      - > "Since `Vπ` is **piecewise linear and convex**, we describe it using sets of `α`-vectors `Γ`={`α1`, ... , `αr`} with each `αi` =[`αi`(`s1`), ... , `αi`(`sn`)]`T` and `αi`(`s`) denoting **value of state `s`∈`S`**. The objective is to find **optimal policy `π∗`** that maximizes `V` denoted as `V∗`. Given an initial belief `b0`, **`V∗` can be iteratively computed** for a time step `t`, **expanding beliefs** at each update resulting in belief `b`."

  - They share the **same `action` space**.
    - {`stop`, `edge forward`, and `go`} decisions which control the **motion along the trajectory**.
    - _How can the ego-car decide to take an obstacle over, as illustrated by `DC4` on the figure, if the trajectory is defined and the decisions only control the longitudinal motion?_
    - > "The **low-level trajectory control** uses these points as an input to its **constrained optimization continual planner**."
  - `state` for `DP.1`: **discrete and abstracted**!
    - `1-` Ego location: {`before-stop`, `at-stop`, `before-gap`, `at-gap`, `goal`, `terminal`}. _I don't understand what `gap` means here._
    - `2-` Other's location: {`before-stop`, `at-stop`, `before-gap`, `at-gap`, `goal`, `empty`}.
    - `3-` Ego **time at the location**: {`short`, `long`}.
    - `4-` **Existence of a gap** when the AV arrives: {`yes`, `no`}. _I am confused by the inclusion of the temporality. What happens if the ego car waits and the gap disappears? Is it still `true`?_
    - In total `6*6*2*2`=**`128` states**. _This can be stored in a_ **_table_**_, and does not require function approximator_.
    - The **high abstraction** in the `state` space can be problematic here.
      - `1-` **Precision** is required, especially when driving close to other objects.
      - `2-` Some **transitions may become impossible**, depending on the **timestep** used.
  - `observation` (_I must say I don't understand their relevance. Shouldn't it be the `location`s with noise added for instance?_):
    - `1-` If the ego car **successfully moved** {`yes`, `no`}. _Why "successfully"?_
    - `2-` If the other vehicle **successfully moved** {`yes`, `no`}.
    - `3-` If a **gap is detected** {`yes`, `no`}.
  - `reward` for `DP.1`:
    - `0` for the **goal `state`**, `−1000` for any other **terminal `state`**, and `-1` for all other (`s`, `a`) pairs.
    - That makes `DP.1` **episodic**.
  - `transition function` for `DP.1`:
    - _I don't understand. More details needed (especially about "etc.")! What `distribution`s for instance?_
    - > "It multiplies parameterized probabilities of quantifiable **events** within the `state`-`action` space including: (`1`) _the AV and/or other vehicle moving forward_, (`2`) _time incrementing_, (`3`) _entering a terminal state_, (`4`) _the other vehicle slowing down for a crossing AV_, (`5`) _the gap’s existence toggling based on other vehicle movement_, etc."
  - `observation function` for `DP.1`:
    - > [_More details would have been appreciated_] "It multiples parameterized probabilities including: (`1`) _localizing correctly within the AV’s location state factor_, (`2`) _correctly detecting the other vehicle that does exist_, (`3`) _correctly matching the other vehicle to its location state factor_, (`4`) _observing the terminal or goal state_, (`5`) _detecting the gap correctly based on predictions_, etc."
  - _What about the timestep?_
    - If too large, then the ego car cannot react. If too small, **some `transitions` between `states` are impossible** and some `states` become absorbing.
  - _What about `DP.2`?_
- _How to deal with occlusion?_
  - **Virtual vehicles** are instantiated.
  - > "We create **virtual vehicles** at the edge of the **detectable visibility range** along all lanes. For example at a T-intersection, **two virtual vehicle `DC`s will always exist**, one for each incoming lane."

</details>

---

**`"Improving Automated Driving through Planning with Human Internal States"`**

- **[** `2020` **]**
**[[:memo:](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321)]**
**[** :mortar_board: `FAU Erlangen`**]**
**[** :car: `AUDI` **]**

- **[** _`ABT`, `TAPIR`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).](media/2020_bey_4.PNG "[Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).")  |
|:--:|
| *Top-left: scenario: the agent cannot recognize whether the **obstacle is traversable** (road blocked, obstacle exists `ep=1`; or not, `ep=0`) until getting closer. The `position` of the obstacle is described by a **continuous variable**. Bottom-left: distribution used for the **`observation model`**: getting closer increases the probability of a `true positive` measurement but also for `false positives`. Very close to the obstacle, it gets detected and correctly classified with high probability. Middle: example of a search in **simplified `belief tree`** with two `action`s: the `distance` of potential object is known, reducing the problem to a **binary** case, with **uniform initial `belief`** (`50/50`). Right: example of trajectories obtained from the **online search**: the `action` that maximizes the **approximated `Q`-function** is selected at each step. Here, future rewards are not discounted: `γ=1`. [Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).* |

| ![[Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).](media/2020_bey_1.PNG "[Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).")  |
|:--:|
| *Given a tree, how to **compute the `Q`(`b`, `a`) values**? Left: Estimate the **expected return** by averaging from **all episodes** passing from this (`b`, `a`) tuple. This leads to a **conservative behaviour** because the outer sum in also **includes episodes that act sub-optimally** (unnecessarily crash into the obstacle) since `ABT` **tries all actions** when hitting a new `belief`. Therefore, the authors switched to the **`max`-option** implemented in `TAPIR` (right). It **does not include suboptimal episodes**: **taking action `a` and acting optimally thereafter**. [Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).* |

| ![[Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).](media/2020_bey_2.PNG "[Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).")  |
|:--:|
| *Left: Influence of the `c-UCT` parameter **balancing the `exploration` / `exploitation`**. **Stochasticity** in the `observations emission` and in the tree construction make results vary. Therefore, **`50` simulations** are performed each time. Middle: Illustration of the **depletion** problem: Because `ABT` only keeps particles that took the same `action` and received the same `observation`, inherently, **particles get lost**. This can be seen at `20s`: the agent receives its first **positive measurement**. At this point **hardly any particles** are close to the observed position. Therefore, **`emergency resampling`** is started, which creates particles close to the **observed location** (in red). This impacts the efficiency as the tree **has to be re-calculated almost from scratch**. Note the **discretisation** with **initial uniform distribution**. Right: **Discretization** is proposed to deal with the **continuous `observation` space**. [Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).* |

| ![[Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).](media/2020_bey_3.PNG "[Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).")  |
|:--:|
| *The expected behaviour is to slow down in front of the **potential** obstacle and **once certain that there is no threat**, accelerate again. A too low `UCT`-factor leads to **too optimistic** (`exploitation`) behaviour: it can plan for up to `20s` but **ignores certain dangerous branches of the tree**. Increasing the `UCT` factor makes it **more conservative**, but a too high `c-UCT` may perform too much **exploration**, leading to a **too short horizon** and also crashes (unable to react). Top-right: **conservative** estimation of the `Q`-values. [Source](https://www.scitepress.org/Link.aspx?doi=10.5220/0009344703120321).* |

Authors: Bey, H., Tratz, M., Sackmann, M., & Lange, A.

- Motivations:
  - `1` Detail how the **`POMDP` online solver** **`ABT`** works.
  - `2` Using a simple example, illustrate the _"difficulties that have to be overcome when trying to_ **_define and solve_** _a_ **_real-world_** `POMDP` _problem"_.
    - Well, **_"real-world"_** means _playing with the algorithm_, and _defining the `POMDP` spaces and models_.
      - But **no consideration** about real-driving or **real-cars**: **no action delay**, `acceleration` as `action`s and **`1s` time-step**.
  - Solver-specific **pitfalls** and impact of **parameters**:
    - `1-` **`UCT`-factor**, balancing `exploration` versus `exploitation`.
    - `2-` Estimation of the **`Q`-value** function.
    - **Continuous** hidden `states` and `observations` makes it hard:
    - `3-` **Particle depletion** in particle filter.
    - `4-` **Degenerated tree** and **space discretization**.
    - `5-` Planning **horizon** and **heuristic** / **default** policy.

- About the scenario:
  - > "The expected behavior is for the vehicle to **slow down** until it is **certain about the obstacle’s state** and then to either stop or accelerate again."

- About [`ABT` (Kurniawati and Yadav, 2016)](https://robotics.itee.uq.edu.au/dokuwiki/papers/isrr13_abt.pdf).
  - `ABT` = **Adaptive Belief Tree**.
  - It is implemented in **"Toolkit for Approximating and Adapting `POMDP` Solutions in Real Time"** ([`TAPIR`](https://robotics.itee.uq.edu.au/~hannakur/dokuwiki/papers/acra14_tapir.pdf)) software [[:octocat:]](https://github.com/rdl-algorithm/tapir).
  - Family of **online** + **sampling-based** solvers:
    - > [approximation] "For more difficult problems it is often desired to **reach a good enough solution** in finite time rather than spending much more time in **search of the optimum**."
    - The **`belief`** (probability distribution over possible `states`) is represented by a **set of sampled `states`** within a **particle filter**.
    - They are **`anytime`-capable**, meaning that they improve the policy as long as they are given time.
    - `ABT` differs from **[Determinized Sparse Partially Observable Tree (`DESPOT`)](https://papers.nips.cc/paper/5189-despot-online-pomdp-planning-with-regularization)** and **[Partially Observable Monte-Carlo Planning (`POMCP`)](https://papers.nips.cc/paper/4031-monte-carlo-planning-in-large-pomdps.pdf)** in that it is able to **keep and only modify the tree if the model changes** (hence _Adaptive_), whereas the others have to **start from scratch**.

- About **particle depletion** (especially in continuous `states`).
  - > "Particles explored with a different `action` at the first step or having received wrong `observations` are lost. Eventually, after several time steps **no particle is left** that fits the actual `observation`."
  - > "The reason being, that the `ABT` does not perform **regular resampling** as it **tries to keep the subtree**, while moving or generating new particles would mean to lose their episodes. This behaviour is opposed to the `DESPOT` solver which uses an **independent conventional particle filtering** including **importance resampling**."
  - > [solution] "`ABT` offers a default implementation to **replenish particles**. It tries to **“force” particles from the previous `belief`** into the new one by always **choosing the `action` actually performed** and “hoping” to receive the correct `observation`, until a minimum number of particles (`1000`) is reached at the new root."

- About **infinite branching factor** (in **continuous** space).
  - > "Whenever a new `observation` is made, a new `observation` branch and subsequent `belief` are opened up. In case of **continuous `observation`** this would mean that **each particle** lands in **its own `belief`**, degenerating the tree just after one step."
  - Solutions:
    - `1-` **Progressive widening**: limits the number of available `observations` at each `action` node, forcing episodes to **run deeper** into the tree.
    - `2-` Space **discretization**: two `observations` are considered equal if a certain distance function falls below a pre-set `threshold`. In that case, the **episode is routed into the same `belief`**.
      - **Small** `threshold` -> `fine`-grained -> many belief nodes -> **shallow search tree**.
      - **Larger** `threshold` -> `coarse`-grained -> less observation branches -> **deeper trees**. But risk:
        - > "If a belief stretches over a large section, there may be trajectories passing through that `belief` and not crashing, even though the obstacle lies within that section. [...] The `belief` may receive a **good value estimate**, making it attractive, even though it is **dangerous**."

- A **`heuristic` function** as a first **estimate** for a **`belief` value**.
  - Can be seen as the **`default` policy** (as opposed to **`tree` policy**) in **`MCTS`**, used for the **rollouts** in the **`simulation`** step.
  - Examples:
    - `1-` The **heuristic** returns simply **zero**.
      - If the `rewards` are constructed to be **always negative**, this leads to **optimistic estimates**.
    - `2-` **`IDM`** model is used.
      - > "This helps to **detect `beliefs`** that **inevitably crash** into the obstacle and should be avoided; the **horizon** is **artificially prolonged**."

- Directions for future works:
  - `1-` **Speed up** the search algorithms by **parallelizing** it. For instance "Hyp-despot: A hybrid parallel algorithm for online planning under uncertainty" [(Cai et al., 2018)](https://arxiv.org/abs/1802.06215).
  - `2-` Better handle **continuous observations**.

</details>

---

**`"Improving Automated Driving through Planning with Human Internal States"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2005.14549)]**
**[[:octocat:](https://github.com/sisl/Multilane.jl/tree/master/thesis)]**
**[** :mortar_board: `Stanford`**]**

- **[** _`MCTS-DPW`, `Q-MDP`, `internal state`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2005.14549).](media/2020_sunberg_1.PNG "[Source](https://arxiv.org/abs/2005.14549).")  |
|:--:|
| *Bottom-left: surrounded cars are modelled with **parametric `IDM` and `MOBIL`** models. These parameters are called `internal state` and are **not observable**. The agent uses a **`belief` tracker** to estimate them. Top: The `action` space has size `10`: `9` combinations of {`change-left`, `change-right`, `stay`} x {`accelerate`, `maintain`, `slow-down`} and one **dynamically determined** `braking` action, computed based on the `speed` and `position` of the vehicle ahead. [Source](https://arxiv.org/abs/2005.14549).* |

| ![[Source](https://arxiv.org/abs/2005.14549).](media/2020_sunberg_2.PNG "[Source](https://arxiv.org/abs/2005.14549).")  |
|:--:|
| *The authors compared different solvers and compare them based on their **Pareto fronts** in this **multi-objective** problem. All approaches are variants of **Monte Carlo tree search** (**`MCTS`**) with **double progressive widening** (**`DPW`**). An approximation consists in assuming that the parameters in the `ìnternal state` are **correlated** (middle). In this case, only the **''type''** of behaviour {`Timid`, `Normal`, `Aggressive`} must be estimated, making the belief tracking and inference simpler: `dim`=`1` compared to `dim`=`8`. [Source](https://arxiv.org/abs/2005.14549).* |

| ![[Source](https://arxiv.org/abs/2005.14549).](media/2020_sunberg_3.PNG "[Source](https://arxiv.org/abs/2005.14549).")  |
|:--:|
| *Two kinds of **robustness** are considered for the `internal state` estimation: How much are the **parameters correlated**. And how **diverse** can their value be (**domain** size). [Source](https://arxiv.org/abs/2005.14549).* |

Authors: Sunberg, Z., & Kochenderfer, M.

- Motivations:
  - > "Our hypothesis is that **planning** techniques that consider **`internal states`** such as **`intentions`** and **`dispositions`** of other drivers can simultaneously improve **_safety_** and **_efficiency_**."

- About **uncertainties**:
  - `1-` **Epistemic** uncertainty: can be reduced through **learning**.
  - `2-` **Aleatory** uncertainty: **cannot be reduced** with any kind of knowledge.
  - > "The **`internal state`** (e.g., `intentions` and `aggressiveness`) of other drivers and road users can **only be indirectly inferred.**"
    - **`internal state` uncertainty** can be said **epistemic**.
  - > "`POMDP` planners **model human drivers’ internal states** as **epistemic uncertainty**, while the MDP methods **only consider aleatory uncertainty**."

- About `(PO)MDP` solvers:
  - > [Only **_approximate_ planning** approaches are considered] "Though `POMDP`s are **very powerful in terms of expression**, even the class of finite-horizon `POMDP`s is **`PSPACE-complete`**, indicating that it is unlikely that **efficient general _exact_ algorithms** for **large problems** will be discovered."
  - Only **online** solutions are considered here, and all the approaches use variants of **Monte Carlo tree search** (**`MCTS`**) with **double progressive widening** (**`DPW`**) - as opposed to **`value iteration`** methods for instance.
    - Main idea of **`MCTS-DPW`**: **restrict the search to relevant regions** of the tree.
    - > "`DPW` further **focuses computation** by considering only a limited, but **gradually increasing**, number of sampled `state`s or `observation`s".

- About the **driver models**:
  - > [**Transition function** in the **`(PO)MDP`** formulation] "The `T` function is implicitly defined by a **generative model** that consists of a **state transition function**, `F(·)`, and a **stochastic noise** process."
  - `IDM` and `MOBIL` **_parametric_ models** are used.
    - Example of parameters: The `MOBIL` parameter `p` in `[0, 1]` is the **politeness factor**, which represents how much the driver values **allowing other vehicles** to increase their acceleration.
  - Their **parameters** are used to define the **`internal state`**.
  - Limitations:
    - > "The **primary weakness** of this investigation is the model of the other drivers. Since the `IDM` and `MOBIL` models were developed to **simulate large scale traffic flow**, simulations with these models **may not be accurate**. Incorporating **models learned from data** would further validate the conclusions drawn here."
    - The authors also point that these models **neglect dynamic intentions** of other drivers.

- About the **correlation** of the **distributions** of the parameters in the **`internal state`s**.
  - The **`internal state`** consists in **many parameters** (from `IDM` and `MOBIL`): e.g. `desired deceleration`, `desired speed`, `desired time gap`, `politeness factor`...
  - `1-` All parameters could be assumed **perfectly correlated**.
    - An **`aggressiveness`** class can be defined {`Timid`, `Normal`, `Aggressive`}, with defined values for each parameter.
    - In this case, **a particle** consists of **only a single value**: **`dimension` = `1`**.
  - `2-` Parameters could be assumed **uncorrelated** and must be **estimated jointly**.
    - A **particle** consists of values of all model parameters: **`dimension` = `8`**.
  - `3-` A **partial correlation** is also investigated.
  - **Robustness analysis `1`**: Simulation models with **varying levels of correlation** are tested:
    - > "When the **parameters are fully correlated**, all of the parameters are **easy to estimate** by **observing only a few**, so there is not a significant performance gap between `MSM`, `QMDP`, and `POMCPOW`."
    - > "On the other hand, when the **parameters are uncorrelated**, the `QMDP` planner performs much better than the **`certainty equivalence planner`** [_`Mean state MDP`_], and `POMCPOW` performs much better than `QMDP`."
  - **Robustness analysis `2`**: the **domain** from which the parameters are drawn is **expanded by a variable factor**.
    - For instance, the **width** of the interval (e.g. [`0.5s`, `2.5s`]), from which the marginal distribution of the `desired time gap` parameter is drawn, is varied.
    - > "Robustness to inaccuracy in the **parameter domain** is **one-sided**: when the true domain is **larger** than that assumed by the planners, performance is **adversely affected**, but when the **true domain is smaller**, there is **no degradation**."
  - Conclusion for human-driver-modelling:
    - > "**Simple solvers are enough** if most human driver behaviour can be correlated with easily measurable quantities."
    - > "Otherwise, need **more advanced planners** that carry the uncertainty further into the future."

- About the **`observation` filtering**:
  - The `belief` at a given time consists of:
    - `1-` The **exactly known physical state**.
    - `2-` A collection of **particles**, along with associated **weights**, representing possible **internal state**.
      - The **online estimation** of the **internal state** `θ` (driver model **parameters**) is accomplished with a **particle filter**.
  - To **update the `belief`** when an `action` is taken:
    - `1-` New **particles** are **sampled** with probability proportional to their **weights**.
    - `2-` Sampled **noise values** and the **state transition function** are used to **generate new `state`s**.
    - `3-` The **new weights** are determined by approximating the **conditional probability** of the particle given the `observation`.

- **Approximate Planning** Approaches:
  - `1`+`2`: **Baselines**, representing ways to **force the epistemic uncertainty** in the `POMDP` into an **`MDP` formulation**.
    - `1-` `Normal`: All **drivers' parameters** are assumed to be known and identical.
      - **No belief tracking** is needed: an `MDP` is formulated.
      - It is **over-optimistic** since it assumes to **knows** the **`internal states`** of other drivers.
      - **Over-confident**: it is able to **reach the goal** a large proportion of the time, but it causes **many safety violations**.
    - `2-` `Naive MDP`: The `internal states` are considered as **random variables**, independent at each timestep, i.e. assumed to be simply **aleatoric**.
      - No **belief tracking** can be done: an `MDP` with a `state` consisting **only of the physical state** is formulated.
      - It is **conservative** since it plans **pessimistically** assuming **it can learn nothing new about the drivers**.
      - **Over-cautious**: it can attain a **high level of safety**, but it is never able to meet the goal more than `80%` of the time.
  - `3`+`4`+`5`: Online estimation of the **internal state** `θ` using a **particle filter**.
  - `3`+`4`: **Passively** online learning. Optimistic assumption. Hence systematically suboptimal.
    - > "These approximations are useful in many domains because they **are much easier to compute** than the full `POMDP` solution since they require only the solution to the **fully observable MDP**."
    - There is **no incentive** for **learning about the state**, and hence these policies will **not take costly actions** for **"active learning"** / **"information gathering"**.
    - `3-` `Mean state MDP`.
      - At each timestep, a **fully observable `MDP`** is constructed using the **mean internal state** values: `s` = `E(s∼b)[s]`.
      - > "This approach is sometimes referred to as **`certainty equivalence control`**".
      - It is **over-confident** (achieving a high success rate, but sacrificing safety) because it plans **without any internal state uncertainty**.
    - `4-` **[`QMDP`](http://www-anw.cs.umass.edu/~barto/courses/cs687/Cassandra-etal-POMDP.pdf)**: hypothetical problem with **partial observability on the current step**, but that **subsequently becomes fully observable**.
      - > "Another natural approach to finding `Q` functions for `POMDP'`s is to make use of the `Q` values of the **underlying `MDP`**. That is, we can **temporarily ignore the `observation` model** and find the `Q-MDP`(`s`, `a`) values for the `MDP` consisting of the `transitions` and `rewards` only." [(Littman et al., 1994)]
      - > "With the `Q-MDP` values in hand, we can treat all the `Q-MDP` values for each action as a **single linear function** and estimate the `Q` value for a **`belief` state** `b`. This estimate amounts to assuming that **any uncertainty** in the agent's **current `belief` state** will be gone after the next `action`." [(Littman et al., 1994)]
      - The `action` selected maximizes the **`expected Q-MDP` value** for the current `belief`: **`Q`(`b`, `a`) = `Σ` `b`(`s`).`Q-MDP`(`s`, `a`)** _(single linear function)_
      - Here, the `Q-MDP` values are estimated through **`MCTS-DPW` instead of `value iteration`**.
      - It performs better than `mean state MDP` because it considers samples from the **entire estimated internal `state` distribution** when planning.

  - `5-` [`POMCPOW`](https://github.com/JuliaPOMDP/POMCPOW.jl).
    - > "Since full **Bayesian belief updates** are computationally expensive, the `POMCPOW` algorithm **extends `MCTS`** to include **approximate beliefs** represented by **weighted particle collections** that are **gradually improved** as the tree is searched".
    - Since the vehicle does not have to take **costly information-gathering `action`s** to accomplish its goal, `POMCPOW` only outperforms `QMDP` in certain cases.
  - `6-` A **omniscient** upper bound planner.

- _How to_ **_compare_** _approaches on this_ **_multi-objective_** _problem?_
  - Two criteria and terms in the `reward` function:
    - `1-` **Safety**: any situation in which any human-driven or autonomous vehicle has to **break hard to avoid a collision** is marked **_"unsafe"_**.
    - `2-` **Efficiency**: accomplishing a **goal** with minimum resource use (`time`).
  - > "At first, it may seem that **`safety` is a strictly higher priority** than `efficiency`, but consumers **will not sacrifice efficiency without limit**."
  - **Multi-objective** comparison with **Pareto frontier**.
    - `reward` function: **Weighted sum** of the **competing goals**, create a single objective function.
      - `1-` Positive reward for **reaching the target lane** within the distance limit.
      - `2-` Negative reward for **hard brakes** and **slow velocity**.
    - Different **weight** values are tested: **`λ`** in {`0.5`, `1`, `2`, `4`, `8`}.
    - **Pareto fronts** are approximated by connecting the resulting **Pareto points** with straight lines.
    - > "Conclusions about different algorithms can be reached by **comparing the resulting curves** generated by those algorithms."

</details>

---

**`"Efficient Uncertainty-aware Decision-making for Autonomous Vehicles Using Guided Branching"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.02746)]**
**[[:octocat:](https://github.com/HKUST-Aerial-Robotics/eudm_planner)]**
**[** :mortar_board: `Hong Kong University`**]**

- **[** _`guided branching`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.02746).](media/2020_zhang_2.PNG "[Source](https://arxiv.org/abs/2003.02746).")  |
|:--:|
| *The **Domain-specific Closed-loop Policy Tree** (`DCP-Tree`) provides a **guided branching mechanism** in the action space. For efficiency, this **semantic-level policy tree** is updated based on the previous best policy. Each `node` in the policy tree is a **finite-horizon semantic behaviour** of the ego car. From the **ongoing action**, each policy sequence will contain **at most one change of action** in one planning cycle. Compared to `MPDM`, this offers the possibility to **change decision in the planning horizon** while, as **humans**, not frequently changing the driving policy back and forth in a single decision cycle. [Source](https://arxiv.org/abs/2003.02746).* |

| ![[Source](https://arxiv.org/abs/2003.02746).](media/2020_zhang_1.PNG "[Source](https://arxiv.org/abs/2003.02746).")  |
|:--:|
| *Decision-making using **domain-specific expert knowledge** to **guide branching**, inspired by [`MPDM`](https://ieeexplore.ieee.org/document/7139412). Branching in the `action` domain is guided by the **`DCP`-Tree**. Branching in the `intention` domain is done by the **`CFB` mechanism** to **pick out risky hidden intentions** of nearby vehicles. [Source](https://arxiv.org/abs/2003.02746).* |

Authors: Zhang, L., Ding, W., Chen, J., & Shen, S.

- Motivations: Propose a simpler alternative to existing online `POMDP` solvers (`DESPOT`, `ABT`, `POMCP`), with focus on **efficiency**.
  - > "The key idea is utilizing **domain-specific expert knowledge** to **guide the branching** in both `(1) action` and `(2) intention` space."
  - The goal is to consider **as few branches as possible**. The **most critical** ones, potentially leading to **risky outcomes**.
- Two related works:
  - ["Multipolicy Decision-Making for AutonomousDriving via Changepoint-based Behavior Prediction"](http://roboticsproceedings.org/rss11/p43.pdf)
  - ["MPDM: Multipolicy decision-making in dynamic, uncertain environments for autonomous driving"](https://ieeexplore.ieee.org/document/7139412)
  - **`MPDM` is extended** here and used as a benchmark.
  - > "`MPDM` approximates the `POMDP` process into the `(1)` **closed-loop simulation** of predefined `(2)` **semantic-level** driving policies (e.g., `LC`, `LK`, etc.). The incorporation of **domain knowledge** greatly **accelerates** the problem-solving."
    - `(1)` The exploration of the state space is guided by **simple** **_closed-loop controllers_**, i.e. **domain knowledge**.
    - `(2)` **Semantic-level policies** instead of traditional "`state`"-level actions such as **discretized accelerations or velocities**
  - > "One major limitation of `MPDM` is that the semantic-level policy of the ego vehicle is **not allowed to change in the planning horizon**."
- Differences with the original `MPDM`:
  - `1-` The **policy** of the ego vehicle is allowed to **change in the planning horizon** according to the `DCP-Tree`.
  - `2-` Focused branching is applied to pick out the **risky scenarios**, even given totally uncertain behaviour prediction, which enhances the **safety** of the framework.
    - > "The `CFB` mechanism is applied to **pick out risky hidden intentions of nearby vehicles** and achieves **guided branching** in **intention** space."
- About formulation:
  - [hidden part of the state] The **intention** about lateral behaviours in {`LK`, `LCL`, `LCR`}.
  - [belief tracker] A **"rule-based lightweight belief tracking module"** generates a probability distribution over these **intentions**. _No much details about this module._
  - [transition model] Two driver models: `intelligent driving model` and `pure pursuit controller`.
  - [action] Longitudinal {} and lateral semantic-level decisions.
- Some terms:
  - **`MPDM`** = multipolicy decision-making.
  - **`EUDM`** = uncertainty-aware decision-making.
  - **`DCP-Tree`** = domain-specific closed-loop policy tree.
  - **`CFB`** = conditional focused branching.

</details>

---

**`"Autonomous Driving at Intersections: A Critical-Turning-Point Approach for Left Turns"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.02409)]**
**[** :mortar_board: `University of Waterloo`, `Tsinghua University` **]**
**[** :car: `Tencent` **]**

- **[** _`intention-aware motion planning`, `ABT`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.02409).](media/2020_shu_1.PNG "[Source](https://arxiv.org/abs/2003.02409).")  |
|:--:|
| *[Source](https://arxiv.org/abs/2003.02409).* |

| ![[Source](https://arxiv.org/abs/2003.02409).](media/2020_shu_2.PNG "[Source](https://arxiv.org/abs/2003.02409).")  |
|:--:|
| *The ego vehicle **updates its beliefs** on the **`route`-intention** of the oncoming vehicle. At start, actions share the same pattern. But when the **`left-turn` intention becomes highly likely**, an `acceleration` action is performed (top-left). In the `straight`-intention case, **the future path is blocked** by the oncoming traffic (right). Instead of stopping and then **waiting at standstill**, the ego vehicle slows down to **creep forward**, targeting **a `CTP` more far away**. [Source](https://arxiv.org/abs/2003.02409).* |

Authors: Shu, K., Yu, H., Chen, X., Chen, L., Wang, Q., Li, L., & Cao, D.

- Motivations:
  - Replicate **_human-like_ efficient behaviour** for a **left-turn** at an unsignalized intersection, trading off between **safety** and **conservatism**, without explicit hand-written rules:
    - > "Before merging into the intersection, the ego vehicle drives into the intersection with high speed. Then **it decelerates to a lower speed and creeps forward**, considering the **potential of collision** with the oncoming vehicle while **waiting for the oncoming vehicle’s intention to become certain**. The ego vehicle then performs **more confident** actions on a proper route when the intention of the oncoming vehicles becomes clear."
- Architecture:
  - `1-` [`high-level`] Paths generation to define `CTP`s. **Offline**.
  - `2-` [`low-level`] **`CTP` selection** and speed planning on the chosen path.  **Online**.
- One term: **"critical turning point" (`CTP`)**.
  - > "The starting points where the vehicle **makes hard `steering`** (**sharp turning**) are identified and extracted as **'turn points'**."
  - They are computed based on the critical zone extraction (`CZE`) which is generated from road geometry.
  - Path candidates are then generated from the `CTP` and sent to the lower-level planner for `path selection` and `speed planning`.
  - [Benefits of `CTP`s]
    - > "Our **candidate paths planned with `CTP`** gives the ego vehicle an option to **keep moving forward** to reach the next `CTP` when one of the paths is blocked by the oncoming traffic."
    - > "The proposed method spends about **`1.5s` less time to pass** through the intersection than the one that does not use `CTP`s."
    - > "When the oncoming vehicle is driving with a **higher speed**, the **shortest and the most aggressive path** is chosen since the **waiting time is shorter**."
- About the **`POMDP`** formulation:
  - [hidden part of `state`] Oncoming vehicles have unknown **intentions**: either `straight` or `left-turn`.
    - The other vehicle is assumed to **ignore the ego-car** and the uncertainty is about the **chosen route**.
    - _Other approaches include the_ **_yield reaction_** _of the other's, e.g. {`stopping`, `hesitation`, `normal`, `aggressive`}._
  - [`observation`] `speeds` and `positions` in **Cartesian** coordinate system.
    - _I think an `orientation` would also be useful to infer which route is taken + give a sense to the `speed` scalar._
  - [`action`] It is **`2`-dimensional**:
    - `1-` (`speed` planning) - The **`ego-acceleration` along the current path** in [`-4 m/s2`, `4 m/s2`] with a step of `1 m/s2`.
    - `2-` (`path` / `CTP` selection) - A **`left-turn` Boolean variable** which _"conveys sharp turn instructions"_. - _I understand it as a change in selected path: where to leave the initial lane_
  - [`transition` model] All vehicles are assumed to move at **constant speed** on predefined routes.
  - [`solver`] [Adaptive Belief Tree](http://robotics.itee.uq.edu.au/dokuwiki/papers/isrr13_abt.pdf) (`ABT`).
    - "Instead of **trimming the entire policy tree** after an action is selected, the `ABT` algorithm **only modifies parts** of the tree that are influenced by the updated belief after executing the selected action".

</details>

---

**`"Integrating Planning and Interpretable Goal Recognition for Autonomous Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2002.02277)]**
**[[🎞️](https://youtu.be/IPa-cxcdT8U?t=5849)]**
**[** :mortar_board: `University of Edinburgh` **]**
**[** :car: `FiveAI` **]**

- **[** _`goal recognition`, `intention-aware motion planning`, `inverse planning`, `MCTS`_ **]**

<details>
  <summary>Click to expand</summary>

| ![Example of scenario (note: **left-hand drive**) where prediction based on goal-recognition can inform the planning. It enables a **less conservative behaviour** (entering the intersection earlier) while **offering interpretability**. [Source](https://arxiv.org/abs/2002.02277).](media/2020_albrecht_1.PNG "Example of scenario (note: **left-hand drive**) where prediction based on goal-recognition can inform the planning. It enables a **less conservative behaviour** (entering the intersection earlier) while **offering interpretability**. [Source](https://arxiv.org/abs/2002.02277).")  |
|:--:|
| *Example of scenario (note: **left-hand drive**) where prediction based on goal-recognition can inform the planning. It enables a **less conservative behaviour** (entering the intersection earlier) while **offering interpretability**. [Source](https://arxiv.org/abs/2002.02277).* |

| ![The main ideas are to **couple `prediction` and `planning`**, try to **infer the `goals`** followed by the other vehicles and use **high-level abstraction** of `manoeuvres` via **`macro actions`**. [Source](https://arxiv.org/abs/2002.02277).](media/2020_albrecht_2.PNG "The main ideas are to **couple `prediction` and `planning`**, try to **infer the `goals`** followed by the other vehicles and use **high-level abstraction** of `manoeuvres` via **`macro actions`**. [Source](https://arxiv.org/abs/2002.02277).")  |
|:--:|
| *The main ideas are to **couple `prediction` and `planning`**, try to **infer the `goals`** followed by the other vehicles and use **high-level abstraction** of `manoeuvres` via **`macro actions`**. [Source](https://arxiv.org/abs/2002.02277).* |

| ![The ego-agent **updates its belief** on the goal follow by the other vehicles (left). As noted below, the **ablation study** (right) raises question about **what really offers benefits** for the time-efficiency of the driving policy. [Source](https://arxiv.org/abs/2002.02277).](media/2020_albrecht_3.PNG "The ego-agent **updates its belief** on the goal follow by the other vehicles (left). As noted below, the **ablation study** (right) raises question about **what really offers benefits** for the time-efficiency of the driving policy. [Source](https://arxiv.org/abs/2002.02277).")  |
|:--:|
| *The ego-agent **updates its belief** on the goal follow by the other vehicles (left). As noted below, the **ablation study** (right) raises question about **what really offers benefits** for the time-efficiency of the driving policy. [Source](https://arxiv.org/abs/2002.02277).* |

Authors: Albrecht, S. V., Brewitt, C., Wilhelm, J., Eiras, F., Dobre, M., & Ramamoorthy, S.

- Motivations:
  - `1-` Improve the **_anticipation_ ability** and hence the **_efficiency_** of driving behaviour at **urban intersections**.
  - `2-` Provide **intuitive interpretations** of the predictions to justify the ego-agent's decisions.
  - `3-` Keep **computational efficiency** low.
- The main **ingredients** to achieve that are:
  - Couple **`planning`** and **`prediction`**.
  - Infer the **goals** (here specifying **target locations**) of other vehicles.
  - Use of **high-level** manoeuvres and **macro actions**.
- Two assumptions:
  - `1-` Each vehicle **seeks to reach some (unknown) goal location** from a set of **possible goals**, and behaves **_rationally_** by driving optimally to achieve goals.
    - Note: for that second point, the definition of the likelihood `p`(`trajectory | goal-i`) still allows for a degree of deviation.
  - `2-` At any time, each vehicle is executing one of **`manoeuvre`** among a **finite set**:
    - `lane-follow`
    - `lane-change-left`/`right`
    - `turn-left`/`right`
    - `give-way`
    - `stop`
- About **goad recognition**:
  - The idea is to **recognise the goals** of other vehicles, in order to perform **rational inverse planning**, i.e. made better _(better informed)_ decisions.
  - > "We must **reason about _why_** – that is, _to what end_ – the vehicle performed its current and past manoeuvres, which will **yield clues as to its intended goal**."
  - > "_[Limitation of the_ **_optimality assumption_**_]_ An important future direction is to account for **human irrational biases**."
- _How to plan ego-action?_ **By leveraging recognition and prediction**.
  - Both **`goal probabilities`** and **`trajectory predictions`** are used to **inform** a **Monte Carlo Tree Search (`MCTS`)** algorithm.
- _How to reduce the search depth?_ **Using `macro actions`**.
  - > "To keep the required **search depth shallow** and hence **efficient**, both `inverse planning` and `MCTS` plan over **macro actions**."
  - In hierarchical `RL`, `macro actions` are sometimes defined by a tuple <`applicability condition`, `termination condition`, `primitive policy`>.
  - Here _(I am a little bit confused)_:
    - Each _manoeuvre_ also specifies **applicability conditions** (`lane-change-left` is only **_applicable_** if there is a lane in same driving direction on the _left_ of the vehicle) and **termination conditions**.
    - > "Macro actions concatenate one or more manoeuvres, [and] automatically set the **parameters** in manoeuvres [e.g. `driving distance` for `lane-follow` or `lane-id`s in `give-way`] based on **context information** (usually **road layout**)".
    - But no underlying _primitive policy_ is used:
    - > "`Macro actions` as used in this work **do not define a hierarchy** of decomposable actions; they simply define sequences of actions."
  - By reasoning on a **high level of abstraction**, `macro actions` offer a **temporal abstraction** that relieves the planner and ensures a low computational cost.
- Main steps of the approach:
  - `1-` **Goal Generation**: Only consider locations that are **reachable**.
  - `2-` **Manoeuvre Detection**: Compute the posterior **probabilities over each goal** for each manoeuvre.
  - `3-` **Inverse Planning**: Using `A*` search over **macro actions**, derive an optimal plan.
  - `4-` **Trajectory Prediction**:  Predict **_multiple_ plausible trajectories** for a given vehicle and goal, rather than a single optimal trajectory.
    - Hence accounting for the **multi-modal** nature of the **future prediction**: given the same context, future may vary.
    - It assumes that trajectories which are **closer to optimal** are **more likely**.
- About `close-loop` / `open-loop` forward simulation:
  - > "The ego vehicle’s motion always uses closed-loop mode, while other vehicles can be simulated in either closed-loop or open-loop mode."
  - `closed-loop` simulation: it uses a combination of **proportional control** and **adaptive cruise control** (`ACC`), based on `IDM`.
  - `open-loop` simulation: **no automatic distance keeping** is used.
    - The vehicle's position and velocity directly are set as specified in trajectory.
- About the **experiment** settings:
  - > "For each _[of the four]_ scenario, we generate `100` instances with **randomly offset initial longitudinal positions** (offset ∼ [`−10`, `+10`] `m`) and **initial speed sampled** from range [`5`, `10`] `m/s` for each vehicle including ego vehicle."
  - Frequency of `MCTS`: **`1 Hz`**.
  - Number of simulations: `D` `=` `30`.
  - Maximum search depth: `d-max` `=` `5`.
  - **Prior** probabilities for **achievable goals**: `uniform`.
- Benefits of the approach:
  - One of the baselines implements `MCTS` **without goal recognition**: the prediction instead assumes on **constant-velocity** lane-following.
    - This candidate suffers from a limited prediction horizon, but **still performs well** in term of **driving time** required to complete scenario.
  - Another baseline also uses `CV` models, together with a **"conservative give-way manoeuvre"** which **waits** until all oncoming vehicles on priority lanes have passed.
    - Hence **no `MCTS`**.
    - This one is **not able to infer goal** and anticipate behaviour, preventing them to safely **enter the road earlier**, for instance when a car is detected to exit the roundabout.
  - Based on this **ablation study**, it is not clear to me _what improves the efficiency of the driving policy_:
    - `1-` Is it the consideration of goals?
    - `2-` Or just the **coupling of `prediction` + `planning`** which gets rid of the conservative **_"wait until clear"_** condition?

</details>

---

**`"Point-Based Methods for Model Checking in Partially Observable Markov Decision Processes"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2001.03809)]**
**[[:octocat:](https://github.com/sisl/POMDPModelChecking.jl)]**
**[** :mortar_board: `Stanford University` **]**
**[** :car: `Honda` **]**

- **[** _`probabilistic garanties`, `safety checkers`, `POMDP`, `SARSOP`_ **]**

<details>
  <summary>Click to expand</summary>

| ![POMDP Model Checker. Source: author provided - taken during the SIPD workshop.](media/2020_bouton_1.PNG "POMDP Model Checker. Source: author provided - taken during the SIPD workshop.")  |
|:--:|
| *__`POMDP` quantitative model checker__. Three parts are followed: `1-` Creation of a `product POMDP`. `2-` **Reduction to reachability** (From `LTL` Satisfaction to **Reachability**). `3-` Solving the **reachability problem**. Source: author provided - taken during the `IV19` `SIPD` workshop - see my report [here](https://github.com/chauvinSimon/IV19#risk-assessment-and-safety-checkers).* |

| ![A **reachability problem** can be interpreted as a **planning problem** where the goal is to **reach the set `B`**. In `LTL` terminology, `F` means 'eventually'. [Source](https://arxiv.org/abs/2001.03809).](media/2020_bouton_2.PNG "A **reachability problem** can be interpreted as a **planning problem** where the goal is to **reach the set `B`**. In `LTL` terminology, `F` means 'eventually'. [Source](https://arxiv.org/abs/2001.03809).")  |
|:--:|
| *A **reachability problem** can be interpreted as a **planning problem** where the goal is to **reach the set `B`**. In `LTL` terminology, the temporal operator `F` means 'eventually'. [Source](https://arxiv.org/abs/2001.03809).* |

Authors: Bouton, M., Tumova, J., & Kochenderfer, M. J.

- Motivations:
  - `1-` Synthesize **policies** that satisfy a **linear temporal logic** (`LTL`) formula in a **`POMDP`**, i.e. make `POMDP` policies exhibit **guarantees on their performance**.
    - **_"Policy synthesis"_** means that some _good_ policy is derived, as opposed to just the **evaluation** of a given policy (computation of the probability of satisfaction for an objective).
  - `2-` **Scale** to larger problem than previous `belief`-`state` techniques (note that only finite **discrete `state` spaces** are considered here).
    - For instance, [`Norman et al.`](https://arxiv.org/abs/1506.06419) addressed the problem of `belief`-`state` planning with `LTL` specifications by **discretizing the belief space** and formulating an **`MDP`** over this space.
    - But when the `state` space has more than a few dimensions, **discretizing the `belief` space** becomes **intractable**.

- About **model checking**:
  - 1- **`Quantitative`** model checking: Compute the maximum **probability** of satisfying a desired logical formula (and compute the associated `belief`-`state` **policy**).
  - 2- **`Qualitative`** model checking: Find a policy satisfying the formula with **probability `1`**.
  - It makes me think of the **_strict_ action masking** methods and masking approaches that consider [**statistical** model checking](https://github.com/chauvinSimon/IV19#risk-assessment-and-safety-checkers), such as **_probabilistic_ reachable sets**.
- About `LTL` formulas:
  - `LTL` is used as a language to **specify the objective** of the problem.
  - Examples:
    - `¬A` `U` `B` means "**avoid** state `A` and **reach** state `B`" (**safe-reachability** objective).
    - `G` `¬C` means "the agent must **never visit** state `C`" (_the temporal operator `G` means "globally"_).
- About **_"reachability problems"_**:
  - > "[the goal is to] Compute the **maximum probability** of **reaching** a given set of `state`s."
- About **_"labelling functions"_** for states of the `POMDP` in the context of `LTL` formulas:
  - The **`labels`** are **atomic propositions** that evaluate to true or false at a given `state`.
  - A **labelling function** maps each `state` of the environment to the set of atomic propositions **holding in that `state`**.
  - > "We do not assume that the labels constituting the **`LTL` formula** are observable. The agent should **infer the labels** from the observations."
  - Concretely, the agent cannot observe whether it has **reached an end component or not**, but the `belief state` characterizes the confidence on whether or not it is in an end component. Therefore, it maintains a belief on **both** the `state` of the environment and the **`state` of the automaton**.

- One major idea: formulate **_"reachability problems"_** (_quantitative_ model checking problem) as **reward maximization** problems.
  - > "We show that the problem of finding a policy **maximizing the satisfaction of the objective** can be formulated as a **reward maximization** problem. This consideration allows us to benefit from **efficient approximate `POMDP` solvers**, such as `SARSOP`."
  - In other words, a **reachability** problem can be interpreted as a **planning** problem where the **goal** is to reach the set `B` (the set of states where the propositional formula expressed by `B` **holds true**).
  - For instance, the **reward function** gives `1` if `s` in `B`.
- Steps of the approach:
  - `1-` Creation of a **`product POMDP`**.
    - > "We define a **new `POMDP`** such that solving the original quantitative model checking problem reduces to a reachability problem in this model."
    - The new `POMDP` is called **`product POMDP`**:
      - The **`state` space** is the **Cartesian product** of the state space of the original `POMDP` and the **deterministic rabin automaton** (`DRA`, representing the `LTL` formula).
        - > "The construction of the `product POMDP` can be interpreted as a principled way to **augment the `state` space** in order to **account for temporal objective**."
        - > "For formulas involving only a single **until (`U`)** or **eventually (`F`)** temporal operators, the problem can be **directly expressed as a reachability problem** and does **not require a state space augmentation**".
      - A **new `transition` function** is also defined, using the fact that any `LTL` formula can be represented by a _deterministic Rabin automaton_ (resulting in a finite state machine).
  - `2-` **Reduction to reachability** (i.e. go from `LTL` satisfaction to **reachability**).
    - Solving the **original quantitative model checking problem** reduces to a **reachability problem** in this **`product POMDP`** model.
      - Reaching a `state` in this **set** guarantees the **satisfaction of the formula**.
    - _What is to be done_:
      - `First` find the **_"end components"_**.
      - `Then` identify the **_success_ `state`s**.
    - The **computation** of the **maximal end components** is one of the two **bottlenecks** of the presented approach (together with the choice of the planning algorithms).
  - `3-` Solving the **reachability problem**.
    - Here, the `state` uncertainty will play a role (distinguishing `MDP`s from `POMDP`s).

- About the **solver** used: [`SARSOP`](http://www.roboticsproceedings.org/rss04/p9.pdf).
  - The idea is to restrict the `policy` space (hence an **approximation**), using `alpha vector`s.
    - `alpha vector`s are `|state space|`-dimensional vectors defining a **linear function** over the `belief` space.
    - They are used to represent **both** the `policy` and the `value function`.
      - Hence, they can serve to approximate the **quantitative model checking** problem and **not only** the **policy synthesis** problem.
  - About **point-based value iteration** (`PBVI`) algorithms.
    - This is a family of `POMDP` solvers that involves applying a **Bellman backup** (hence _"value iteration"_) to a set of **`alpha vectors`** in order to **approximate** the optimal value function.
    - _Why it is said "point-based"?_
      - Vanilla value iteration (`VI`) algorithms cannot scale for `POMDP`s.
      - In **`PBVI`** algorithms, the `belief` space is **sampled**.
      - An `alpha vector` associated to **each belief _point_** is then computed to approximate the value function at **that _point_**.
  - _What is the specificity of `SARSOP`?_
    - It stands for _"Successive Approximations of the_ _**Reachable Space**_ _under Optimal Policies"_.
    - It relies on a **tree search** to explore the `belief` space.
      - It maintains **_upper_ and _lower_ bounds** on the **value function**, which are used to **guide the search** close to optimal trajectories (i.e. only exploring relevant regions).
      - In other words, it focuses on regions that can be **reached** from the **initial belief point** under optimality conditions.
    - This makes `SARSOP` one of the most **scalable** **_offline_** `POMDP` planners.

- Another major idea: use the _upper_ and _lower_ bounds of `SARSOP` to estimate the **probability of satisfaction of the `LTL` formula**.
  - In `PBVI` algorithms, **convergence guarantees** are offered, specified in _upper_ and _lower_ bound on the **value function** (e.g. one can **control the convergence** of the value function by controlling the **depth of the tree** in `SARSOP`).
  - > "For a given **precision parameter**, we can directly **translate the bounds** on the **value function** in the `product POMDP` in terms of **probability of success** for our problem of **_quantitative_ model checking**".
  - The user can specify the **precision parameter**.

</details>

---

**`"Self-Driving Under Uncertainty - aa228 Course Project"`**

- **[** `2020` **]**
**[[:memo:](http://files.coldattic.info/stanford/aa228/AA228_Final.pdf)]**
**[[:octocat:](https://github.com/pshved/aa228)]**
**[** :mortar_board: `Stanford University` **]**

- **[** _`POMCPOW`, `JuliaPOMDP`_ **]**

<details>
  <summary>Click to expand</summary>

| ![On the **graphical model**, it can be seen that only the **position of the obstacle** `b-xy` is **not fully observable**. One contribution is the use of **second reward model** to help. It first encourages high speed. And also linearly reward the **approaching of the goal** to address the limitation of **depth-bounded** online tree search methods. [Source](http://files.coldattic.info/stanford/aa228/AA228_Final.pdf).](media/2019_shved_1.PNG "On the **graphical model**, it can be seen that only the **position of the obstacle** `b-xy` is **not fully observable**. One contribution is the use of **second reward model** to help. It first encourages high speed. And also linearly reward the **approaching of the goal** to address the limitation of **depth-bounded** online tree search methods. [Source](http://files.coldattic.info/stanford/aa228/AA228_Final.pdf).")  |
|:--:|
| *On the **graphical model**, it can be seen that only the **position of the obstacle** `b-xy` is **not fully observable**. One contribution is the use of **second reward model** to help. It first encourages high speed. And also linearly reward the **approaching of the goal** to address the limitation of **depth-bounded** online tree search methods. [Source](http://files.coldattic.info/stanford/aa228/AA228_Final.pdf).* |

Author: Shved, P.

- A university project completed for the **[aa228 - Decision Making under Uncertainty](https://web.stanford.edu/class/aa228/cgi-bin/wp/)** course at Stanford.
  - The scenario is a simple straight road with **single moving obstacle** (whose behaviour is **non-adversarial** and **does not change** in response to the actions taken by the ego-agent).
  - Results were not very brilliant, but the author proposes a good analysis:
    - > "We find that with **certain fine-tuning**, `POMCPOW` produces driving decisions that result in mission completion albeit sub-optimally. However, for more complicated scenarios, the quality of planning decisions degrades and the **agent gets stuck** without completing the mission."
- Initial motivation:
  - Evaluate the **robustness** of the planner, i.e. study its performance while **varying the perception precision**.
  - That means the **`observation` model is stochastic** (_Gaussian_) while the `transition` model is kept deterministic.
- Some elements about the `POMDP` formulation:
  - The `state` space is continuous and not discretized.
  - The task is **episodic**: A `state` is **terminal** if
    - `1-`_(either)_ The **end of the road** is reached.
    - `2-` _(or)_ A **collision** occurs.
- One idea: use a **second `reward` model**.
  - `1-` The **first** reward model evaluates the driving behaviours: _rewarding reaching the goal_, _penalizing collisions_ and _encouraging progress_.
    - > "However, the solvers used were unable to attain good results on that model directly. We **augment** this reward model and use model `R2`."
  - `2-` The second was added after testing the **online search** and serves two purposes:
    - > "In the challenge model, the **large positive reward** for **mission completion** is **_spread out_** across the state space, and we **reward high velocity**."
    - Concretely, it adds a term proportional to `r-mission-completion`*`dist-to-goal`.
- About the [`POMCPOW`](https://github.com/JuliaPOMDP/POMCPOW.jl) solver:
  - > "It is a **fixed-depth tree search** algorithm that uses **Double Progressive Widening** to explore **continuous `actions` space**, and **observation binning** to constrain the **continuous `observation` space**."
  - This is an **_online_** solver, which may not be the best option for this **simple scenario**:
    - > "Inability to **learn from prior experience** and produce an _offline_ policy which results in **unnecessary computation** in **simple situations**."
- Another idea: use **two rollout policies** for the tree search.
  - > "The **default priors** (e.g. random rollouts) were not sufficient to achieve high rewards."
  - > "We addressed this problem by using **two rollout policies**: `always brake` and `always maintain`, comparing the _expected_ `rewards`, and choosing the `action` produced by the **highest policy**."
- **Findings**. Key ingredients for the method to work:
  - `1-` **Tuning** of the reward function.
  - `2-` Carefully **craft the rollout policy** for the tree search.
  - `3-` **Spreading the rewards** across the world.
  - It all results in a lot of **manual tuning**:
    - > "We find that the agent behavior depends more on the **domain knowledge embedded** into the **policy algorithm** that on the **sensor quality**, but detailed exploration of this effect is left to future work."

</details>

---

**`"Context and Intention Aware Planning for Urban Driving"`**

- **[** `2019` **]**
**[[:memo:](https://ieeexplore.ieee.org/document/8967873)]**
**[[🎞️](https://www.youtube.com/watch?v=psm6juPltJs)]**
**[** :mortar_board: `National University of Singapore`, `MIT` **]**

- **[** _`interaction-aware planning`, `LSTM-prediction`, `DESPOT`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://ieeexplore.ieee.org/document/8967873).](media/2019_meghjani_1.PNG "[Source](https://ieeexplore.ieee.org/document/8967873).")  |
|:--:|
| *Modular and **hierarchical** architecture. The planner is **`road context`- and `intention`-aware**. [Source](https://ieeexplore.ieee.org/document/8967873).* |

| ![[Source](https://www.youtube.com/watch?v=psm6juPltJs).](media/2019_meghjani_1.gif "[Source](https://www.youtube.com/watch?v=psm6juPltJs).")  |
|:--:|
| *Scenario showing the benefit of **integrating `intention` inference** into planning: after **inferring the exo-vehicles’ `intentions`** of driving to the right lane, the ego-car immediately changes to the left lane which **saves travelling time** and **avoids near collisions**. [Source](https://www.youtube.com/watch?v=psm6juPltJs).* |

| ![[Source](https://www.youtube.com/watch?v=psm6juPltJs).](media/2019_meghjani_2.gif "[Source](https://www.youtube.com/watch?v=psm6juPltJs).")  |
|:--:|
| *The authors tried their approach on **real scenarios**, which is quite rare in the field! Contrary to the **simulation**, measurements are not optimal: **inaccurate perception** module fails **assigning** the second and third car to their correct lanes. In addition, the **`turn signal`** of the first car is **ignored**, which could have led to an accident. It would be relevant to **consider this `turn signal`** to **infer `intention`**. A second idea would be to consider **occlusion**. [Source](https://www.youtube.com/watch?v=psm6juPltJs).* |

Authors: Meghjani, M., Luo, Y., Ho, Q. H., Cai, P., Verma, S., Rus, D., & Hsu, D.

- Motivation:
  - Show the benefit, for _prediction_, _planning_ and _decision making_ in **urban driving**, of using:
    - `1-` **Road contextual information**.
      - This reduces the **uncertainties in `intention`** and trajectory **predictions** of exo-vehicles.
      - It also reduces the **computation cost** of the `POMDP` planner for the ego-vehicle: the search is assisted by **pruning invalid actions** and **shaping the rewards**.
    - `2-` **Intention inference** for **`behaviour` and `trajectory` prediction**.
      - The **interaction** with **multiple exo-vehicles** is considered by the planner.
    - `3-` **Long-term planning**.
  - > [About the scenarios] "We argue that the **`lane merging` problem** is **much harder** than the **intersection case**, because exo-vehicles have a lot more freedom. [...] **Long-term interactions** are often required."

- About **intention inference** and **trajectory prediction**.
  - The **context-aware prediction** model **decouples `intention` and `trajectory` predictions**:
    - `1-` The `intention` is predicted by a **neural network** _(mentioned below)_.
    - `2-` The `trajectory` of **predicted intention** is obtained based on **polynomial fitting** and **extrapolating** the real-time vehicle state.
      - The `time-to-collision` model (`TTC`) is used to predict the longitudinal speed. And hence embed **interaction** in the prediction.
  - This [video](https://www.youtube.com/watch?v=e_GjCq0qsSE) describes the context-aware prediction module.
  - This **generative model** is used in the `POMDP` **transition function**, for forward simulations.

- About the **decision making** hierarchical architecture.
  - Input: observed **trajectories** of _exo_-vehicles.
  - Output: **high-level** lateral `action` for _ego_-vehicle.
  - Three components:
    - `1-` Road context database.
      - The context include the `number of lanes`, the `direction` of the lanes, the `lane width`, and the `positions` of the `start` and `end` points of each lane.
    - `2-` **`LSTM` intention predictor**.
      - > "We formalize the **`intention`** as the **high-level action**, i.e., the action of `lane-keeping`, `left-lane-changing`, or `right-lane-changing`."
      - The input includes: `changes` in lateral and longitudinal `pose`, flags indicating if `right lane` and `right lane` exist, as well as `lateral distance` from the centre of the lane.
      - The output of the net is a **`belief`**, i.e., **probability distribution**, over three **`intention` classes**: `lane-keeping`, `right lane-changing`, and `left lane changing`.
        - The use of a **net** contrasts with **particle filters** for **`belief` tracking**, which are prone to **computationally expensive Bayesian belief updates**.
    - `3-` `POMDP` high-level planner.
      - It determines **long-term high-level** `action`s of the ego-vehicle under **uncertainties** of exo-vehicles' **intentions**.
        - > "The **`belief` tree search** takes as input the current `belief` and the `state` of all vehicles and performs **Monte Carlo simulations** for the **uncertain future** to provide the largest rewarding high-level `action`."
      - > [About the solver] "The key idea of **[`DESPOT`](https://papers.nips.cc/paper/5189-despot-online-pomdp-planning-with-regularization)** is to search a **`belief` tree** under **`K` sampled scenarios** only, which greatly reduces computational complexity, making it an efficient solver for our `POMDP` model."

- About the `POMDP` formulation.
  - `state` space:
    - The `intention`: hidden variables.
    - The `road contextual information`.
    - The `pose` (`x`, `y`, `θ`) of each vehicle.
    - A `4`-time-step **history** of the **past `poses`**, representing `1s`.
  - `action` space:
    - {`LANE-KEEP`, `LEFT-LANE-CHANGE`, `RIGHT-LANE-CHANGE`} for the next time-step.
    - > "We further **prune** the **forbidden `actions`** in different lanes with the **help of road contextual information**."
  - `transition` function:
    - The above-mentioned **trajectory predictor** is used to predict its **next-step `pose`**, given its `intention` and the `road contextual information`.
    - A Gaussian noise is added _(no value reported)_.
- About the baselines:
  - `1-` **Reactive** controller.
    - It reacts based on the comparison between `headway distances` with a `distance threshold`.
  - `2-` **Greedy** controller.
    - It chooses to drive in the lane that is **shortest to the destination lane** at each time step **regardless of exo-vehicles**.
  - `3-` **`SimMobilityST`**.
    - A **rule-based** algorithm used in **[`SimMobility`](https://github.com/smart-fm/simmobility-prod) simulator**.
- About the criteria for **performance comparison**:
  - For **safety**: the `collision rate`.
  - For **efficiency**: the `success rate` and `travel time`.
  - For **smoothness**: the number of `lane changes` per 100 meters.
  - A **timeout** is also implemented.
- Limitations:
  - `1-` The **intention prediction** module considers the **exo-vehicles independently**, **ignoring the influence of interactions** between them.
  - `2-` No **feedback loop**: The high-level `action`s provided by the planner are **decoupled from the low-level control**. It is possible that low-level controller **cannot execute the high-level `action`** given by the planner.
    - In particular, the planner suffers from **decision switches** between `lane-change` and `keep-lane`. Instable high-level commands may become struggling for the **low-level controller** to implement.

</details>

---

**`"Belief State Planning for Autonomous Driving: Planning with Interaction, Uncertain Prediction and Uncertain Perception"`**

- **[** `2019` **]**
**[[:memo:](https://publikationen.bibliothek.kit.edu/1000120841)]**
**[** :mortar_board: `KIT`**]**
**[** :car: `BMW` **]**

- **[** _`ABT`, `information gathering`, `occlusion`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://publikationen.bibliothek.kit.edu/1000120841).](media/2019_hubmann_1.PNG "[Source](https://publikationen.bibliothek.kit.edu/1000120841).")  |
|:--:|
| *Various sources of **uncertainty** and how to model the scene when dealing with **occlusion**. The variable `g` is a Boolean, indicating whether there is a car (`g=1`) in the occluded area. It is estimated by the `belief tracker`. `Ψ` denotes the edge of the **field of view** on the path of a **phantom vehicle**. It is part of the `observation` and used for the `transition` model. The idea is to **sample if the phantom vehicle is detected or not**. The probability of this sampling is proportional to the **revealed occluded area** during the **transition** from `state`(`t`) to `state`(`t+1`). I admit I did not fully understand. [Source](https://publikationen.bibliothek.kit.edu/1000120841).* |

| ![[Source](https://publikationen.bibliothek.kit.edu/1000120841).](media/2019_hubmann_2.PNG "[Source](https://publikationen.bibliothek.kit.edu/1000120841).")  |
|:--:|
| *Hierarchy: The **`behaviour` planner** (**`A*`-based** or **`POMDP`-based**) generates an optimal **plan** first, but under different optimization criteria than the **`trajectory` planner**. The result of the `behaviour` planner is a **`policy`, not a trajectory**! Different plans can be retrieved from it, depending on the scene evolution. In a second step, the **most probable trajectory** in the **generated policy of the `POMDP`** is retrieved and optimized by a **local `trajectory` planning** algorithm on a **shorter horizon**. [Source](https://publikationen.bibliothek.kit.edu/1000120841).* |

| ![[Source](https://publikationen.bibliothek.kit.edu/1000120841).](media/2019_hubmann_3.PNG "[Source](https://publikationen.bibliothek.kit.edu/1000120841).")  |
|:--:|
| *The **`rollout` stage** of the `MCTS` uses a **deterministic `A*`** that solves a **simplified (deterministic)** problem, followed by **constant velocity rollouts**. Thus, new **belief nodes** can be quickly initialized with an **estimated `value`** (heuristic). Note that `observations` (measurements) come at `1Hz`. In between, the tree is ''recycled'' and **produces `4` `reference trajectories`**. Right: `observation` clustering. If a **new `observation`** arrives, it is tried to match it on one of the **existing `observation` clusters**. [Source](https://publikationen.bibliothek.kit.edu/1000120841).* |

| ![[Source](https://publikationen.bibliothek.kit.edu/1000120841).](media/2019_hubmann_4.PNG "[Source](https://publikationen.bibliothek.kit.edu/1000120841).")  |
|:--:|
| *Top: `prediction`, `manoeuvre selection` and `trajectory planning` are done separately. It assumes that the **probabilistic future behaviour** of the other agents is **independent** of the (future) ego-decisions, which may work well for **simple scenarios**. Bottom: In `MCTS`, an **external `prediction`** algorithm is not needed as the other agents are **simulated stepwise ahead** as part of a **forward simulation**. One challenge is to define these **`transition` models** though. Bottom-right: considering **_possible_ future `observation`s** (`measurements`) leads to a **closed-loop** policy planner that respects not only the **current `belief` state** but also the **most likely future scenarios**. This enables **less-conservative behaviours**, for instance by **postponing decisions** until more information is collected. [Source](https://publikationen.bibliothek.kit.edu/1000120841).* |

| ![[Source](https://publikationen.bibliothek.kit.edu/1000120841).](media/2019_hubmann_5.PNG "[Source](https://publikationen.bibliothek.kit.edu/1000120841).")  |
|:--:|
| *The __deterministic__ formulation. It is solved with an **`A*` graph search** on a **receding horizon**. Three types of **`Event`** are defined to populate the **`s`-`t` cost map**. [Source](https://publikationen.bibliothek.kit.edu/1000120841).* |

| ![[Source](https://publikationen.bibliothek.kit.edu/1000120841).](media/2019_hubmann_6.PNG "[Source](https://publikationen.bibliothek.kit.edu/1000120841).")  |
|:--:|
| *The __stochastic__ formulation. Here for the `intersection` scenario. The `belief` tracker uses `observations` to **estimate the current `route`** (hidden `intention`) followed by the other vehicles and **maintain `belief`** about the **possible futures**. This leads to **less conservative** behaviours. [Source](https://publikationen.bibliothek.kit.edu/1000120841).* |

| ![[Source](https://github.com/chauvinSimon/IV19).](media/2019_hubmann_1.gif "[Source](https://github.com/chauvinSimon/IV19).")  |
|:--:|
| *The ego car maintains **multiple hypotheses** wrt. the path the other car is following and prepare **multiple plans** (in a `policy`) to react accordingly. The **`observations` are aggregated** over time to estimate the **probability of each path** (`belief tracking`). [Source](https://github.com/chauvinSimon/IV19).* |

Author: Hubmann, C.

- PhD Thesis.

- One idea: combine `MCTS` with a **deterministic `A*` roll-out heuristic** for fast convergence to the optimal policy.

- Motivations:
  - `1-` **Uncertainty**-aware planner.
    - Since the **current `state` is not _fully_ known**, a **`belief` over state** is maintained from `observations`. It is described by `b`(`s`), i.e. **the probability of being in a certain state `s`**.
  - `2-` **Interaction**-aware planner.
    - > "Modeling **intertwined `prediction` and `planning`** allows to consider the **reaction** of other agents to the trajectory of the autonomous car."
  - `3-` **"_Globally_ optimal"**, [complete], **anytime** and **online** planner.
    - > [Real time capability] "This is possible by extending state of the art solvers with domain **specific heuristics** which allows to **focus on promising branches** in an otherwise intractable graph search."
  - `4-` Rely neither on **manually designed logic rules**, nor on **hand-selected `manoeuvres`**.
    - This would become infeasible in **complex urban environments**.
  - `5-` Avoid **over-conservatism** of rule-based planners.
    - The idea is to consider **_possible_ future `observation`s** (`measurements`) during planning in this **_sequential_** decision-making problem.
    - This leads to a **`closed-loop`** planner which generates a `policy` over an **uncertain `belief` space**, offering:
      - The ability to **postpone (conservative) decisions**, waiting for more information to be gathered.
        - The policy contains **reactive plans** for possible **future `observations`**, i.e. measurements of the **uncertain behaviour** of the other agents.
      - The ability to take `action`s in order to **reduce uncertainty**, i.e. **active `information gathering`**.
        - E.g. **step laterally** off the predefined path to **increase the `field of view`** in **occluded scenarios** (`lateral exploration`).
    - > "By considering **_possible_ future `observations` explicitly**, the algorithm is able to predict in what ways the **current `belief` state** may change in the future. This enables the **postponing of decisions**, such as merging _before_ or _after_ another vehicle, as the algorithm is able to predict that future `observations` will lead to a **less uncertain prediction**."
    - > "This behavior (also known as `information gathering`) is the result of the policy because the **`observation` model has simulated**, that the next measurements will lead to a **less uncertain `belief` state**. Because of the `observation` model, it can even infer **at what point in time the `belief` becomes less uncertain** and approach the intersection accordingly."

- About **`manoeuvres`**.
  - > "While a precise **definition** of a **`maneuver`** does not exist, it is often compared to the mathematical concept of **`homotopies`**. Two continuous trajectories are in the **same `homotopy` class** if a continuous, collision-free projection exists that transforms one trajectory to the other one. For example, two different trajectories, **one overtaking an obstacle on the right side** and one on the **left side** lie in **different `homotopies`**."
  - Many works **define beforehand a set of `manoeuvres`**. For each, **trajectories** are planned and then **evaluated** with a cost function.
    - The author wants to avoid this **_a priori_ enumeration of `manoeuvres`** which **constrains the solution space** and may prevent **global and optimal** behaviours.
    - > "While the approach of only considering a **limited amount of predefined `maneuvers`** is feasible on highways, this may become intractable in **urban environments** due to the high amount of varying topological situations and corresponding maneuver possibilities."

- _What types of_ **_uncertainty_**_?_
  - Uncertain **perception**:
    - `1-` Noisy **sensor measurements**
    - `2-` **Occlusions**
  - Uncertain **prediction**:
    - `3-` Unknown **intentions** of other drivers
    - `4-` Unknown **driver models** for other drivers, including interaction capabilities

- _How to plan under uncertainty?_
  - > "The problem of optimally considering the uncertainty of future `states` can be addressed by planning in the **space of `policies`** instead of in the **space of `trajectories`**."
  - A `policy` is generated over a **`belief` state** and optimizes the expected `reward`, starting from an initial `belief` state.

- About `open-loop`/`close-loop` planners.
  - `1-` **`open-loop` motion planning** algorithms **do not consider future (possible) `measurements`** which arrive during the execution of the planned motion.
    - > "Respecting every **possible prediction** leads to safe but potentially **conservative behavior**. Another possibility is to consider only the most likely prediction(s)."
    - > "The **`open-loop`** planner has to **slow down immediately** as it is not able to incorporate **future `observations`** in the `planning` phase. This means, that the planner **reacts to both possible future situations** simultaneously."
  - `2-` **`close loop` motion planning** on the other hand allows to **consider the possible future observations** in the planning stage.
    - > "The policy **contains `2` plans** about **how to react** to the `observation` which arrives at `t=1`."
    - > "On the contrary, the `POMDP` planner is able to **reason about both possible scenarios**. This results in a policy that **postpones the decision** of `crossing` vs. `braking` to a **future point in time** when more observations have been recorded."

- _How to make_ **_interaction-aware_** _planning?_ By **not separating `planning` and `prediction`**.
  - > "A common approach is to **separate `prediction` and `planning`**. In this case, all the trajectories of the other agents are predicted first. Given the predicted trajectories, a **maneuver** is selected for the autonomous car (by the **behavior layer**) and a correspondent trajectory is planned by the **`trajectory` planner**."
  - A **separation** would assume that the **probabilistic future behaviour** of the other agents is **independent** of the (future) ego-decisions.
    - It would work for simple scenarios (e.g. highway).
    - But not for urban situations where **interaction** must be explicitly considered.
  - _How to deal with the uncertainty in_ **_prediction_**_, when_ **_separation_** _is used?_
    - **Frequent replanning** is one option. Especially when the prediction is **unimodal**.
  - In `MCTS`, an **external prediction** algorithm is not needed as the other agents are **simulated stepwise ahead** as part of a **forward simulation**. One challenge is to define these **`transition` models**, usually one for each of the different possible manoeuvres. For instance `IDM`.

- Making the problem simpler: the **_"path-velocity decomposition"_ assumption**.
  - It is made for most of scenarios.
  - Except for the strongly coupled problems **`lane change`** and `occlusions` which require **combined _longitudinal_ and _lateral_ optimization**.

- _What if the_ **_`transition` model is deterministic_** _and the_ **_`state` fully observable_**_?_
  - Then the planning problem is simpler and can be represented as a **graph**:
    - `vertices` = `state`s
    - `edges` = `action`s
  - An **`A*` graph search on a receding horizon** derives the **longitudinal `acceleration`**.
    - The path to follow is given by another module, based on **variational methods**, i.e. local convex optimization around a reference path.
  - The `A*` algorithm uses a **heuristic** to speed up the graph search by **truncating non-promising branches early**.
    - Here the heuristic is based on the idea of **[Inevitable Collision States](https://www.researchgate.net/publication/4046235_Inevitable_Collision_States_A_Step_Towards_Safer_Robots) (`ICS`)**.
    - > "When a new `state` is generated, it is tested for **being an `ICS`**. If this is the case, the remaining **estimated costs** are at least the **collision costs**."
  - The **traffic rules** and the **predicted behaviours** of other agents are represented into a **spatio-temporal cost map**.
    - Only the **most likely prediction** is considered for each agent.
    - The uncertainty in the (**unimodal**) prediction is addressed by **frequent replanning (`10Hz`)**.
  - Information of **possible future `observations`** is ignored. Hence **open-loop**.

- About the **`cost` definition** for the `A*` search.
  - Three types of **events** are defined:
    - `1-` **`static`**. It prohibits the ego vehicle to **traverse a certain `position` on its path at a certain position** during a **`time` interval**.
    - `2-` **`dynamic`**. The prohibited `position` is **`time` dependent**. For instance leading car.
      - Also traffic lights, otherwise the `green` and `red` phases are assumed to last forever.
      - > "During a `yellow` phase, the **legal length of the `yellow` phase** is used to **predict the traffic signal switch**, such that `t-start` is the **predicted start of the `red` phase** and `t-end` is set to infinity. That way, the algorithm implicitly handles the decision **to pass or not to pass** a (recently switched) traffic light."
    - `3-` **`area`**. It prohibits the ego vehicle to enter certain areas when they **cannot be left again**.

- _How to penalize deviation from the_ **`desired speed`** _in the `reward`?_
  - The `desired speed` depends on the **road’s `curvature`** and the **current legal `speed limit`**.
  - Too **high** speeds are punished **_quadratically_**.
  - Too **low** speeds are punished **_linearly_**.
    - This allows lower velocities during **decelerating upon `events`** such as _red traffic lights_.
    - > "Punishing lower velocities in a **linear way** motivates the planner to drive with the desired velocity but allows for **slower solutions** (e.g. because of a **temporarily occupied lane**)."

- _How to ensure_ **_consistent decision_** _over time and prohibit jumps in behaviours?_
  - The **current _desired_ `state`** on the **_reference_ trajectory** is considered during replanning, instead of the **_measured_ `state`**.
  - > "Instead of planning from the actual, **measured state** `x-meas(t0)` the **currently _desired_ state** `x-des(t0)`, retrieved from the **previous planning step**, is used as the **start state `x-start`**."

- _What if the_ **_`transition` model is stochastic_** _and the_ **_`state` partially observable_**_?_
  - Modelling with `POMDP`
  - Challenges: **curse of history** and **curse of dimensionality**.
    - > "One of the main difficulties in solving `POMDPs` is the **continuous `belief` state**."
    - The `observation` space is continuous. But **observation clustering** is performed to construct the tree.
    - > During the simulation of the `belief` tree, the `observations` which are following a certain `action` must be **clustered into a discrete number of possible `observations`**. This is the case as the **structure of a tree** can only be generated when a **discrete number of `observations`** exist.
    - If a **new `observation`** arrives, it is tried to match it on one of the **existing `observation` clusters**.

- `POMDP` formulations (multiple since different scenarios are considered ).
  - `action`.
    - A **discrete** set. Describing `longitudinal acceleration`.
  - `state`.
    - `longitudinal position` and `speed` on a path.
    - **Hidden parts** which cannot be measured directly but only **inferred over time** can be, depending on the **scenario**:
      - `1-` [`Crossing`] The `path` followed by the other vehicle, i.e. its `intention`.
      - `2-` [`Merging`] **`Willingness`** for yielding.
        - > "The variable `m` is used to describe the **_friendliness_** of the other driver i.e. if he will **react by yielding** (`m=1`) to a merge attempt or not (`m=0`)."
      - `3-` [`Occlusion`] A Boolean, indicating whether there is **a car in the occluded area**. _But no information about its `position`?_
  - `observation`.
    - `position` as (`x, y`) instead of (`s`, [`lat-offset`], `path`) since `path` is not observable.
    - _no heading? this could be useful to estimate the path it follows_
    - [`Occlusion`] The `FoV` on every lane.

  - `observation` model.
    - Noise is added to the true `position` and `speed`.
    - [`Occlusion`] For every **potential phantom** vehicle, an observation is also generated. _Ok, but what `position` is given?_

  - `transition` model.
    - To convert `acceleration` to `position`: simple physical models (**point-mass object** with `1st` order integration).
    - To derive the `action` of other agents. Depending on the scenario:
      - `IDM` is used to compute the `longitudinal acceleration`.
        - Potentially extended with **interaction-based** term, when **paths intersect** in the near future.
        - > "The `acceleration` of the other vehicles is additionally perturbed by use of **Gaussian noise** to represent the **model uncertainty**."
      - A **learnt classifier** predicts the **probability of `yielding`** to a possible merge attempt / `ignoring` and simply following the existing front vehicles.
  - `reward`
    - Considering `collision`, `comfort` and `desired speed`.

- _How to represent_ **_phantom vehicles_** _[`occlusion`]?_
  - The idea is to represent all the possible vehicle configurations in the occluded area by **one reachable set**:
    - > "As it is infeasible to describe **all possible vehicle configurations in occlusions** by particular `states`, the idea is to describe all possible configurations on a occluded lane by **one `set`**."
    - For each **occluded lane**, a **`phantom` vehicle** is placed at the start of the **field of view `FoV`**.
    - It has **infinite length** and drives faster than the **speed limit** (i.e. worst-case).
  - > "If every possible, occluded vehicle configuration would be represented explicitly, a certain **subset of these configurations** would be **discovered, when the `FoV` is expanded**."
  - > "Nonetheless, in this work, the idea is to represent **all these configurations** by **one reachable set**. Instead of splitting the set into many discretized subsets, the idea is to **sample if the `phantom` vehicle is detected or not**. The probability of this sampling is **proportional to the revealed occluded** area during the **transition from `state`(`t`) to `state`(`t+1`)**." [_not very clear to me_]

- _How the_ **_`belief` state_** _can be_ **_estimated over time_**_?
  - By **recursive Bayesian estimation**.
  - [`Intersection` scenario] A Bayes classifier with `2`-dimensional **feature** (`position` and `speed`) computes the **probability of each vehicle being on a certain route `r`**.
    - The **likelihood** term includes `Euclidean distances` to the considered path.
  - _How is it done for the other two scenarios? I did not understand._

- _How `POMDP`s can be solved?_
  - `1-` Offline.
    - > "The `value` function of a `POMDP` is always **piece-wise linear and convex** in the `belief`."
    - The optimal value function over the continuous `belief` is represented by a set of **alpha-vectors**.
    - **Point-based solvers**, such as `SARSOP`, `PBVI`, `HSVI`.
      - Why _"point-"_?
      - > "The idea of **point-based** algorithms is to overcome that problem by **backing up the `value` function** only for a **discrete set of chosen `belief-points`**."
  - `2-` Online: with an online **graph/tree search**.
    - Traditional graph-searches are not appropriate due to the **non-deterministic** nature of the `transition` model.
    - **Monte Carlo Tree Search** (`MCTS`) is one option: it combines a **deterministic tree search** with **random sampling**.
    - For instance, **`Adaptive Belief Tree`** ([`ABT`](https://link.springer.com/chapter/10.1007/978-3-319-28872-7_35)) uses `MCTS`, optimized for `POMDP`s, where the search is done in a **`belief` tree**.
      - > "The combination of a **smart selection method** (e.g. the `UCT` algorithm) and the **fast estimation of future `rewards`** by a sufficiently good **default `policy`** allows to reduce the **search space** drastically, which gives the algorithm its **online capabilities**."
      - Another characteristic of `ABT`: reuse previous parts of the tree.
    - Here the `ABT` implementation **["Toolkit for Approximating and Adapting POMDP Solutions in Real Time"](https://robotics.itee.uq.edu.au/~hannakur/dokuwiki/papers/acra14_tapir.pdf) (`TAPIR`)** ([[:octocat:]](https://github.com/rdl-algorithm/tapir)) is used to solve the `POMDP` online.

- _How to make the_ **_search in the `belief` tree_** _faster?_
  - By **initializing the `value`** of **new `belief` nodes** with **_optimized_ roll-outs**.
    - A **heuristic** is used to estimate the `value` function of the nodes.
    - > "The **heuristic value** is calculated by solving a **deterministic, simplified problem online** as soon as a **new `belief` state** is encountered."
    - Here using the previously developed `A*` graph-search.
  - More precisely:
    - `1-` Optimization of the **simplified (`deterministic`) problem** is used for the **first three steps** ...
    - `2-` ... Followed by a **constant velocity roll-out** until the optimization horizon.

- _What_ **_decision frequency_** _and_ **_tree depth_**_, i.e. `planning horizon`?_
  - > [On the one hand ...] "The more time is used for **sampling** of episodes, the better the policy is approximated."
  - > [On the other hand ...] "The earlier the `policy` is sent to the trajectory layer, the **less delay** is introduced between the sensor measurements and the corresponding policy."
  - Here, the algorithm returns a decision every `200ms` (`5Hz`).
  - > "While **the solution is optimized for `200ms`**, a **step size of `∆t=1s`** is used to **construct the tree** to allow for a **planning horizon of `8-10s`**." [_i.e. `depth`~`8-10`_]

- Future works:
  - `1-` Constrained `POMDPs` for **safety guarantees**.
  - `2-` Combining `learning` and `planning`: The **heuristic** that **guides the online search** could be **learnt offline**.

</details>

---

**`"SUMMIT: A Simulator for Urban Driving in Massive Mixed Traffic"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1911.04074)]**
**[[:octocat:](https://github.com/AdaCompNUS/summit)]**
**[[:octocat:](https://adacompnus.github.io/summit/)]**
**[[🎞️](https://www.youtube.com/watch?v=dNiR0z2dROg)]**
**[** :mortar_board: `National University of Singapore` **]**

- **[** _`simulator`, `dense traffic`, `Hyp-DESPOT`, [`SUMO`](https://sumo.dlr.de/docs/index.html), [`CARLA`](http://carla.org/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1911.04074).](media/2019_cai_2.PNG "[Source](https://arxiv.org/abs/1911.04074).")  |
|:--:|
| *`SUMMIT` has been developed to simulate realistic **dense**, **unregulated** urban traffic for **heterogeneous agents** at any **worldwide locations**. [Source](https://arxiv.org/abs/1911.04074).* |

| ![[Source](https://www.youtube.com/watch?v=dNiR0z2dROg).](media/2019_cai_1.gif "[Source](https://www.youtube.com/watch?v=dNiR0z2dROg).")  |
|:--:|
| *[Source](https://www.youtube.com/watch?v=dNiR0z2dROg).* |

| ![[Source](https://arxiv.org/abs/1911.04074).](media/2019_cai_1.PNG "[Source](https://arxiv.org/abs/1911.04074).")  |
|:--:|
| *The motion model used is `Context-GAMMA` that applies **`velocity`-space optimization** under **kinematic** (e.g. non-holonomic motion of car), **geometric** (collision avoidance with nearby agents) and context-related constraints to generate sophisticated, unregulated **crowd behaviors**. [Source](https://arxiv.org/abs/1911.04074).* |

| ![[Source](https://www.youtube.com/watch?v=dNiR0z2dROg).](media/2019_cai_2.gif "[Source](https://www.youtube.com/watch?v=dNiR0z2dROg).")  |
|:--:|
| *[Source](https://www.youtube.com/watch?v=dNiR0z2dROg).* |

Authors: Cai P., Lee Y., Luo Y., & Hsu D.

- > "Driving in **unregulated**, **crowded** urban environments, like in **uncontrolled roads** or **unsignalized intersections** in less-developed countries remains an open problem."
- Motivation for a new simulator:
  - Simulates realistic **dense**, **unregulated** urban traffic for **heterogeneous agents** at any **worldwide locations**.
  - Driving simulators already exist. But here the idea is:
    - To **scale up** from **complex interactions** between agents to **crowded urban** scenes.
    - Not to be restricted to **predefined maps**.
    - To simulate **large crowds**, but with the quality of **interactions**.
    - To closely represent **real-world** scenarios and generate **high-fidelity** interactive data.
- _How to work on any worldwide locations?_
  - With **`OpenStreetMaps`**.
  - `SUMMIT` relies on **[`SUMO`](https://sumo.dlr.de/docs/index.html)** to automatically convert **`OSM` maps** to **lane networks**.
  - > "`SUMMIT` fetches real-world maps from the `OpenStreetMap`, and constructs **two topological graphs**: a **`lane network`** for vehicles, and a `sidewalk network` for pedestrians."
- _How to generate complex and realistic crowd interactions? What crowd behaviour model?_
  - The authors present a model: **`Context-GAMMA`**, built on top from **[`GAMMA`](https://arxiv.org/abs/1906.01566)** (Luo & Cai, 2019) ([video](https://www.youtube.com/watch?v=5xAB0-8XceQ)), a "General **Agent Motion Prediction** Model".
  - It uses **`velocity`-space optimization** to generate sophisticated, unregulated **crowd behaviours**.
  - > "`GAMMA` assumes that each traffic agent **optimizes its velocity** based on the **navigation goal**, while being **constrained** by **kinematic constraints** (e.g. non-holonomic motion of car) and **geometric constraints** (collision avoidance with nearby agents)."
  - `Context-GAMMA` introduce a **third constraint** and another objective about the **road context**.
    - > "We suggest that algorithms should leverage the **road contexts** to help **long-term planning**."

- _How to generate realistic data?_
  - > "`SUMMIT` is based on `CARLA` to benefits from its desirable features such as **high-fidelity physics**, **realistic rendering**, weather control, and rich sensors".

- _What decision-making module?_
  - A **context-aware `POMDP`** is proposed and compared to **`TTC`**-based **reactive** system. Two components:
  - `1-` The **belief tracker** for the **hidden `state` variables**.
    - > "At each time step, it uses a **motion model** to compute the likelihood of transitions and observations, and **updates the posterior belief** using the Bayes rule."
  - `2-` The **online solver** **[`hyp-DESPOT`](https://arxiv.org/abs/1802.06215)** that takes the **current belief** and computes the **optimal driving action**.
  - About the `state` space:
    - **_observable_ ego** (**continuous**): `position`, `speed`, and `heading`.
    - **_observable_ other** (**discrete**): `position`, `speed`
      - _No information about the_ **_discretisation_**.
    - **_hidden_ other**:
      - `type`: An agent can be either **`distracted`**, thus **not interacting** with the ego-vehicle, or be **`attentive`**, thus cooperatively **avoid collision** with the ego-vehicle.
      - `intention` wrt. a set of path candidates.
    - > "We assume that the ego-vehicle can observe its **own `state`** and **discretized values** of the **observable `state`s** of **exo-agents**. The **hidden `state`s** of exo-agents can only be **inferred and modelled with `belief`s**."
  - About the `action` space:
    - **Path-velocity decomposition**:
      - > "We restrict the `POMDP` to compute the **acceleration along the intended path**, while the steering angle is generated using a **pure-pursuit algorithm**."
    - {`ACC`, `MAINTAIN`, `DEC`}.
  - About the **`transition`** model = **motion model**:
    - > "`distracted` traffic agents are assumed to track their intended path with the **current speed**".
    - > "`attentive` traffic agents also tend to follow the sampled path, but use [`PORCA`](https://arxiv.org/abs/1805.11833), an **interactive collision avoidance model** that is similar to [`GAMMA`](https://arxiv.org/abs/1906.01566) but considerably **simpler**, to generate the actual **local motion**".
    - To model **stochastic transitions** of exo-agents, their motion is perturbed by **Gaussian noises on the displacement**.
  - About the `reward` model:
    - Multi-objective: `safety`, `efficiency`, and `smoothness` of driving.
- Baseline and evaluation:
  - > "We compare `Context-GAMMA` with a **_reactive_ model** that moves agents along lane center-curves and uses **time-to-collision** (`TTC`) to calculate the vehicle’s speed."
  - Criteria:
    - `average speed` of traffic agents.
    - `congestion factor` defined as the percentage of agents being jammed in the crowd. (Jammed agents are removed from the crowd after being stationary for `5s`.)
  - Findings:
    - The **congestion factor** of the `TTC`-controlled traffic **grows quickly** with the simulation time, indicating that **agents fail to coordinate** with each other.
    - > "We thus conclude that **sophisticated long-term planning** is important for driving in **unregulated traffic**, and `Context-POMDP` establishes a reference for future **crowd-driving** algorithms."

</details>

---

**`"Crossing of Road Intersections : Decision-Making Under Uncertainty for Autonomous Vehicles"`**

- **[** `2019` **]**
**[[:memo:](https://hal.inria.fr/tel-02424655/document)]**
**[** :mortar_board: `INRIA` **]**
**[** :car: `Renault` **]**

- **[** _`POMCP`, `interaction-aware`, [`SCANeR`](https://www.avsimulation.fr/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![The author calls for a **probabilistic** framework to reason and make decision, due to the inherent `perception` **uncertainty** and `behaviour` (`interaction`) **uncertainty**. Also, learning-based methods are avoided due to their susceptibility to **over-fit** if the dataset is not balanced. [Source](https://hal.inria.fr/tel-02424655/document).](media/2019_barbier_3.PNG "The author calls for a **probabilistic** framework to reason and make decision, due to the inherent `perception` **uncertainty** and `behaviour` (`interaction`) **uncertainty**. Also, learning-based methods are avoided due to their susceptibility to **over-fit** if the dataset is not balanced. [Source](https://hal.inria.fr/tel-02424655/document).")  |
|:--:|
| *The author calls for a **probabilistic** framework to reason and make decision, due to the inherent `perception` **uncertainty** and `behaviour` (`interaction`) **uncertainty**. Also, learning-based methods are avoided due to their susceptibility to **over-fit** if the dataset is not balanced. [Source](https://hal.inria.fr/tel-02424655/document).* |

| ![The author prefers **probabilistic methods**, in order to deal with **uncertainties** while trying to offer some **interpretability**. The navigation module outputs a **single action** to be implemented. Another option would have been to deliver some `policy` which could be followed for several steps, limiting inconsistent transitions (especially for **comfort**) and favouring **long-horizon reasoning**. [Source](https://hal.inria.fr/tel-02424655/document).](media/2019_barbier_25.PNG "The author prefers **probabilistic methods**, in order to deal with **uncertainties** while trying to offer some **interpretability**. The navigation module outputs a **single action** to be implemented. Another option would have been to deliver some `policy` which could be followed for several steps, limiting inconsistent transitions (especially for **comfort**) and favouring **long-horizon reasoning**. [Source](https://hal.inria.fr/tel-02424655/document).")  |
|:--:|
| *The author prefers **probabilistic methods**, in order to deal with **uncertainties** while trying to offer some **interpretability**. The navigation module outputs a **single action** to be implemented. Another option would have been to deliver some `policy` which could be followed for several steps, limiting inconsistent transitions (especially for **comfort**) and favouring **long-horizon reasoning**. [Source](https://hal.inria.fr/tel-02424655/document).* |

| ![The **intended manoeuvre** is **inferred** based on observed `speed`, `acceleration` and heading - no `position` term - and will be treated as an **observation** in the `POMDP`. [Source](https://hal.inria.fr/tel-02424655/document).](media/2019_barbier_13.PNG "The **intended manoeuvre** is **inferred** based on observed `speed`, `acceleration` and heading - no `position` term - and will be treated as an **observation** in the `POMDP`. [Source](https://hal.inria.fr/tel-02424655/document).")  |
|:--:|
| *The **intended manoeuvre** is **inferred** based on observed `speed`, `acceleration` and heading - no `position` term - and will be treated as an **observation** in the `POMDP`. [Source](https://hal.inria.fr/tel-02424655/document).* |

| ![As noted in my [report of `IV19`](https://github.com/chauvinSimon/IV19), **risk assessment** can be performed by comparing the **expectated behaviour** (`expectation`) to the **inferred behaviour** (`intention`), i.e. what **should be done** in the situation and what is **actually observed**. A discrepancy can detect some **misinterpretation** of the scene. [Source](https://hal.inria.fr/tel-02424655/document).](media/2019_barbier_6.PNG "As noted in my [report of `IV19`](https://github.com/chauvinSimon/IV19), **risk assessment** can be performed by comparing the **expectated behaviour** (`expectation`) to the **inferred behaviour** (`intention`), i.e. what **should be done** in the situation and what is **actually observed**. A discrepancy can detect some **misinterpretation** of the scene. [Source](https://hal.inria.fr/tel-02424655/document).")  |
|:--:|
| *As noted in my [report of `IV19`](https://github.com/chauvinSimon/IV19), **risk assessment** can be performed by comparing the **expectated behaviour** (`expectation`) to the **inferred behaviour** (`intention`), i.e. what **should be done** in the situation and what is **actually observed**. A discrepancy can detect some **misinterpretation** of the scene. [Source](https://hal.inria.fr/tel-02424655/document).* |

| ![The problem is formulated as a `POMDP`. [Source](https://hal.inria.fr/tel-02424655/document).](media/2019_barbier_22.PNG "The problem is formulated as a `POMDP`. [Source](https://hal.inria.fr/tel-02424655/document).")  |
|:--:|
| *The problem is formulated as a `POMDP`. [Source](https://hal.inria.fr/tel-02424655/document).* |

| ![Decomposition of the probabilistic **transition** function. Only the longitudinal control via discrete `acceleration` is considered. The state `x` consists of **physical** and **behavioural** parts. In particular, it includes the `behaviour expectation` for each vehicle, i.e. what **should be done** according to the traffic rules. It also come with a `behavioural intention` for which is the **inferred manoeuvre** followed by the **observed vehicle**. `intention continuation` is used to describe the transition about `intention`, while [`gap acceptance model`](https://www.sciencedirect.com/science/article/pii/S0191261501000248) are used for the transition about expected behaviour. Finally, note that the selected `acceleration` action only influences the **physical term** of the ego vehicle. [Source](https://hal.inria.fr/tel-02424655/document).](media/2019_barbier_23.PNG "Decomposition of the probabilistic **transition** function. Only the longitudinal control via discrete `acceleration` is considered. The state `x` consists of **physical** and **behavioural** parts. In particular, it includes the `behaviour expectation` for each vehicle, i.e. what **should be done** according to the traffic rules. It also come with a `behavioural intention` for which is the **inferred manoeuvre** followed by the **observed vehicle**. `intention continuation` is used to describe the transition about `intention`, while [`gap acceptance model`](https://www.sciencedirect.com/science/article/pii/S0191261501000248) are used for the transition about expected behaviour. Finally, note that the selected `acceleration` action only influences the **physical term** of the ego vehicle. [Source](https://hal.inria.fr/tel-02424655/document).")  |
|:--:|
| *Decomposition of the probabilistic **transition** function. Only the longitudinal control via discrete `acceleration` is considered. The state `x` consists of **physical** and **behavioural** parts. In particular, it includes the `behaviour expectation` for each vehicle, i.e. what **should be done** according to the traffic rules. It also come with a `behavioural intention` for which is the **inferred manoeuvre** followed by the **observed vehicle**. `intention continuation` is used to describe the transition about `intention`, while [`gap acceptance model`](https://www.sciencedirect.com/science/article/pii/S0191261501000248) are used for the transition about expected behaviour. Finally, note that the selected `acceleration` action only influences the **physical term** of the ego vehicle. [Source](https://hal.inria.fr/tel-02424655/document).* |

| ![One contribution is called `Functional Discretisation`. So-called `motion patterns` are stored within an HD-map as **polygon delimiting the intersection entrance and crossing zones**. This discrete `crossing` and `merging` zones are not manually defined but learnt based on simulated vehicle trajectories. The continuous intersection space is therefore divided into non-uniform discrete areas. Top-right: three **crossing scenarios** are considered, with different **pairs of priorities**. [Source](https://hal.inria.fr/tel-02424655/document).](media/2019_barbier_24.PNG "One contribution is called `Functional Discretisation`. So-called `motion patterns` are stored within an HD-map as **polygon delimiting the intersection entrance and crossing zones**. This discrete `crossing` and `merging` zones are not manually defined but learnt based on simulated vehicle trajectories. The continuous intersection space is therefore divided into non-uniform discrete areas. Top-right: three **crossing scenarios** are considered, with different **pairs of priorities**. [Source](https://hal.inria.fr/tel-02424655/document).")  |
|:--:|
| *One contribution is called `Functional Discretisation`. So-called `motion patterns` are stored within an HD-map as **polygon delimiting the intersection entrance and crossing zones**. This discrete `crossing` and `merging` zones are not manually defined but learnt based on simulated vehicle trajectories. The continuous intersection space is therefore divided into non-uniform discrete areas. Top-right: three **crossing scenarios** are considered, with different **pairs of priorities**. [Source](https://hal.inria.fr/tel-02424655/document).* |

| ![The **`trust` KPI** is based the `time gap`, i.e. the delta in **predicted time** of when each vehicle will **reach the crossing point**. This should be **''maintained''** over `4s` over all the approach. Hence the use of **''temporal''** logic. The **`unsafe stop` KPI** states that the vehicle should never be stand still within the unsafe area. [Source](https://hal.inria.fr/tel-02424655/document).](media/2019_barbier_21.PNG "The **`trust` KPI** is based the `time gap`, i.e. the delta in **predicted time** of when each vehicle will **reach the crossing point**. This should be **''maintained''** over `4s` over all the approach. Hence the use of **''temporal''** logic. The **`unsafe stop` KPI** states that the vehicle should never be stand still within the unsafe area. [Source](https://hal.inria.fr/tel-02424655/document).")  |
|:--:|
| *The **`trust` KPI** is based the `time gap`, i.e. the delta in **predicted time** of when each vehicle will **reach the crossing point**. This should be **''maintained''** over `4s` over all the approach. Hence the use of **''temporal''** logic. The **`unsafe stop` KPI** states that the vehicle should never be stand still within the unsafe area. [Source](https://hal.inria.fr/tel-02424655/document).* |

Author: Barbier M.

- _What?_
  - A **PhD thesis**.
- Motivation:
  - **Interaction-aware** and **uncertainty-aware** handling of a _signed_ intersection.
- _How to capture and deal with_ **_interaction_** _in the decision?_
  - The **intended manoeuvre** is **inferred** (`behavioural classification`) and subsequently treated as an **`observation`**.
  - By comparing it with the **expected manoeuvre**, the agent should determine **how to interact** with the other vehicle.
- About the **behavioural classification**.
  - `lateral` part: {`Turn right`, `Turn left`, `Go straight`}.
  - `longitudinal` part: {`Stop`, `Yield`, `Cross`}.
  - Six **features** are used:
    - `max` and `min` **speed**.
    - `max` and `min` **acceleration**
    - Maximum `right` and `left` **deviation** from the mean heading angle.
  - _I would be afraid that maximum and minimum values could_ **_come from outliers_** _and would rather have worked with quantiles (e.g. `10%` and `90%`)._
- About **risk assessment**:
  - > "The `intended` manoeuvre represents what the driver **is doing**, whereas the `expected` manoeuvre represents what the **situation requires**."
  - One idea to compute the **difference** between what the **other driver `IS DOING`** (called `intention`) and what **one driver `SHOULD DO`** (called `expectation`)
    - The `expected` behaviour can be derived e.g. according to **priority rules** or to [gap acceptance models](https://www.sciencedirect.com/science/article/pii/S0191261501000248).
  - This discrepancy is useful for **risk assessment** since it can detect some **_misinterpretation_** of the scene:
    - Situations where `intention` and `expectation` do not match could result in a **risky interaction**.
  - By penalizing states with a large difference, the **reward function** incorporates feedbacks about **interaction** and encourages the agent to select actions that **reduce this risk**.
- About the **(large) search horizon**:
  - > "The configuration with `γ = 0.85` and `C = 30` is chosen as it meets these characteristics. The discount value results in a search horizon of **`12` seconds**".
- About the **online `POMDP` solver**:
  - [`POMCP`](https://papers.nips.cc/paper/4031-monte-carlo-planning-in-large-pomdps.pdf).
  - **`action continuation`** is used as **rollout policy**.
  - > "[One could] include **imitation learning** to initiate `V`(`ha`) with **knowledge** obtained by looking at **human drivers**."
  - > "A **memory** could be used to initialize the value function from previous exploration, **accelerating the search** for the optimal policy."
- _How to_ **_evaluate_** _the decision-making framework?_
  - The author argues that evaluation should **be decorrelated from the reward crafting**, hence having **separated `KPI`s**:
    - The reason is that systems that used their **performances indicators** in their **value estimation** are likely to **over-fit**.
    - > "[Goodhart's law](https://en.wikipedia.org/wiki/Goodhart%27s_law) stating that **'when a metric is used as a target, it ceases to be a good metric'**"
  - Another idea to avoid **reward hacking**: the reward function is designed with multiple objectives: trade-off between `performances`, `risks` and `interactions`.
- _How to decide the threshold in `KPI`s?_
  - **Statistical Model Checking** is applied to vary the **_bound_** of **KPIs**.
    - **Bounded Linear Temporal Logic** (`BLTL`) allows to **state conditions** that will **_eventually_** be true.
  - The author works for instance with the `probability of crossing the intersection in less than a given time`.
- About [`ENABLE-S3`](https://www.enable-s3.eu/domains/automotive/):
  - The author uses the **validation and verification** architecture of this European project.
  - > From [`ENABLE-S3` website](https://www.enable-s3.eu/about-project/): "**Pure simulation** cannot cover physics in detail due to its limitations in modelling and computation. **Real-world tests** are too expensive, too time consuming and potentially dangerous. Thus, `ENABLE-S3` aims at developing an innovative solution capable of combining both worlds in an optimized manner [...] and facilitate the market introduction of automated systems in **Europe**."

</details>

---

**`"DESPOT-α: Online POMDP Planning With Large State And Observation Spaces"`**

- **[** `2019` **]**
**[[:memo:](http://www.roboticsproceedings.org/rss15/p06.pdf)]**
**[** :mortar_board: `National University Of Singapore` **]**

- **[** _`POMDP`, `online solver`, `DESPOT`, `parallelization`, `large observation space`_ **]**

<details>
  <summary>Click to expand</summary>

| ![Unlike standard **belief tree**, some **`observation` branches** are removed in a `DESPOT`. [Source](https://arxiv.org/abs/1609.03250).](media/2017_ye_1.PNG "Unlike standard **belief tree**, some **`observation` branches** are removed in a `DESPOT`. [Source](https://arxiv.org/abs/1609.03250).")  |
|:--:|
| *Unlike standard **belief tree**, some **`observation` branches** are removed in a `DESPOT`. [Source](https://arxiv.org/abs/1609.03250).* |

| ![Top - Illustration of the **`particle divergence` problem**: When **`observation` space** is large, particles quickly diverge into **separate belief nodes** in the belief tree, each of which contains only a **single particle**. This causes **over-optimistic** behaviours. Bottom - In a `DESPOT-α`, each node has the **same number of particles as the root** of the tree and weighting is performed based on the `observation`s. This **prevents the over-optimistic evaluation** of **value** of the belief. [Source](http://www.roboticsproceedings.org/rss15/p06.pdf).](media/2019_garg_1.PNG "Top - Illustration of the **`particle divergence` problem**: When **`observation` space** is large, particles quickly diverge into **separate belief nodes** in the belief tree, each of which contains only a **single particle**. This causes **over-optimistic** behaviours. Bottom - In a `DESPOT-α`, each node has the **same number of particles as the root** of the tree and weighting is performed based on the `observation`s. This **prevents the over-optimistic evaluation** of **value** of the belief. [Source](http://www.roboticsproceedings.org/rss15/p06.pdf).")  |
|:--:|
| *Top - Illustration of the **`particle divergence` problem**: When **`observation` space** is large, particles quickly diverge into **separate `belief` nodes** in the belief tree, each of which contains only a **single particle**. This causes **over-optimistic** behaviours. Bottom - In a `DESPOT-α`, each node has the **same number of particles as the root** of the tree and **weighting** is performed based on the `observation`s. This **prevents the over-optimistic evaluation** of **value** of the `belief`. [Source](http://www.roboticsproceedings.org/rss15/p06.pdf).* |

Authors: Garg, N. P., Hsu, D., & Lee, W. S.

- Previous work: **"Determinized Sparse Partially Observable Tree" (`DESPOT`)** by [(Ye, Somani, Hsu & Lee. 2017)](https://arxiv.org/abs/1609.03250).
- About **`DESPOT`**:
  - _Why_ `Partially Observable` _?_
    - As the `state` is **not fully observable**, the agent must **reason** (_and maintain_) **with `belief`s**, which are **probability distributions over the `state`s** given history h.
    - The `belief` is a **sufficient statistic** that contains all the information from the **history** of **`action`s** and **`observation`s** (`a1`, `z1`, `a2`, `z2`, ... , `at`, `zt`).
    - > "By reasoning in `belief` space, `POMDP`s are able to maintain a balance between **_exploration_** and **_exploitation_** and hence provide a principled framework for [sequential] decision making **under uncertainty**."

  - _Why_ `Tree` _?_
    - Because a **search tree of histories** is constructed, online.
    - The **_"belief tree search"_** aspect has to do with the **`online`** nature of the solver _(as opposed to `offline` methods that compute an approximately optimal policy _ **_over the entire `belief` space_**_, prior to execution)_:
      - > "At each time step, it plans **locally** and chooses an optimal `action` for the **current `belief` only**, by performing **lookahead search** in the **neighborhood** of the **current `belief`**. It then **executes the chosen `action`** immediately."
    - > "Many `POMDP` solvers do **online planning** by doing **forward search** from the **current `belief`**, constructing a tree which **branches** each time an `action` is required, and also each time an `observation` may be observed".
      - Each **node** implicitly represents a `belief`.
        - **_"Implicitly"_** since it contains a **particle set** that approximates the `belief`. This contrasts with other approaches that **_explicitly_** represent the `belief` as a **probability distribution** over the `state` space, e.g. with exact updates using Bayes' theorem.
        - Planning is only performed from the **current `belief`**, which is the _root_ node.
      - Each **node branches** into `|A|` `action` edges.
      - Each **action edge** further branches into `|Z|` **observation edges**.
    - A `DESPOT` is built through **trials**, consisting of `exploration` and `backup` on **sampled** scenarios.

  - _Why_ `Determinized` _?_
    - Because the search is focused on a **set of randomly _sampled_ "scenarios"** that are sampled **_a priori_**.
      - A set of **random numbers** are generated in advance, as the first belief is given.
        - As I understood, they are called `scenarios` (_"abstract simulation trajectories"_).
      - > "A **small number of sampled scenarios** is **sufficient** to give a good estimate of the true value of any policy."
        - These determinized scenarios make `DESPOT` differ from [`POMCP`](https://papers.nips.cc/paper/4031-monte-carlo-planning-in-large-pomdps.pdf) which performs **`MCTS` on a belief tree using `UCT`**.
    - Here is **my interpretation**:
      - Imagine you are playing a **game** where your motion relies on the outcome of some dice, e.g. [`Monopoly`](https://en.wikipedia.org/wiki/Monopoly_(game)) or [`game of snakes and ladders`](https://en.wikipedia.org/wiki/Snakes_and_Ladders)
        - `Option 1-` At **each timestep**, you roll the dice and move accordingly.
        - `Option 2-` **Before starting**, you roll the dice `x` times. You then **put the dice away** and start playing: at each timestep, you move according to the `i`-th generated number.
      - Here, these generated numbers (_`scenarios`_) are used to decide the **`noise` injected** in the evaluation of the **two models** used for the **tree expansion**: `measurement` and `transition` functions.
        - That means it is known in advance, before starting building the tree, that the `n`-th `belief`-leaf will be generated from the `measurement` function using the **`n`-th sampled number** as **`noise` parameter**.
    - > "Like `DESPOT`, `DESPOT-α` uses the **"particle belief approximation"** and searches a **determinized** sparse belief tree".

  - _Why_ `Sparse` _?_
    - It is related to the question: _How to represent a `belief`?_
      - `DESPOT` represents the `belief` as a **set of particles** (particles are **sampled `state`s**), as for **[`POMCP`](https://papers.nips.cc/paper/4031-monte-carlo-planning-in-large-pomdps.pdf)**.
        - This enables to overcome the issue of **large `state` space**.
      - > "While a **standard belief tree** captures the execution of all policies under **_all_ possible scenarios**, a `DESPOT` captures the execution of all policies **under _a set of_ sampled scenarios**."
      - Because some **observation branches** are removed, a `DESPOT` can be viewed as a **_sparse_ approximation** of the **standard belief tree**:
        - The tree contains **all the action branches**, but only the **observation branches** under the **sampled scenarios**.
        - This also implies that `DESPOT` does not perform **belief update** over the **entire `state` space** (addressing the `curse of dimensionality`).
    - In other words, a `DESPOT` is structurally similar to **standard belief trees**, but contains only `belief` nodes **reachable under the `K` sampled scenarios**.
      - Size of `SparseSampling`: `|A|^D`.`C^D` (_sparse_ because only **`C` observations** are sampled for each action branch, and `D` is the **depth**).
      - Size of `DESPOT`: `|A|^D`.`K` (for `K` sampled **scenarios**).

- Additional notes about `DESPOT`:
  - Motivations: Address two **curses**.
    - `1-` Curse of **dimensionality**: the `state` space, and correspondingly the dimensionality of the `belief` size, grows exponentially with the number of `state` variables.
    - `2-` Curse of **history**: the `belief` tree grows exponentially with `depth`.
    - **`DESPOT`** (as for **[`POMCP`](https://papers.nips.cc/paper/4031-monte-carlo-planning-in-large-pomdps.pdf)**) breaks the two **curses** through **`sampling`**:
      - > "It alleviates the `curse of dimensionality` by **sampling `state`s from a `belief`** and alleviates the `curse of history` by **sampling `observation`s**."
  - `DESPOT` contains all the main [ideas](https://www.aaai.org/Papers/JAIR/Vol32/JAIR-3217.pdf) for **online planning** via **belief tree search**:
    - `1-` **Heuristic search**: The tree is incrementally constructed under the **guidance of a heuristic**.
    - `2-` **Branch-and-bound pruning**: `Upper bounds` _(computed from state-based heuristics)_ and `lower bounds` _(computed from default policies)_ on the value at each `belief` node are used to **prune suboptimal subtrees**.
      - Note that the **gap** between the `lower bound` and `upper bound` can represent the **uncertainty** at the `belief` node.
    - `3-` **Monte Carlo sampling**: Only a randomly **sampled subset of observation** branches is explored at each node.
  - **Regularization**.
    - Since many scenarios are not sampled, and because the chosen policy optimizes for **the sampled scenarios**, it can happen that the policy does not perform well.
    - _Regularization_ can be used to address that **overfitting**.

- More about "`1- heuristic search`": **Search-guidance** based on the **value function**.
  - > "To make sure that even the partially constructed tree is able to compute a good policy, **heuristics** based on **`upper bound`** and **`lower bound`** on the **value of `belief` nodes** are used to **guide** the search".
  - Note that this requires the **computation** of the `value` of `belief` nodes: `V(b)`.
  - _How to estimate the value?_ Using `α`-vectors.
- One concept: **`α`-vectors**.
  - One important property:
    - > "The **value function** of a `POMDP` can be approximated arbitrarily well by a **convex piece-wise linear** function of the `belief`".
    - `V`(`b`) = `max over α` [`∑ over s` (`b(s)`.`α(s)`)]
  - > "An `α`-vector is associated with a **conditional plan** and, for each `state` `s`, captures the reward of executing the plan starting from `state` `s`."
  - Note that the number of components in an `α`-vector correspond to the **number of states** and hence can be **exponentially large**.
  - In a `DESPOT-α`, **`α`-vectors** will be efficiently approximated to reduce computation, to approximate the `lower bound` on value of `belief` nodes.
  - Hence the name `Determinized Sparse Partially Observable Tree` **`With α-Vector Update`**.

---

- Main **motivation** for **`DESPOT-α`**:
  - Address the problem **`particle divergence`** to scale to **large `observation` spaces**.
  - > "When the **`observation` space is large**, particles quickly diverge into **separate `belief` nodes** in the belief tree, each of which contains only a **single particle**."
  - The **uncertainty can be underestimated** by the derived policy, leading to poor and **over-optimistic actions**.
- Main idea of `DESPOT-α`:
  - To prevent the **over-optimistic evaluation** of value of the `belief`, the idea is to keep a **constant number of particles**, and **weight** them (as for [`POMCPOW` and `PFT-DPW`](https://arxiv.org/abs/1709.06196) that extend `POMCP`).
  - > "Instead of propagating **only the particles producing the same observation** to the child of a `belief`-`action` node, we **propagate _all_ the particles** to the child nodes and update the weights of particles according to relative **likelihood of observation** `p`(`z`|`s`, `a`)."
  - This is similar to **`particle filters`**.
- New issue: when computing the **heuristics**, propagating each particle to **every child `belief` node** impacts the **computational efficiency**.
  - > "Always having `C` child `belief` nodes **prevents over optimistic evaluation of value** of `belief` but also makes the **tree size** (`C.|A|`)`^D`".
- Solution (not in [`POMCPOW` and `PFT-DPW`](https://arxiv.org/abs/1709.06196)):
  - **Share the value function calculation** among different (but similar) `belief` nodes, by **grouping observations** together.
    - > "We can **merge the observations**, when the value of the resulting `belief`s is maximized by the same **`α`-vector**."
    - > "We can use `α-vectors` to **share the computation** done for one trial among _"sibling"_ `belief` nodes for **improving `lower bounds`**".
  - This leads to the concept of **"_sibling_ `belief` nodes"**: Nodes which **differ** from each other only in **last observation**.
    - > "We are _implicitly_ **grouping `belief`s** whose values are maximised by same `α`-vector by **sharing `α`-vectors between sibling `belief` nodes**."
    - > "As sibling `belief` nodes **share the same set of scenarios** with different weights, `α`-vector calculated for one `belief` node **can be used to calculate approximate lower bound for the sibling `belief` nodes** by simply doing an inner product of weights of the particles and the `α`-vector".

---

- To sum up - Contributions:
  - `1-` Sample a **fixed number of observations** for each action branch like in `sparse sampling`, while still using **determinized scenarios** like `DESPOT` (it still contains only the `observation` branches reachable by **sampled scenarios**).
  - `2-` Introduce a **particle approximation** of the **`α`-vector** to improve the **efficiency** of online policy search.
  - `3-` Further **speed-up** the search by leveraging `CPU` and `GPU` **parallelization** introduced in [`HyP-DESPOT`](https://arxiv.org/abs/1802.06215).
    - Here **`K` particles** can be **expanded** in parallel, which is efficient since **each node contains all the particles**.

</details>

---

**`"Risk-Aware Reasoning for Autonomous Vehicles"`**

- **[** `2019` **]**
**[[:memo:](http://arxiv.org/abs/1910.02461)]**
**[** :mortar_board: `Khalifa University, Abu Dhabi` **]**

- **[** _`risk-bounded planning`, `chance constraint`, `POMDP`, `hierachical planning`_ **]**

<details>
  <summary>Click to expand</summary>

| ![Architecture to deal with **uncertainty** and produce **risk-aware** decisions. **Probabilistic vehicle motions** are modelled using **Probabilistic Flow Tube** ([`PFT`](https://dspace.mit.edu/handle/1721.1/76824)). These `PFTs` learnt from demonstrating trajectories represent a sequence of **probabilistic reachable sets**, and are used to calculate the **risk of collision**. This risk quantification serves in the **`CC-POMDP` formulation** of the `short-horizon` planner, where the ego-agent should plan the best sequence of actions while respecting a **bound on the probability** of **collision**. Uncertainty is also propagated in the **higher modules** of the hierarchical planning where _Temporal Plan Networks with Uncertainty_ ([`STNUs`](https://www.ijcai.org/proceedings/2019/0765.pdf)) are used to derive **short-term objectives**. [Source](http://arxiv.org/abs/1910.02461).](media/2019_khonji_1.PNG "Architecture to deal with **uncertainty** and produce **risk-aware** decisions. **Probabilistic vehicle motions** are modelled using **Probabilistic Flow Tube** ([`PFT`](https://dspace.mit.edu/handle/1721.1/76824)). These `PFTs` learnt from demonstrating trajectories represent a sequence of **probabilistic reachable sets**, and are used to calculate the **risk of collision**. This risk quantification serves in the **`CC-POMDP` formulation** of the `short-horizon` planner, where the ego-agent should plan the best sequence of actions while respecting a **bound on the probability** of **collision**. Uncertainty is also propagated in the **higher modules** of the hierarchical planning where _Temporal Plan Networks with Uncertainty_ ([`STNUs`](https://www.ijcai.org/proceedings/2019/0765.pdf)) are used to derive **short-term objectives**. [Source](http://arxiv.org/abs/1910.02461).")  |
|:--:|
| *Architecture to deal with **uncertainty** and produce **risk-aware** decisions. **Probabilistic vehicle motions** are modelled using **Probabilistic Flow Tube** ([`PFT`](https://dspace.mit.edu/handle/1721.1/76824)). These `PFTs` learnt from demonstrating trajectories represent a sequence of **probabilistic reachable sets**, and are used to calculate the **risk of collision**. This risk quantification serves in the **`CC-POMDP` formulation** of the `short-horizon` planner, where the ego-agent should plan the best sequence of actions while respecting a **bound on the probability** of **collision**. Uncertainty is also propagated in the **higher modules** of the hierarchical planning where _Temporal Plan Networks with Uncertainty_ ([`STNUs`](https://www.ijcai.org/proceedings/2019/0765.pdf)) are used to derive **short-term objectives**. [Source](http://arxiv.org/abs/1910.02461).* |

Authors: Khonji, M., Dias, J., & Seneviratne, L.

- One remark: _Not too many details are given about the implementation, but it is interesting to read reformulation of concepts met in other works_.
- One related work:
  - Several ideas (**`RAO*`**, **`PFT`**, **`CC-POMDP`**) reminded me the work of _[(Huang, Hong, Hofmann, & Williams, 2019)](https://arxiv.org/abs/1904.02341)_ - `Online Risk-Bounded Motion Planning for Autonomous Vehicles in Dynamic Environments` - detailed further above.
  - _The first author has been actually collaborating with this research group._
- One idea: **hierarchical planning**.
  - The uncertainty-aware decision-making task is decomposed between a **`high-level`** planner, a **`short-horizon`** planner and some **`MPC`**-based precomputed and learned manoeuvre trajectories.
  - Three levels of actions are distinguished for `short-horizon` planner:
    - **`Micro Actions`** are **primitive** actions, e.g. `accelerate`, `decelerate`, `maintain`.
    - **`Manoeuvre Actions`** are **sequences of micro actions**, e.g. `merge left`. `merge right`.
    - **`Macro Actions`** are **sequences of manoeuvre actions**, e.g. `pass the front vehicle`, `go straight until next intersection`.
- One concept: **_"chance constraint"_** optimization.
  - Some **measure of uncertainty** (e.g. about _perception_, about _unknown intention_, about _control_) is available to the `short-horizon` planner.
  - To goal is to solve the **optimization problem** (as for vanilla `POMDP` formulations) i.e. find the optimal sequence of ego-vehicle actions, while ensuring that the **probability of meeting a certain constraint** (e.g. _too small gap_ or _collision_) is above a **certain level**.
    - In other words, and contrary to _strict_ constrained optimization, here there is a **bound on the probability** of **violating constraints**.
    - The **policymaker** can set the desired **level of conservatism** in the plan.
  - The authors mention [**`RAO*`**](https://dspace.mit.edu/handle/1721.1/101416). This is solver for **_"chance-constrained POMDP"_** (**`CC-POMDP`**).
    - During the search, it uses **heuristics** to quickly detect and **prune overly-risky policy branches**.

</details>

---

**`"Tactical decision-making for autonomous driving: A reinforcement learning approach"`**

- **[** `2019` **]**
**[[:memo:](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf)]**
**[** :mortar_board: `Chalmers University` **]**
**[** :car: `Volvo` **]**

- **[** _`POMDP`, `MCTS`_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![ `MCTS` is especially beneficial when it is necessary to plan relatively __far into the future__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).](media/2019_hoel_3.PNG "`MCTS` is especially beneficial when it is necessary to plan relatively __far into the future__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).")  |
|:--:|
| *`MCTS` is especially beneficial when it is necessary to plan relatively __far into the future__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).* |

| ![ The `RL`-learnt neural network predicts two values used to __guide the search__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).](media/2019_hoel_2.PNG "The `RL`-learnt neural network predicts two values used to __guide the search__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).")  |
|:--:|
| *The `RL`-learnt neural network predicts two values used to __guide the search__. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).* |

| ![ Treat surrounding vehicles as __interchangeable objects__ using __`CNN`__ layers. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).](media/2019_hoel_1.PNG "Treat surrounding vehicles as __interchangeable objects__ using __`CNN`__ layers. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).")  |
|:--:|
| *Treat surrounding vehicles as __interchangeable objects__ using __`CNN`__ layers. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).* |

| ![ Comparison of __sampling efficiency__ - need for `domain knowledge` and `computational speed` should also be considered. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).](media/2019_hoel_4.PNG "Comparison of __sampling efficiency__ - need for `domain knowledge` and `computational speed` should also be considered. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).")  |
|:--:|
| *Comparison of __sampling efficiency__ - need for `domain knowledge` and `computational speed` should also be considered. [Source](https://research.chalmers.se/publication/511929/file/511929_Fulltext.pdf).* |

Author: Hoel, C.-J.

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
- One idea: Treat surrounding vehicles as **interchangeable objects** using **`CNN`** layers.
  - Using CNN layers with `max-pooling` creates a **translational invariance** between the vehicles.
  - > "The output is independent on the ordering of the vehicles in the input vector, and it also removes the problem of specifying a fixed input vector size, which instead can be made larger than necessary and padded with dummy values for the extra slots"
- About **_"sampling efficiency_"**, **_"domain knowledge_"** and trade-off of `speed` vs. `generality`:
  - The **`GA`** agent requires much **domain knowledge** in the form of **handcrafted features** (form of the instructions).
  - The `DQN` agent requires between `2` and `3` **orders of magnitude** **less driving time** than the `GA` agent.
  - > "The `MCTS`/`NN` agent requires the **most domain knowledge**, since it needs a **generative model `G`** of the environment, a **belief state estimator**, and possibly knowledge on how to prune actions that lead to collisions."
- Results:
  - The baseline is a **rule-based** approach built with **`IDM`** and **`MOBIL`** driver models (also used in the _generative model_ and to _simulate other vehicles_).
  - > "All methods outperform the baseline `IDM`/`MOBIL` model by taking decisions that allows the vehicle to navigate through traffic between `5%` and `10%` faster."
  - `MCTS` is especially beneficial when it is necessary to **plan relatively far into the future** (e.g. _highway exit case_).

</details>

---

**`"WiseMove: A Framework for Safe Deep Reinforcement Learning for Autonomous Driving"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1902.04118)]**
**[[:octocat:](https://git.uwaterloo.ca/wise-lab/wise-move)]**
**[** :mortar_board: `University of Waterloo` **]**

- **[** _`MCTS`, `options framework`, `LTL`, `hierarchical decision making`, `POMDP`_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://arxiv.org/abs/1902.04118).](media/2019_lee.PNG "[Source](https://arxiv.org/abs/1902.04118).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1902.04118).* |

Authors: Lee, J., Balakrishnan, A., Gaurav, A., & Feb, L. G.

- One related work: The presented approach reminds me the work of [Paxton, C., Raman, V., Hager, G. D., & Kobilarov, M.](https://arxiv.org/abs/1703.07887).
- One term: **"WiseMove"**: the presented **options**-based **modular** safe DRL framework.
  - The _modular_, or _hierarchical_, aspect comes from the [**option framework**](http://www-anw.cs.umass.edu/~barto/courses/cs687/Sutton-Precup-Singh-AIJ99.pdf). Sometimes called **macro-actions**.
  - For more on **Hierarchical RL**, check out [this `thegradient.pub` post](https://thegradient.pub/the-promise-of-hierarchical-reinforcement-learning/).
  - The idea is to decompose the decision by working with **temporal abstracted actions** (e.g. _slow down_, _turn left_) on a high-level (like a **behaviour planner**).
  - Each of these so called _options_ rely on **low-level primitive policies** that implement their manoeuvres (similar to a **geometrical trajectory optimizer**).
- One idea: **LTL** formalism is used to check the validity of high-level decisions.
  - An option is defined by (1) a **underlying primitive policy**, but also by (2) an **initial condition** and (3) a **terminal condition**.
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

</details>

---

**`"A Simulation-Based Reinforcement Learning Approach for Long-Term Maneuver Planning in Highway Traffic Scenarios"`**

- **[** `2019` **]**
**[[:memo:](https://www.researchgate.net/publication/334745733_A_Simulation-Based_Reinforcement_Learning_Approach_for_Long-Term_Maneuver_Planning_in_Highway_Traffic_Scenarios)]**
**[** :mortar_board: `Technische Universität Darmstadt` **]**
**[** :car: `Opel` **]**

- **[** _`combining learning/planning`, `hierarchical/modular decision making`, `POMDP`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

- One diagram is better than 100 words:

| ![The term `action` comprises a lateral manoeuvre decision and a set speed request. [Source](https://www.researchgate.net/publication/334745733_A_Simulation-Based_Reinforcement_Learning_Approach_for_Long-Term_Maneuver_Planning_in_Highway_Traffic_Scenarios).](media/2019_augustin.PNG "The term `action` comprises a lateral manoeuvre decision and a set speed request. [Source](https://www.researchgate.net/publication/334745733_A_Simulation-Based_Reinforcement_Learning_Approach_for_Long-Term_Maneuver_Planning_in_Highway_Traffic_Scenarios).")  |
|:--:|
| *The term `action` comprises a lateral manoeuvre decision and a set speed request. [Source](https://www.researchgate.net/publication/334745733_A_Simulation-Based_Reinforcement_Learning_Approach_for_Long-Term_Maneuver_Planning_in_Highway_Traffic_Scenarios).* |

Authors: Augustin, D., Schucker, J., Tschirner, J., Hofmann, M., & Konigorski, L.

- One remark: I like the **hierarchy** and **modularity** of the approach.
  - Especially the fact that the `action` stays high-level (`speed desire` and `high-level manoeuvre`), as opposed to `steering angle` and `throttle` commands that are often used in RL.
- One promising tool: [FLOW](https://flow-project.github.io/)
  - `FLOW` is a Python library that interfaces the RL libraries [RLlib](https://ray.readthedocs.io/en/latest/rllib.html) and [rllab](https://github.com/rll/rllab) with `SUMO`. It has been developed and is supported by _UC Berkeley_.
  - It has not been used many times (_because of the lack of Windows support?_). Instead, many research using `SUMO` develop their own interface, which makes comparison and reproduction difficult.
  - A few recent `FLOW`-based works can be mentioned though:
    - "Simulation to scaled city: zero-shot policy transfer for traffic control via autonomous vehicles" by [(Jang et al., 2018)](https://arxiv.org/abs/1812.06120)
    - "Benchmarks for reinforcement learning in mixed-autonomy traffic" by [(Vinitsky et al., 2018)](https://arxiv.org/abs/1812.06120)

</details>

---

**`"Towards Human-Like Prediction and Decision-Making for Automated Vehicles in Highway Scenarios"`**

- **[** `2019` **]**
**[[:memo:](https://tel.archives-ouvertes.fr/tel-02184362/document)]**
**[[:octocat:](https://github.com/marioney/hybrid_simulation/tree/decision-making)]**
**[[🎞️](https://www.youtube.com/watch?v=Xx5OmV86CsM)]**
**[** :mortar_board: `INRIA` **]**
**[** :car: `Toyota` **]**

- **[** _`MCTS`, `online POMDP`, `POMCP`, `progressive widening`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_  **]**

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

Author: Sierra Gonzalez, D.

- The author targets some **_"human-like tactical planning"_**.
  - The **POMDP** formulation is ideal since it considers uncertainty in the `controls`, `states`, and the `intentions` of the traffic participants.
  - The idea is to include **estimation of intentions** for **long-term anticipation**.

- One idea: about the **rollout policy** used for the construction of the search tree.
  - One option is to use a **random** rollout policy.
  - Here, the previously-derived models are used to **predict approximately the long-term development** of traffic scenes.

- Another idea: adapt the combination of **_model-based_** and **_manoeuvre-estimation-based_** predictions, depending on **how far** the **rollout looks into the future**.

> "As we go **deeper into the history tree** (that is, into the future), the **observed dynamics** of the targets at the root node become **less relevant** and so we **rely increasingly** in the model to **predict the behaviour** of the obstacles."

- Other related works:
  - The **Hierarchical architecture** of [(Sonu, E., Sunberg, Z., & Kochenderfer, M. J. (2018). _"Exploiting Hierarchy for Scalable Decision Making in Autonomous Driving"_)](https://www.researchgate.net/publication/328455111_Exploiting_Hierarchy_for_Scalable_Decision_Making_in_Autonomous_Driving).
  - The **`Double Progressive Widening (DPW)`** or `progressive unpruning` of [(Sunberg & Kochenderfer, 2017)](https://arxiv.org/abs/1709.06196) to deal with continuous observations.
  - The general approach of [(Bouton, M., Cosgun, A., & Kochenderfer, M. J. (2017). _"Belief state planning for autonomously navigating urban intersections"_)](https://arxiv.org/abs/1704.04322). The main difference is the substitution of the **Interacting Multiple Model** (`IMM`) with **DBN**-based model to **consider the interactions** between vehicles.

- One quote about the _(relative)_ benefits of POMDP formulations:

> "There have not been significative differences between the decisions taken by the **proposed POMDP planner** and the **reactive SUMO model**. This is due to the fact that neither of those scenes truly required to analyse the **long-term consequences** of a maneuver".

</details>

---

**`"Value Sensitive Design for Autonomous Vehicle Motion Planning"`**

- **[** `2018` **]**
**[[:memo:](https://ddl.stanford.edu/publications/value-sensitive-design-autonomous-vehicle-motion-planning)]**
**[** :mortar_board: `Stanford University` **]**
**[** :car: `Ford` **]**

- **[** _`POMDP`, `QMDP`_ **]**

<details>
  <summary>Click to expand</summary>

| ![The `POMDP` policy **better deals with uncertainties** in the detection of the pedestrian. It accounts for the possible **transition** between `detected` and `not detected` cases, leading to **smoother `action`s** across the `state` space. [Source](https://ddl.stanford.edu/publications/value-sensitive-design-autonomous-vehicle-motion-planning).](media/2018_thornton_1.PNG "The `POMDP` policy **better deals with uncertainties** in the detection of the pedestrian. It accounts for the possible **transition** between `detected` and `not detected` cases, leading to **smoother `action`s** across the `state` space. [Source](https://ddl.stanford.edu/publications/value-sensitive-design-autonomous-vehicle-motion-planning).")  |
|:--:|
| *The `POMDP` policy **better deals with uncertainties** in the detection of the pedestrian. It accounts for the possible **transition** between `detected` and `not detected` cases, leading to **smoother `action`s** across the `state` space. [Source](https://ddl.stanford.edu/publications/value-sensitive-design-autonomous-vehicle-motion-planning).* |

Authors: Thornton, S. M., Lewis, F. E., Zhang, V., Kochenderfer, M. J., & Christian Gerdes, J.

- Motivation:
  - Apply the **`VSD` methodology** / **formalism** to the problem of **speed control** for the scenario of an **occluded pedestrian crosswalk**.
- About [Value Sensitive Design](https://en.wikipedia.org/wiki/Value_sensitive_design) (`VSD`):
  - > "[Wikipedia]: A theoretically grounded approach to the **design** of technology that **accounts for human values** in a principled and comprehensive manner."
  - In **(`PO`)`MDP`s**, engineers account for some **"human values"** in the **design of the reward function**.
  - **Values** are converted to **specifications**:
    - `safety`: harm reduction or collision avoidance.
    - `legality`: care when approaching a crosswalk.
    - `mobility`: efficiency.
    - `smoothness`: comfort.
  - **Stakeholders** are also identified:
    - the **AV** and its occupants.
    - the obstructing **vehicle parked**.
    - the **pedestrian** potentially crossing the street.
    - the **authority** of traffic laws.
- About the `POMDP` formulation:
  - The `belief` of **a pedestrian crossing** is tracked with some Bayesian filter.
    - The pedestrian detection is a **Boolean value** because the pedestrian is **either crossing or not**.
    - > "There is `observation` uncertainty for the pedestrian crossing with a **false positive of `5%` for detecting** and a **false positive of `5%` for not detecting** the pedestrian, which **captures sensor uncertainty**.
    - > "When the pedestrian is detected, there is a `90%` probability the pedestrian will **continue to be detected** at the next time step. When the pedestrian is not detected, then there is a `50%` chance he or she will **continue to not be detected**, which captures the **uncertainty due to the occlusion**.
  - The **[`QMDP`](http://www-anw.cs.umass.edu/~barto/courses/cs687/Cassandra-etal-POMDP.pdf)** [solver](https://github.com/JuliaPOMDP/QMDP.jl) from **[`JuliaPOMDP`](http://juliapomdp.github.io/POMDPs.jl/latest/)** is used.
    - > "Although `QMDP` assumes that at **the next time step the state will be fully observable**, it is well suited for this problem because the actions are **not information gathering**, meaning the actions do not directly reduce the uncertainty of the scenario."
    - _I do not agree with that statement: information gathering is key in this scenario to resolve the ambiguity in the detection._
  - `state` and `action` spaces are **discrete**.
    - But **_"continuousness"_** is maintained using **_"multilinear grid interpolations"_** for the state transitions, as in [(Davies 1997)](https://pdfs.semanticscholar.org/5ee0/d2ccbda910b32876754da4ce6f32b2e0c3d6.pdf).
- Benefits of `POMDP`:
  - > "A **stochastic optimization** problem can account for **modeled uncertainty** present in the driving scenario while balancing the **identified values** through the objective function."
  - The baseline on the other hand is **reactive**:
    - `if` detection, `then` decelerate to stop at crosswalk.
    - `else` target the desired velocity with a proportional controller.
  - For this scenario where the **detection is key** but **uncertain**, one need to **anticipate `transitioning`** from one set of logic to the other.
    - When the pedestrian is detected: the baseline is **safe**, but it **lacks efficiency**.
    - When the pedestrian is not detected: the baseline is **efficient**, but **not safe**.
  - This **rule-based dichotomy** makes the baseline control have **full speed** when the pedestrian appears and prevents it from to **legally yielding** to the pedestrian.

</details>

---

**`"Decision Making Under Uncertainty for Urban Driving"`**

- **[** `2018` **]**
**[[:memo:](https://web.stanford.edu/class/aa228/reports/2018/final100.pdf)]**
**[[:octocat:](https://github.com/PhilippeW83440/ACT)]**
**[** :mortar_board: `Stanford` **]**

- **[** _`POMDP`, `MCTS`, `julia`, `probabilistic risk assessment`, `value iteration`_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![Comparing the vanilla POMCP and proposed safe variant of it. [Source](https://web.stanford.edu/class/aa228/reports/2018/final100.pdf).](media/2018_weingertner.PNG "Comparing the vanilla POMCP and proposed safe variant of it. [Source](https://web.stanford.edu/class/aa228/reports/2018/final100.pdf).")  |
|:--:|
| *Comparing the vanilla POMCP and proposed safe variant of it. [Source](https://web.stanford.edu/class/aa228/reports/2018/final100.pdf).* |

Authors: Weingertner, P., Autef, A., & Le Cleac’h, S.

- One algorithm: [`POMCP`](https://papers.nips.cc/paper/4031-monte-carlo-planning-in-large-pomdps).
  - Presented in 2010, `POMCP` is an extension of the traditional **MCTS algorithm to POMDP**.
  - Together with [`DESPOT`](https://papers.nips.cc/paper/5189-despot-online-pomdp-planning-with-regularization), `POMCP` is an often-used POMDP online solver.
- One term: **"observation class"**.
  - Different extensions of `POMCP` and `DESPOT` have been proposed. In the presented approach, the goal is to work with **continuous observations**, while ensuring safety.
  - The idea is to **limit the number of observation nodes** in the tree by **grouping observations** based on some **utility function**.
  - This utility function should not to be confused with the **offline-learn _value function_** representing the probability of collision.
  - The safe **clusterization of observations** can be based on _smallest `TTC`_ or _smallest distance_ to other participants.
- One idea: **guide the online graph search** using an **offline methods** to improve safety.
  - This is based on the work of [(Bouton, Karlsson, et al., 2019)](https://arxiv.org/abs/1904.07189), where **offline `VI`** (value iteration) is used to compute `P_collision`(`s`, `a`).
  - This **safety criterion** is then used to limit the **set of safe available actions**.
  - In the presented work, the author reason over the `belief` instead of the `state`.
- Another idea: Use a **Kalman Filter** (instead of some particle filters) for **belief updater**.
- One quote:

> "Using an online method based on sparse sampling may lead to safety issues. Rare events with critical consequences may not be sampled leading to sub-optimal and potentially dangerous decisions."

- One promising tool: [POMDPs.jl](https://github.com/JuliaPOMDP/POMDPs.jl)
  - `POMDPs.jl` is an interface for defining, solving, and simulating `MDPs` and `POMDPs` on discrete and continuous spaces. It has been developed and is supported by a team from _Stanford_.
- Two ideas for future works:
  - In their [repo](https://github.com/PhilippeW83440/ACT), the authors suggest **combining learning** (e.g. model-free RL used as a heuristic and/or for rollout) with **planning** (MCTS), mentioning the success of AlphaGo Zero.
  - Another improvement concerns the **transition model** for the observed vehicles. Instead of `CV` (constant velocity) models, one could assume surrounding vehicles are **following a driver model** (e.g. `IDM`) and the task would be to **infer its hidden parameters**.

</details>

---

**`"Baidu Apollo EM Motion Planner"`**

- **[** `2018` **]**
**[[:memo:](https://arxiv.org/abs/1807.08048)]**
**[[:octocat:](https://github.com/ApolloAuto/apollo/tree/master/modules/planning)]**
**[** :car: `Baidu` **]**
- **[** _`path-velocity decomposition`, [`apollo`](https://github.com/ApolloAuto/apollo/)_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1807.08048).](media/2018_fan_3.PNG "[Source](https://arxiv.org/abs/1807.08048).")  |
|:--:|
| *Top-left: __Hierarchical__ framework: **lane-level** trajectories are **generated in parallel** and eventually compared to decide of lane-changes. They can be **`passive`** (e.g. when the default lane is blocked by an obstacle) or **`non-passive`** (request triggered by the **routing module** for the purpose of **reaching the final destination**). For each lane, both `path` and `speed` optimizations are **iteratively** solved in the **`Frenet` frame** using a combination of **dynamic programming** for rough but feasible **decision**-proposals (`E-step`) and **spline-based quadratic programming** for smoothing and optimization (`M-step`). [Source](https://arxiv.org/abs/1807.08048).* |

| ![[Source](https://arxiv.org/abs/1807.08048).](media/2018_fan_2.PNG "[Source](https://arxiv.org/abs/1807.08048).")  |
|:--:|
| *Bottom: note the **`path`<->`speed` communication**: `1-` The `speed` profile from the last cycle is used to evaluate the **dynamic obstacle `interactions`**, for instance to estimate the **time-to-collision** with oncoming and low-speed dynamic obstacles in the `path` optimizer. `2-` The generated `path` is sent to the **`speed` optimizer** to compute an optimal `speed` profile. [Source](https://arxiv.org/abs/1807.08048).* |

| ![[Source](https://arxiv.org/abs/1807.08048).](media/2018_fan_1.PNG "[Source](https://arxiv.org/abs/1807.08048).")  |
|:--:|
| *Top: __Search__ within a grid is performed using **dynamic programming** (`DP`). In addition to the **boundary constraint** and the **dynamic constraints** (`acceleration`, `jerk limits` and `monotonicity`), the **generated path** shall match the ego car’s **initial lateral position** and **derivatives**. Bottom: Since all **constraints** are **linear** with respect to spline parameters, a **quadratic programming** solver (`QP`) can be used to solve the problem very fast. [Source](https://arxiv.org/abs/1807.08048).* |

| ![[Source](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/images/architecture_5.5.png).](media/2020_apollo_1.PNG "[Source](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/images/architecture_5.5.png).")  |
|:--:|
| *__`Scenario`-based planning__ was introduced in `v3.5`. [Source1](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/images/architecture_5.5.png) [Source2](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/images/Apollo_3_5_software_architecture.png).* |

Authors: Fan, H., Zhu, F., Liu, C., Zhang, L., Zhuang, L., Li, D., Zhu, W., Hu, J., Li, H. & Kong, Q.

- Motivations:
  - > "This **planner** targets **safety** and ride experience with a **multilane**, **path-speed iterative**, **traffic rule** and **decision** combined design."
  - Avoid **`state`-based descriptions** used in **`hand-tuning` decisions** (_tuneable_ but not _scalable_) and **`model-based` decisions** (e.g. data-driven-tuned `FSM`).
    - > "It is true that a **heavy-decision-based** algorithm, or **heavily rule-based algorithm**, is easily **understood** and **explained**. The disadvantages are also clear: it may be trapped in **corner cases** (while its frequency is closely related to the complexity and magnitude of the number of rules) and **not always be optimal**."
  - **Real-world** industrial applications are targeted. As opposed to theoretical simulator-based research experiments. Especially about **`safety`**.
    - > "We aim to provide a trajectory with **at least an `8s`** or **`200m` meter** motion planning trajectory."
    - > "In the case of an **emergency**, the system could react within `100ms`, compared with a `300ms` reaction time for a normal human driver."

- About **`path-velocity` decomposition** and **`Fernet` frame**.
  - > "Many autonomous driving motion planning algorithms are developed in **`Frenet` frames with time (`SLT`)** to reduce the planning dimension with the help of a **reference line**. Finding the optimal trajectory in a `Frenet` frame is essentially a **`3D` constrained optimization** problem."
  - Instead of a direct **`3D`** optimization, the problem is converted into **two `2D`** problems: **`path` optimization** and **`speed` optimization**.

- **_Why `EM`?_**
  - It is based on an **[`EM`-type iterative](https://en.wikipedia.org/wiki/Expectation%E2%80%93maximization_algorithm)** algorithm:
    - > "[Wikipedia] It alternates between performing an **expectation (`E`)** step, which creates a **function for the expectation** of the log-likelihood evaluated using the current estimate for the parameters, and a **maximization (`M`)** step, which computes parameters **maximizing the expected log-likelihood** found on the `E` step."
    - Motivations for **two-step optimization**:
      - > "Finding a best `path` / `speed` profile on the **`SL` / `ST` graph** is a **non-convex** optimization problem."
      - > "In `Apollo` **`EM`-planner**, we **make decisions** prior to providing a **smooth trajectory**. The decision process is designed to make **on-road intentions clear** and **reduce the search space** for finding the **optimal** trajectory."
  - Here:
    - `1-` `E-path`. Based on the information projected in the `Frenet` frame (`SL` = **arclength `S`** vs `Lateral` gap), a **smooth `path`** is generated.
      - **Search** within a grid is performed using **dynamic programming** (`DP`) to reach a **rough resolution**.
        - > "Some necessary **pruning** based on vehicle dynamic constraints is also applied to **accelerate the process**."
      - The `DP` results are used to generate a **convex domain** and guide the **spline-based quadratic programming** (`QP`).
        - This solution can provide **obstacle decisions** such as **`nudge`**, `yield` and `overtake`.
    - `2-` `M-path`. **Find** a feasible **smooth `path`** solution in this region.
      - `QP` is used to search for the **optimal solution** in the **convex region supplied by the `DP` step**, balancing between **following the guidance line** and **smoothness** (`heading`, `curvature` and `derivative of curvature`).
      - `QP` requires a **_decision_** from the `DP` step, such as `nudge from the left`, `nudge from the right`, `follow`, or `overtake`, to **generate its constraint**.
      - > "For both the `path` and `speed` optimizers, we find that **piecewise `quintic` polynomials** are good enough. The spline generally contains **`3` to `5` polynomials** with approximately **`30` parameters**. The quadratic programming problem has a relatively **small objective function** but **large number of constraints** [`boundary constraints` and `dynamic feasibility`]. Consequently, an active set `QP` solver is good for solving the problem."
      - The result calculated in the **last cycle** is re-used as a **hot start**.
      - Average **solving time: `3ms`**.
    - `3-` `E-speed`. Obstacles are projected on the station-**time** graph (`ST` = **arclength `S`** vs `Time`) and find a first solution with `DP`.
      - Since the **piecewise linear** `speed` profile cannot satisfy **dynamic requirements**, the **spline `QP`** step is needed to fill this gap.
    - `4-` `M-speed`. **Find** a feasible **smooth `speed`** solution in this region.
      - > "The **spline `QP` speed** step includes three parts: `cost functional`, `linearized constraint` and spline `QP` solver."

- How to tune a **`cost` functional** that can adapt to **different scenarios**?
  - As the scenario becomes more complicated, **tuning** to improve the motion planner performance becomes **increasingly difficult**.
  - One idea is to **learn these parameters** from **human demonstrated driving data**: [`auto-tuning`](https://arxiv.org/pdf/1808.04913.pdf).

- About `Apollo`:
  - Features are progressively added from the _`GPS` waypoint following_ in `v1.0` (2017) to _urban driving_ in `v5.5` (2020).
  - **Scenario-based planning** e.g. `follow-lane`, `intersection`, `park`, `emergency`, was introduced in **`v3.5`**:
    - > "We felt the need to move to a **more modular**, **scenario specific** and wholistic approach for planning its trajectory. In this approach, each **driving use case** is treated as a **different driving scenario**. This is useful because an issue now reported in a particular scenario can be fixed **without affecting the working of other scenarios** as opposed to the previous versions."
  - `v5.5` focuses on **`curb-to-curb`** autonomous driving on urban roads, with a `Park-and-go` scenario, useful in situations like **`curb`-side delivery** or **passenger pick-up** or **drop-off**.
  - License: **Apache-2.0**.

</details>

---

**`"On Monte Carlo Tree Search and Reinforcement Learning"`**

- **[** `2017` **]**
**[[:memo:](https://pdfs.semanticscholar.org/3d78/317f8aaccaeb7851507f5256fdbc5d7a6b91.pdf)]**
**[** :mortar_board: `Universities of Ljubljana and Essex` **]**

- **[** _`RL`, `MCTS`, `learning`, `planning`_ **]**

<details>
  <summary>Click to expand</summary>

| ![Four parameters introduced in a TD-Tree Search (`TDTS`) algorithm related to `forgetting`, `first-visit updating`, `discounting` and `initial bias`. [Source](https://pdfs.semanticscholar.org/3d78/317f8aaccaeb7851507f5256fdbc5d7a6b91.pdf).](media/2017_vodopivec_1.PNG "Four parameters introduced in a TD-Tree Search (`TDTS`) algorithm related to `forgetting`, `first-visit updating`, `discounting` and `initial bias`. [Source](https://pdfs.semanticscholar.org/3d78/317f8aaccaeb7851507f5256fdbc5d7a6b91.pdf).")  |
|:--:|
| *Four parameters introduced in a TD-Tree Search (`TDTS`) algorithm related to `forgetting`, `first-visit updating`, `discounting` and `initial bias`. [Source](https://pdfs.semanticscholar.org/3d78/317f8aaccaeb7851507f5256fdbc5d7a6b91.pdf).* |

Author: Vodopivec, T., Samothrakis, S., & Ster, B.

- Goal: The authors aim at promoting a _"unified view of `learning`, `planning`, and `search`"_.
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
  - Both exhibits **anytime** behaviours.

- Two major `RL`/`MCTS` differences:
  - RL methods do not recognize the **playout phase**, i.e. a **separate policy for non-memorized** (i.e., non-represented) parts of space.
    - In MCTS it is often called the **_"default policy"_**.
    - It would be beneficial to have a **more complex** default policy (where **expert knowledge** could be included).
  - They present two different **memorization** approaches and two different **approximation accuracies**:
    - `RL` methods based on value function approximation (e.g. NN) can **weakly describe the whole state space**.
      - The `RL` theory should **acknowledge a non-represented** (i.e., non-memorized) part of the state space, i.e. the part that is **not described (estimated) by the representation model** at a given moment.
    - `MCTS` algorithms initially **approximate only a part of the state space** (with high accuracy).
      - Therefore `MCTS` maintains the distinction between a **_memorized_** and **_non-memorized_** part of the **state space**.
      - The state-space representation is changed online: it is an **_"adaptive (incremental) representation method"_**.
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
    - The **transposition table** is a **kind of cache**: on encountering a new position, the program checks the table to see whether the **state has already been analysed**. If yes, the value (stored) can be used instead of calculating it (which would require expending a subtree).
  - **_"afterstates"_**:
    - I understand it as a third metric to quantify the "value":
      - V(`s`): **state value** of **being** in state `s` and following policy `π`.
      - V(`s'`): **afterstate value** of **arriving** at state `s'` and thereafter following policy `π`.
      - Q(`s`, `a`): **action value** of **being** in state `s`, taking action `a` and thereafter following policy `π`.
    - This is used in the presented Sarsa-UCT(λ) algorithm.

</details>

---

**`"Online decision-making for scalable autonomous systems"`**

- **[** `2017` **]**
**[[:memo:](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515)]**
**[[🎞️](https://pdfs.semanticscholar.org/1df1/8e66c9117852468f8e327a67622b5738558a.pdf) (slides)]**
**[** :mortar_board: `University of Massachusetts` **]**
**[** :car: `Nissan` **]**

- **[** _`POMDP`, `scaling`, `scene decomposition`_ **]**

<details>
  <summary>Click to expand</summary>

| ![Two **solvers** have been developed `offline`: one to deal with **one `vehicle`** (Decision-Problem 1 (`DP1`)), and another that can deal with **one `pedestrian`** (`DP2`). `DP`s are instantiated for each **detected entity**: here **two cars** and **one pedestrian**. At each timestep, three **recommendations** are issued. The **most conservative** one is kept and implemented (here `stop`). [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).](media/2017_wray_1.PNG "Two **solvers** have been developed `offline`: one to deal with **one `vehicle`** (Decision-Problem 1 (`DP1`)), and another that can deal with **one `pedestrian`** (`DP2`). `DP`s are instantiated for each **detected entity**: here **two cars** and **one pedestrian**. At each timestep, three **recommendations** are issued. The **most conservative** one is kept and implemented (here `stop`). [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).")  |
|:--:|
| *Two **solvers** have been developed `offline`: one to deal with **one `vehicle`** (Decision-Problem `P1`), and another that can deal with **one `pedestrian`** (`P2`). `DP`s are instantiated for each **detected entity**: here **two cars** and **one pedestrian**. Therefore, at each timestep, three **recommendations** are issued. The **most conservative** one is kept and implemented (here `stop`). [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).* |

| ![The proposed solution offers number advantages over the direct use of a **massive monolithic `POMDP`** for planning and learning. First, it remains tractable by **growing linearly** in the number of decision-making problems encountered. Secondly, its component-based form **simplifies the design** and **analysis** and offers **easier interpretability**. [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).](media/2017_wray_2.PNG "The proposed solution offers number advantages over the direct use of a **massive monolithic `POMDP`** for planning and learning. First, it remains tractable by **growing linearly** in the number of decision-making problems encountered. Secondly, its component-based form **simplifies the design** and **analysis** and offers **easier interpretability**. [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).")  |
|:--:|
| *The proposed solution offers number advantages over the direct use of a **massive monolithic `POMDP`** for planning and learning. First, it remains tractable by **growing linearly** in the number of decision-making problems encountered. Secondly, its component-based form **simplifies the design** and **analysis** and offers **easier interpretability**. [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).* |

| ![The authors consider that **urban** deployment of AVs requires **mid-level** decision-making. Hence both `state` and `action` are rather abstract. [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).](media/2017_wray_3.PNG "The authors consider that **urban** deployment of AVs requires **mid-level** decision-making. Hence both `state` and `action` are rather abstract. [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).")  |
|:--:|
| *The authors consider that **urban** deployment of AVs requires **mid-level** decision-making. Hence both `state` and `action` are rather abstract. [Source](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515).* |

Author: Wray, K. H., Witwicki, S. J., & Zilberstein, S.

- Motivation:
  - **Scale!**
  - More precisely, provide a **general solution** to **intersection** decision-making that can cope with **complex real-world scenarios**, with varying **number of lanes** and **number of entities** (e.g. `car` and `pedestrian`).
- One option: a **single (big) policy**.
  - > "A **single all-encompassing `POMDP`** with these state factors quickly becomes **utterly infeasible**, and will vary greatly among intersections."
  - Instead, the authors suggest **decomposing the problem**.
- Main idea:
  - `1-` **Decompose** the **scene** into **_canonical_** simple scenarios.
  - `2-` Solve each instance separately with solvers that have been learnt **offline**. It leads to one **recommendation** per instance.
  - `3-` Aggregate all the recommendations using some (_conservative_) **_"aggregation operator"_**.
    - It could have been **`min()`** if the action were about the **`acceleration`**.
    - Here, the action is more **abstract** (**`go`**, **`stop`**) and the operator uses **lexicographic preference** for safest action (concept called **`LEAF`**): `stop` > `go`.
- About the **`POMDP` `offline` solver**:
  - The authors parallelize the **Point-Based Value Iteration** (`PBVI`) algorithm using `GPU`.
  - They called it **`nova`**: [`paper`](https://www.aaai.org/ocs/index.php/FSS/FSS15/paper/view/11645/11515) [`slides`](https://pdfs.semanticscholar.org/6f86/9e895cc9205b0497697bc230398df95d0888.pdf).

</details>

---

**`"Planning under Uncertainties for Autonomous Driving on Urban Road"`**

- **[** `2016` **]**
**[[:memo:](https://scholarbank.nus.edu.sg/handle/10635/126233)]**
**[[🎞️](https://www.youtu.be/W37haHhfU34)]**
**[** :mortar_board: `National University of Singapore` **]**

- **[** _`POMDP`, `DESPOT`, `urban driving`, `intention estimation`_ **]**

<details>
  <summary>Click to expand</summary>

| ![The author prefers the **`reaction`**-based over **`goal`**-based **`motion intention` estimation**. It relies on **_deviation measurement_** between the **observed behaviour** and some **reference vehicle state**. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).](media/2016_wei_1.PNG "The author prefers the **`reaction`**-based over **`goal`**-based **`motion intention` estimation**. It relies on **_deviation measurement_** between the **observed behaviour** and some **reference vehicle state**. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).")  |
|:--:|
| *The author prefers the **`reaction`**-based over the **`goal+motion`**-based **`motion intention` estimation**. It relies on **_deviation measurement_** between the **observed behaviour** and some **reference vehicle state**. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).* |

| ![The agent **maintains a belief** over the **`motion intention`** of the other vehicles to decide of the **longitudinal discrete `action`** in {`accelerate`, `maintain speed`, `decelerate`}. The **`motion intention`** is in {`stopping`, `hesitation`, `normal`, `aggressive`}. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).](media/2016_wei_4.PNG "The agent **maintains a belief** over the **`motion intention`** of the other vehicles to decide of the **longitudinal discrete `action`** in {`accelerate`, `maintain speed`, `decelerate`}. The **`motion intention`** is in {`stopping`, `hesitation`, `normal`, `aggressive`}. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).")  |
|:--:|
| *The agent **maintains a belief** over the **`motion intention`** of the other vehicles to decide of the **longitudinal discrete `action`** in {`accelerate`, `maintain speed`, `decelerate`}. The **`motion intention`** is in {`stopping`, `hesitation`, `normal`, `aggressive`}. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).* |

| ![[Source](http://youtu.be/W37haHhfU34).](media/2016_wei_1.gif "[Source](http://youtu.be/W37haHhfU34).")  |
|:--:|
| *[Source](http://youtu.be/W37haHhfU34).* |

| ![The `observation` model can be factorized. Then, the **emission probability** depends on both the **hidden intention** and **physical** state. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).](media/2016_wei_7.PNG "The `observation` model can be factorized. Then, the **emission probability** depends on both the **hidden intention** and **physical** state. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).")  |
|:--:|
| *The `observation` model can be factorized. Then, the **emission probability** depends on both the **hidden intention** and **physical** state. Question: **can someone explain me the decomposition, and what to take for the marginal probability?** [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).* |

| ![Construction of the `transition` model: The **`speed action` of the other vehicles** depends on both their `motion intention` and the **correlations** with the other vehicles. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).](media/2016_wei_8.PNG "Construction of the `transition` model: The **`speed action` of the other vehicles** depends on both their `motion intention` and the **correlations** with the other vehicles. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).")  |
|:--:|
| *Construction of the `transition` model: The **`speed action` of the other vehicles** depends on both their `motion intention` and the **correlations** with the other vehicles. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).* |

| ![In the `POMDP`, the `transition model` for the other vehicle works as followed: an **acceleration `action`** is computed based on both its `motion intention` and its **correlations** with the other vehicles, which means that it wants to **avoid a potential collision** with **any other vehicles**. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).](media/2016_wei_2.PNG "In the `POMDP`, the `transition model` for the other vehicle works as followed: an **acceleration `action`** is computed based on both its `motion intention` and its **correlations** with the other vehicles, which means that it wants to **avoid a potential collision** with **any other vehicles**. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).")  |
|:--:|
| *In the `POMDP`, the `transition model` for the other vehicle works as followed: an **acceleration `action`** is computed based on both its `motion intention` and its **correlations** with the other vehicles, which means that it wants to **avoid a potential collision** with **any other vehicles**. A `reference state` `rc` (containing among others a **reference speed `vref`** that the observed vehicle **should follow**) is also derived - more precisely sampled - which enables the computation of an expected `action` for each observed vehicle. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).* |

| ![A **risk-aware** motion planning algorithm `CC-RRT*-D` is proposed to address the **internal motion uncertainty**. It leverages the **RRT\* algorithm** for space exploring and utilized the **chance-constrained** approach to evaluate the trajectories' collision risks. In each planning loop, the decision-maker will call the **motion planner** first with a **goal** being provided. It then will proceed to decide a **proper acceleration command** and select **which trajectory** to commit. The `control frequency` is set at `1Hz` while the `planning task time` is fixed to `0.1s`. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).](media/2016_wei_5.PNG "A **risk-aware** motion planning algorithm `CC-RRT*-D` is proposed to address the **internal motion uncertainty**. It leverages the **RRT\* algorithm** for space exploring and utilized the **chance-constrained** approach to evaluate the trajectories' collision risks. In each planning loop, the decision-maker will call the **motion planner** first with a **goal** being provided. It then will proceed to decide a **proper acceleration command** and select **which trajectory** to commit. The `control frequency` is set at `1Hz` while the `planning task time` is fixed to `0.1s`. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).")  |
|:--:|
| *A **risk-aware** motion planning algorithm `CC-RRT*-D` is proposed to address the **internal motion uncertainty**. It leverages the **RRT\* algorithm** for space exploring and utilized the **chance-constrained** approach to evaluate the trajectories' collision risks. In each planning loop, the decision-maker will call the **motion planner** first with a **goal** being provided. It then will proceed to decide a **proper acceleration command** and select **which trajectory** to commit. The `control frequency` is set at `1Hz` while the `planning task time` is fixed to `0.1s`. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).* |

| ![The `action` space is extended to **coordinate the `speed` and `steering` control**. While choosing the **acceleration**, the agent selects either the **reference path** or a path **proposed by the motion planner**. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).](media/2016_wei_6.PNG "The `action` space is extended to **coordinate the `speed` and `steering` control**. While choosing the **acceleration**, the agent selects either the **reference path** or a path **proposed by the motion planner**. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).")  |
|:--:|
| *The `action` space is extended to **coordinate the `speed` and `steering` control**. While choosing the **acceleration**, the agent selects either the **reference path** or a path **proposed by the motion planner**. [Source](https://scholarbank.nus.edu.sg/handle/10635/126233).* |

Author: Wei L.

- The quote in this **PhD thesis**:
  - > "As far as the laws of mathematics refer to reality, they are not certain; and as far as they are certain, they do not refer to reality." - Albert Einstein, 1922
- Motivations / main ideas:
  - Propose an **interaction**-aware and **uncertainty**-aware decision making for **urban scenarios**.
  - Have a **general** and **scalable method**, with a focus on **real-time applicability**.
  - Cope with both `internal motion uncertainty` and `external situation uncertainty`.
  - Consider both `road context` and the obstacle vehicle's `motion intention` in the decision-making.
- About **_motion intention_** estimation:
  - To cope with the **lack of an `intention` sensor**.
  - > "The most popular strategy is to define the `motion intention` as their **hidden destinations**."
  - > "Instead of relying on the **hidden goals**, this study aims at employing the **obstacle vehicle’s** **_reaction_** to model the `motion intention`."
  - Compared to the **hidden goal method**, employing the **reactions** to model the `motion intention` is **more general and scalable**. The measurement of deviation is not restricted to any specific _region of interest_ (`ROI`).
  - _How to model the reaction?_
    - Using a **_deviation measurement_** of the observed vehicle state from the corresponding **reference vehicle state**.
    - For that purpose, some `reference state` containing the **reference speed `vref`** that the observed vehicle **should follow** is derived.
  - _What are the intentions for crossing in urban driving?_
    - Four intentions are abstracted from human driving behaviours and **evidences** are based on `speed deviation`:
    - `1-` **`Stopping` intention**: The `obstacle vehicle's speed` is close to zero, while the `reference speed` is much higher.
    - `2-` **`Hesitating` intention**: The `obstacle vehicle's speed` is lower than the `reference speed` but not close to zero.
    - `3-` **`Normal` intention**: The `obstacle vehicle's speed` matches well with the `reference speed`.
    - `4-` **`Aggressive` intention**: The `obstacle vehicle's current` speed is much higher than the `reference speed`.

- About the **`POMDP` formulation**:
  - The episode is not **episodic** but continuous: there is no termination `state` such as _"exit of intersection"_ in other approaches.
    - That makes this method very **general**, addressing `junctions`, `leader-following` and `round-about` scenarios with a **single `POMDP` formulation**.
    - _What is the discount rate?_
  - `state`:
    - It contains:
      - `1-` **Physical** (or _metric_) information: **`pose`** and **`speed`** (`x`, `y`, `θ`, `v`), which is assumed to be **fully observed**.
      - `2-` **Intention** information: `i`, which is **non-observable**. Obviously this is not applicable for the ego-vehicle.
    - The author explains that the **`reference state`** (containing the `reference speed`) should not be part of the `state`:
    - > "The **road context** can be employed as a **reference knowledge**, and is consequently **excluded from the vehicle state**".
  - `action`:
    - The action space is about **longitudinal control** and is made **discrete**: `accelerate`, `decelerate` or `maintain current speed`.
      - Values are set to **`±0.25 m/s2`** in simulation and **`±0.5 m/s2`** in the real-world experiment.
    - > "The `steering` control of the ego vehicle, on the other hand, is accomplished by tracking the reference routes closely."
    - The author proposes a modification to include the **lateral control** as well, e.g. for **`overtaking`** or **`lane-changing` manoeuvres** and a strategy to properly **coordinate the `speed` and `steering` control**.
      - > "An improperly extended action space can break down the **real-time applicability** of our situation-aware decision making algorithm. As an alternative, the **_trajectory selection_** strategy is employed in this study".
      - The `steering action` space includes **two trajectory candidates** to be selected: [`Commit`, `NewPlan`] and the agent decides whether or not the **trajectory planned by the motion planner** should be **committed** in each planning loop.
  - `transition model`:
    - The **behaviour** of other vehicles is driven by both the **road context** and their **motion intentions**.
    - An `action` about the acceleration is derived for each vehicle.
      - It depends on both the **motion intention** and the **correlations with the other vehicles** to avoid a potential collision with any other vehicles:
      - `1-` If the **collision risk** with other vehicles is low, the `action` is mapped from the **motion intention** only as `P`(`a`|`intention`).
      - `2-` Otherwise, the obstacle vehicle would either follow the **inferred motion intention** or choose the `decelerate` action: the higher the speed of the nearest vehicle, the higher the probability to decide to decelerate.
  - `observation model`:
    - > "Obstacle vehicles' observation functions are **more complicated** due to the **inclusion of their motion intentions**".
    - > "Given the `reference speed` vref(zi), the **observation function** for each `motion intention` **`ι`** `∈` `I` is modeled as a **Gaussian** with **mean `m(vref(zi), ι)`** and **covariance `vref(zi)/σ`**."
  - `reward function`:
    - The reward `R`(`a`, `s`) obtained by executing action `a` in state `s` balances the **driving efficiency** and **safety**.
    - _What is weighting between the different terms?_

- Another topic: learning the **road context** to perform **vehicle behaviour analysis**.
  - The **inference** over the **road context** is divided into three stages:
    - `1-` The topology learning to model the road network.
    - `2-` The motion learning to generalize the typical vehicle motion pattern.
    - `3-`The rule learning for traffic rules reasoning.
  - > "Given the **road context** and the **vehicle behavior analysis**, a **situation-aware** decision making algorithm was introduced to address the **situation uncertainty**."

- The corresponding paper: [Situation-aware decision making for autonomous driving on urban road using online POMDP](https://www.researchgate.net/publication/307904751_Situation-aware_Decision_Making_for_Autonomous_Driving_on_Urban_Road_using_Online_POMDP) by (Liu, Kim, Pendleton, & Ang, 2015).
  - It emphasises the need for a proper **environment representation**, integrating both the **road context** and the **obstacle vehicle's motion intention**, leading to the definition of **`Urban Road Situation`**:
    - > "The **road context** should comprise not only the **road network** and **traffic rules**, but also the **typical vehicle motion patterns**."
  - It also explains the **intention estimation** based on motion rather than **target destinations** (more _general_ and _scalable_, because the **measurement of deviation** is not restricted to any specific `ROI`):
    - > "The **motion intention** is inferred from the **vehicle reactions**, i.e., the **deviation** of the observed vehicle states to the reference vehicle behaviors [here the **`reference speed`** and transition probabilities between regions] that are represented by the **road context**."
    - > "The **urban road situation** (`URS`) features not only the **motion intention** and the **road context**, but also the **implicit correlations** between them. [...] The **motion intention** is abstracted as the **conditional distribution over the reactions**, where the implicit correlations between the **road context** and **motion intention** are acknowledged."
  - It also details the two most complicated components of the `POMDP` - the `transition` and `observation` models - and explains how to **learn reference behaviours** from data.
  - Finally, it compare the approach so to **classic rule-based policies**:
    - The `ROS`-based **[`Stage`](http://wiki.ros.org/stage) simulator** is employed for evaluation.
    - > [About _frozen robot_ problem] "While the **reactive approach** seems able to maintain a nice success rate, the **efficiency is scarified**. For most of the trials using the reactive approach, the ego vehicle always passively decided to wait, even when the gap between itself and the obstacle vehicles is large enough for safe merging."

</details>

---

**`"Probabilistic Decision-Making under Uncertainty for Autonomous Driving using Continuous POMDPs"`**

- **[** `2014` **]**
**[[:memo:](https://www.researchgate.net/publication/267040968_Probabilistic_Decision-Making_under_Uncertainty_for_Autonomous_Driving_using_Continuous_POMDPs)]**
**[** :mortar_board: `KIT` **]**

- **[** _`state representation`, `occlusion`, `offline solver`_ **]**

<details>
  <summary>Click to expand</summary>

| ![A **physical `transition model`** is defined for all vehicles: **p(`x'`** given **`x`, `a`)**. The **`ego-action`** is known. For the other road users, the **action is inferred / estimated**, using the **`situational context`** `Cj`, the map knowledge and an **estimated `planned route`** `Pj` (derived from the latter two). This `p`(`aj` given `cj`, `pj`) inference / estimation is motivated by the fact that other drivers usually follow the course of the road and **interact** with each other and the ego car. The authors introduce a **context-dependent** term for the **ego-action** too: `p`(`ae` given `a`, `ce`). It should account for **interaction with other road users** such as ''hold the velocity of the car in front''. Should not the **`policy`** be responsible for that?. [Source](https://www.researchgate.net/publication/267040968_Probabilistic_Decision-Making_under_Uncertainty_for_Autonomous_Driving_using_Continuous_POMDPs).](media/2014_brechtel_1.PNG "A **physical `transition model`** is defined for all vehicles: **p(`x'`** given **`x`, `a`)**. The **`ego-action`** is known. For the other road users, the **action is inferred / estimated**, using the **`situational context`** `Cj`, the map knowledge and an **estimated `planned route`** `Pj` (derived from the latter two). This `p`(`aj` given `cj`, `pj`) inference / estimation is motivated by the fact that other drivers usually follow the course of the road and **interact** with each other and the ego car. The authors introduce a **context-dependent** term for the **ego-action** too: `p`(`ae` given `a`, `ce`). It should account for **interaction with other road users** such as ''hold the velocity of the car in front''. Should not the **`policy`** be responsible for that?. [Source](https://www.researchgate.net/publication/267040968_Probabilistic_Decision-Making_under_Uncertainty_for_Autonomous_Driving_using_Continuous_POMDPs).")  |
|:--:|
| *A **physical `transition model`** is defined for all vehicles: **p(`x'`** given **`x`, `a`)**. The **`ego-action`** is known. For the other road users, the **action is inferred / estimated**, using the **`situational context`** `Cj`, the map knowledge and an **estimated `planned route`** `Pj` (derived from the latter two). This `p`(`aj` given `cj`, `pj`) inference / estimation is motivated by the fact that other drivers usually follow the course of the road and **interact** with each other and the ego car. The authors introduce a **context-dependent** term for the **ego-action** too: `p`(`ae` given `a`, `ce`). It should account for **interaction with other road users** such as ''hold the velocity of the car in front''. Should not the **`policy`** be responsible for that?. [Source](https://www.researchgate.net/publication/267040968_Probabilistic_Decision-Making_under_Uncertainty_for_Autonomous_Driving_using_Continuous_POMDPs).* |

| ![Up: **intersection** scenario, considering **noise in measurement** as well as **occlusion**. Bottom: Instead of **a priori (equidistant) state discretization**, the state representation is **learnt** offline, while performing value iteration. It offers a **more appropriate description** of the **current scene**, deducing what is relevant to the decision-making and what is not. For instance, state when the other car is far away can be **aggregated**, while **finer distinctions** are required when driving close.. [Source](https://www.researchgate.net/publication/267040968_Probabilistic_Decision-Making_under_Uncertainty_for_Autonomous_Driving_using_Continuous_POMDPs).](media/2014_brechtel_2.PNG "Up: **intersection** scenario, considering **noise in measurement** as well as **occlusion**. Bottom: Instead of **a priori (equidistant) state discretization**, the state representation is **learnt** offline, while performing value iteration. It offers a **more appropriate description** of the **current scene**, deducing what is relevant to the decision-making and what is not. For instance, state when the other car is far away can be **aggregated**, while **finer distinctions** are required when driving close.. [Source](https://www.researchgate.net/publication/267040968_Probabilistic_Decision-Making_under_Uncertainty_for_Autonomous_Driving_using_Continuous_POMDPs).")  |
|:--:|
| *Up: **intersection** scenario, considering **noise in measurement** as well as **occlusion**. Bottom: Instead of **a priori (equidistant) state discretization**, the state representation is **learnt** offline, while performing value iteration. It offers a **more appropriate description** of the **current scene**, deducing what is relevant to the decision-making and what is not. For instance, state when the other car is far away can be **aggregated**, while **finer distinctions** are required when driving close. [Source](https://www.researchgate.net/publication/267040968_Probabilistic_Decision-Making_under_Uncertainty_for_Autonomous_Driving_using_Continuous_POMDPs).* |

Authors: Brechtel, S., Gindele, T., & Dillmann, R.

- Motivations:
  - Adopt an **appropriate _state representation_** to solve a **`POMDP`**-formulated intersection problem.
  - **Equidistant discretization** or any **_a priori_** discretization of the `state` space are to be avoided.
    - Instead, the problem should be formulated and solved directly in its natural **continuous space**.
    - More precisely, the idea is to **Learn** a **discrete** representation of the **continuous `state` space** to solve the integrals in continuous POMDPs.

- `POMDP` formulation:
  - The ego-car interacts with **`2` other cars** at an **intersection**.
    - Each vehicle is described by `4` continuous variables: `speed`, global **`pose`** (`x`, `y`, `ψ`).
    - Leading to a **continuous `state`** of **size `12`**.
  - **Occlusions** are detected by checking if there is a **direct line of sight** to the road user that does not intersect with other objects, which are represented as polygons.
    - _To be honnest, I did not understand where the `occlusion` random variable is used/considered. The authors mention a probability of beeing occluded. But where/how is that tracked by the belief tracker?_
- **Learning** the state representation:
  - > [Idea is to] "Apply a **learning algorithm** during **value iteration** in order to only **distinguish states** if the discrimination is necessary to represent the optimal decision."
  - > "The **`value function`** serves as a mathematically sound basis to **deduce what is relevant to the decision-making and what is not**. [In discrete `POMDP`s, `state`s with same value can be **aggregated** without influencing the `policy`.]"
  - The **problem-specific _compression_** is refined during the solving process:
    - The `POMDP` solver learned a **decision tree** with **`442` leaves**.
    - **`4 million` continuous state samples**, collected from `1000` simulations, are used to learn the state representation, i.e. the **assignment of an index** to a state vector (`12` continuous values).
  - Plotting the **histogram of "situations per state index"**, it can be seen that:
    - Particles that **do need not be differentiated** can be **aggregated**.
    - **Refinements** are performed for states that **model critical situations**, where **accuracy** in the information is important.

- Some previous work by the authors:
  - **"Solving Continuous POMDPs: Value Iteration with Incremental Learning of an Efficient Space Representation"** - [(Brechtel, Gindele, & Dillmann, 2013)](http://proceedings.mlr.press/v28/brechtel13.pdf).
  - **"Probabilistic MDP-behavior planning for cars"** - [(Brechtel, Gindele, & Dillmann, 2011)](https://www.researchgate.net/publication/256079154_Probabilistic_MDP-Behavior_Planning_for_Cars).

- One quote I like:
  - > "The `process` [`transition`] and `observation` models and the `reward` function can be seen as the **_glue_** between the **spaces**, as they define the meaning of the spaces for the decision process."

</details>
