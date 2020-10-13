## `Inverse Reinforcement Learning` `Inverse Optimal Control` and `Game Theory`

---

**`"Deep Inverse Q-learning with Constraints"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2008.01712)]**
**[[🎞️](https://www.youtube.com/watch?v=5m21ibhWcXw)]**
**[** :mortar_board: `University of Freiburg` **]**
**[** :car: `BMW` **]**

- **[** _`constrained Q-learning`, `constrained imitation`, `Boltzmann distribution`, [`SUMO`](https://sumo.dlr.de/docs/index.html)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2008.01712).](../media/2020_kalweit_2.PNG "[Source](https://arxiv.org/abs/2008.01712).")  |
|:--:|
| *Left: both the `reward` and `Q`-function are estimated based on demonstrations and a **set of constraints**. Right: Comparing the expert, the **unconstrained (blue) and constrained (green) imitating agents** as well as a `RL` agent trained to optimize the **true `MDP`** with constraints (yellow). The **`constrained imitation`** can keep high speed while ensuring **no constrain violation**. Not to mention that it can **converge faster** than the `RL` agent (yellow). [Source](https://arxiv.org/abs/2008.01712).* |

| ![[Source](https://arxiv.org/abs/2008.01712).](../media/2020_kalweit_3.PNG "[Source](https://arxiv.org/abs/2008.01712).")  |
|:--:|
| *Compared to existing `IRL` approaches, the proposed methods can **enforce additional constraints** that were **not part of the original demonstrations**. And it does not requires solving the  **`MDP` multiple times**. [Source](https://arxiv.org/abs/2008.01712).* |

| ![[Source](https://arxiv.org/abs/2008.01712).](../media/2020_kalweit_4.PNG "[Source](https://arxiv.org/abs/2008.01712).")  |
|:--:|
| *Derivation for the **model-based** case (Inverse Action-value Iteration): The `IRL` problem is transformed to solving a **system of linear equations**. ((`3`) reminds me the `law of total probability`). The demonstrations are assumed to come from an **expert following a stochastic policy with an underlying `Boltzmann` distribution** over optimal `Q`-values (which enables working with `log`). With this formulation, it is possible to calculate a matching `reward` function for the observed (optimal) behaviour **analytically in closed-form**. An extension based on **sampling** is proposed for **model-free** problems. [Source](https://arxiv.org/abs/2008.01712).* |

| ![[Source](https://arxiv.org/abs/2008.01712).](../media/2020_kalweit_1.gif "[Source](https://arxiv.org/abs/2008.01712).")  |
|:--:|
| *While the **Expert** agent was not trained to include the `Keep Right` constraint (**US-highway** demonstrations), the **Deep Constrained Inverse Q-learning (`DCIQL`) agent** is satisfying the `Keep Right` (**German highway**) and `Safety` constraints while still imitating to overtake the other vehicles in an anticipatory manner. [Source](https://arxiv.org/abs/2008.01712).* |

Authors: Kalweit, G., Huegle, M., Werling, M., & Boedecker, J.

- Previous related works:
  - `1-` [`Interpretable multi time-scale constraints in model-free deep reinforcement learning for autonomous driving`](https://arxiv.org/abs/2003.09398), (Kalweit, Huegle, Werling, & Boedecker, 2020)
    - About **Constrained `Q`-learning**.
    - Constraints are considered at **different time scales**:
      - Traffic rules and constraints are ensured in predictable **short-term** horizon.
      - **Long-term** goals are optimized by **optimization** of long-term return: With an **expected sum of discounted or average `constraint` signals**.
  - `2-` [`Dynamic input for deep reinforcement learning in autonomous driving`](https://arxiv.org/abs/1907.10994), (Huegle, Kalweit, Mirchevska, Werling, & Boedecker, 2019)
    - A **`DQN` agent is learnt** with the following definitions and used as the **_expert_** to **produce the demonstrations**.
      - `simulator`: `SUMO`.
      - `state` representation: [`DeepSet`](https://arxiv.org/abs/1909.13582) to **model interactions** between an **arbitrary number of objects** or lanes.
      - `reward` function: minimize **deviation to some `desired speed`**.
      - `action` space: **high-level** manoeuvre in {`keep lane`, `perform left lane change`, `perform right lane change`}.
        - `speed` is controlled by low-level controller.
  - `2-` [`Off-policy multi-step q-learning`](https://arxiv.org/abs/1909.13518), (Kalweit, Huegle, & Boedecker, 2019)
    - Considering two methods inspired by **multi-step `TD`-learning** to enhance **data-efficiency** while remaining **off-policy**:
      - `(1)` **Truncated** `Q`-functions: representing the `return` for the first `n` steps of a policy rollout.
      - `(2)` **Shifted** `Q`-functions: acting as the **far-sighted `return`** after this **truncated rollout**.

- Motivations:
  - `1-` **Optimal _constrained_ imitation**.
    - The goal is to **imitate an expert** while **respecting constraints**, such as _traffic rules_, that **may be violated** by the expert.
      - > "`DCIQL` is able to guarantee satisfaction of constraints on the **long-term for optimal constrained imitation**, even if the **original demonstrations violate these constraints**."
    - For instance:
      - Imitate a driver observed on the **US highways**.
      - And **transfer** the policy to **German highways** by including a **`keep-right` constraint**.
  - `2-` Improve the **training efficiency** of `MaxEnt IRL` to offer **faster training convergence**.
    - > "Popular `MaxEnt IRL` approaches require the **computation of expected `state visitation` frequencies** for the optimal policy under an **estimate** of the `reward` function. This usually requires **intermediate `value` estimation** in the **inner loop** of the algorithm, slowing down convergence considerably."
    - > "One general limitation of `MaxEnt IRL` based methods, however, is that **the considered `MDP`** underlying the demonstrations **has to be solved MANY times** inside the **inner loop** of the algorithm."
    - The goal is here to **solve the `MDP` underlying the demonstrated behaviour once** to recover the expert policy.
      - > "Our approach needs to **solve the `MDP`** underlying the demonstrated behavior **only once**, leading to a **speedup of up to several orders** of magnitude compared to the popular **`Maximum Entropy IRL`** algorithm and some of its variants."
      - > "Compared to our **learned `reward`**, the agent **trained on the true `reward` function** has a **higher demand for training samples** and requires **more iterations** to achieve a well-performing policy. [...] Which we hypothesize to result from the **bootstrapping formulation** of `state-action` visitations in our `IRL` formulation, suggesting a strong link to **[successor features](https://www.gatsby.ucl.ac.uk/~dayan/papers/d93b.pdf)**."
  - `3-` Focus on **off-policy** `Q-learning`, where an optimal policy can be found on the basis of a **given transition set**.

- Core assumption.
  - The **expert** follows a **`Boltzmann` distribution** over **optimal `Q-values`**.
    - > "We assume a policy that only **maximizes the entropy** over actions **locally** at each step as an approximation."
- Outputs.
  - Each time, not only `r` but also **`Q` is estimated**, i.e. a `policy`.
    - Deriving the `policy` by such imitation seems faster than solving the `MDP` with the true `reward` function:
    - > "Compared to our **learned `reward`**, the agent trained on the **true `reward` function** has a **higher demand for training samples** and **requires more iterations** to achieve a well-performing policy."
- Two families:
  - `1-` **model-based**.
    - > "If the **observed `transitions`** are samples from the **true optimal `Boltzmann`** distribution, we can recover the **true `reward` function** of the `MDP` in **closed-form**. In case of an infinite control problem or if **no clear reverse topological order exists**, we solve the `MDP` by **iterating multiple times** until convergence."
    - If the **`transition` model is known**, the `IRL` problem is converted to a system of linear equations: it is possible to calculate a matching reward function for the observed (optimal) behaviour **analytically in closed-form**.
    - Hence named "Inverse Action-value **Iteration**" (`IAVI`).
    - > "Intuitively, this formulation of the **immediate `reward`** encodes the **local probability of action `a`** while also ensuring the probability of the **maximizing next action under `Q-learning`**. Hence, we note that this formulation of **bootstrapping visitation frequencies** bears a strong resemblance to the **Successor Feature Representation.**"

  - `2-` **model-free**.
    - > "To relax the **assumption** of an **existing `transition` model and `action` probabilities**, we extend `IAVI` to a **sampling-based** algorithm."
    - > "We extend `IAVI` to a **sampling based** approach using **stochastic approximation**, which we call **Inverse Q-learning (`IQL`)**, using **`Shifted` Q-functions** proposed in [[6]](https://arxiv.org/abs/1909.13518) to make the approach **model-free**.
      - The shifted `Q-value` `QSh`(`s`, `a`) **skips the immediate `reward`** for taking `a` in `s` and **only considers the discounted `Q-value` of the `next state`**.
    - `1-1.` **_Tabular_** Inverse Q-learning algorithm:
      - The `action` probabilities are approximated with **`state-action` visitation counter `ρ`(`s`, `a`)**.
      - > [`transition` model] "In order to avoid the need of a **model `M`** [in `η`(`a`, `s`)], we evaluate all other actions via **Shifted `Q-functions`.**"
    - `1-2.` Deep (**non-tabular**) `IQL`.
      - **Continuous `state`s** are now addressed.
      - The `reward` function is estimated with **function approximator `r(·, ·|θr)`**, parameterized by `θr`.
      - The **`Q`-function and `Shifted-Q` function** are also estimated with nets.
      - > "We approximate the **state-action visitation** by classifier **ρ(·, ·|`θρ`)**, parameterized by `θρ` and with linear output."
- _How to_ **_enforce_** _(additional)_ **_constraints_**_?_
  - In [one previous work](https://arxiv.org/abs/2003.09398), two sets of constraints were considered.
    - `1-` One about the **`action`** (`action masking`).
    - `2-` One about the **`policy`**, with **multi-step constraint signals** with **horizon**.
      - An **expected sum of discounted or average `constraint` signals** is estimated.
    - Here, only the **`action` set** is considered.
      - A set of **constraints functions** **C={`ci`}** is defined. Similar to the `reward` function, it considers (`s, a`): `ci`(`s`, `a`).
      - A **"safe" `action` set** can be defined based on some **threshold values** for each `ci`: `Safe`[`i`](`s`) = {`a ∈ A` | `ci(s, a)` ≤ `βci` }
    - In addition to the `Q`-function in `IQL`, a **constrained `Q`-function `QC`** is estimated.
      - > "For **policy extraction from `QC`** after `Q-learning`, only the `action-values` of the **constraint-satisfying actions** must be considered."
  - Including constraints directly in `IQL` leads to **optimal constrained imitation** from **unconstrained demonstrations**.

</details>

---

**`"Planning on the fast lane: Learning to interact using attention mechanisms in path integral inverse reinforcement learning"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2007.05798)]**
**[** :mortar_board: `TU Darmstadt` **]**
**[** :car: `Volkswagen` **]**

- **[** _`max-entropy`, `path integral`, `sampling`, `MPC`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1905.00229).](../media/2019_rosbach_1.PNG "[Source](https://arxiv.org/abs/1905.00229).")  |
|:--:|
| *About **`path integral IRL`** and how the **`partition function`** in the `MaxEnt-IRL` formulation is approximated via **sampling**. Note the need to integrate over **all possible trajectories** (`Π`) in the `partition function`. Besides, note the **transition model `M`** that produces the `features` but is also able to **generate the `next-state`**. Such a model-based **generation** is simple for **static scenes**, but can it work in **dynamic environments** (unknown system dynamics), where the **future is way more uncertain**? [Source](https://arxiv.org/abs/1905.00229).* |

| ![[Source](https://arxiv.org/abs/1905.00229).](../media/2019_rosbach_2.PNG "[Source](https://arxiv.org/abs/1905.00229).")  |
|:--:|
| *Top-left: **sampling-based** and 'general-purpose' (**non-hierarchical**) planner that **relies** on some **transition model**. Top-right: the **`features`** used and their learnt/hard-coded associated `weights`. It can be seen that the **weights are changing** depending on the **context** (`straight` / `curvy` road). Bottom: For every planning cycle, **a restricted set of demonstrations `ΠD`** is considered, which are **"geometrically" close** (c.f. `projection metric` that transfers the `actions` of a manual drive into the `state-action` space of the planning algorithm) to the **odometry record `ζ`** (not very clear to me). Also note the **`labelling` function** that assigns **categorical labels** to **transitions**, e.g., a label associated with `collision`. [Source](https://arxiv.org/abs/1905.00229).* |

| ![[Source](https://arxiv.org/abs/2007.05798).](../media/2020_rosbach_1.PNG "[Source](https://arxiv.org/abs/2007.05798).")  |
|:--:|
| *To ensure **temporally consistent** prediction, an analogy with the **temporal abstraction** of `HRL` is made. [Source](https://arxiv.org/abs/2007.05798).* |

| ![[Source](https://arxiv.org/abs/2007.05798).](../media/2020_rosbach_2.PNG "[Source](https://arxiv.org/abs/2007.05798).")  |
|:--:|
| *To **dynamically update** the `reward` function while ensuring **temporal consistency**, the deep `IRL` architecture is separated into a **`policy` attention** and a **`temporal` attention** mechanism. The first one **encodes** the **context** of a situation and should learns to **focus on collision-free policies** in the configuration space. It also helps for **dimension reduction**. The second one predicts a **mixture `reward` function** given a **history of `context` vectors**. [Source](https://arxiv.org/abs/2007.05798).* |

Authors: Rosbach, S., Li, X., Großjohann, S., Homoceanu, S., & Roth, S.

- Previous related works:
  - `0-` [`Planning Universal On-Road Driving Strategies for Automated Vehicles`](https://www.springer.com/gp/book/9783658219536), (Heinrich, 2018) and [`Optimizing a driving strategy by its sensor coverage of relevant environment information`](https://ieeexplore.ieee.org/document/7535423) (Heinrich, Stubbemann, & Rojas, 2016).
    - A **_"general-purpose"_** (i.e. **no `behavioural`/`local path` hierarchy**) planner.
  - `1-` [`Driving with Style: Inverse Reinforcement Learning in General-Purpose Planning for Automated Driving`](https://arxiv.org/abs/1905.00229), (Rosbach, James, Großjohann, Homoceanu, & Roth, 2019).
    - **Path integral** (`PI`) maximum entropy `IRL` method to **learn `reward` functions** for/with the **above planner**.
    - The **structure of the planner** is leveraged to compute the `state visitation`, enabling `MaxEnt-IRL` despite the **high-dimensional `state` space**.
  - `2-` [`Driving Style Encoder: Situational Reward Adaptation for General-Purpose Planning in Automated Driving`](https://arxiv.org/abs/1912.03509), (Rosbach et al., 2019).
    - Motivation: learn **situation-dependent** `reward` functions for the **planner**.
    - The (complex) **mapping** between the `situation` and the `weights` of the `reward function` is approximated by a `NN`.

- Motivations:
  - `1-` Automate the **tuning** of the `reward` function for a **_"general-purpose"_** (non hierarchical) planner, using **human driving demonstrations**.
    - The **absence of temporal abstraction** brings a constraint: a high-dimensional **`state` space** with continuous actions.
  - `2-` Be able to **update** the `reward` function **dynamically**, i.e. predict **situation-dependent** `reward` functions.
  - `3-` Predict **temporally-consistent** `reward` functions.
    - Since **"Non-_temporally-consistent_ `reward` functions"** `=>` **"Non-_persistent_ behaviours / interactions"**.

- About the **planner**.
  - It is **model-based**, i.e. relies on some `transition model` to performed a **forward search of `actions`** via **sampling**, starting from some **initial state `s0`**.
    - > "The algorithm is similar to **parallel breadth first search** [leveraging **GPU** and made efficient with **`pruning`**] and **forward value iteration**."
  - **Optimization under constraints**: The most promising sequence is selected based on some **`reward` function** while respecting **`kinematic` / `dynamic` constraints**.
    - > "The planner **explores** the subspace of **feasible policies `Π`** by **sampling `actions`** from a distribution **conditioned on vehicle `dynamics`** for each state `s`."
    - > "The final **driving policy** is selected based on the **policy value `V(π)`** and **model-based constraints**."
- _Why no_ **_hierarchical_** _planner, e.g. some_ **_tactical `manoeuvre` selection_** _above some_ **_operational local `trajectory` optimization_**_?_
  - > "`Behavior` planning becomes difficult in **complex and unforeseen driving situations** in which the behavior **fails to match predefined admissibility templates**."
  - > [**kinematic constraints**] "These **hierarchical** planning architectures suffer from **uncertain behavior planning** due to **insufficient knowledge about motion constraints**. As a result, a maneuver may either be **infeasible** due to over-estimation or discarded due to under-estimation of the **vehicle capabilities**."
  - One idea is instead to **sample** of a **large set of `actions`** that **respect `kinematic` constraints**. And then evaluate the candidates with some **`cost` / `reward` function**.
  - The **sequences** of sampled actions can **represent complex manoeuvres**, **implicitly** including multiple behaviours, e.g., `lane following`, `lane changes`, `swerving`, and `emergency stops`.
- Advantages of the **flat planning architecture** (no `planning`-task decomposition).
  - > "These **general-purpose planners** allow **`behavior`-aware motion planning** given a **SINGLE `reward` function**." _[Which would probably have to **be adapted** depending on the situation?]_
  - Also, it can become **more scalable** since it does not rely on **behaviour implementations**: it does not **decompose** the decision-making based on **behaviour templates** for instance.
    - _But, again,_ **_there will not be a `One-size-fits-all` function_**. _So now the challenge is to constantly adapt the `reward` function based on the situation_.
- About the `action` space.
  - **Time-continuous polynomial** functions:
    - **`Longitudinal` actions** described by **`velocity` profiles**, up to the `5th`-order.
    - **`Lateral` actions** described by **`wheel`** angle, up to the `3th`-order.

- About `IRL`.
  - `1- Idea.` Find the `reward` function **weights `θ`** that enable the optimal policy `π∗` to be **at least as good as the demonstrated policy**.
    - Issue: learning a `reward` function given an **_optimal_ policy** is ambiguous since **many `reward` functions** may lead to the same **_optimal_ policy**.
  - `2- Solution.` **Max-margin** classification.
    - Issue: it suffers from drawbacks in the case of **imperfect demonstrations**.
  - `3- Solution.` Use a **probabilistic** model. For instance maximize the **entropy** of the distribution on `state`-`actions` under the learned policy: `MaxEnt-IRL`.
    - > "It solves the **ambiguity of imperfect demonstrations** by recovering a **distribution** over potential reward functions while avoiding any bias."
    - _How to compute the **_gradient of the `entropy`_**_?_
      - Many use **state visitation calculation**, similar to **backward value iteration** in `RL`.
    - Issue: this is intractable in the **high-dimensional `state` space**.
      - > [Because **no `hierarchy` / `temporal abstraction`**] "Our desired driving style requires **high-resolution sampling** of **time-continuous actions**, which produces a **high-dimensional `state` space** representation."
  - `4- Solution.` Combines **search-based planning** with `MaxEnt-IRL`.
    - Solution: Use the **graph representation** of the planner (i.e. starting from `s0`, sampling `actions` and using a **`transition model`**) to **approximate the required empirical feature expectations** and to allow `MaxEnt-IRL`.
    - > "The **parallelism of the `action` sampling** of the **search-based planner** allows us to explore a **high-resolution state representation `St`** for each discrete planning horizon increment `t`."
    - > "Our sample-based planning methodology allows us to **approximate the `partition function`** similar to **Markov chain Monte Carlo** methods."
    - > "Due to the **high-resolution sampling of actions**, we ensure that there are policies that are **geometrically close to human-recorded odometry** and resemble human driving styles. The task of `IRL` is to find the **unknown reward function** that increases the **likelihood of these trajectories** to be considered as **optimal policies**."

- About the `features`.
  - The vector of `features` is generated by the **environment model `M`** at each step: **`f`**(`s`,`a`).
  - The **mapping** from the **complex `feature` space** to the `reward` is here **linear**.
    - The `reward` is computed by **weighting the `features` in a sum**: `r`(`s`,`a`) = **`f`**(`s`,`a`) . **`θ`**.
  - The **`feature path integral`** for a policy `π` is defined by `fi`(`π`) = −`Integral-over-t` [`γt.fi`(`st, at`)`.dt`].
    - The **path integral** is approximated by the **iterative execution of sampled `state-action` sets**.

- Why is it called **_"path integral"_** `MaxEnt-IRL`?
  - It builds on **[`Maximum entropy inverse reinforcement learning in continuous state spaces with path integrals`](https://www.ias.informatik.tu-darmstadt.de/uploads/Research/ICML2011/Aghasadeghi_Bretl_ICML_2011.pdf)**, (Aghasadeghi & Bretl, 2011).
    - > "Similar to (Aghasadeghi et al.), we **optimize** [maximization of the `log-likelihood` of the expert behavior] under the **constraint of matching the feature path integrals `fπ`** of the demonstration and **feature expectations** of the explored policies."
    - The **expected `PI` feature values** `Ep`(`π|θ`)[`fπ`] of the **policy set `Π`** should match the **empirical feature values `fˆΠD`** of the demonstrations for each planning cycle of the `MPC`.

- _How to adapt to_ **_continuously-changing_** objectives?_ I.e. learn **situation-dependent** reward functions.
  - > "The probabilistic model `p(π|θ)` that recovers a **single reward function** for the demonstrated trajectories **does not scale**."
  - > "The **tuned linear reward functions** do not **generalize** well over different situations as the **objectives change continuously**, e.g., the importance of **keeping to the lane center** in straight road segments while allowing deviations in curvy parts."
  - `Idea 1.` `PI-clustered IRL`: Consider that there are **`N` different reward functions**.
    - Reward functions (their `weights` for the **linear combination**) are computed for **each cluster**.
    - > "We utilize **Expectation Maximization (`EM`)** in `IRL`, where `βπDc` is the probability that a demonstration `πD` **belongs to a cluster `c`** [`E`-step], and `ψ`(`c`) is the estimated **prior probability** of a cluster `c` [`M`-step]."
  - `Idea 2.` Neural net as **function approximator**.
    - Input: `PI features` and `actions` of **sampled driving policies** of an `MPC`-based planner.
    - Output: a **set of linear `reward` function `weights`** for upcoming planning cycles: `reward-weights`(`t+1`) `≈` `net`[`Θ`](`fk`, `ak`).
    - Hence the net learns a representation of the `statics` and `kinematics` of the situation.
      - > "Previously **sampled driving policies** of the `MPC` are used as inputs to our neural network. The network learns a **representation of the driving situation** by matching distributions of **features and actions** to reward functions on the basis of the **maximum entropy principle**."
    - It uses **`1-d` convolutions** over trajectories. With ideas similar to [`PointNet`](https://arxiv.org/abs/1612.00593):
      - > "The **average pooling layers** are used for **dimensionality reduction** of the features. Since we use only **one-dimensional convolutional layers**, no relationship is established between **policies of a planning cycle** by then. These **inter-policy relationships** are established by a series of `8` fully-connected layers at the end."
  - During **inference**.
    - The `MPC` re-plans in discrete time-steps `k`
    - After receiving the `features` and `actions` of the latest planning cycle, the **neural network infers the new `reward` weights**.
    - > "To enable **smooth transitions** of the `reward` functions, we utilize a predefined **history size `h`** to calculate the empirical mean of **weights `θˆ`**. The **weights** hence obtained are used to **continuously re-parameterize** the planning algorithm for the subsequent planning cycle."

- _How to_ **_dynamically update_** _the `reward` function while enabling_ **_persistent behaviour_** _over an extended time horizon?_
  - > "Continuous reward function **switches** may result in **non-stationary behavior** over an extended planning horizon."
  - > "The interaction with **dynamic objects** requires an **extended planning horizon**, which requires **_sequential_** context modeling."
  - The reward functions for the **next planning cycle at time `t+1`** is predicted with a net. With two **attention** mechanisms:
  - `1-` **`Policy` (trajectory) attention** mechanism.
    - Generate a **low dimensional `context` vector** of the driving situation from `features` sampled-driving policies.
    - > "Inputs are a **set of planning _cycles_** each having a **set of _policies_**."
    - > "The **attention vector** essentially **filters non-human-like trajectories** from the policy encoder."
    - It also helps for **dimension reduction**.
      - > "The size of the policy set used to understand the spatio-temporal scene can be significantly reduced by **concentrating on relevant policies having a human-like driving style**. In this work, we use a **policy attention mechanism** to achieve this **dimension reduction** using a **situational `context` vector**."
      - > "The attention networks stand out, having **less parameters and a low-dimensional `context` vector** while **yielding similar performance** as compared to **larger neural network** architectures."
  - `2-` **`Temporal` attention** network (`TAN`) with **recurrent** layers.
    - Predict a **mixture `reward` function** given a **history of `context` vectors**.
    - > "We use this **context vector** in a **sequence model** to predict a **temporal `reward` function attention vector**."
    - > "This **`temporal` attention vector** allows for **stable `reward` transitions** for upcoming planning cycles of an `MPC`-based planner."
  - > "We are able to produce **stationary reward functions** if the driving task does not change while at the same time addressing **situation-dependent task switches** with rapid response by giving the **highest weight** to the reward prediction of the **last planning cycle**."

</details>

---

**`"Efficient Sampling-Based Maximum Entropy Inverse Reinforcement Learning with Application to Autonomous Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2006.13704)]**
**[** :mortar_board: `UC Berkeley` **]**

- **[** _`max-entropy`, `partition function`, `sampling`, [`INTERACTION`](http://interaction-dataset.com/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2006.13704).](../media/2020_wu_1.PNG "[Source](https://arxiv.org/abs/2006.13704).")  |
|:--:|
| *The **intractable partition `Z` function** of `Max-Entropy` method is approximated by a **sum of sampled trajectories**. [Source](https://arxiv.org/abs/2006.13704).* |

| ![[Source](https://arxiv.org/abs/2006.13704).](../media/2020_wu_2.PNG "[Source](https://arxiv.org/abs/2006.13704).")  |
|:--:|
| *Left: Prior knowledge is injected to make the sampled trajectories **feasible**, hence **improving the efficiency** of the `IRL` method. Middle: Along with `speed-desired_speed`, `long-acc`, `lat-acc` and `long-jerk`, two **interactions `features`** are considered. Bottom-right: **Sample re-distribution** is performed since generated samples are not necessarily **uniformly distributed in the selected feature space**. Top-right: The learned weights indicate that **humans care more about longitudinal accelerations** in both non-interactive and interactive scenarios. [Source](https://arxiv.org/abs/2006.13704).* |

Authors: Wu, Z., Sun, L., Zhan, W., Yang, C., & Tomizuka, M.

- Motivations:
  - `1-` The trajectories of the observed vehicles satisfy **car kinematics constraints**.
    - This **should be considered** while learning `reward` function.
  - `2-` **Uncertainties** exist in real traffic demonstrations.
    - The demonstrations in naturalistic driving data are **not necessarily optimal or near-optimal**, and the `IRL` algorithms should be compatible with such uncertainties.
    - `Max-Entropy` methods (**probabilistic**) can cope with this **sub-optimality**.
  - `3-` The approach should **converge quickly** to **scale** to problems with **large continuous-domain** applications with **long horizons**.
    - The critical part in **max-entropy `IRL`**: **_How to estimate the intractable partition `Z`_?**

- Some assumptions:
  - > "We do not consider scenarios where human drivers **change their `reward` functions** along the demonstrations."
  - > "We also do not specify the **diversity of `reward` functions** among different human drivers. Hence, the acquired `reward` function is **essentially an averaged result** defined on the demonstration set."

- _Why "sampling-based"?_
  - The **integral** of the **partition function** is approximated by a **sum over generated samples**.
    - It reminds me the **Monte Carlo** integration techniques.
    - The **sampled are not random**. Instead they are **feasible** and **represent** long-horizon trajectories, leveraging **prior knowledge** on **vehicle kinematics** and motion planning.
  - Efficiency:
    - `1-` Around `1 minute` to **generate all samples** for the entire training set.
    - `2-` The sampling process is **one-shot** in the algorithm through the training process (_do they mean that the set needs only to be created once?_).
  - Sample **Re-Distribution**.
    - > "The samples are not necessarily **uniformly distributed in the selected feature space**, which will cause **biased evaluation of probabilities**."
    - > "To address this problem, we propose to use **`Euclidean` distance** [_better metrics will be explored in future works_] in the **feature space** as a **similarity metric** for **re-distributing the samples**."
  - The sampling time of all trajectories is `∆t=0.1s`.

- Features:
  - `1-` **Non-interactive**: `speed` **deviation** to `desired_speed`, longitudinal and lateral `accelerations`, longitudinal `jerk`.
  - `2-` **Interactive**:
    - `future distance`: **minimum** spatial distance of **two interactive vehicles** within a predicted horizon `τ-predict` assuming that they are **maintaining their current speeds**.
    - `future interaction distance`: minimum distance between **their distances** to the **collision point**.
  - All are normalized in (`0, 1`).

- Metrics:
  - `1-` Deterministic: **feature deviation** from the ground truth.
  - `2-` Deterministic: **mean `Euclidean` distance** to the ground truth.
  - `3-` Probabilistic: the **likelihood** of the ground truth.

- Baselines:
  - They all are based on the principle of **maximum entropy**, but differ in the **estimation of `Z`**:
    - `1-` [Continuous-domain `IRL`](https://graphics.stanford.edu/projects/cioc/cioc.pdf) (`CIOC`).
      - `Z` is estimated in a continuous domain via **Laplace approximation**: the `reward` at an arbitrary trajectory `ξ˜` can be **approximated** by its **second-order Taylor expansion** at a demonstration trajectory `ˆξD`.
    - `2-` [Optimization-approximated `IRL`](http://ais.informatik.uni-freiburg.de/publications/papers/kuderer15icra.pdf) (`Opt-IRL`).
      - An optimal trajectory `ξopt` can be obtained by **minimizing the updated `reward`** function. Then, `Z` ≈ `exp`(`βR`(`θ`,`ξopt`)).
      - > "In the **forward problem** at each iteration, it directly **solves the optimization** problem and use the **optimal trajectories** to represent the **expected feature counts**."
    - `3-` [Guided cost learning](https://arxiv.org/abs/1603.00448) (`GCL`).
      - This one is not model-based: it **does not need manually crafted `features`**, but automatically learns features via neural networks.
      - It uses rollouts (**samples**) of the `policy` network to estimate `Z` in each iteration.
      - However, **all these samples must be re-generated** in every training iteration, while the proposed method **only needs to generate all samples once**.

</details>

---

**`"Analyzing the Suitability of Cost Functions for Explaining and Imitating Human Driving Behavior based on Inverse Reinforcement Learning"`**

- **[** `2020` **]**
**[[:memo:](https://ras.papercept.net/proceedings/ICRA20/0320.pdf)]**
**[** :mortar_board: `FZI`, `KIT`, `UC Berkeley` **]**

- **[** _`max-entropy`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://ras.papercept.net/proceedings/ICRA20/0320.pdf).](../media/2020_naumann_1.PNG "[Source](https://ras.papercept.net/proceedings/ICRA20/0320.pdf).")  |
|:--:|
| *Left: Definition of the **`features`** retrieved from **trajectory demonstrations** and the **`evaluation function`**. Right: `max-Entropy IRL` enable only requires **locally optimal** demonstrations because the `gradient` and `Hessian` of the reward function is only considered in **proximity of the demonstration**. Note that the `features` are only based on the `states`, while the `actions` remain disregarded. And that their approach assumes that the **`cost` function** is parameterized as a **linear combination of cost terms**. [Source](https://ras.papercept.net/proceedings/ICRA20/0320.pdf).* |

| ![[Source](https://ras.papercept.net/proceedings/ICRA20/0320.pdf).](../media/2020_naumann_2.PNG "[Source](https://ras.papercept.net/proceedings/ICRA20/0320.pdf).")  |
|:--:|
| *General **cost function structures** and commonly used **trajectory features**. Only one work considers `crossing` scenarios. To account for the `right of way` at intersections, the time that elapses between one vehicle leaving a conflict zone, i.e. an area where paths overlap, and another vehicle entering this zone, is considered: `tTZC` = `dsecond`/`vsecond`. Bottom: Due to the **similarity of the `variance`-`mean`-ratio** under different **`evaluation` functions**, the authors limit their experiments to the consideration of `sum[f(t)²]`, which is most used. [Source](https://ras.papercept.net/proceedings/ICRA20/0320.pdf).* |

Authors: Naumann, M., Sun, L., Zhan, W., & Tomizuka, M.

- Motivations:
  - `1-` Overview of trajectory `features` and `cost` structures.
  - `2-` About **demonstration selection**: _What are the requirements when_ **_entire trajectories are not available_** _and_ **_trajectory segments_** _must be used?_
    - _Not very clear to me._
    - > "**Bellman’s principle of optimality** states that parts of optimal decision chains are also optimal decisions. **Optimality**, however, **always refers to the entire decision chain**. [...] _Braking in front of a stop sign is only_ **_optimal_** _as soon as the stop sign is considered within the `horizon`._"
    - > "The key insight is that **selected segments** have to **end in a `timestep` that is _optimal_**, independent of the weights that are to be learned."
    - > "Assuming a **_non-negative_ cost** definition, this motivates the choice of **arbitrary trajectory segments** ending in a timestep `T` such that `cT−d+1` ... `cT+d` (depending on `xT−2d+1`...`xT+2d`) are zero, i.e. **optimal**, independent of `θ`."
    - > "While this **_constraint_** limits the approach to cost functions that **yield zero cost for some sections**, it also yields the meaningful assumption that humans are not driven by a permanent dissatisfaction through their entire journey, but reach desirable states from time to time."

- Miscellaneous: about `cost function` structures in related works:
  - **Trajectory `features`** can depend on:
    - `1-` A **single** trajectory only. They are based on **ego- `acceleration`, `speed` and `position`** for instance.
    - `2-` Trajectory **ensembles**. I.e. they describe quality of one trajectory with respect to the trajectories of **other traffic participants**. For instance `TTC`.
  - As most approaches did not focus on crossings, the **traffic rule features** were not used by the related approaches.
  - All approaches use a **convenience term** to prevent that **being at a full stop is an optimal** state with zero cost.
    - > "In order to prevent that being at a **full stop** is beneficial, **progress** towards the target must be rewarded, that is, `costs` must be added in case of little progress. This can be done by considering the **deviation from the `desired velocity`** or the **`speed limit`**, or via the **deviation from a `reference position`**."
    - > "For stop signs, similarly, the deviation from a complete stop, i.e. the driven velocity at the stop line, can be used as a feature."
  - All approaches incorporate both `smoothness` (longitudinal) and `curve comfort` (lateral).
  - The `lane geometry` is incorporated in the cost, unless it was already incorporated by using a **predefined path**.

- _What `feature` for `interaction` with others traffic participants?_
  - Simply **relative** `speeds` and `positions` (`gap`).
  - Most approaches assume that the **future trajectory** of others is known or provided by an **upstream prediction** module. The **effect of the ego vehicle** on the traffic participant can then be measured. For instance the **induced cost**, such as **`deceleration`**.
  - > "Other approaches do not rely on an **upstream prediction**, but incorporate the prediction of others into the planning by **optimizing a global cost functional**, which weights other traffic participants equally, or allows for more **egoistic behavior** based on a **`cooperation factor`**."

- Some findings when applying `IRL` on [`INTERACTION`](http://interaction-dataset.com/) dataset on three scenarios: `in-lane driving`, `right turn` and `stop`:
  - `1-` Among all scenarios, human drivers **weight `longitudinal acceleration` higher than `longitudinal jerks`**.
  - `2-` The weight for **`longitudinal` and `lateral acceleration` are similar** per scenario, such that neither seems to be preferred over the other. If implied by the scenario, as in the right turn, the weight decreases.
  - `3-` In the `right turn` scenario, the weight of the `lateral deviation` from the centerline is very large.
    - > "Rather than assuming that the `centerline` is especially important in turns, we hypothesize that a large weight on `d-cl` is necessary to **prefer turning over simply going straight**, which would cause less `acceleration` cost."
  - > "We found that the **key `features` and human preferences differ largely**, even in different single lane scenarios and disregarding interaction with other traffic participants."

</details>

---

**`"Modeling Human Driving Behavior through Generative Adversarial Imitation Learning"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2006.06412)]**
**[** :mortar_board: `Stanford` **]**

- **[** _`GAIL`, `PS-GAIL`, `RAIL`, `Burn-InfoGAIL`, [`NGSIM`](https://github.com/sisl/ngsim_env)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2006.06412).](../media/2020_bhattacharyya_3.PNG "[Source](https://arxiv.org/abs/2006.06412).")  |
|:--:|
| *Different variations of **Generative Adversarial Imitation Learning** (`GAIL`) are used to **model human drivers**. These **augmented `GAIL`-based models** capture many desirable properties of both **rule-based** (`IDM`+`MOBIL`) and **machine learning** (`BC` predicting single / multiple Gaussians) methods, while avoiding common pitfalls. [Source](https://arxiv.org/abs/2006.06412).* |

| ![[Source](https://arxiv.org/abs/2006.06412).](../media/2020_bhattacharyya_2.PNG "[Source](https://arxiv.org/abs/2006.06412).")  |
|:--:|
| *In **Reward Augmented Imitation Learning (`RAIL`)**, the imitation learning agent receives a **second source of `reward` signals** which is **hard-coded** to discourage undesirable driving behaviours. The reward can be either **`binary`**, receiving penalty when the collision actually occurs, or **`smoothed`**, via **increasing penalties** as it approaches an **undesirable event**. This should address the **credit assignment problem** in `RL`. [Source](https://arxiv.org/abs/2006.06412).* |

Authors: Bhattacharyya, R., Wulfe, B., Phillips, D., Kuefler, A., Morton, J., Senanayake, R., & Kochenderfer, M.

- Related work:
  - ["Application of Imitation Learning to Modeling Driver Behavior in Generalized Environments"](https://www.bernardlange.com/s/Application-of-Imitation-Learning-to-Modeling-Driver-Behavior-in-Generalized-Environments.pdf), (Lange & Brannon, 2019), detailed in this page too.
- Motivation: Derive realistic **models** of **human** drivers.
  - Example of applications: populate surrounding vehicles with **human-like behaviours** in the **simulation**, to learn a driving policy.

- Ingredients:
  - `1-` **Imitation learning** instead of `RL` since the **`cost` function is unknown**.
  - `2-` **`GAIL`** instead of `apprenticeship learning` to not **restrict the class of `cost` functions** and avoid computationally **expensive `RL` iterations**.
  - `3-` Some **variations of `GAIL`** to deal with the **specificities of driver modelling**.

- **Challenges** and solutions when **modelling the _driving_ task** as a **sequential decision-making** problem (`MDP` formulation):
  - `1-` **Continuous** `state` and `action` spaces. And **high dimensionality** of the `state` representation.
  - `2-` **Non-linearity** in the desired mapping from `states` to `actions`.
    - For instance, **large corrections** in `steering` are applied to avoid collisions caused by **small changes** in the current `state`.
    - Solution to `1-`+`2-`: Neural nets.
      - > "The **feedforward `MLP`** is limited in its ability to adequately address **partially observable environments**. [...] By **maintaining sufficient statistics of _past_ `observations`** in memory, **recurrent policies disambiguate** perceptually similar states by acting with respect to **histories** of, rather than **individual `observations`**."
      - **`GRU`** layers are used: **fewer parameters** and still good performances.
  - `3-` **Stochasticity**: humans may take **different `actions`** each time they encounter a given traffic scene.
    - Solution: Predicting a [Gaussian] **distribution** and **sampling** from it: `at` **`∼`** `πθ`(`at` | `st`).
  - `4-` The **underlying `cost` function** is **unknown**. Direct `RL` is not applicable.
    - Solution: **Learning from demonstrations** (**imitation learning**). E.g. `IRL`+`RL` or `BC`.
    - > "The goal is to **infer this human policy** from a dataset consisting of a sequence of (`state`, `action`) tuples."
  - `5-` **Interaction** between agents needs to be modelled, i.e. it is a **multi-agent problem**.
    - Solution: `GAIL` extension. A **parameter-sharing `GAIL`** (`PS-GAIL`) to tackle **multi-agent driver modelling**.
  - `6-` `GAIL` and `PS-GAIL` are **domain agnostic**, making it difficult to **encode specific knowledge** relevant to **driving** in the learning process.
    - Solution: `GAIL` extension. **Reward Augmented Imitation Learning** (`RAIL`).
  - `7-` The human demonstrations dataset is a **mixture** of **different driving styles**. I.e. human demonstrations are dependent upon **latent factors** that may not be captured by `GAIL`.
    - Solution: `GAIL` extension. **[`Burn-`]Information Maximizing `GAIL`** (`Burn-InfoGAIL`) to **disentangle the latent variability in demonstrations**.

- Issues with **behavioural cloning (`BC`)** (supervised version of `imitation learning`).
  - > "`BC` trains the policy on the **distribution of `states`** encountered by the expert. During testing, however, the policy acts within the environment for **long time horizons**, and **small errors** in the learned policy or **stochasticity** in the environment can cause the agent to **encounter a different distribution of `states`** from what it observed during **training**. This problem, referred to as **_covariate shift_**, generally results in the policy making **increasingly large errors** from which it **cannot recover**."
  - > "`BC` can be effective when a **large number of demonstrations are available**, but in many environments, it is not possible to obtain sufficient quantities of data."
  - Solutions to the **covariate shift** problem:
    - `1-` Dataset **Aggregation** (`DAgger`), assuming **access to an expert**.
    - `2-` Learn a **replacement for the `cost` function** that generalizes to unobserved `states`.
      - Inverse reinforcement learning (`IRL`) and `apprenticeship learning`.
      - > "The goal in **apprenticeship learning** is to find a policy that **performs no worse than the expert** under the true [unknown] `cost` function."

- Issues with `apprenticeship learning`:
  - A **class of cost functions** is used.
    - `1-` It is often defined as the span of a **set of basis functions** that must be **defined manually** (as opposed to **learned from the observations**).
    - `2-` This class may be **restricting**. I.e. **no guarantee** that the learning agent will perform no worse than the expert, and the agent can **fail at imitating** the expert.
    - > "There is no reason to assume that the **`cost` function of the human drivers** lies within a **small function class**. Instead, the `cost` function could be **quite complex**, which makes `GAIL` a suitable choice for **driver modeling**."
  - `3-` It generally involves running **`RL` repeatedly**, hence **large computational cost**.

- About **Generative Adversarial Imitation Learning** (`GAIL`):
  - Recommended video: [This CS285 lecture](https://www.youtube.com/watch?v=DP0SJrNgV60&list=PLkFD6_40KJIwhWJpGazJ9VSj9CFMkb79A&index=14) of **Sergey Levine**.
  - It is derived from an alternative approach to imitation learning called **Maximum Causal Entropy `IRL` (`MaxEntIRL`)**.
  - > "While `apprenticeship learning` attempts to find a policy that **performs at least as well as the expert** across `cost` functions, `MaxEntIRL` seeks a **`cost` function** for which the expert is **uniquely optimal**."
  - > "While existing `apprenticeship learning` formalisms used the `cost` function as the **descriptor of desirable behavior**, `GAIL` relies instead on the **divergence** between the **demonstration occupancy distribution** and the **learning agent’s occupancy distribution**."
  - Connections to `GAN`:
    - It performs **binary classification** of (`state`, `action`) pairs drawn from the **occupancy distributions** `ρπ` and `ρπE`.
    - > "Unlike `GANs`, `GAIL` considers the environment as a **black box**, and thus the **objective is not differentiable** with respect to the parameters of the policy. Therefore, **simultaneous gradient descent** [for `D` and `G`] is not suitable for solving the `GAIL` optimization objective."
    - > "Instead, **optimization** over the `GAIL` objective is performed by alternating between a **gradient step** to increase the **objective function** with respect to the **discriminator parameters** `D`, and a **Trust Region Policy Optimization** (`TRPO`) step (Schulman et al., 2015) to decrease the **objective function** with respect to the parameters `θ` of the **policy** `πθ`."

- Advantages of `GAIL`:
  - `1-` It removes the **restriction** that the `cost` belongs to a **highly limited class of functions**.
    - > "Instead allowing it to be learned using **expressive function approximators** such as neural networks".
  - `2-` It scales to **large `state` / `action` spaces** to work for practical problems.
    - `TRPO` for `GAIL` works with **direct policy search** as opposed to **finding intermediate value functions**.
  - > "`GAIL` proposes a **new cost function regularizer**. This regularizer allows scaling to **large state action spaces** and removes the requirement to **specify basis cost functions**."

- Three extensions of `GAIL` to account for the **specificities** of **driver modelling**.
  - `1-` **Parameter-Sharing `GAIL` (`PS-GAIL`)**.
    - Idea: account for the **multi-agent nature** of the problem resulting from the **interaction** between traffic participants.
    - "We formulate multi-agent driving as a **Markov game** (Littman, 1994) consisting of `M` agents and an **unknown `reward` function**."
    - It combines `GAIL` with **`PS-TRPO`**.
    - > "`PS-GAIL` training procedure **encourages stabler interactions** between agents, thereby making them **less likely to encounter extreme** or unlikely driving situations."
  - `2-` **Reward Augmented Imitation Learning (`RAIL`).**
    - Idea: **reward augmentation** during **training** to provide **domain knowledge**.
    - It helps to improve the **`state` space exploration** of the learning agent by **discouraging bad `states`** such as those that could potentially lead to **collisions**.
    - > "These include **penalties** for **`going off the road`**, **`braking hard`**, and **`colliding`** with other vehicles. All of these are **undesirable driving behaviors** and therefore should be **discouraged** in the learning agent."
    - Two kinds of penalties:
      - `2.1-` **Binary** penalty.
      - `2.2-` **Smoothed** penalty.
        - > "We hypothesize that **providing advanced warning** to the imitation learning agent in the form of **smaller, increasing penalties** as the agent approaches an event threshold will address the **credit assignment problem** in `RL`."
        - > "For off-road driving, we **linearly increase the penalty** from `0` to `R` when the vehicle is within `0.5m` of the edge of the road. For **hard braking**, we **linearly increase** the penalty from `0` to `R/2` when the acceleration is between `−2m/s2` and `−3m/s2`."
    - > "`PS-GAIL` and `RAIL` policies are **less likely to lead vehicles into collisions**, extreme decelerations, and off-road driving."
    - It looks like now a **combination of `cloning` and `RL`** now: the agent receives `rewards` for **imitating** the `actions` and gets **hard-coded** `rewards`/`penalties` defined by the human developer.
  - `3-` **Information Maximizing `GAIL` (`InfoGAIL`).**
    - Idea: assume that the expert policy is a **mixture of experts**.
    - > [**Different driving style** are present in the dataset] "**Aggressive drivers** will demonstrate significantly **different driving trajectories** as compared to **passive drivers**, even for the same road geometry and traffic scenario. To **uncover these latent factors of variation**, and learn policies that produce trajectories corresponding to these **latent factors**, `InfoGAIL` was proposed."
    - To ensure that the learned policy utilizes the **latent variable `z`** as much as possible, `InfoGAIL` tries to **enforce high `mutual information`** between `z` and the **`state-action` pairs** in the generated trajectory.
    - Extension: `Burn-InfoGAIL`.
      - Playback is used to **initialize** the ego vehicle: the **"`burn-in` demonstration"**.
      - > "If the policy is initialized from a `state` sampled at the end of a **demonstrator’s trajectory** (as is the case when initializing the ego vehicle from a human playback), the driving policy’s actions should be **consistent with the driver’s past behavior**."
      - > "To address this issue of **inconsistency** with real driving behavior, `Burn-InfoGAIL` was introduced, where a **policy must take over** where an expert demonstration trajectory ends."
    - When trained in a **simulator**, different parameterizations are possible, defining the **style `z`** of each car:
      - `Aggressive`: **High `speed` and large `acceleration`** + small `headway distances`.
      - `Speeder`: same but **large `headway distances`**.
      - `Passive`: **Low `speed` and `acceleration`** + large `headway distances`.
      - `Tailgating`: same but **small `headway distances`**.

- Experiments.
  - `NGSIM` dataset:
    - > "The trajectories were **smoothed** using an **extended Kalman filter** on a **bicycle model** and **projected to lanes** using centerlines extracted from the `NGSIM` roadway geometry file."
  - Metrics:
    - `1-` **Root Mean Square Error** (`RMSE`) metrics.
    - `2-` Metrics that **quantify** _undesirable_ traffic phenomena: `collisions`, `hard-braking`, and `offroad driving`.
  - Baselines:
    - `BC` with **single** or **mixture** Gaussian regression.
    - Rule-based controller: `IDM`+`MOBIL`.
      - > "A small amount of **noise** is added to both the lateral and longitudinal accelerations to make the **controller nondeterministic**."
  - Simulation:
    - > "The effectiveness of the resulting driving **policy trained using `GAIL`** in imitating human driving behavior is **assessed by validation in rollouts** conducted on the simulator."
  - Some results:
    - > [`GRU` helps `GAIL`, but not `BC`] "Thus, we find that **recurrence** by itself is insufficient for addressing the detrimental effects that **cascading errors** can have on `BC` policies."
    - > "Only `GAIL`-based policies (and of course `IDM`+`MOBIL`) **stay on the road** for extended stretches."

- Future work: _How to refine the integration modelling?_
  - > "Explicitly modeling the interaction between agents in a **centralized manner** through the use of **Graph Neural Networks**."

</details>

---

**`"Deep Reinforcement Learning for Human-Like Driving Policies in Collision Avoidance Tasks of Self-Driving Cars"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2006.04218)]**
**[** :mortar_board: `University of the Negev` **]**

- **[** _`data-driven reward`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2006.04218).](../media/2020_emuna_2.PNG "[Source](https://arxiv.org/abs/2006.04218).")  |
|:--:|
| *Note that the `state` variables are **normalized** in [`0`, `1`] or [`-1`, `1`] and that the **previous `actions`** are part of the **`state`**. Finally, both the **previous** and the **current** `observations` (only the current one for the `scans`) are included in the `state`, in order to **appreciate the temporal evolution**. [Source](https://arxiv.org/abs/2006.04218).* |

| ![[Source](https://arxiv.org/abs/2006.04218).](../media/2020_emuna_3.PNG "[Source](https://arxiv.org/abs/2006.04218).")  |
|:--:|
| *Left: `throttle` and `steering` actions are not predicted as **single scalars** but rather as **distributions**. In this case a **mixture of `3` Gaussian**, each of them parametrized by a `mean` and a `standard deviation`. **Weights** are also learnt. This enable modelling **multimodal distribution** and offers better **generalization** capabilities. Right: the `reward` function is designed to make the agent **imitate the expert driver's behaviour**. Therefore the differences in term of **mean `speed`** and **mean `track position`** between the **agent** and **expert** driver are penalized. The mean `speed` and `position` of the **expert** driver is obtained from the **learnt `GP` model**. It also contains a **non-learnable** part: penalties for `collision` and `action changes` are **independent of human driver observations**. [Source](https://arxiv.org/abs/2006.04218).* |

| ![[Source](https://arxiv.org/abs/2006.04218).](../media/2020_emuna_1.PNG "[Source](https://arxiv.org/abs/2006.04218).")  |
|:--:|
| *Human **`speeds` and `lateral` positions** on the track are recorded and **modelled using a `GP` regression**. It is used to define the `human-like` behaviours in the **`reward`** function (instead of `IRL`) as well as for **comparison** during test. [Source](https://arxiv.org/abs/2006.04218).* |

Authors: Emuna, R., Borowsky, A., & Biess, A.

- Motivations:
  - Learn **human-like** behaviours via `RL` without **traditional `IRL`**.
  - `Imitation` should be considered in term of **`mean`** but also **in term of `variability`**.
- Main idea: **hybrid (`rule-based` and `data-driven`) reward shaping.**
  - The idea is to **build a model** based on **observation of human behaviours**.
    - In this case a **Gaussian Process** (`GP`) describes the **distribution of `speed` and `lateral position`** along a track.
  - **Deviations** from these learnt parameters are then **penalized** in the **`reward` function**.
  - Two variants are defined:
    - `1-` The `reward` function is **fixed**, using the **`means`** of the **two `GPs`** are reference `speeds` and `positions`.
    - `2-` The `reward` function **varies** by **sampling each time a trajectory** from the learnt `GP` models and using its values are **reference `speeds` and `positions`**.
      - The goal here is not only to **imitate `mean` human behaviour** but to recover also the **variability in human driving**.
  - > "Track `position` was **recovered better than `speed`** and we concluded that the latter is related to an agent acting in a **partially observable** environment."
  - Note that the **weights** of feature in the `reward` function stay **arbitrary** (they are not learnt, contrary to `IRL`).
- About the **dynamic batch update**.
  - > "To improve **exploration** and avoid **early termination**, we used [`reference state initialization`](https://arxiv.org/abs/1804.02717). We initialized the `speed` by **sampling from a uniform distribution** between `30` to `90km/h`. **High variability** in the policy at the **beginning** of training caused the agent to **terminate after a few number of steps** (`30-40`). A **full round** of the track required about `2000` steps. To improve learning we implemented a **dynamic batch size** that **grows with the agent’s performance**."

</details>

---

**`"Reinforcement Learning with Iterative Reasoning for Merging in Dense Traffic"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2005.11895)]**
**[** :mortar_board: `Stanford` **]**
**[** :car: `Honda`, `Toyota` **]**

- **[** _`curriculum learning`, `level-k reasoning`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2005.11895).](../media/2020_bouton_3.PNG "[Source](https://arxiv.org/abs/2005.11895).")  |
|:--:|
| *`Curriculum learning`: the `RL` agent solves `MDP`s with **iteratively increasing complexity**. At each step of the **curriculum**, the behaviour of the cars in the environment is **sampled from the previously learnt `k-levels`**. Bottom left: **`3` or `4` iterations** seem to be enough and larger **reasoning levels** might not be needed for this merging task. [Source](https://arxiv.org/abs/2005.11895).* |

Authors: Bouton, M., Nakhaei, A., Isele, D., Fujimura, K., & Kochenderfer, M. J.

- Motivations:
  - `1-` Training `RL` agents **more efficiently** for **complex traffic scenarios**.
    - The goal is to avoid standard **issues with `RL`**: `sparse rewards`, `delayed rewards`, and `generalization`.
    - Here the agent should **merge in dense traffic**, requiring **interaction**.
  - `2-` Cope with **dense** scenarios.
    - > "The lane change model `MOBIL` which is at the core of this **rule-based policy** has been designed for **SPARSE** traffic conditions _[and performs poorly in comparison]_."
  - `3-` Learn a **robust** policy, able to deal with **various behaviours**.
    - Here learning is done iteratively, as the **reasoning level** increases, the learning agent is exposed to a **larger variety of behaviours**.
  - Ingredients:
    - > "Our **training curriculum** relies on the `level-k` **cognitive hierarchy model** from **behavioral game theory**".
- About **`k-level`** and game theory:
  - > "This model consists in assuming that **an agent performs a limited number of iterations** of strategic reasoning: **_(“I think that you think that I think”)_**."
  - A **level-`k`** agent acts **optimally** against the strategy of a **level-`(k-1)`** agent.
  - The **level-`0`** is not learnt but uses an `IDM` + `MOBIL` hand-engineered **rule-based** policy.
- About **curriculum learning**:
  - The idea is to **iteratively increase the complexity** of the problem. Here increase the **diversity** and the **optimality** of the surrounding cars.
  - Each **cognitive level** is trained in a `RL` environment populated with vehicles of **any lower cognitive level**.
    - > "We then train a level-`3` agent by populating the top lane with level-`0` and level-`2` agents and the bottom lane with level-`0` or level-`1` agents."
    - > "Note that a **level-`1`** policy corresponds to a **standard `RL` procedure** [no further iteration]."
  - Each learnt policy is learnt with `DQN`.
    - To accelerate training at each time step, the authors **re-use the weights** from the previous iteration to start training.
- `MDP` formulation.
  - Actually, **two policies** are learnt:
    - Policies `1`, `3`, and `5`: **change-lane** agents.
    - Policies `2` and `4`: **keep-lane** agents.
  - `action`
    - > "The learned policy is intended to be **high level**. At deployment, we expect the agent to decide on a **desired speed** (`0 m/s`, `3 m/s`, `5 m/s`) and a **lane change command** while a **lower lever controller**, operating at **higher frequency**, is responsible for executing the motion and **triggering emergency braking system** if needed."
    - Simulation runs at `10Hz` but the agent takes an action **every five simulation steps**: **`0.5 s`** between **two actions**.
    - The authors chose **high-level** `action`s and to **rely on `IDM`**:
      - > "By using the `IDM` rule to **compute the acceleration**, the behavior of **braking if there is a car in front** will not have to be learned."
      - > "The **longitudinal action space** is **_safe_** by design. This can be thought of as a **form of shield** to the `RL` agent from **taking unsafe actions**."
        - _Well, all learnt agent exhibit_ **_at least `2%` collision rate_** _??_
  - `state`
    - Relative `pose` and `speed` of the **`8` closest** surrounding vehicles.
    - **Full observability** is assumed.
      - > "**Measurement uncertainty** can be **handled online (after training)** using the **`QMDP`** approximation technique".
  - `reward`
    - Penalty for **collisions**: `−1`.
    - Penalty for deviating from a **desired velocity**: `−0.001|v-ego − v-desired|`.
    - Reward for being in the **top lane**: `+0.01` for the _merging-agent_ and `0` for the _keep-lane_ agent.
    - Reward for **success** (passing the blocked vehicle): `+1`.

</details>

---

**`"Using Counterfactual Reasoning and Reinforcement Learning for Decision-Making in Autonomous Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.11919)]**
**[** :mortar_board: `Technische Universität München` **]**
**[** :car: `fortiss` **]**
**[[:octocat:](https://github.com/bark-simulator/bark)]**

- **[** _`counterfactual reasoning`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.11919).](../media/2020_hart_1.PNG "[Source](https://arxiv.org/abs/2003.11919).")  |
|:--:|
| *The idea is to first `train` the agent interacting with **different driver models**. This should lead to a **more robust** policy. During `inference` the possible outcomes are **first evaluated**. If too many **predictions** result in collisions, a **non-learnt controller** takes over. Otherwise, the **learnt policy** is executed. [Source](https://arxiv.org/abs/2003.11919).* |

Authors: Hart, P., & Knoll, A.

- Motivations:
  - Cope with the **behavioural uncertainties** of other traffic participants.
- The idea is to perform **predictions** considering **multiple** interacting **driver models**.
  - `1-` During `training`: expose **multiple behaviour models**.
    - The **parametrized model `IDM`** is used to describe more **passive** or **aggressive** drivers.
    - Model-free `RL` is used. The **diversity** of driver models should improve the **robustness**.
  - `2-` During `application`: at each step, the learned policy is first **evaluated before being executed**.
    - The **evolution** of the present scene is **simulated** using the different driver models.
    - The **outcomes** are then aggregated:
      - `1-` **Collision rate**.
      - `2-` **Success rate** (reaching the destination).
      - Based on these _risk_ and _performance_ metrics, the **policy is applied or not**.
        - If the collision rate is too high, then the ego vehicle **stays on its current lane**, controlled by `IDM`.
        - > "Choosing the **thresholds** is nontrivial as this could lead to too passive or risky behaviors."
    - It could be seen as some **_prediction-based `action masking`_**.
    - These **multi-modal predictions** make me also think of the **roll-out** phase in **tree searches**.
    - Besides it reminds me the concept of **`concurrent MDP`**, where the agent tries to infer in which `MDP` (parametrized) it has been placed.
- Not clear to me:
  - _Why not doing_ **_planning_** if you explicitly know the **`transition`** (`IMD`) and the `reward` models? It would substantially increase the **sampling efficiency**.
- About the simulator:
  - **[`BARK`](https://github.com/bark-simulator/bark)** standing for **_Behavior benchmARKing_** and developed at [`fortiss`](https://www.fortiss.org/en/research/fields-of-research).

- About **_"counterfactual reasoning"_**:
  - From [wikipedia](https://en.wikipedia.org/wiki/Counterfactual_thinking): "Counterfactual thinking is, as it states: **'counter to the facts'**. These thoughts consist of the _'What if?'_ ..."
  - > "We use **_causal_ counterfactual reasoning**: [...] **sampling behaviors** from a model pool for other traffic participants can be seen as **assigning nonactual behaviors** to other traffic participants.

</details>

---

**`"Modeling pedestrian-cyclist interactions in shared space using inverse reinforcement learning"`**

- **[** `2020` **]**
**[[:memo:](https://www.researchgate.net/publication/339568981_Modeling_pedestrian-cyclist_interactions_in_shared_space_using_inverse_reinforcement_learning)]**
**[** :mortar_board: `University of British Columbia, Vancouver` **]**
- **[** _`max-entropy`, `feature matching`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://www.researchgate.net/publication/339568981_Modeling_pedestrian-cyclist_interactions_in_shared_space_using_inverse_reinforcement_learning).](../media/2020_alsaleh_1.PNG "[Source](https://www.researchgate.net/publication/339568981_Modeling_pedestrian-cyclist_interactions_in_shared_space_using_inverse_reinforcement_learning).")  |
|:--:|
| *Left: The contribution of each `feature` in the **linear `reward` model** differs between the **Maximum Entropy** (`ME`) and the **Feature Matching** (`FM`) algorithms. The `FM` algorithm is **inconsistent across levels** and has a **higher `intercept to parameter weight ratio`** compared with the estimated weights using the `ME`. Besides, why does it penalize all lateral distances and all speeds in these `overtaking` scenarios? Right: good idea how to visualize `reward` function for `state` of **dimension `5`**. [Source](https://www.researchgate.net/publication/339568981_Modeling_pedestrian-cyclist_interactions_in_shared_space_using_inverse_reinforcement_learning).* |

Authors: Alsaleh, R., & Sayed, T.

- In short: A simple but good illustration of `IRL` concepts using **Maximum Entropy** (`ME`) and **Feature Matching** (`FM`) algorithms.
  - It reminds me some experiments I talk about in this video: ["From RL to Inverse Reinforcement Learning: Intuitions, Concepts + Applications to Autonomous Driving"](https://www.youtube.com/watch?v=wBfd2Kn-IgU).
- Motivations, here:
  - `1-` Work in **non-motorized shared spaces**, in this case a _cyclist-pedestrian_ zone.
    - It means **high degrees of freedom** in motions for all participants.
    - And offers **complex road-user interactions** (behaviours different than on conventional streets).
  - `2-` Model the behaviour of cyclists in this share space using **_agent-based_** modelling.
    - _`agent-based`_ as opposed to **physics-based prediction** models such as `social force model` (`SFM`) or `cellular automata` (`CA`).
    - The agent is trying to maximize an unknown **`reward` function**.
    - The **recovery of that reward function** is the core of the paper.
- First, **`2` interaction types** are considered:
  - The cyclist `following` the pedestrian.
  - The cyclist `overtaking` the pedestrian.
  - This **distinction** avoids the search for a _1-size-fits-all_ model.
- About the `MDP`:
  - The **_cyclist_** is the **`agent`**.
  - **`state`** (absolute for the _cyclist_ or relative compared to the _pedestrian_):
    - `longitudinal distance`
    - `lateral distance`
    - `angle difference`
    - `speed difference`
    - `cyclist speed`
  - `state` discretization:
    - > "Discretized for each interaction type by dividing **each state feature into `6` levels** based on **equal frequency observation** in each level."
    - This **non-constant bin-width** partially addresses the **imbalanced dataset**.
    - `6^5` = `7776` states.
  - `action`:
    - `acceleration`
    - `yaw rate`
  - `action` discretization:
    - > "Dividing the acceleration into five levels based on **equal frequency observation** in each level."
    - `5^2` = `25` actions.
  - `discount factor`:
    - > "A discount factor of `0.975` is used assuming **`10%` effect of the reward at a state `3 sec` later** (`90` time steps) from the current state."
- About the dataset.
  - Videos of two streets in Vancouver, for a total of `39` hours.
  - `228` cyclist and `276` pedestrian trajectories are extracted.
- `IRL`.
  - The two methods assume that the `reward` is a **linear combination of `features`**. Here `features` are `state` components.
  - `1-` **Feature Matching** (`FM`).
    - It **matches the feature counts** of the expert trajectories.
    - The authors do not details the `max-margin` part of the algorithm.
  - `2-` **Maximum Entropy** (`ME`).
    - It estimates the `reward` function parameters by **maximizing the likelihood of the expert demonstrations** under the **maximum entropy distribution**.
    - Being probabilistic, it can account for **non-optimal observed behaviours**.
- The recovered reward model can be used for **prediction** - _How to measure the similarity between two trajectories?_
  - `1-` `Mean Absolute Error` (`MAE`).
    - It compares elements of same indices in the two sequences.
  - `2-` [`Hausdorff Distance`](https://en.wikipedia.org/wiki/Hausdorff_distance).
    - > "It computes the largest distance between the simulated and the true trajectories while **ignoring the time step alignment**".
- Current limitations:
  - **`1-to-1` interactions**, i.e. a **single pedestrian/cyclist pair**.
  - **Low-density** scenarios.
  - > "[in future works] **neighbor condition** (i.e. other pedestrians and cyclists) and **shared space density** can be explicitly considered in the model."

</details>

---

**`"Accelerated Inverse Reinforcement Learning with Randomly Pre-sampled Policies for Autonomous Driving Reward Design"`**

- **[** `2019` **]**
**[[:memo:](https://www.researchgate.net/publication/337624367_Accelerated_Inverse_Reinforcement_Learning_with_Randomly_Pre-sampled_Policies_for_Autonomous_Driving_Reward_Design)]**
**[** :mortar_board: `UC Berkeley`, `Tsinghua University, Beijin` **]**
- **[** _`max-entropy`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://www.researchgate.net/publication/337624367_Accelerated_Inverse_Reinforcement_Learning_with_Randomly_Pre-sampled_Policies_for_Autonomous_Driving_Reward_Design).](../media/2019_xin_1.PNG "[Source](https://www.researchgate.net/publication/337624367_Accelerated_Inverse_Reinforcement_Learning_with_Randomly_Pre-sampled_Policies_for_Autonomous_Driving_Reward_Design).")  |
|:--:|
| *Instead of the **costly `RL` optimisation** step at each iteration of the **vanilla `IRL`**, the idea is to **randomly sample a massive of policies in advance** and then to **pick one** of them as the optimal policy. In case the **sampled policy set does not contain** the optimal policy, **exploration** of policy is introduced as well for supplement. [Source](https://www.researchgate.net/publication/337624367_Accelerated_Inverse_Reinforcement_Learning_with_Randomly_Pre-sampled_Policies_for_Autonomous_Driving_Reward_Design).* |

| ![[Source](https://www.researchgate.net/publication/337624367_Accelerated_Inverse_Reinforcement_Learning_with_Randomly_Pre-sampled_Policies_for_Autonomous_Driving_Reward_Design).](../media/2019_xin_2.PNG "[Source](https://www.researchgate.net/publication/337624367_Accelerated_Inverse_Reinforcement_Learning_with_Randomly_Pre-sampled_Policies_for_Autonomous_Driving_Reward_Design).")  |
|:--:|
| *The **approximation** used in [**Kuderer et al. (2015)**](http://ais.informatik.uni-freiburg.de/publications/papers/kuderer15icra.pdf) is applied here to compute the second term of gradient about the **expected feature values**. [Source](https://www.researchgate.net/publication/337624367_Accelerated_Inverse_Reinforcement_Learning_with_Randomly_Pre-sampled_Policies_for_Autonomous_Driving_Reward_Design).* |

Authors: Xin, L., Li, S. E., Wang, P., Cao, W., Nie, B., Chan, C., & Cheng, B.

- Reminder: Goal of `IRL` = **Recover the reward function** of an expert from demonstrations (here _trajectories_).
- Motivations, here:
  - `1-` Improve the **efficiency** of **"weights updating"** in the **iterative** routine of `IRL`.
    - More precisely: generating **optimal policy** using `model-free RL` suffers from **low sampling efficiency** and should therefore be avoided.
    - Hence the term **_"accelerated"_** **`IRL`**.
  - `2-` Embed **human knowledge** where restricting the **search space** (policy space).
- One idea: **"Pre-designed policy subspace"**.
  - > "An intuitive idea is to **randomly sample a massive of policies in advance** and then to **pick** one of them as the **optimal policy** instead of finding it via `RL` optimisation."
- _How to construct the_ **_policies sub-space_**_?_
  - **Human knowledge** about vehicle controllers is used.
  - Parametrized **linear controllers** are implemented:
    - **`acc`** = `K1`*`∆d` + `K2`*`∆v` + `K3`*`∆a`, where `∆` are **relative to the leading vehicle**.
    - By **sampling** tuples of **<`K1`, `K2`, `K3`> coefficients**, `1 million` (_candidates_) policies are generated to **form the sub-space**.
- Section about `Max-Entropy IRL` (_btw. very well explained, as for the section introducing `IRL`_):
  - > "Ziebart et al. (2008) employed the principle of `maximum entropy` to **resolve ambiguities** in choosing trajectory distributions. This principle **maximizes the uncertainty** and leads to the **distribution** over behaviors **constrained to matching feature expectations**, while being **no more committed** to any particular trajectory than this constraint requires".
  - > "**Maximizing the entropy of the distribution over trajectories** subject to the feature constraints from expert’s trajectories implies to **maximize the likelihood** under the maximum entropy (exponential family) distributions. The problem is **convex for `MDPs`** and the optima can be obtained using **gradient-based optimization** methods".
  - > "The **gradient** [of the Lagrangian] is the difference between **empirical feature expectations** and the **learners expected feature expectations**."
- _How to compute the_ *_second term_* _of this gradient?_
  - It implies **integrating over all possible trajectories**, which is infeasible.
  - As [Kuderer et al. (2015)](http://ais.informatik.uni-freiburg.de/publications/papers/kuderer15icra.pdf), one can compute the feature values of the **most likely trajectory** as an **approximation of the feature expectation**.
  - > "With this approximation, only the **optimal trajectory** associated to the **optimal policy** is needed, in contrast to regarding the **generated trajectories** as a **probability distribution**."
- About the **features**.
  - As noted in my [experiments about `IRL`](https://www.youtube.com/watch?v=wBfd2Kn-IgU), they serve **two purposes** (in `feature-matching`-based `IRL` methods):
    - `1-` In the **reward function**: they should represent _"things we want"_ and _"things we do not want"_.
    - `2-` In the **feature-match**: to **compare two policies** based on their sampled trajectories, they should **capture relevant properties** of driving behaviours.
  - Three features for this _longitudinal acceleration_ task:
    - `front-veh time headway`.
    - `long. acc`.
    - `deviation to speed limit`.
- _Who was the expert?_
  - Expert followed a **modified linear car-following** (`MLCF`) model.
- Results.
  - **Iterations** are stopped after `11` loops.
  - It would have been interesting for **comparison** to test a _"classic"_ `IRL` method where `RL` optimizations are applied.

</details>

---

**`"Jointly Learnable Behavior and Trajectory Planning for Self-Driving Vehicles"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1910.04586)]**
**[[🎞️](https://youtu.be/j8qypWV2L2g?t=1610)]**
**[** :car: `Uber` **]**
- **[** _`max-margin`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1910.04586).](../media/2019_sadat_1.PNG "[Source](https://arxiv.org/abs/1910.04586).")  |
|:--:|
| *Both `behavioural planner` and `trajectory optimizer` share the same cost function, whose **weigth parameters are learnt from demonstration**. [Source](https://arxiv.org/abs/1910.04586).* |

Authors: Sadat, A., Ren, M., Pokrovsky, A., Lin, Y., Yumer, E., & Urtasun, R.

- Main motivation:
  - Design a decision module where both the **behavioural planner** and the **trajectory optimizer** share the **same objective** (i.e. cost function).
  - Therefore **_"joint"_**.
  - > "[In approaches not-joint approaches] the final trajectory outputted by the **trajectory planner** might differ significantly from the one generated by the **behavior planner**, as they do not share the **same objective**".
- Requirements:
  - `1-` Avoid time-consuming, error-prone, and iterative **hand-tuning** of cost parameters.
    - E.g. Learning-based approaches (`BC`).
  - `2-` Offer **interpretability** about the costs **jointly imposed** on these modules.
    - E.g. Traditional modular `2`-stage approaches.
- About the structure:
  - The **driving scene** is described in `W` (`desired route`, `ego-state`, `map`, and `detected objects`). _Probably `W` for "World"_?
  - The **behavioural planner** (`BP`) decides two things based on `W`:
    - `1-` A high-level **behaviour** `b`.
      - The path to converge to, based on one chosen manoeuvre: `keep-lane`, `left-lane-change`, or `right-lane-change`.
      - The `left` and `right` **lane boundaries**.
      - The **obstacle `side assignment`**: whether an obstacle should stay in the `front`, `back`, `left`, or `right` to the ego-car.
    - `2-` A coarse-level **trajectory** `τ`.
    - The loss has also a **regularization term**.
    - This decision is "simply" the `argmin` of the shared cost-function, obtained by **sampling** + **selecting the best**.
  - The **"trajectory optimizer"** refines `τ` based on the **constraints** imposed by `b`.
    - E.g. an **overlap cost** will be incurred if the `side assignment` of `b` is violated.
  - A **cost function** parametrized by `w` assesses the quality of the selected <`b`, `τ`> pair:
    - **`cost`** = **`w^T`** **.** **`sub-costs-vec`(`τ`, `b`, `W`)**.
    - Sub-costs relate to _safety_, _comfort_, _feasibility_, _mission completion_, and _traffic rules_.
- _Why_ **_"learnable"_**_?_
  - Because the **weight vector `w`** that captures the **importance of each sub-cost** is learnt based on human demonstrations.
    - > "Our planner can be **trained jointly end-to-end** without requiring **manual tuning** of the costs functions".
  - They are **two losses** for that objective:
    - `1-` **Imitation loss** (with `MSE`).
      - It applies on the <`b`, `τ`> produced by the **`BP`**.
    - `2-` **Max-margin loss** to penalize trajectories that have **small cost** and are different from the human driving trajectory.
      - It applies on the <`τ`> produced by the **trajectory optimizer**.
      - > "This encourages the human driving trajectory to have smaller cost than other trajectories".
      - It reminds me the **`max-margin` method in `IRL`** where the weights of the reward function should make the **expert demonstration better** than any other policy candidate.

</details>

---

**`"Adversarial Inverse Reinforcement Learning for Decision Making in Autonomous Driving"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1911.08044)]**
**[** :mortar_board: `UC Berkeley, Chalmers University, Peking University` **]**
**[** :car: `Zenuity` **]**
- **[** _`GAIL`, `AIRL`, `action-masking`, `augmented reward function`_ **]**

<details>
  <summary>Click to expand</summary>

Author: Wang, P., Liu, D., Chen, J., & Chan, C.-Y.

| ![In Adversarial `IRL` (`AIRL`), the discriminator tries to distinguish learnt actions from demonstrated expert actions. Action masking is applied, removing some combinations that are not preferable, in order to reduce the **unnecessary exploration**. Finally, the reward function of the discriminator is extended with some manually-designed **semantic reward** to help the agent successfully complete the lane change and not to collide with other objects. [Source](https://arxiv.org/abs/1911.08044).](../media/2019_wang_1.PNG "In Adversarial `IRL` (`AIRL`), the discriminator tries to distinguish learnt actions from demonstrated expert actions. Action masking is applied, removing some combinations that are not preferable, in order to reduce the **unnecessary exploration**. Finally, the reward function of the discriminator is extended with some manually-designed **semantic reward** to help the agent successfully complete the lane change and not to collide with other objects. [Source](https://arxiv.org/abs/1911.08044).")  |
|:--:|
| *In Adversarial `IRL` (`AIRL`), the **discriminator** tries to distinguish learnt actions from demonstrated expert actions. `Action-masking` is applied, removing some `action` combinations that are not preferable, in order to reduce the **unnecessary exploration**. Finally, the reward function of the discriminator is extended with some **manually-designed** **semantic reward** to help the agent successfully complete the lane change and not to collide with other objects. [Source](https://arxiv.org/abs/1911.08044).* |

- One related concept _(detailed further on this page)_: **Generative Adversarial Imitation Learning** (**`GAIL`**).
  - An **imitation learning** method where the goal is to **learn a policy** against a **discriminator** that tries to distinguish learnt actions from expert actions.
- Another concept used here: **Guided Cost Learning** (**`GCL`**).
  - A **`Max-Entropy` `IRL` method** that makes use of **importance sampling** (`IS`) to approximate the **partition function** (the term in the gradient of the log-likelihood function that is **hard to compute** since it involves an **integral of over all possible trajectories**).
- One concept introduced: **Adversarial Inverse Reinforcement Learning** (**`AIRL`**).
  - It **combines `GAIL` with `GCL` formulation**.
    - > "It uses a special form of the **discriminator** different from that used in `GAIL`, and **recovers a cost function and a policy simultaneously** as that in `GCL` but in an **adversarial way**."
  - Another difference is the use of a **model-free `RL` method** to compute the new optimal policy, instead of **model-based** `guided policy search` (`GPS`) used in `GCL`:
    - > "As the **dynamic driving environment** is too complicated to learn for the driving task, we instead use a **model-free** policy optimization method."
  - One motivation of `AIRL` is therefore to cope with **changes in the dynamics** of environment and make the learnt policy **more robust** to **system noises**.
- One idea: **Augment the learned reward** with some **_"semantic reward"_** term to improve **learning efficiency**.
  - The motivation is to manually **embed some domain knowledge**, in the **generator reward function**.
  - > "This should provide the agent **some informative guidance** and assist it to **learn fast**."
- About the task:
  - > "The task of our focus includes a `longitudinal` decision – the **selection of a target gap** - and a `lateral` decision – whether to **commit the lane change** right now."
  - It is a rather **"high-level"** decision:
    - A **low-level controller**, consisting of a `PID` for lateral control and `sliding-mode` for longitudinal control, is the use to **execute the decision**.
  - The authors use some **`action-masking` technics** where only valid action pairs are allowed to reduce the **agent’s unnecessary exploration**.

</details>

---

**`"Predicting vehicle trajectories with inverse reinforcement learning"`**

- **[** `2019` **]**
**[[:memo:](https://kth.diva-portal.org/smash/get/diva2:1366887/FULLTEXT01.pdf)]**
**[** :mortar_board: `KTH` **]**
- **[** _`max-margin`_ **]**

<details>
  <summary>Click to expand</summary>

Author: Hjaltason, B.

| ![The `φ` are **distances read from the origin of a vision field** and are represented by red dotted lines. They take value in [`0`, `1`], where `φi` = `1` means the dotted line does not hit any object and `φi` = `0` means it hits an object at origin. In this case, **two objects are inside the front vision field**. Hence `φ1` = `0.4` and `φ2` = `0.6`.. [Source](https://kth.diva-portal.org/smash/get/diva2:1366887/FULLTEXT01.pdf).](../media/2019_hjaltason_1.PNG "The `φ` are **distances read from the origin of a vision field** and are represented by red dotted lines. They take value in [`0`, `1`], where `φi` = `1` means the dotted line does not hit any object and `φi` = `0` means it hits an object at origin. In this case, **two objects are inside the front vision field**. Hence `φ1` = `0.4` and `φ2` = `0.6`.. [Source](https://kth.diva-portal.org/smash/get/diva2:1366887/FULLTEXT01.pdf).")  |
|:--:|
| *About the **features**: The `φ` are **distances read from the origin of a vision field** and are represented by red dotted lines. They take value in [`0`, `1`], where `φi` = `1` means the dotted line does not hit any object and `φi` = `0` means it hits an object at origin. In this case, **two objects are inside the front vision field**. Hence `φ1` = `0.4` and `φ2` = `0.6`. [Source](https://kth.diva-portal.org/smash/get/diva2:1366887/FULLTEXT01.pdf).* |

| ![Example of `max-margin` `IRL`. [Source](https://kth.diva-portal.org/smash/get/diva2:1366887/FULLTEXT01.pdf).](../media/2019_hjaltason_2.PNG "Example of `max-margin` `IRL`. [Source](https://kth.diva-portal.org/smash/get/diva2:1366887/FULLTEXT01.pdf).")  |
|:--:|
| *Example of **`max-margin`** `IRL`. [Source](https://kth.diva-portal.org/smash/get/diva2:1366887/FULLTEXT01.pdf).* |

- A good example of **max-margin `IRL`**:
  - > "There are two classes: The **expert behaviour from data** gets a label of **`1`**, and the **"learnt" behaviours** a label of **`-1`**. The framework performs a **`max-margin` optimization** step to maximise the difference between both classes. The result is an **orthogonal vector `wi`** from the max margin hyperplane, **orthogonal to the estimated expert feature vector `µ(πE)`**".
  - From this new `R=w*f`, an **optimal policy** is derived using `DDPG`.
  - **Rollouts** are performed to get an **estimated feature vector** that is added to the set of "learnt" behaviours.
  - The process is repeated until convergence (when the estimated values `w*µ(π)` are close enough).
- Note about the reward function:
  - Here, **r(`s, a, s'`)** is also function of the action and the next state.
  - Here a [post](https://thinkingwires.com/posts/2018-02-11-reward-function-domain.html) about different forms of reward functions.

</details>

---

**`"A Survey of Inverse Reinforcement Learning: Challenges, Methods and Progress"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1806.06877)]**
**[** :mortar_board: `University of Georgia` **]**
- **[** _`reward engineering`_ **]**

<details>
  <summary>Click to expand</summary>

Authors: Arora, S., & Doshi, P.

| ![Trying to generalize and classify `IRL` methods. [Source](https://arxiv.org/abs/1806.06877).](../media/2019_arora_1.PNG "Trying to generalize and classify `IRL` methods. [Source](https://arxiv.org/abs/1806.06877).")  |
|:--:|
| *Trying to generalize and classify `IRL` methods. [Source](https://arxiv.org/abs/1806.06877).* |

| ![I learnt about **state visitation frequency**: `ψ`(`π`)(`s`) and the **feature count expectation**: `µ`(`π`)(`φ`). [Source](https://arxiv.org/abs/1806.06877).](../media/2019_arora_3.PNG "I learnt about **state visitation frequency**: `ψ`(`π`)(`s`) and the **feature count expectation**: `µ`(`π`)(`φ`). [Source](https://arxiv.org/abs/1806.06877).")  |
|:--:|
| *I learnt about **state visitation frequency**: `ψ`(`π`)(`s`) and the **feature count expectation**: `µ`(`π`)(`φ`). [Source](https://arxiv.org/abs/1806.06877).* |

- This large review does not focus on `AD` applications, but it provides a good picture of `IRL` and can give ideas. Here are my take-aways.
- Definition:
  - > "Inverse reinforcement learning (**`IRL`**) is the problem of **modeling the preferences** of another agent using its **observed behavior** _[hence class of `IL`]_, thereby avoiding a manual specification of its **reward function**."
- Potential `AD` applications of `IRL`:
  - **Decision-making**: _If I find your underlying reward function, and I consider you as an expert,_ **_I can imitate you_**.
  - **Prediction**: _If I find your underlying reward function, I can imagine_ **_what you are going to do_**
- I start rethinking `Imitation Learning`. The goal of `IL` is to **derive a policy** based on some (expert) **demonstrations**.
  - Two branches emerge, depending on **what structure is used to model the expert behaviour**. Where is that model **captured**?
    - `1-` In a **policy**.
      - This is a **"direct approach"**. It includes `BC` and its variants.
      - The task is to learn that `state` `->` `action` **mapping**.
    - `2-` In a **reward function**.
      - **Core assumption**: Each driver has an **internal reward function** and **acts optimally** w.r.t. it.
      - The main task it to learn that **reward function** (`IRL`), which captures the expert's preferences.
      - The second step consists in **deriving the optimal policy** for this derived reward function.
        > As Ng and Russell put it: "The `reward function`, rather than the `policy`, is **the most succinct, robust, and transferable definition of the task**"
  - _What happens if some states are_ **_missing in the demonstration_**_?_
    - `1-` **Direct methods** will not know what to do. And will try to **interpolate** from similar states. This could be risky. (c.f. `distributional shift` problem and `DAgger`).
      - > "If a policy is used to **describe a task**, it will be **less succinct** since **for each state** we have to give a description of what the behaviour should look like". From [this post](https://thinkingwires.com/posts/2018-02-13-irl-tutorial-1.html)
    - `2-` `IRL` methods acts **optimally** w.r.t. the underlying reward function, which _could_ be better, since it is **more robust**.
      - This is particularly useful if we have an expert policy that is **only approximately optimal**.
      - In other words, a policy that is better than the "expert" can be derived, while having very **little exploration**. This **"minimal exploration"** property is useful for tasks such as `AD`.
      - This is [sometimes](https://thegradient.pub/learning-from-humans-what-is-inverse-reinforcement-learning/) refers to as `Apprenticeship learning`.
- One new concept I learnt: **`State-visitation frequency`** (it reminds me some concepts of _Markov chains_).
  - Take a **policy `π`**. Let run the agent with it. Count how **often it sees each state**. This is called the `state-visitation frequency` (note it is for a **specific `π`**).
  - Two ideas from there:
    - **Iterating** until this `state-visitation frequency` **stops changing** yields the `converged frequency function`.
    - Multiplying that `converged state-visitation frequency` with `reward` gives another **perspective to the `value function`**.
      - The `value function` can now be seen as a **linear combination** of the **expected feature count** **`µ`**(`φk`)(`π`) (also called `successor feature`).
- One common assumption:
  -> "The solution is a **weighted linear combination** of a set of **reward features**".
  - This greatly reduces the **search space**.
  - > "It allowed the use of **feature expectations** as a **sufficient statistic** for representing the **value** of trajectories or the **value** of an expert’s policy."
- Known `IRL` **issues** (and solutions):
  - `1-` This is an **under-constrained** learning problem.
    - > "Many reward functions **could explain** the observations".
    - Among them, they are highly **"degenerate"** functions with **all reward values zero**.
    - One solution is to impose **constraints** in the **optimization**.
      - For instance try to maximize the sum of **"value-margins"**, i.e. the difference between the value functions of the best and the second-best actions.
      - > "`mmp` makes the solution policy have **state-action visitations** that **align** with those in the expert’s demonstration."
      - > "`maxent` distributes probability mass based on entropy but **under the constraint** of **feature expectation matching**."
    - Another common constraint is to **encourage** the reward function to **be as simple as possible**,  similar to **`L1` regularization** in supervised learning.
  - `2-` Two incomplete models:
    - `2.1-` _How to deal with incomplete/absent model of transition probabilities?_
    - `2.2-` _How to select the_ **_reward features_**_?_
      - > "_[One could]_ use **neural networks** as **function approximators** that avoid the **cumbersome hand-engineering** of appropriate reward features".
    - > "These extensions share similarity with **`model-free RL`** where the `transition` model and `reward function` features are also unknown".
  - `3-` How to deal with **noisy demonstrations**?
    - Most approaches assume a Gaussian noise and therefore apply **Gaussian filters**.
- _How to_ **_classify_** _`IRL` methods?_
  - It can be useful to ask yourself **two questions**:
    - `1-` What are the **parameters** of the **Hypothesis `R` function**`?
      - Most approaches use the **_"linear approximation"_** and try to **estimate the weights** of the **linear combination of features**.
    - `2-` What for **"Divergence Metric"**, i.e. how to evaluate the _discrepancy_ to the expert demonstrations?
      - > "_[it boils down to]_ a **search** in **reward function space** that terminates when the behavior derived from the current solution **aligns** with the observed behavior."
      - How to measure the **closeness** or the **similarity** to the expert?
        - `1-` Compare the **policies** (i.e. the behaviour).
          - E.g. how many <`state`, `action`> pairs are matching?
          - > "A difference between the two policies in just one state could still have a significant impact."
        - `2-` Compare the **value functions** (they are defined over **all states**).
          - The authors mention the **`inverse learning error`** (`ILE`) = `||` `V`(`expert policy`) `-` `V`(`learnt policy`) `||` and the `value loss` (use as a **margin**).
  - Classification:
    - **`Margin-based` optimization**: Learn a reward function that explains the demonstrated policy **better than alternative policies** by a **`margin`** (address `IRL`'s **"solution ambiguity"**).
      - The intuition here is that we want a reward function that **clearly distinguishes** the optimal policy from other possible policies.
    - **`Entropy-based` optimization**: Apply the [**"maximum entropy principle"**](https://en.wikipedia.org/wiki/Principle_of_maximum_entropy) (together with the **"feature expectations matching"** constraint) to obtain a **distribution over potential reward functions**.
    - **`Bayesian` inference** to derive `P`(`^R`|`demonstration`).
      - What for the **likelihood** `P`(<`s`, `a`> | `ˆR`)? This probability is proportional to the exponentiated **value function**: `exp`(`Q`[`s`, `a`]).
    - **`Regression` and `classification`**.

</details>

---

**`"Learning Reward Functions for Optimal Highway Merging"`**

- **[** `2019` **]**
**[[:memo:](https://web.stanford.edu/class/aa228/reports/2018/final101.pdf)]**
**[** :mortar_board: `Stanford` **]**
**[[🎞️](https://www.youtube.com/watch?v=icJz32L24JI)]**
- **[** _`reward engineering`_ **]**

<details>
  <summary>Click to expand</summary>

Author: Weiss, E.

| ![The **assumption-free** reward function that uses a **simple polynomial** form based on `state` and `action` values at each time step does better at **minimizing both `safety` and `mobility` objectives**, even though it **does not incorporate human knowledge** of typical reward function structures. About **Pareto optimum**: at these points, it becomes impossible to improve in the minimization of **one objective** without **worsening** our minimization of the **other objective**). [Source](https://web.stanford.edu/class/aa228/reports/2018/final101.pdf).](../media/2019_weiss_1.PNG "The **assumption-free** reward function that uses a **simple polynomial** form based on `state` and `action` values at each time step does better at **minimizing both `safety` and `mobility` objectives**, even though it **does not incorporate human knowledge** of typical reward function structures. About **Pareto optimum**: at these points, it becomes impossible to improve in the minimization of **one objective** without **worsening** our minimization of the **other objective**). [Source](https://web.stanford.edu/class/aa228/reports/2018/final101.pdf).")  |
|:--:|
| *The **assumption-free** reward function that uses a **simple polynomial** form based on `state` and `action` values at each time step does better at **minimizing both `safety` and `mobility` objectives**, even though it **does not incorporate human knowledge** of typical reward function structures. About **Pareto optimum**: at these points, it becomes impossible to improve in the minimization of **one objective** without **worsening** our minimization of the **other objective**). [Source](https://web.stanford.edu/class/aa228/reports/2018/final101.pdf).* |

- _What?_
  - A Project from the Stanford course ([AA228/CS238 - Decision Making under Uncertainty](https://web.stanford.edu/class/aa228/cgi-bin/wp/)). Examples of student projects can be found [here](https://web.stanford.edu/class/aa228/cgi-bin/wp/old-projects/).
- My main takeaway:
  - A **simple problem** that illustrates the need for (_learning more about_) **`IRL`**.
- The **merging task** is formulated as a **simple `MDP`**:
  - The **state space** has **size `3`** and is **discretized**: `lat` + `long` ego position and `long` position of the other car.
  - The other vehicle **transitions stochastically** (`T`) according to three simple behavioural models: `fast`, `slow`, `average speed` driving.
  - The main contribution concerns the **reward design**: _how to_ **_shape the reward function_** _for this_ **_multi-objective_** _(_**_trade-off_** `safety` _/_ `efficiency`_)_ _optimization problem?_
- Two **reward functions** (`R`) are compared:
  - > `1-` "The first formulation models rewards based on **our prior knowledge** of how we **would expect** autonomous vehicles to operate, **directly encoding human values** such as `safety` and `mobility` into this problem as a positive reward for merging, a penalty for `merging close` to the other vehicle, and a penalty for `staying` in the on-ramp."
  - > `2-` "The second reward function formulation **assumes no prior knowledge** of **human values** and instead comprises a **simple degree-one polynomial** expression for the components of the `state` and the `action`."
    - The **parameters** are tuned using a sort of **grid search** (no proper `IRL`).
- _How to compare them?_
  - Since **both `T` and `R` are known**, a **planning** (as [opposed](https://github.com/chauvinSimon/IV19#difference-learning-vs-planning) to **learning**) algorithm can be used to find the **optimal policy**. Here `value iteration` is implemented.
  - The resulting agents are then evaluated based on **two conflicting objectives**:
    - > "Minimizing the distance along the road at which **point merging occurs** and **maximizing the `gap`** between the two vehicles when merging."
  - Next step will be proper **`IRL`**:
    - > "We can therefore conclude that there may exist **better reward functions** for capturing optimal driving policies than either the **intuitive prior knowledge** reward function or the **polynomial reward function**, which **doesn’t incorporate any human understanding of costs** associated with `safety` and `efficiency`."

</details>

---

**`"Game-theoretic Modeling of Traffic in Unsignalized Intersection Network for Autonomous Vehicle Control Verification and Validation"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1910.07141v2)]**
**[** :mortar_board: `University of Michigan and Bilkent University, Ankara` **]**
- **[** _`DAgger`, `level-k control policy`_ **]**

<details>
  <summary>Click to expand</summary>

Authors: Tian, R., Li, N., Kolmanovsky, I., Yildiz, Y., & Girard, A.

- This paper builds on several works _(also analysed further below)_:
  - ["Adaptive Game-Theoretic Decision Making for Autonomous Vehicle Control at Roundabouts"](https://arxiv.org/abs/1810.00829) - (Tian, Li, Li, et al., 2019).
  - ["Game Theoretic Modeling of Vehicle Interactions at Unsignalized Intersections and Application to Autonomous Vehicle Control"](https://www.researchgate.net/publication/323428804_Game_Theoretic_Modeling_of_Vehicle_Interactions_at_Unsignalized_Intersections_and_Application_to_Autonomous_Vehicle_Control) - (N. Li, Kolmanovsky, Girard, & Yildiz, 2018).
  - ["Game-theoretic modeling of driver and vehicle interactions for verification and validation of autonomous vehicle control systems"](https://arxiv.org/abs/1608.08589) - (N. Li et al., 2016).

- Addressed problem: **unsignalized intersections** with **heterogenous driving styles** (`k` in [`0`, `1`, `2`])
  - The problem is formulated using the **level-`k`** game-theory formalism _(See analysed related works for more details)_.
- One idea: use **imitation learning** (`IL`) to obtain an **explicit level-`k` control policy**.
  - A level-`k` policy is a mapping `pi`: <**`ego state`**, **`other's states`**, **`ego k`**> `->` <**`sequence of ego actions`**>.
  - The ego-agent **maintains belief over the level `k`** of other participants. These estimates are updated using _maximum likelihood_ and _Bayes rule_.
  - A first attempt with **supervised learning on a fix dataset** (_`behavioural cloning`_) was not satisfying enough due to its **drift shortcomings**:
    - > "A small error may cause the vehicle to **reach a state that is not exactly included in the dataset** and, consequently, a large error may occur at the next time step."
  - The solution is to also **aggregate experience sampled from the currently learnt policy**.
    - The [**`DAgger`**](https://www.cs.cmu.edu/~sross1/publications/Ross-AIStats11-NoRegret.pdf) algorithm (**_Dataset Aggregation_**) is used in this work.
    - _One point I did not understand: I am surprised that no initial "off-policy" demonstrations is used. The dataset `D` is initialized as empty._
    - The policy is represented by a neural network.

</details>

---

**`"Interactive Decision Making for Autonomous Vehicles in Dense Traffic"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1909.12914)]**
**[** :car: `Honda` **]**

- **[** _`game tree search`, `interaction-aware decision making`_ **]**

<details>
  <summary>Click to expand</summary>

| ![In the **rule-based stochastic driver model** describing the other agents, `2` thresholds are introduced: The `reaction threshold`, sampled from the range {`−1.5m`, `0.4m`}, describes whether or not the **agent reacts to the ego car**. The `aggression threshold`, uniformly sampled {`−2.2`, `1.1m`}, describes **how the agent reacts**. [Source](https://arxiv.org/abs/1909.12914).](../media/2019_isele_2.PNG "In the **rule-based stochastic driver model** describing the other agents, `2` thresholds are introduced: The `reaction threshold`, sampled from the range {`−1.5m`, `0.4m`}, describes whether or not the **agent reacts to the ego car**. The `aggression threshold`, uniformly sampled {`−2.2`, `1.1m`}, describes **how the agent reacts**. [Source](https://arxiv.org/abs/1909.12914).")  |
|:--:|
| *In the **rule-based stochastic driver model** describing the other agents, `2` thresholds are introduced: The `reaction threshold`, sampled from the range {`−1.5m`, `0.4m`}, describes whether or not the **agent reacts to the ego car**. The `aggression threshold`, uniformly sampled {`−2.2`, `1.1m`}, describes **how the agent reacts**. [Source](https://arxiv.org/abs/1909.12914).* |

| ![Two **tree searches** are performed: The first step is to **identify a target merging gap** based on the probability of a **successful merge** for each of them. The second search involves **forward simulation** and **collision checking** for multiple ego and traffic intentions. In practice the author found that ''the **coarse tree** - i.e. with intention only - was sufficient for **long term planning** and **only one intention depth** needed to be considered for the fine-grained search''. This reduces this second tree to a **matrix game**. [Source](https://arxiv.org/abs/1909.12914).](../media/2019_isele_1.PNG "Two **tree searches** are performed: The first step is to **identify a target merging gap** based on the probability of a **successful merge** for each of them. The second search involves **forward simulation** and **collision checking** for multiple ego and traffic intentions. In practice the author found that ''the **coarse tree** - i.e. with intention only - was sufficient for **long term planning** and **only one intention depth** needed to be considered for the fine-grained search''. This reduces this second tree to a **matrix game**. [Source](https://arxiv.org/abs/1909.12914).")  |
|:--:|
| *Two **tree searches** are performed: The first step is to **identify a target merging gap** based on the probability of a **successful merge** for each of them. The second search involves **forward simulation** and **collision checking** for multiple ego and traffic intentions. In practice the author found that ''the **coarse tree** - i.e. with intention only - was sufficient for **long term planning** and **only one intention depth** needed to be considered for the fine-grained search''. This reduces this second tree to a **matrix game**. [Source](https://arxiv.org/abs/1909.12914).* |

Author: Isele, D.

- Three motivations when working on decision-making for **merging in dense traffic**:
  - `1-` Prefer `game theory` approaches over `rule-based` planners.
    - To avoid the **`frozen robot`** issue, especially in dense traffic.
    - > "If the ego car were to wait for an opening, it may have to **wait indefinitely**, greatly frustrating drivers behind it".
  - `2-` Prefer the `stochastic game` formulation over `MDP`.
    - Merging in dense traffic involves **interacting with self-interested agents** (_"self-interested"_ in the sense that they want to **travel as fast as possible** without crashing).
    - > "`MDPs` assume agents follow a **set distribution** which limits an autonomous agent’s ability to **handle non-stationary agents** which **change their behaviour over time**."
    - > "**`Stochastic games`** are an extension to `MDPs` that generalize to **multiple agents**, each of which has its **own policy** and **own reward function**."
    - In other words, `stochastic games` seen more appropriate to **model interactive behaviours**, especially in the **forward rollout** of tree search:
      - An _interactive prediction model_ based on the concept of [**`counterfactual reasoning`**](https://en.wikipedia.org/wiki/Counterfactual_thinking) is proposed.
      - It describes how **behaviour might change in response to ego agent intervention**.
  - `3-` Prefer `tree search` over `neural networks`.
    - > "Working with the **`game trees`** directly produces **interpretable decisions** which are better suited to **safety guarantees**, and ease the **debugging** of undesirable behaviour."
    - In addition, it is possible to include **stochasticity** for the tree search.
      - More precisely, the **probability of a successful merge** is computed for each **potential gap** based on:
        - The traffic participant’s **willingness to yield**.
        - The **size of the gap**.
        - The **distance to the gap** (from our current position).
- _How to_ **_model other participants_**_, so that they act "intelligently"?_
  - > "In order to validate our behaviour we need **interactive agents** to test against. This produces a `chicken and egg` problem, where we **need to have an intelligent agent** to develop and test our agent. To address this problem, we develop a **stochastic rule-based merge behaviour** which can give the appearance that agents are changing their mind."
  - This _merging-response_ driver model builds on the ideas of `IDM`, introducing **two thresholds** (c.f. figure):
    - One threshold governs **whether or not the agent reacts** to the ego car,
    - The second threshold determines **how the agent reacts**.
    - > "This process can be viewed as a **rule-based variant of negotiation strategies**: an agent proposes he/she go first by **making it more dangerous for the other**, the other agent accepts by backing off."
- _How to_ **_reduce the computational complexity_** _of the probabilistic game tree search, while keeping_ **_safely considerations_** _?_
  - The **_forward simulation_** and the **_collision checking_** are costly operations. Especially when the depth of the tree increases.
  - Some approximations include reducing the **number of actions** _(for both the ego- and the other agents)_, reducing the **number of interacting participants** and reducing the **branching factor**, as can been seen in the steps of the presented approach:
    - **`1-`** `Select an intention class based on a coarse search.` - the ego-actions are decomposed into a `sub-goal selection task` and a `within-sub-goal set of actions`.
    - **`2-`** `Identify the interactive traffic participant.` - it is assumed that at any given time, the ego-agent **interacts with only one other agent**.
    - **`3-`** `Predict other agents’ intentions.` - working with **`intentions`**, the **continuous action space** can be **discretized**. It reminds me the concept of **`temporal abstraction`** which **reduces the depth of the search**.
    - **`4-`** `Sample and evaluate the ego intentions.` - a set of safe (absence of collision) ego-intentions can be generated and assessed.
    - **`5-`** `Act, observe, and update our probability models.` - the probability of safe successful merge.

</details>

---

**`"Adaptive Robust Game-Theoretic Decision Making for Autonomous Vehicles"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1909.02586)]**
**[** :mortar_board: `University of Michigan` **]**
**[[:octocat:](https://github.com/gokulsivasankar/RobustDecisionMaking)]**

- **[** _`k-level strategy`, `MPC`, `interaction-aware prediction`_ **]**

<details>
  <summary>Click to expand</summary>

| ![The agent maintain belief on the `k` parameter for other vehicles and updates it at each step. [Source](https://arxiv.org/abs/1902.09068).](../media/2019_sankar_1.PNG "The agent maintain belief on the `k` parameter for other vehicles and updates it at each step. [Source](https://arxiv.org/abs/1902.09068).")  |
|:--:|
| *The agent maintain belief on the `k` parameter for other vehicles and updates it at each step. [Source](https://arxiv.org/abs/1902.09068).* |

Authors: Sankar, G. S., & Han, K.

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
    - > "The min-max strategy considers the **worst-case disturbance** affecting the behaviour/performance of the system and provides control actions to mitigate the effect of the worst-case disturbance."
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

</details>

---

**`"Towards Human-Like Prediction and Decision-Making for Automated Vehicles in Highway Scenarios"`**

- **[** `2019` **]**
**[[:memo:](https://tel.archives-ouvertes.fr/tel-02184362/document)]**
**[[:octocat:](https://github.com/marioney/hybrid_simulation/tree/decision-making)]**
**[[🎞️](https://www.youtube.com/watch?v=Xx5OmV86CsM)]**
**[** :mortar_board: `INRIA` **]**
**[** :car: `Toyota` **]**

- **[** _`maximum entropy IRL`_  **]**

<details>
  <summary>Click to expand</summary>

- Note:
  - this **`190`-page thesis** is also referenced in the sections for **prediction** and **planning**.
  - I really like how the author organizes synergies between three modules that are split and made independent in most modular architectures:
    - **`(1)` driver model**
    - **`(2)` behaviour prediction**
    - **`(3)` decision-making**

Author: Sierra Gonzalez, D.

- Related work: there are close concepts to the approach of `(Kuderer et al., 2015)` referenced below.
- One idea: **encode the driving preferences** of a human driver with a **reward function** (or **cost function**), mentioning a quote from Abbeel, Ng and Russell:

> “The reward function, rather than the policy or the value function, is the most **succinct**, **robust**, and **transferable** definition of a task”.

- Other ideas:
  - Use IRL to **avoid the manual tuning** of the parameters of the reward model. Hence learn a cost/reward function from demonstrations.
  - Include **dynamic features**, such as the `time-headway`, in the linear combination of the cost function, to take the **interactions** between traffic participants into account.
  - Combine IRL with a trajectory planner based on **_"conformal spatiotemporal state lattices"_**.
    - The motivation is to deal with _continuous_ state and action spaces and handle the presence of _dynamic obstacles_.
    - Several advantages (_I honestly did not understand that point_): the ability to exploit the structure of the environment, to **consider time as part of the state-space** and respect the non-holonomic motion constraints of the vehicle.

- One term: **"_planning-based motion prediction_"**.
  - The resulting reward function can be used to **generate trajectory** (for prediction), using optimal control.
  - Simply put, it can be assumed that each vehicle in the scene behaves in the **"risk-averse"** manner **encoded by the model**, i.e. choosing actions leading to the lowest cost / highest reward.
  - This method is also called "**model**-based prediction" since it relies on a reward function or on the models of an MDP.
  - This prediction tool is not used alone but rather coupled with some **DBN-based manoeuvre estimation** (detailed in the [section on prediction](#Prediction_and_Manoeuvre_Recognition)).

</details>

---

**`"An Auto-tuning Framework for Autonomous Vehicles"`**

- **[** `2018` **]**
**[[:memo:](https://arxiv.org/abs/1808.04913)]**
**[**:car: `Baidu`**]**

- **[** _`max-margin`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1808.04913).](../media/2018_fan_4.PNG "[Source](https://arxiv.org/abs/1808.04913).")  |
|:--:|
| *Two ideas of rank-based conditional `IRL` framework (`RC`-`IRL`): **`Conditional` comparison** (left) and **`Rank`-based learning** (middle - is it a `loss`? I think you want to maximize this term instead?). Right: Based on the idea of the **`maximum margin`**, the goal is to find the **direction** that clearly **separates** the **demonstrated trajectory** from **randomly generated ones**. Illustration of the benefits of using `RC` to prevent **`background shifting`**: Even if the optimal reward function direction is the **same under the two scenarios**, it may not be ideal to train them together because the **optimal direction** may be impacted by **overfitting the `background shifting`**. Instead, the idea of **conditioning on scenarios** can be viewed as a **pairwise comparison**, which can **remove the background differences**. [Source](https://arxiv.org/abs/1808.04913).* |

| ![[Source](https://arxiv.org/abs/1808.04913).](../media/2018_fan_5.PNG "[Source](https://arxiv.org/abs/1808.04913).")  |
|:--:|
| *The **human expert** trajectory and **randomly generated** sample trajectories are sent to a `SIAMESE` network in a **pair-wise** manner. Again, I do not understand very well. [Source](https://arxiv.org/abs/1808.04913).* |

Authors: Fan, H., Xia, Z., Liu, C., Chen, Y., & Kong, Q.

- Motivation:
  - Define an **automatic tuning** method for the **cost** function used in the `Apollo` **`EM`-planning** module to **address many different scenarios**.
  - The idea is to learn these parameters from **human demonstration** via `IRL`.

- Two main ideas _(to be honest, I have difficulties understanding their points)_:
- `1-` **Conditional comparison**.
  - _How to measure similarities between the `expert policy` and a `candidate policy`?_
    - Usually: compare the **`expectation`** of their `value functions`.
    - Here: compare their `value functions` **evaluated `state` by `state`**.
  - _Why "conditional"?_
    - Because the **loss function** is **conditional on `states`**.
      - This can _allegedly_ significantly **reduce the _`background variance`_**.
      - The authors use the term **"background variance"** to refer to the _"differences in behaviours metrics"_, due to the **diversity of scenarios**. _(Not very clear to me.)_
    - > "Instead, the idea of **conditioning on scenarios** can be viewed as a **_pairwise comparison_**, which can remove the **background differences**."
- `2-` **Rank-based** learning.
  - > "To accelerate the training process and extend the coverage of corner cases, we **sample random policies** and compare against the expert demonstration instead of **generating the optimal policy first**, as in `policy gradient`."
  - _Why "ranked"?_
    - > "Our assumption is that the human demonstrations **rank near the top** of the distribution of policies **conditional on initial state** on average."
    - > "The **value function** is a **rank** or search objective for selecting best trajectories in the online module."

</details>

---

**`"Car-following method based on inverse reinforcement learning for autonomous vehicle decision-making"`**

- **[** `2018` **]**
**[[:memo:](https://authors.library.caltech.edu/92021/1/1729881418817162.pdf)]**
**[** :mortar_board: `Tsinghua University, California Institute of Technology, Hunan University` **]**

- **[** _`maximum-margin IRL`_ **]**

<details>
  <summary>Click to expand</summary>

| ![__Kernel functions__ are used on the **continuous** state space to obtain a **smooth reward function** using **linear function approximation**. [Source](https://authors.library.caltech.edu/92021/1/1729881418817162.pdf).](../media/2018_gao_2.PNG "__Kernel functions__ are used on the **continuous** state space to obtain a **smooth reward function** using **linear function approximation**. [Source](https://authors.library.caltech.edu/92021/1/1729881418817162.pdf).")  |
|:--:|
| *__Kernel functions__ are used on the **continuous** state space to obtain a **smooth reward function** using **linear function approximation**. [Source](https://authors.library.caltech.edu/92021/1/1729881418817162.pdf).* |

| ![As often, the __divergence metric__ - to measure the `gap` between one candidate and the expert - is the __expected value function__. Example of how to use **`2` `other candidate` policies**. I am still __confused__ that each of their decision is __based on a state seen by the expert__, i.e. they are not building their own full trajectory. [Source](https://authors.library.caltech.edu/92021/1/1729881418817162.pdf).](../media/2018_gao_1.PNG "As often, the __divergence metric__ - to measure the `gap` between one candidate and the expert - is the __expected value function__. Example of how to use **`2` `other candidate` policies**. I am still __confused__ that each of their decision is __based on a state seen by the expert__, i.e. they are not building their own full trajectory. [Source](https://authors.library.caltech.edu/92021/1/1729881418817162.pdf).")  |
|:--:|
| *As often, the __divergence metric__ (to measure the `gap` between one candidate and the expert) is the __expected value function__ estimated on **sampled trajectories**. Example of how to use **`2` `other candidate` policies**. I am still __confused__ that each of their decision is __based on a state seen by the expert__, i.e. they are not building their own **full trajectory**. [Source](https://authors.library.caltech.edu/92021/1/1729881418817162.pdf).* |

Authors: Gao, H., Shi, G., Xie, G., & Cheng, B.

- One idea: A simple and "educationally relevant" application to `IRL` and a good implementation of the algorithm of **(Ng A. & Russell S., 2000)**: [Algorithms for Inverse Reinforcement Learning](https://ai.stanford.edu/~ang/papers/icml00-irl.pdf).
  - Observe human behaviours during a "car following" task, **assume** his/her behaviour is optimal w.r.t. an **hidden reward function**, and try to **estimate** that function.
  - Strong assumption: `no lane-change`, `no overtaking`, `no traffic-light`. In other worlds, just concerned about the **longitudinal control**.
- _Which `IRL` method?_
  - `Maximum-margin`. Prediction aim at learning a reward function that **explains the demonstrated policy better than alternative policies** by a **margin**.
  - The **"margin"** is there to address **IRL's** [**solution ambiguity**](https://arxiv.org/pdf/1806.06877.pdf).
- Steps:
  - `1-` Define a **simple `2d` continuous state** space `s` = (`s0`, `s1`).
    - **`s0`** = **`ego-speed`** divided into **`15` intervals** (each `centre` will serve to build `means` for **Gaussian kernel functions**).
    - **`s1`** = **`dist-to-leader`** divided into **`36` intervals** (same remark).
    - A **normalization** is additionally applied.
  - `2-` **Feature transformation**: Map the `2d` **continuous state** to a finite number of **features** using **kernel functions**.
    - I recommend this short [video](https://www.youtube.com/watch?v=RdkPVYyVOvU) about **feature transformation** using **kernel functions**.
    - Here, **Gaussian radial kernel** functions are used:
      - _Why "radial"?_ The **closer the state** to the **centre of the kernel**, the higher the **response of the function**. And the further you go, the larger **the response "falls"**.
      - _Why "Gaussian"?_ Because the **standard deviation** describes how sharp that "fall" is.
      - Note that this functions are **`2d`**: `mean` = (the **centre of one `speed` interval**, the **centre of one `dist` interval**).
    - The distance of the continuous state `s` `=` (`s0`, `s1`) to each of the `15`*`36`=**`540`** `means` `s`(`i`, `j`) can be computed.
    - This gives **`540`** **kernel features** **`f`**(`i`, `j`) = **K**(`s`, `s`(`i`, `j`)).
  - `3-` The **one-step `reward`** is assumed to be **linear combination** of that features.
    - Given a policy, a **trajectory** can be constructed.
      - This is a list of `states`. This list can be mapped to a **list of `rewards`**.
      - The discounted sum of this list leads to the **`trajectory return`**, seen as **expected `Value function`**.
    - One could also form **`540` lists** for this trajectory (one per kernel feature). Then reduce them by **`discounted_sum()`**, leading to **`540` `V_f`(`i`, `j`)** per trajectory.
      - The **`trajectory return`** is then a simple the linear combination: **`theta`(`i`, `j`) `*` `V_f`(`i`, `j`)**.
    - This can be computed for the **demonstrating expert**, as well as for many other **policies**.
    - Again, the task it to **tune the weights** so that the **expert results in the largest values**, against all possible other policies.
  - `4-` The goal is now to find the `540` `theta`(`i`, `j`) **weights parameters** solution of the **`max-margin` objective**:
    - One goal: [**`costly single-step deviation`**](https://ai.stanford.edu/~ang/papers/icml00-irl.pdf).
      - Try to **maximize the smallest difference** one could find.
        - I.e. select the **best non-expert-policy action** and try to **maximize the difference to the expert-policy action** in each state.
        - **`max`**[over `theta`] **`min`**[over `π`] of the `sum`[over `i`, `j`] of **`theta`**(`i`, `j`) `*` [**`f_candidate`**(`i`, `j`) - **`f_expert`**(`i`, `j`)].
      - As often the `value function` serves as **"divergence metric"**.
    - One side **heuristic** to remove **_degenerate_** solutions:
      - > _"The reward functions with_ **_many small rewards_** are **_more natural_** _and should be preferred"._ from [here](https://thinkingwires.com/posts/2018-02-13-irl-tutorial-1.html).
      - Hence a **regularization constraint** (_a constraint, not a loss like `L1`!_) on the `theta`(`i`, `j`).
    - The optimization problem with **strict constraint** is transformed into an optimization problem with **"inequality" constraint**.
      - **Violating constraints** is allowed by **penalized**.
      - As I understood from my [readings](https://thinkingwires.com/posts/2018-02-13-irl-tutorial-1.html), that **relaxes the linear assumption** in the case the true `reward function` cannot be expressed as a linear combination of the fixed basis functions.
    - The resulting system of equations is solved here with **Lagrange multipliers** (**linear programming** was recommended in the [orginal `max-margin` paper](https://ai.stanford.edu/~ang/papers/icml00-irl.pdf)).
  - `5-` Once the `theta`(`i`, `j`) are estimated, the `R` can be expressed.
- About the other **policy "candidates"**:
  - > "For each optimal car-following state, one of the other car-following **actions** is **randomly selected** for the solution".
  - In other words, in `V`(**`expert`**) `>` `V`(**`other_candidates`**) goal, "`other_candidates`" refers to **random policies**.
  - It would have been interesting to have **"better" competitors**, for instance policies that are **optional w.r.t. the current estimate of `R` function**. E.g. **learnt with `RL`** algorithms.
    - That would lead to an **iterative** process that stops when **`R` converges**.

</details>

---

**`"A Human-like Trajectory Planning Method by Learning from Naturalistic Driving Data"`**

- **[** `2018` **]**
**[[:memo:](https://ieeexplore.ieee.org/document/8500448)]**
**[** :mortar_board: `Peking University` **]**
**[** :car: `Groupe PSA` **]**

- **[** _`sampling-based trajectory planning`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://ieeexplore.ieee.org/document/8500448).](../media/2018_he.PNG "[Source](https://ieeexplore.ieee.org/document/8500448).")  |
|:--:|
| *[Source](https://ieeexplore.ieee.org/document/8500448).* |

Authors: He, X., Xu, D., Zhao, H., Moze, M., Aioun, F., & Franck, G.

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

</details>

---

**`"Learning driving styles for autonomous vehicles from demonstration"`**

- **[** `2015` **]**
**[[:memo:](http://ais.informatik.uni-freiburg.de/publications/papers/kuderer15icra.pdf)]**
**[** :mortar_board: `University of Freiburg` **]**
**[** :car: `Bosch` **]**

- **[** _`MaxEnt IRL`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](http://ais.informatik.uni-freiburg.de/publications/papers/kuderer15icra.pdf).](../media/2015_kuderer_1.PNG "[Source](http://ais.informatik.uni-freiburg.de/publications/papers/kuderer15icra.pdf).")  |
|:--:|
| *[Source](http://ais.informatik.uni-freiburg.de/publications/papers/kuderer15icra.pdf).* |

Authors: Kuderer, M., Gulati, S., & Burgard, W.

- One important contribution: Deal with **continuous** features such as `integral of jerk` over the trajectory.
- One motivation: **Derive a cost function** from **observed trajectories**.
  - The trajectory object is first mapped to some **feature vector** (`speed`, `acceleration` ...).
- One Q&A:  _How to then derive a `cost` (or `reward`) from these features?_
  - The authors assume the cost function to be a **linear combination of the features**.
  - The goal is then about **learning the weights**.
  - They acknowledge in the conclusion that it may be a too simple model. Maybe **neural nets** could help to capture some more complex relations.
- One concept: **"Feature matching"**:
  - > "Our goal is to find a **generative model** `p`(`traj`| `weights`) that yields trajectories that are **_similar_** to the observations."
  - _How to define the "Similarity"?_
    - The **"features"** serve as a **measure of similarity**.
- Another concept: **"`ME-IRL`"** = **Maximum Entropy** `IRL`.
  - One issue: This **"feature matching"** formulation is ambiguous.
    - There are potentially many (degenerated) solutions `p`(`traj`| `weights`). For instance `weights` = zeros.
  - One idea is to introduce an **additional goal**:
    - In this case: "Among all the distributions that **match features**, they to select the one that **maximizes the entropy**."
  - The probability distribution over trajectories is in the form `exp`(`-cost[features(traj), θ]`), to model that **agents are exponentially more likely to select trajectories with lower cost**.
- About the **_maximum likelihood approximation_** in `MaxEnt-IRL`:
  - The gradient of the Lagrangian cost function turns to be the difference between two terms:
    - `1-` The **empirical** feature values _(easy to compute from the recorded)_.
    - `2-` The **expected** feature values _(hard to compute: it requires_ **_integrating over all possible trajectories_**_)_.
      - An approximation is made to estimate the **expected feature values**: the authors compute the **feature values** of the **_"most"_** **likely trajectory**, instead of computing the expectations by **sampling**.
    - Interpretation:
      - > "We assume that the demonstrations are in fact generated by minimizing a **cost function (`IOC`)**, in contrast to the assumption that demonstrations are **samples from a probability distribution (`IRL`)**".
- One related work:
  - ["Learning to Predict Trajectories of Cooperatively Navigating Agents"](http://www2.informatik.uni-freiburg.de/~kretzsch/pdf/kretzschmar14icra.pdf) by (Kretzschmar, H., Kuderer, M., & Burgard, W., 2014).

</details>
