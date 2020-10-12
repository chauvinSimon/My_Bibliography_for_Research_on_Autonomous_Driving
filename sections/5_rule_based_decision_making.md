# `Rule-based` `Decision Making`

---

**`"Formalizing Traffic Rules for Machine Interpretability"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2007.00330)]**
**[** :car: `Fortiss` **]**

- **[** _`LTL`, `Vienna convention`, [`SPOT`](https://spot.in.tum.de), [`INTERACTION Dataset​`](http://interaction-dataset.com/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2007.00330).](media/2020_esterle_1.PNG "[Source](https://arxiv.org/abs/2007.00330).")  |
|:--:|
| *Top left: Different techniques on how to **model the rules** have been employed: **formal logics** such as **Linear Temporal Logic `LTL`** or **Signal Temporal Logic (`STL`**), as well as **real-value constraints**. Middle and bottom: **Rules** are separated into **`premise`** and **`conclusion`**. The initial `premise` and `exceptions` (red) are combined by `conjunction`. [Source](https://arxiv.org/abs/2007.00330)* |

Authors: Esterle, K., Gressenbuch, L., & Knoll, A.

- Motivation:
  - > "Traffic rules are **fuzzy** and **not well defined**, making them **incomprehensible to machines**."
  - The authors **formalize traffic rules** from **legal texts** (here `StVO`) to a **formal language** (here `LTL`).
  
- _Which legal text defines rules?_
  - For instance the **[Straßenverkehrsordnung](https://de.wikipedia.org/wiki/Stra%C3%9Fenverkehrs-Ordnung_(Deutschland))** (`StVO`), which is the **German concretization** of the **[`Vienna Convention` on Road Traffic](https://en.wikipedia.org/wiki/Vienna_Convention_on_Road_Traffic)**.
- _Why_ **_Linear Temporal Logic (`LTL`)_** _as the_ **_formal language_** _to specify traffic rules?_
  - > "During the legal analysis, `conjunction`, `disjunction`, `negation` and `implication` proved to be powerful and useful tools for formalizing rules. As traffic rules such as **`overtaking` consider temporal behaviors**, we decided to use `LTL`."
  - > "Others have used **Signal Temporal Logic (`STL`)** to obtain **quantitative semantics** about **rule satisfaction**. Quantitive semantics might be beneficial for **relaxing the requirements** to satisfy a rule."

- **Rules** are separated into **`premise`** and **`conclusion`**.
  - > "This allows rules to be separated into a `premise` about the **current state of the environment**, i.e. when a rule applies, and the **legal behavior** of the ego agent in that situation (`conclusion`). Then, **exceptions** to the rules can be modeled to be part of the assumption."
- Tools:
  - [`INTERACTION`](http://interaction-dataset.com/): a dataset​ which focuses on **dense interactions** and *analyze the compliance** of each vehicle to the **traffic rules**.
  - [`SPOT`](https://spot.in.tum.de): a **`C++` library for model checking**, to translate the formalized **`LTL` formula** to a **deterministic finite automaton** and to manipulate the automatons.
  - [`BARK`](https://bark-simulator.github.io/): a benchmarking framework.

- Evaluation of **rule-violation** on **public** data:
  - > "Roughly **every fourth `lane change`** does **not keep a `safe distance`** to the rear vehicle, which is similar for the `German` and `Chinese` Data."

</details>

---

**`"A hierarchical control system for autonomous driving towards urban challenges"`**

- **[** `2020` **]**
**[[:memo:](https://www.mdpi.com/2076-3417/10/10/3543)]**
**[** :mortar_board: `Chungbuk National University, Korea` **]**

- **[** _`FSM`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://www.mdpi.com/2076-3417/10/10/3543).](media/2020_van_1.PNG "[Source](https://www.mdpi.com/2076-3417/10/10/3543).")  |
|:--:|
| *__Behavioural planning__ is performed using a **two-state `FSM`**. Right: **transition** conditions in the **`M-FSM`**. [Source](https://www.mdpi.com/2076-3417/10/10/3543)* |

Authors: Van, N. D., Sualeh, M., Kim, D., & Kim, G. W.

- Motivations:
  - > "In the `DARPA` Urban Challenge, **Stanford `Junior`** team succeeded in applying **`FSM` with several scenarios** in urban traffic roads. However, the main **drawback of `FSM`** is the difficulty in **solving uncertainty** and in **large-scale scenarios**."
  - Here:
    - The **uncertainty** is not addressed.
    - The diversity of scenarios is handled by a **two-stage Finite State Machine** (`FSM`).
- About the **two-state `FSM`**:
  - `1-` A **Mission `FSM`** (`M-FSM`).
    - Five **states**: `Ready`, `Stop-and-Go` (`SAG`) _(main mode)_, `Change-Lane` (`CL`), `Emergency-stop`, `avoid obstacle mode`.
  - `2-` A **Control `FSM`** (`C-FSM`) in each **`M-FSM` state**.
- The `decision` is then converted into `speed` and `waypoints` objectives, handled by the **local path planning**.
  - It uses a **real-time hybrid `A*` algorithm** with an **occupancy grid map**.
  - The communication `decision` -> `path planner` is **unidirectional**: **No feedback** is given regarding the _feasibility_ for instance.

</details>

---

**`"Trajectory Optimization and Situational Analysis Framework for Autonomous Overtaking with Visibility Maximization"`**

- **[** `2019` **]**
**[[:memo:](http://www.alonsomora.com/docs/19-andersen-t-iv.pdf)]**
**[[🎞️](https://www.youtube.com/watch?v=qCcrjBfSKOU)]**
**[** :mortar_board: `National University of Singapore, Delft University, MIT` **]**
- **[** _`FSM`, `occlusion`, `partial observability`_  **]**

<details>
  <summary>Click to expand</summary>

| ![Left: previous work [Source](http://people.csail.mit.edu/jalonsom/docs/17-andersen-ITSC.pdf). Right: The `BP` **`FSM`** consists in **`5` states** and **`11` transitions**. Each transition from one state to the other is triggered by specific **alphabet** unique to the state. For instance, `1` is `Obstacle to be overtaken in ego lane detected`. Together with the `MPC` set of parameters, a **guidance path** is passed to the **trajectory optimizer**. [Source](http://www.alonsomora.com/docs/19-andersen-t-iv.pdf).](media/2019_andersen_1.PNG "Left: previous work [Source](http://people.csail.mit.edu/jalonsom/docs/17-andersen-ITSC.pdf). Right: The `BP` **`FSM`** consists in **`5` states** and **`11` transitions**. Each transition from one state to the other is triggered by specific **alphabet** unique to the state. For instance, `1` is `Obstacle to be overtaken in ego lane detected`. Together with the `MPC` set of parameters, a **guidance path** is passed to the **trajectory optimizer**. [Source](http://www.alonsomora.com/docs/19-andersen-t-iv.pdf).")  |
|:--:|
| *Left: previous work [Source](http://people.csail.mit.edu/jalonsom/docs/17-andersen-ITSC.pdf). Right: The `BP` **`FSM`** consists in **`5` states** and **`11` transitions**. Each transition from one state to the other is triggered by specific **alphabet** unique to the state. For instance, `1` is `Obstacle to be overtaken in ego lane detected`. Together with the `MPC` set of parameters, a **guidance path** is passed to the **trajectory optimizer**. [Source](http://www.alonsomora.com/docs/19-andersen-t-iv.pdf).* |

Authors: Andersen, H., Alonso-mora, J., Eng, Y. H., Rus, D., & Ang Jr, M. H.

- Main motivation:
  - Deal with **occlusions**, i.e. **_partial observability_**.
  - Use case: a car is illegally parked on the vehicle’s ego lane. It may fully **occlude the visibility**. But has to be **overtaken**.
- One related works:
  - ["Trajectory Optimization for Autonomous Overtaking with Visibility Maximization"](http://people.csail.mit.edu/jalonsom/docs/17-andersen-ITSC.pdf) - (Andersen et al., 2017)
  - **[[🎞️](https://www.youtube.com/watch?v=mHye5V1iC70)]**.
  - **[[🎞️](https://www.youtube.com/watch?v=iKXvOs6Drw0)]**.
  - **[[🎞️](https://www.youtube.com/watch?v=iwnlYLaQWLI)]**.
- About the **hierarchical** structure.
  - `1-` A high-level **behaviour planner** (`BP`).
    - It is structured as a **deterministic finite state machine** (`FSM`).
    - States include:
      - `Follow ego-lane`
      - `Visibility Maximization`
      - `Overtake`
      - `Merge back`
      - `Wait`
    - Transition are based on some **_deterministic_** **`risk assessment`**.
      - The authors argue that the **_deterministic_** methods (e.g. _formal verification_ of trajectory using `reachability analysis`) are **simpler** and computationally more efficient than **_probabilistic_** versions, while being very adapted for this **information maximization**:
      - > This is due to the fact that the designed behaviour planner **explicitly breaks the traffic rule** in order to progress along the vehicle’s course.
  - Interface `1-` > `2-`:
    - Each state correspond to specific **set of parameters** that is used in the **trajectory optimizer**.
    - > "In case of `Overtake`, a suggested **guidance path** is given to both the `MPC` and `**backup trajectory generator**".
  - `2-` A trajectory **optimizer**.
    - The problem is formulated as **receding horizon planner** and the task is to solve, in real-time, the non-linear **constrained optimization**.
      - **Cost** include `guidance path deviation`, `progress`, `speed deviation`, `size of blind spot` (visible area) and `control inputs`.
      - **Constraints** include, among other, `obstacle avoidance`.
      - The **prediction horizon** of this **`MPC`** is `5s`.
    - Again (_I really like this idea_), `MPC` **parameters** are set by the `BP`.
      - For instance, the **cost for `path deviation`** is high for `Follow ego-lane`, while it can be reduced for `Visibility Maximization`.
      - > "Increasing the **visibility maximization** cost resulted in the **vehicle deviating from the path** earlier and more abrupt, leading to **frequent wait or merge back** cases when an oncoming car comes into the vehicle’s sensor range. Reducing visibility maximization resulted in **later and less abrupt deviation**, leading to **overtaking** trajectories that are **too late to be aborted**. We tune the costs for a good trade-off in performance."
      - Hence, depending on the state, the task might be to maximize the **amount of information** that the autonomous vehicle **gains along its trajectory**.
  - > "Our method considers **visibility** as a part of both `decision-making` and `trajectory generation`".

</details>

---

**`"Jointly Learnable Behavior and Trajectory Planning for Self-Driving Vehicles"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1910.04586)]**
**[** :car: `Uber` **]**
- **[** _`max-margin`_  **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/1910.04586).](media/2019_sadat_1.PNG "[Source](https://arxiv.org/abs/1910.04586).")  |
|:--:|
| *[Source](https://arxiv.org/abs/1910.04586).* |

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
    - `1-` An high-level **behaviour** `b`.
      - The path to converge to based on one chosen manoeuvre: `keep-lane`, `left-lane-change`, or `right-lane-change`.
      - The `left` and `right` **lane boundaries**.
      - The **obstacle `side assignment`**: whether an obstacle should stay in the `front`, `back`, `left`, or `right` to the ego-car.
    - `2-` A coarse-level **trajectory** `τ`.
    - The loss has also a **regularization term**.
    - This decision is "simply" the `argmin` of the shared cost-function, obtained by **sampling**+**selecting the best**.
  - The **"trajectory optimizer"** refines `τ` based on the **constraints** imposed by `b`.
    - For instance an **overlap cost** will be incurred if the `side assignment` of `b` is violated.
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

**`"Liability, Ethics, and Culture-Aware Behavior Specification using Rulebooks"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1902.09355)]**
**[[:octocat:](https://github.com/marioney/hybrid_simulation/tree/decision-making)]**
**[[🎞️](http://rulebooks.tech/)]**
**[[🎞️](https://video.ethz.ch/events/2019/rsl/28a0302b-64eb-4ebe-a609-1dc05fcdb038.html)]**
**[** :mortar_board: `ETH Zurich` **]**
**[** :car: `nuTonomy`, `Aptiv` **]**

- **[** _`sampling-based planning`, `safety validation`, `reward function`, `RSS`_  **]**

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

Authors: Censi, A., Slutsky, K., Wongpiromsarn, T., Yershov, D., Pendleton, S., Fu, J., & Frazzoli, E.

- Allegedly how **nuTonomy** (an [**Aptiv**](https://www.aptiv.com/autonomous-mobility) company) cars work.

- One main concept: **_"rulebook"_**.
  - It contains multiple **`rules`**, that specify the **desired behaviour** of the self-driving cars.
  - A rule is simply a **scoring function**, or **“violation metric”**, on the _realizations_ (= trajectories).
  - The **degree of violation** acts like some **penalty term**: here some examples of evaluation of a realization `x` evaluated by a rule `r`:
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
    - **Top**: Guarantee **safety** of humans.
      - This is written **analytically** (e.g. a precise expression for the kinetic energy to minimize harm to people).
    - **Bottom**: _Comfort_ constraints and _progress_ goals.
      - Can be **learnt** based on observed behaviour (and also tend to be platform- and implementation- specific).
    - **Middle**: All the other priorities among rule groups
      - There are somehow **open for discussion**.
- How to build a rulebook:
  - Rules can be defined **analytically** (e.g. `LTL` formalism) or **learnt** from data (for non-safety-critical rules).
  - **Violation functions** can be **learned from data** (e.g. `IRL`).
  - **Priorities** between rules can also be learnt.
- One idea: **manipulation** of rulebooks.
  - **Regulations** and cultures differ depending on the country and the state.
  - A rulebook <**`R`**, **`<`**> can easily be **adapted** using three operations (`priority refinement`, `rule augmentation`, `rule aggregation`).

- Related work: Several topics raised in this paper reminds me subjects addressed in [Emilio Frazzoli, CTO, nuTonomy - 09.03.2018](https://www.youtube.com/watch?v=dWSbItd0HEA)
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
    - Driving baheviours are **hard to code**.
    - Driving baheviours are **hard to learn**.
    - But driving baheviours are **easy to assess**.
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

</details>

---

**`"Provably Safe and Smooth Lane Changes in Mixed Traffic"`**

- **[** `2019` **]**
**[[:memo:](https://www.mrt.kit.edu/z/publ/download/2019/Naumann2019LaneChange.pdf)]**
**[[:octocat:](https://github.com/coincar-sim)]**
**[[🎞️](https://www.mrt.kit.edu/z/publ/download/2019_LaneChange_Naumann.mp4)]**
**[** :mortar_board: `FZI`, `KIT` **]**

- **[** _`path-velocity decomposition`, `IDM`, `RSS`_ **]**

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

Authors: Naumann, M., Königshof, H., & Stiller, C.

- Main ideas:
  - The notion of **_safety_** is based on the **responsibility sensitive safety** (`RSS`) definition.
    - As stated by the authors, _"A **`safe`** lane change is guaranteed not to **`cause`** a collision according to the previously defined rules, while a single vehicle cannot ensure that it will never be involved in a collision."_
  - Use _set-based reachability analysis_ to prove the "RSS-safety" of lane change manoeuvre based on **gap evaluation**.
    - In other words, it is the **responsibility** of the ego vehicle to **maintain safe distances** during the lane change manoeuvre.

- Related works: A couple of safe distances are defined, building on
  - **`RSS`** principles (_after IV19, I tried to summarize some of the RSS concepts [here](https://github.com/chauvinSimon/IV19#rss)_).
  - [_"Verifying the Safety of Lane Change Maneuvers of Self-driving Vehicles Based on Formalized Traffic Rules"_](https://mediatum.ub.tum.de/doc/1379669/794156.pdf), (Pek, Zahn, & Althoff, 2017)

</details>

---

**`"Decision-Making Framework for Autonomous Driving at Road Intersections: Safeguarding Against Collision, Overly Conservative Behavior, and Violation Vehicles"`**

- **[** `2018` **]**
**[[:memo:](https://ieeexplore.ieee.org/document/8370800)]**
**[[🎞️](https://www.youtube.com/watch?v=h7ExZ040wyk)]**
**[** :mortar_board: `Daejeon Research Institute, South Korea` **]**

- **[** _`probabilistic risk assessment`, `rule-based probabilistic decision making`_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![[Source](https://ieeexplore.ieee.org/document/8370800).](media/2018_noh.PNG "[Source](https://ieeexplore.ieee.org/document/8370800).")  |
|:--:|
| *[Source](https://ieeexplore.ieee.org/document/8370800).* |

Author: Noh, S.

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

</details>
