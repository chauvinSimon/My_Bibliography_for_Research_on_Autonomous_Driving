# `Behavioural Cloning` `End-To-End` and `Imitation Learning`

---

**`"Learning Scalable Self-Driving Policies for Generic Traffic Scenarios"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2011.06775)]**
**[[🎞️](https://sites.google.com/view/dignet-self-driving/)]**
**[** :mortar_board: `Hong Kong University` **]**

- **[** _`BEV`, `VAE`, `GAT`, `close-loop evaluation`, `generalization`, [`carla`](http://carla.org/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2011.06775).](../media/2020_cai_1.png "[Source](https://arxiv.org/abs/2011.06775).")  |
|:--:|
| *The core idea is to **encode the `BEV` with `VAE`**. And then incorporate this **latent vector** with **`state` vectors** (e.g., locations and speeds) into `GNNs` to model the **complex interactions** among road agents. The resulting `state` is finally fed to a **`policy` net** that **clones demonstrations** and produce **high-level commands** (**target `speed`** `vT` and **course angle `θT`**) implemented by a `PID` controller. [Source](https://arxiv.org/abs/2011.06775).* |

| ![[Source](https://sites.google.com/view/dignet-self-driving/).](../media/2020_cai_2.gif "[Source](https://sites.google.com/view/dignet-self-driving/).")  |
|:--:|
| *[Source](https://sites.google.com/view/dignet-self-driving/).* |

Authors: Cai, P., Wang, H., Sun, Y., & Liu, M.

- Motivations:
  - `1-` Derive a `policy` that can **generalize** to **multiple scenarios**.
    - As opposed to most works that address **one particular scenario** such as `T-intersections`.
      - A **state machine** could aggregate individual the **scattered cases**.
      - > "Current learning models have not been well designed for **scalable** self-driving in a **uniform setup**."
    - > "We propose a **graph-based** deep network to achieve **unified and scalable** self-driving in **diverse dynamic environments**."
  - `2-` Obey traffic rules such as **speed limits** and **traffic lights**.
    - As opposed to works that **neglect the environmental structures**.
      - > "... which is not that important for indoor robot navigation in restricted areas, but is **non-negligible** for outdoor self-driving problems where **traffic rules** should be strictly obeyed."
  - `3-` **Open-loop** evaluation of `BC` is not always a good indicator.
    - > "As reported in [`ChauffeurNet`](https://arxiv.org/abs/1812.03079) and [`Exploring the limitations of behavior cloning for autonomous driving`](https://arxiv.org/abs/1904.08980), the **offline evaluation** of driving models may **not reflect the real driving performance**."
    - For instance, a `MLP`-based model has smaller or similar prediction errors than `GNN`-based methods, but its **closed-loop performance is much worse**.
    - > "We conduct `19,200` episodes to thoroughly evaluate `8` driving models (over `7,500` km)."

- Key idea: **`mid-to-mid` behavioural cloning**.
  - The **driving scene** is encoded as a semantic `BEV`, as in [`ChauffeurNet`](https://arxiv.org/abs/1812.03079).
  - Benefits of a `BEV`:
    - `1-` Can cope with **arbitrary number of objects**.
    - `2-` Can include environmental **structures**, such as **drivable area** and **lane markings**.
      - > "Very helpful because it provides valuable **structural priors** on the **motion** of surrounding road agents. For example, vehicles normally **drive on lanes** rather than on sidewalks. Another benefit is that vehicles need to drive according to **traffic rules**, such as not crossing solid lane markings."
    - `3-` `sim-2-real`: there is **no domain difference** between the simulation and real world, thus the **policy transfer** problem can be alleviated.
  
  - Output:
    - > "we adopt a **mid-level** output representation indicating the **target `speed`** `vT` and **course angle `θT`** rather than **direct vehicle control** commands (e.g., `steering` and `throttle`)."
    - `1-` First, the `policy` **regresses two scalars**:
      - `κv` in [`0`, `1`]
      - `κc` in [`−1`, `1`]
    - `2-` Then the **target control values** can be computed:
      - `vT` = `vlim` × `κv`
      - `θT` = `90◦` × `κc`

- Should the `BEV` be **used directly as an input** for the "cloner" net?
  - **No, it is too big!**
    - > "Such **high-dimensionality** makes it not only hard to learn good policies in **data scarce tasks**, but also suffer from **over-fitting problems**. Therefore, a **lower-dimensional embedding** for the multi-channel `BEV` input is needed for us to train a high-performance driving policy."
  - Rather some **compressed** version of it.
    - A variational autoencoder (`VAE`) is trained to:
    - `1-` **Reconstruct** the input `BEV`.
    - `2-` Make sure that the **latent space** is **regular** enough with a standard normal distribution `N`(`0`, `I`).

- **Input** of the **cloning `policy`**. A fusion of:
  - `1-` Processed (`MLP`) **`ego-route`**.
    - > "The **first waypoint** is the **closest waypoint** in `Gf` to the **current vehicle location**, and the distance of every two adjacent points is **`0.4` m**."
    - > "Note the **redundancy of route information** in `G` and `BEV` is meaningful, where the `state` vector here provides **more specific waypoint locations** for the vehicle to follow, while the **`route` mask of `BEV`** can indicate if there are **any obstacles on conflicting lanes** of planned routes, as well as encode traffic light information."
  - `2-` Processed (`MLP`) **ego-motion** vector `m`:
    - Current **control command** (`steering`, `throttle` and `brake`).
    - `speed limit`.
    - Difference between this `speed limit` and the current `speed`.
    - **Cross track error** and `heading angle` error.
    - Two binary indicators that indicate whether the **neighbouring line** is **crossable** (a `broken` line) or not (`solid` line).
  - `3-` Processed (`MLP`) graph (two `GAT` layers) fed with:
    - The **context embedding**, i.e. the **`BEV` encoded by the `VAE`**.
    - Processed (`MLP`) **`node` information** for each vehicle:
      - `location`, distance to the ego-vehicle, `yaw` angle, `velocity`, `acceleration` and `size`.
    - > "We are interested in the result `ho1` of the **first `node`**, which represents the **influence on the ego-vehicle**."

- `BC` task:
  - Dataset: `260` episodes = `7.6` hours = **`150` km**.
  - **`L1` loss** in terms of `vT` and `θT`.
    - _Weighting between the two?_

- About **graph convolutional networks (`GCN`s)**:
  - > "They generalize the **`2D` convolution on grids to graph-structured** data. When training a `GCN`, a **fixed adjacency matrix** is commonly adopted to **aggregate feature information** of neighboring `nodes`."
  - > "Graph attention network ([`GAT`](https://arxiv.org/abs/1710.10903)) is a `GCN` variant which **aggregates node information** with **weights learned in a self-attention mechanism**. Such adaptiveness of `GAT` makes it more effective than `GCN` in graph representation learning."
  - `GAT` is used here to model the **interaction** among road agents during driving, which is composed of **multiple graph layers**.

- To sum up: ingredients of `DiGNet` (**_"driving in graphs"_**):
  - `1-` Semantic bird’s-eye view (`BEV`) images to **model road topologies** and **traffic rules**.
    - **`7` channels** of `80`×`140`:
      - HD map: `drivable area` and lane markings (`solid` and `broken` lines).
      - `ego-route`.
      - Objects: `ego-vehicle`, other `vehicles` and `pedestrians`.
      - _No `traffic light`?_
  - `2-` Variational auto-encoder (`VAE`) to **extract** effective and interpretable environmental features.
  - `3-` **Graph attention** networks (`GAT`) to **model the complex interactions** among traffic agents.

</details>

---

**`"Driving Through Ghosts: Behavioral Cloning with False Positives"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2008.12969)]**
**[[🎞️](https://www.youtube.com/watch?v=2o2F5m1vwS0)]**
**[** :mortar_board: `ETH Zurich` **]**
**[** :car: `Toyota` **]**

- **[** _`uncertainty`, `false positive`, `high recall`, `noise injection`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2008.12969).](../media/2020_buehler_1.PNG "[Source](https://arxiv.org/abs/2008.12969).")  |
|:--:|
| *The `observation` is represented in a `birds-eye-view` grid, where detections are coded as **pairs**: each dimension represents an `object` or `feature type` together with the **respective estimated confidences**. To build the grid, **ghosts are added** based on a sampled **confidence value**. [Source](https://arxiv.org/abs/2008.12969).* |

| ![[Source](https://www.youtube.com/watch?v=2o2F5m1vwS0).](../media/2020_buehler_1.gif "[Source](https://www.youtube.com/watch?v=2o2F5m1vwS0).")  |
|:--:|
| *A **threshold** on the **`confidence scores`** can be **hand-crafted**. This leads to **conservative** or **inconsistent behaviours**. On the other hand, the `Soft BEV` agent remains invariant to the noise in the input features: it is able to **ignore ghosts** but **stop before real vehicles**. Without the need for **tuning**. [Source](https://www.youtube.com/watch?v=2o2F5m1vwS0).* |

Authors: Bühler, A., Gaidon, A., Cramariuc, A., Ambrus, R., Rosman, G., & Burgard, W.

- Motivations:
  - `1-` Reduce **conservative behaviour** in **uncertain** situations, while simultaneously **acting safely**.
    - In particular, to better deal with **`false positives`** without the **need for thresholds**.
  - `2-` Focus on **uncertain `inputs`**, i.e. **imperfect perception**, as opposed to _dynamics_ or _intention_ uncertainties for instance.
    - > "We focus on **critical perception mistakes** in the form of **detection `false positives`**, as this is typically the **main failure mode** of safe systems biased towards negligible `false negative` rates (as `false negatives` are **irrecoverable in `mediated perception`** approaches)."
  - `3-` Aim at robustness.
    - In particular cope with the fact that the **`noise distribution` during `training`** may differ from the `test` one.
    - > "In the case of **imitation learning**, perceptual errors at **training time** can lead to learning difficulties, as expert demonstrations might be **inconsistent** with the world state perceived by the vehicle."
    - > "We therefore show that our `Soft BEV` model is able to maintain the **same level of safety** despite **not knowing the true underlying noise distribution** and even when this **distribution diverges from the training distribution**, which is the more realistic scenario."

- Behavioural cloning.
  - Challenge: it is operating over a **different input space**.
  - The **expert** has access to the **ground truth** while the robot might suffer from **false positive perceptions**.
  - > "A key challenge lies in **potential inconsistencies** between **observations `o`** and the **true state `s`**, for instance in the presence of **false positives in `o`**."

- How to **represent/encode** the **perceptual uncertainty**?
  - **Probabilistic birds-eye-view (`Soft BEV`) grid**.
  - _Why `soft`?_
    - Because **`observation` are pairs**: the **object presence is not binary**, but have a **`confidence value`** in [`0`; `1`].
    - > "We model **observations `o` = (`ˆs`, `c`)** as pairs of **estimated perceptual states `ˆs`** and **black-box `confidence values` `c`** in [`0`; `1`] for each `state` variable."
  - About the **training noise**.
    - > "We do not make explicit assumptions about the **distribution** of `ˆs` w.r.t. the **true state `s`**. Instead, we assume the perception system is **tuned for high recall**, i.e., that all critical state variables are (noisily) captured in the estimated state. This comes at the potential expense of **false positives**, but corresponds to the practical setup where **safety requirements** are generally designed to avoid partial observability issues, as **false negatives are hard to recover from**.
    - > "We use **two truncated `Normal` distributions** (values between `0` and `1`) to simulate the **confidence score** generated by a generic perception stack, and approximate the perception system’s confidence estimate on its failure modes."
    - > "After sampling **true positive confidence values** for the actual objects, we **sample ghost-objects** in the vicinity of the ego agent with a **probability of `10%`** and sample a **confidence value** for this **false positive** object as well."

- Two agents:
  - `1-` **`BEV` Agent** is trained on **ground truth noise-free data**, meaning it will have **no notion of uncertainty**.
  - `2-` **`Soft BEV` Agent** is trained on **noisy data** and its input is the **`Soft BEV` grid**.

- Test setup.
  - [NoCrash Carla Benchmark](https://github.com/carla-simulator/driving-benchmarks). Unfortunately for an old version: `carla 0.8.4`. A [non-official version](https://github.com/dianchen96/LearningByCheating) for `0.9.6` is available.
  - Metrics include the `time to completion`.
  - _How to assess the `average speed` of agent in scenarios where traffic lights or jams require the agent to stop?_
    - > "To reduce the influence of **traffic lights**, we only consider **`velocities` above `1m/s` when computing the `average velocity`**. The reason for this is that traffic lights act as a **binary gate** and essentially either **increase variance significantly**, or eliminate the influence of **speed difference** along the routes. Similarly, we only consider **`accelerations` with an absolute value above zero**."
  - About the **testing noise**:
    > "To model **temporal consistency** of the **false positives** we employ a **_survival model_**. In other words a **false positive** is created with **probability `pFP`**, and it will be present in the consecutive frames with a **decaying probability**. In our experiments, we set the decay rate to `0.8`, thus allowing false positives to **typically survive `2-3` frames**."

</details>

---

**`"Action-based Representation Learning for Autonomous Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2008.09417)]**
**[[:octocat:](https://github.com/yixiao1/Action-Based-Representation-Learning)]**
**[[🎞️](https://www.youtube.com/watch?v=fFywCMlLbyE)]**
**[** :mortar_board: `University of Moncton` **]**

- **[** _`representation learning`, `affordances`, `self-supervised`, `pre-training`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2008.09417).](../media/2020_xiao_1.PNG "[Source](https://arxiv.org/abs/2008.09417).")  |
|:--:|
| *So--called **`direct` perception approach** uses **`affordances`** to select low-level driving commands. Examples of `affordances` include `is there a car in my lane within 10m?` or `relative angles deviation between my car's heading and the lane` to control the car. This approach offers **interpretability**. The question here is **''how to efficiently extract these `affordances`?''**. For this `classification` / `regression` **supervised learning** task, the encoder is first **pre-trained** on another task (**proxy tasks**) involving learning from **`action` demonstration** (e.g. `behavioural cloning`). The intuition is that for **a learnt `BC` model** able to take good driving decisions most of the time, **relevant information should have been captured in its encoder**. [Source](https://arxiv.org/abs/2008.09417).* |

| ![[Source](https://arxiv.org/abs/2008.09417).](../media/2020_xiao_2.PNG "[Source](https://arxiv.org/abs/2008.09417).")  |
|:--:|
| *Different `self-supervised learning` tasks based on **`action` prediction** can be used to produce the **encoder**. An other idea is to use models trained with **supervised learning**. For instance `ResNet`, whose encoder performs worse. Probably because of the **synthetic images**? [Source](https://arxiv.org/abs/2008.09417).* |

| ![[Source](https://github.com/yixiao1/Action-Based-Representation-Learning).](../media/2020_xiao_1.gif "[Source](https://github.com/yixiao1/Action-Based-Representation-Learning).")  |
|:--:|
| *Four **`affordances`** are **predicted from images**. They represent the **explicit detection of hazards** involving pedestrians and vehicles, respecting traffic lights and considering the **heading of the vehicle** within the current lane. `PID` controllers convert these `affordances` into `throttle`, `brake` and `steering` commands. [Source](https://github.com/yixiao1/Action-Based-Representation-Learning).* |

Authors: Xiao, Y., Codevilla, F., Pal, C., & López, A. M.

- One sentence:
  - > "**Expert demonstrations** can act as an effective `action`-based **representation learning** technique."

- Motivations:
  - `1-` Leverage **driving demonstration data** that can be easily obtained by **simply recording** the `actions` of good drivers.
  - `2-` Be more **interpretable** than pure `end-to-end` imitation methods.
  - `3-` Be **less annotation dependent**, i.e. rely preferably on **self-supervision** and try to **reduce data supervision** (i.e., human annotation).
    - > "Our method uses **much less densely annotated data** and does not use **dataset aggregation (`DAgger`)**."

- Main idea:
  - Use both **manually annotated data** (`supervised learning`) and **expert demonstrations** (`imitation learning`) to **learn to extract `affordances` from images** (`representations learning`).
  - This combination is beneficial, compared to taking each approach separately:
    - `1-` Pure `end-to-end` imitation methods such as `behavioural cloning` could be used to directly **predict `control` actions** (`throttle`, `brake`, `steering`).
      - > "In this pure **data-centered approach**, the supervision required to train deep `end-to-end` driving models does not come from **human annotation**; instead, the vehicle’s state variables are used as **self-supervision** (e.g. `speed`, `steering`, `acceleration`, `braking`) since these can be **automatically collected** from fleets of human-driven vehicles."
      - But it **lacks interpretability**, can have difficulty dealing with **spurious correlations** and training may be unstable.
    - `2-` Learning to extract the `affordances` from scratch would not be very efficient.
      - One could use a **pre-trained backbone** such as `ResNet`. But it does not necessary **capture the relevant information** of the driving scene to make decisions.
      - > "**Action-based pre-training** (`Forward`/`Inverse`/`BC`) outperforms all the other reported **pre-training** strategies (e.g. `ImageNet`). However, we see that the **action-based pre-training** is mostly beneficial to help on **reliably estimating the vehicle’s relative angle** with respect to the road."

- About **`driving affordances`** and **`direct` perception approach**:
  - > "A different paradigm, conceptually midway between **pure modular** and **end-to-end** driving ones, is the so-called **_`direct perception` approach_**, which focuses on learning deep models to **predict driving `affordances`**, from which an additional controller can maneuver the AV. In general, such **`affordances`** can be understood as a **relatively small set of interpretable variables** describing **events that are relevant for an agent acting** in an environment. Driving `affordances` bring **interpretability** while **only requiring weak supervision**, in particular, human annotations just at the **image level** (i.e., not pixel-wise)."
  - **Four `affordances`** used by a **rule-based** controller to select `throttle`, `brake` and `steering` commands:
    - `1-` **Pedestrian** hazard (`bool`): _Is there is a_ **_pedestrian_** _in our lane at a distance lower than `10m`?_
    - `2-` **Vehicle** hazard (`bool`): _Is there is a_ **_vehicle_** _in our lane at a distance lower than `10m`?_
    - `3-` **Red** traffic light (`bool`): _Is there is a_ **_traffic light in red affecting our lane_** _at a distance lower than `10m`?_
    - `4-` Relative **heading angle** (`float`): **relative angle** of the longitudinal vehicle axis with respect to the lane in which it is navigating.
  - Extracting these `affordances` from the sensor data is called **"representation learning"**.
  - Main idea here:
    - Learn to **extract these `affordances`** (supervised learning), with the **encoder being pre-trained** on the task of **`end-to-end` driving**, e.g. `BC`.

- Two stages with two datasets: A **_self_-supervised** dataset and a **_weakly_-supervised** dataset.
  - `1-` Train an **`end-to-end` driving model** (e.g. `BC`) from (e.g. expert) **demonstrations**.
    - The **final layers for `action` prediction** are discarded.
    - Only the first part, i.e. the **encoder**, is used for the second stage.
    - It is called **"_self_-supervised"** since it does not require **external manual annotations**.
  - `2-` Use this **pre-trained encoder** together with a multi-layer perceptron (`MLP`) to **predict `affordances`**.
    - The **pre-training** (stage `1`) is beneficial because all the **relevant information for driving** should have been **extracted by the encoder**.
    - It is called **"_weakly_-supervised"** since it only requires **_image-level_ `affordance` annotations**, i.e. not pixel-level.

- _Why is it called "`action`-based" method?_
  - So far, I mention **`behaviour cloning`** (`BC`) as a learning method that **focus on predicting the control `actions`** and whose **learnt encoder** can be used.
  - For instance, **`inverse dynamics` models**.
    - > "Predicting the **next states** of an agent, or the **action between state transitions**, yields useful representations."
  - > [_Is it a good idea to use_ **_random `actions`_**_?_] "We show that learning from **expert data** in our approach leads to **better representations** compared to **training `inverse dynamics` models**. This shows that expert **driving** data (i.e. coming from human drivers) is an important source for **representation learning**."

</details>

---

**`"Learning to drive by imitation: an overview of deep behavior cloning methods"`**

- **[** `2020` **]**
**[[:memo:](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods)]**
**[** :mortar_board: `University of Moncton` **]**

- **[** _`overview`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).](../media/2020_ly_1.PNG "[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).")  |
|:--:|
| *Some **simulators** and **datasets** for supervised learning of `end-to-end` driving. [Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).* |

| ![[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).](../media/2020_ly_2.PNG "[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).")  |
|:--:|
| *Instead of just **single front-view camera** frames (top and left), **other sensor modalities** can be used as **inputs**, for instance **event-based** cameras (bottom-right). [Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).* |

| ![[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).](../media/2020_ly_3.PNG "[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).")  |
|:--:|
| *The **temporal** evolution of the scene can be captured by considering a **sequence of past frames**. [Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).* |

| ![[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).](../media/2020_ly_4.PNG "[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).")  |
|:--:|
| *Other approaches also address the **`longitudinal` control** (top and right), while some try to **exploit intermediate representations** (bottom-left). [Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).* |

| ![[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).](../media/2020_ly_5.PNG "[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).")  |
|:--:|
| *[Source](https://www.researchgate.net/publication/342190981_Learning_to_drive_by_imitation_an_overview_of_deep_behavior_cloning_methods).* |

Authors: Ly, A. O., & Akhloufi, M.

- Motivation:
  - An **overview** of the current state of the art **deep `behaviour cloning`** methods for lane stable driving.
  - > [No `RL`] "By `end-to-end`, we mean **supervised methods** that map **raw visual inputs** to **low-level** (`steering` angle, `speed`, etc) or **high-level** (driving `path`, driving `intention`, etc.) of actuation commands using almost only deep networks."

- **Five classes** of methods:
  - `1-` **Pure imitative** methods that make use of vanilla `CNNs` and take **standard camera frames** only as input.
    - The **loss** can be computed using the Mean Squared Error (`MSE`) between **predictions** and **`steering` labels**.
    - > "Recovery from mistakes is made possible by **adding synthesized data** during training via simulations of **car deviation** from the center of the lane."
    - > "**Data augmentation** was performed using a **basic viewpoint transformation** with the help of the `left` and `right` cameras."
  - `2-` Models that use **other types of perceptual sensors** such as **`events` or `fisheye` cameras** etc.
    - > "A **more realistic label augmentation** is achieved with the help of the **wide range** of captures from the front **fisheye** camera compared to previous methods using shearing with side (right and left) cameras."
    - > "**`Events` based cameras** consist of **independent pixels** that record intensity variation in an asynchronous way. Thus, giving **more information** in a time interval than traditional video cameras where changes taking place between two consecutive frames are not captured."
  - `3-` Methods that consider **previous driving history** to estimate future driving commands.
  - `4-` Systems that predicts **both `lateral` and `longitudinal` control** commands.
    - > "It outputs the **vehicle `curvature`** instead of the `steering angle` as generally found in the literature, which is justified by the fact that `curvature` is **more general** and **does not vary from vehicle to vehicle**."

  - `5-` Techniques that leverage the power of **`mid-level` representations** for **transfer learning** or give **more explanation** in regards to taken actions.
    - > "The motivation behind using a `VAE` architecture is to automatically **mitigate the bias issue** which occurs because generally the driving scenes in the datasets does not have the same proportions. In previous methods, this issue is solved by **manually reducing** the over represented scenes such as `straight driving` or `stops`."

- Some take-aways:
  - > "Models making use of **non-standard cameras** or **intermediate representations** are showing a lot of potential in comparison to **pure imitative** methods that takes conventional **video frames** as input."
  - > "The **diversity in metrics and datasets** used for reporting the results makes it very hard to strictly weigh the different models against each other."
  - **Explainability and transparency** of taken decisions is important.
    - > "A common approach in the literature is to analyse the **pixels** that lead to the **greatest activation** of neurons."

</details>

---

**`"Advisable Learning for Self-driving Vehicles by Internalizing Observation-to-Action Rules"`**

- **[** `2020` **]**
**[[:memo:](http://openaccess.thecvf.com/content_CVPR_2020/papers/Kim_Advisable_Learning_for_Self-Driving_Vehicles_by_Internalizing_Observation-to-Action_Rules_CVPR_2020_paper.pdf)]**
**[** :mortar_board: `UC Berkeley` **]**
**[[:octocat:](https://github.com/JinkyuKimUCB/Advisable-Learning)]**

- **[** _`attention`, `advisability`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](http://openaccess.thecvf.com/content_CVPR_2020/papers/Kim_Advisable_Learning_for_Self-Driving_Vehicles_by_Internalizing_Observation-to-Action_Rules_CVPR_2020_paper.pdf).](../media/2020_kim_1.PNG "[Source](http://openaccess.thecvf.com/content_CVPR_2020/papers/Kim_Advisable_Learning_for_Self-Driving_Vehicles_by_Internalizing_Observation-to-Action_Rules_CVPR_2020_paper.pdf).")  |
|:--:|
| *[Source](http://openaccess.thecvf.com/content_CVPR_2020/papers/Kim_Advisable_Learning_for_Self-Driving_Vehicles_by_Internalizing_Observation-to-Action_Rules_CVPR_2020_paper.pdf).* |

| ![[Source](http://openaccess.thecvf.com/content_CVPR_2020/papers/Kim_Advisable_Learning_for_Self-Driving_Vehicles_by_Internalizing_Observation-to-Action_Rules_CVPR_2020_paper.pdf).](../media/2020_kim_2.PNG "[Source](http://openaccess.thecvf.com/content_CVPR_2020/papers/Kim_Advisable_Learning_for_Self-Driving_Vehicles_by_Internalizing_Observation-to-Action_Rules_CVPR_2020_paper.pdf).")  |
|:--:|
| *[Source](http://openaccess.thecvf.com/content_CVPR_2020/papers/Kim_Advisable_Learning_for_Self-Driving_Vehicles_by_Internalizing_Observation-to-Action_Rules_CVPR_2020_paper.pdf).* |

Authors: Kim, J., Moon, S., Rohrbach, A., Darrell, T., & Canny, J.

- Related **PhD thesis**: [Explainable and Advisable Learning for Self-driving Vehicles](https://www2.eecs.berkeley.edu/Pubs/TechRpts/2020/EECS-2020-16.pdf), (Kim. J, 2020)

- Motivation:
  - An **`end-to-end` model** should be **explainable**, i.e. provide **easy-to-interpret rationales** for its behaviour:
  - `1-` Summarize / the **visual observations** (input) in **natural language**, e.g. _"light is red"_.
    - `Visual attention` is not enough, **verbalizing** is needed.
  - `2-` Predict an **appropriate action response**, e.g. _"I see a pedestrian crossing, so I stop"_.
    - I.e. Justify the decisions that are made and explain why they are reasonable in a **human understandable manner**, i.e., again, in **natural language**.
  - `3-` Predict a **control signal**, accordingly.
    - The command is **conditioned** on the predicted **high-level action command**, e.g. _"maintain a slow speed"_.
    - The output is a **sequence of waypoints**, hence **`end-to-mid`**.

- About the dataset:
  - **Berkeley [`DeepDrive-eXplanation`](https://github.com/JinkyuKimUCB/BDD-X-dataset)** (`BDD-X`) dataset _(by the first author)_.
  - Together with _camera front-views_ and _IMU signal_, the dataset provides:
    - `1-` **Textual descriptions** of the vehicle's actions: **_what_** _the driver is doing_.
    - `2-` **Textual explanations** for the driver's actions: **_why_** _the driver took that action from the point of view of a driving instructor_.
      - For instance the pair: (_`"the car slows down"`_, _`"because it is approaching an intersection"`_).

</details>

---

**`"Feudal Steering: Hierarchical Learning for Steering Angle Prediction"`**

- **[** `2020` **]**
**[[:memo:](http://openaccess.thecvf.com/content_CVPRW_2020/papers/w60/Johnson_Feudal_Steering_Hierarchical_Learning_for_Steering_Angle_Prediction_CVPRW_2020_paper.pdf)]**
**[** :mortar_board: `Rutgers University` **]**
**[** :car: `Lockheed Martin` **]**

- **[** _`hierarchical learning`, `temporal abstraction`, `t-SNE embedding`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](http://openaccess.thecvf.com/content_CVPRW_2020/papers/w60/Johnson_Feudal_Steering_Hierarchical_Learning_for_Steering_Angle_Prediction_CVPRW_2020_paper.pdf).](../media/2020_johnson_1.PNG "[Source](http://openaccess.thecvf.com/content_CVPRW_2020/papers/w60/Johnson_Feudal_Steering_Hierarchical_Learning_for_Steering_Angle_Prediction_CVPRW_2020_paper.pdf).")  |
|:--:|
| *__Feudal learning__ for `steering` prediction. The **worker** decides the next `steering` angle **conditioned on a `goal`** (`subroutine id`) determined by the **manager**. The **manager** learns to predict these `subroutine ids` from a sequence of **past `states`** (`break`, `steer`, `throttle`). The ground truth `subroutine ids` are the **centres of centroids** obtained by **unsupervised clustering**. They should contain observable **semantic meaning** in terms of driving tasks. [Source](http://openaccess.thecvf.com/content_CVPRW_2020/papers/w60/Johnson_Feudal_Steering_Hierarchical_Learning_for_Steering_Angle_Prediction_CVPRW_2020_paper.pdf).* |

Authors: Johnson, F., & Dana, K.

- Note: Although terms and ideas from hierarchical reinforcement learning (`HRL`) are used, **no `RL` is applied here!**

- Motivation: **Temporal abstraction**.
  - Problems in `RL`: **delayed rewards** and **sparse credit assignment**.
  - Some solutions: **intrinsic rewards** and **temporal abstraction**.
  - The idea of **temporal abstraction** is to **break down** the problem into more tractable pieces:
    - > "At all times, human drivers are **paying attention** to **two levels** of their environment. The first level goal is on a **finer grain**: _don’t hit obstacles in the immediate vicinity of the vehicle_. The second level goal is on a **coarser grain**: _plan actions a few steps ahead to maintain the proper course efficiently_."

- The idea of **feudal learning** is to **divide the task** into:
  - `1-` A **manager** network.
    - It **operates at a lower temporal resolution** and produces **`goal` vectors** that it passes to the **worker network**.
    - This `goal` vector should encapsulate a **_temporally extended action_** called a `subroutine`, `skill`, `option`, or `macro-action`.
    - Input: Sequence of previous `steering`.
    - Output: `goal`.
  - `2-` A **worker** network: **_conditioned_** on the `goal` decided by the manager.
    - Input: `goal` decided by the manager, `previous own prediction`, sequence of `frames`.
    - Output: `steering`.
  - The **`subroutine ids`** (manager net) and the `steering angle` prediction (worker net) are jointly learnt.

- _What are the ground truth `goal` used to train the manager?_
  - They are **ids** of the **centres of centroids formed by clustering** (unsupervised learning) all the training data:
    - `1-` Data: `Steering`, `braking`, and `throttle` data are concatenated every `m=10` time steps to make a vector of length `3m=30`.
    - `2-` Encoding: projected in a **`t-SNE` `2d`-space**.
    - `3-` Clustering: `K-means`.
    - The `2d`-coordinates of **centroids of the clusters** are the **`subroutine ids`**, i.e. the possible `goals`.
      - _How do they convert the `2d`-coordinates into a single scalar?_
  - > "We aim to **classify the `steering` angles into their temporally abstracted subroutines**, also called `options` or `macro-actions`, associated with highway driving such as `follow the sharp right bend`, `bumper-to-bumper traffic`, `bear left slightly`."

- _What are the decision frequencies?_
  - The worker considers the last `10` actions to decide the `goal`.
  - It seems like a **smoothing process**, where a **window is applied**?
    - _It should be possible to achieve that with a recurrent net, shouldn't it?_

- About **`t-SNE`**:
  - > "**`t`-Distributed Stochastic Neighbor Embedding (`t-SNE`)** is an **unsupervised**, non-linear technique primarily used for **data exploration and visualizing** high-dimensional data. In simpler terms, `t-SNE` gives you a feel or intuition of how the data is arranged in a **high-dimensional space** [[from `towardsdatascience`](https://towardsdatascience.com/an-introduction-to-t-sne-with-python-example-5a3a293108d1)]."
  - Here it is used as an **embedding space** for the driving data and as the **subroutine ids** themselves.

</details>

---

**`"A Survey of End-to-End Driving: Architectures and Training Methods"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.06404)]**
**[** :mortar_board: `University of Tartu` **]**

- **[** _`review`, `distribution shift problem`, `domain adaptation`, `mid-to-mid`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.06404).](../media/2020_tampuu_3.PNG "[Source](https://arxiv.org/abs/2003.06404).")  |
|:--:|
| *Left: example of `end-to-end` architecture with key terms. Right: difference `open-loop` / `close-loop` evaluation. [Source](https://arxiv.org/abs/2003.06404).* |

| ![[Source](https://arxiv.org/abs/2003.06404).](../media/2020_tampuu_2.PNG "[Source](https://arxiv.org/abs/2003.06404).")  |
|:--:|
| *[Source](https://arxiv.org/abs/2003.06404).* |

| ![[Source](https://arxiv.org/abs/2003.06404).](../media/2020_tampuu_1.PNG "[Source](https://arxiv.org/abs/2003.06404).")  |
|:--:|
| *[Source](https://arxiv.org/abs/2003.06404).* |

Authors: Tampuu, A., Semikin, M., Muhammad, N., Fishman, D., & Matiisen, T.

- _A rich literature overview and some useful reminders about general `IL` and `RL` concepts with focus to `AD` applications._
  - It constitutes a good complement to the **_"Related trends in research"_** part of **my video ["From RL to Inverse Reinforcement Learning: Intuitions, Concepts + Applications to Autonomous Driving"](https://youtu.be/wBfd2Kn-IgU?t=8688)**.
- I especially like the structure of the document: It shows **what one should consider when starting an `end-to-end` / `IL` project for `AD`**:
  - _I have just noted here some ideas I find interesting. In no way an exhaustive summary!_

- `1-` **Learning methods**: working with `rewards` (`RL`) or with `losses` (`behavioural cloning`).
  - About **`distribution shift problem`** in `behavioural cloning`:
    - > "If the driving decisions lead to **unseen situations** (not present in the training set), the model might no longer know how to behave".
    - Most solutions try to **diversify the training data** in some way - either by **_collecting_** or **_generating_** additional data:
      - `data augmentation`: e.g. one can place **two additional cameras pointing forward-left and forward-right** and associate the images with commands to `turn right` and `turn left` respectively.
      - `data diversification`: addition of **temporally correlated noise** and synthetic **trajectory perturbations**. Easier on "semantic" inputs than on camera inputs.
      - `on-policy learning`: **recovery annotation** and **`DAgger`**. The **expert provides examples** how to solve situations the model-driving leads to. Also ["Learning by cheating"](http://vladlen.info/papers/learning-by-cheating.pdf) by (Chen et al. 2019).
      - `balancing the dataset`: by **upsampling the rarely** occurring angles, **downsampling the common ones** or by **weighting** the samples.
        - > "Commonly, the collected datasets contain large amounts of **repeated traffic situations** and only few of those **rare events**."
        - The authors claim that only the **_joint_ distribution** of `inputs` and `outputs` defines the **rarity of a data point**.
        - > "Using more training data from `CARLA` `Town1` decreases generalization ability in `Town2`. This illustrates that **more data without more _diversity_ is not useful**."
        - Ideas for augmentation can be taken from the field of `supervised Learning` where it is already an largely-addressed topic.
  - About `RL`:
    - Policies can be first trained with IL and then fine-tuned with RL methods.
    - > "This approach reduces the long training time of RL approaches and, as the RL-based fine-tuning happens online, also helps overcome the problem of IL models learning off-policy".
  - About **`domain adaptation`** and **`transfer`** from simulation to real world (`sim2real`).
    - Techniques from `supervised` learning, such as `fine tuning`, i.e. **adapting the driving model** to the new distribution, are rarely used.
    - Instead, one can instead **adapt the incoming data** and **keep the driving model fixed**.
      - A first idea is to **transform _real_ images** into _simulation-like_ images (the opposite - generating **real-looking images** - is challenging).
      - One can also extract the **semantic segmentation** of the scene from both the **real** and the **simulated images** and use it as the input for the driving policy.

- `2-` **Inputs**.
  - In short:
    - `Vision` is key.
    - `Lidar` and `HD-map` are nice to have but expensive / tedious to maintain.
    - Additional inputs from independent modules (`semantic segmentation`, `depth map`, `surface normals`, `optical flow` and `albedo`) can improve the robustness.
  - About the **`inertia problem`** / **`causal confusion`** when for instance **predicting the next `ego-speed`**.
    - > "As in the vast majority of samples the **current** _[observed]_ and next **speeds** _[to be predicted]_ are **highly correlated**, the model learns to base its **speed prediction** exclusively on **current speed**. This leads to the model being **reluctant to change its speed**, for example to **start moving again** after stopping behind another car or a at traffic light."
  - About **`affordances`**:
    - > "Instead of parsing all the objects in the driving scene and performing robust localization (as **modular approach**), the system focuses on a **small set of crucial indicators**, called `affordances`."

- `3-` **Outputs**.
  - > "The outputs of the model define the **level of understanding** the model is expected to achieve."
  - Also related to the **`time horizon`**:
    - > "When predicting **instantaneous low-level commands**, we are not explicitly forcing the model to **plan a long-term trajectory**."
  - Three types of **predictions**:
    - `3-1` **_Low-level_** commands.
      - > "The majority of end-to-end models yield as output the **`steering angle` and `speed`** (or `acceleration` and `brake` commands) for the next timestep".
      - Low-level commands may be **car-specific**. For instance vehicles answer differently to the same `throttle` / `steering` commands.
        - > "The function between **steering wheel angle** and the **resulting turning radius** depends on the car's geometry, making this measure specific to the car type used for recording."
      - > [About the regression loss] "Many authors have recently optimized `speed` and `steering` commands using **`L1` loss** (mean absolute error, `MAE`) instead of `L2` loss (mean squared error, `MSE`)".
    - `3-2` Future **waypoints** or desired **trajectories**.
      - This **higher-level** output modality is **independent of car geometry**.
    - `3-3` **Cost map**, i.e. information about where it is safe to drive, leaving the **trajectory generation** to another module.
  - About **multitask learning** and **auxiliary tasks**:
    - The idea is to **simultaneously train** a separate set of networks to predict for instance `semantic segmentation`, `optical flow`, `depth` and other **human-understandable representations** from the camera feed.
    - > "Based on the **same extracted visual features** that are fed to the decision-making branch (_main_ task), one can also predict `ego-speed`, `drivable area` on the scene, and `positions` and `speeds` of other objects".
    - It offers **more learning signals** - at least for the shared layers.
    - And can also help **understand the mistakes** a model makes:
      - > "A failure in an **auxiliary task** (e.g. object detection) might suggest that necessary information was **not present already in the intermediate representations** (layers) that it shared with the main task. Hence, also the main task did not have access to this information and might have **failed for the same reason**."

- `4-` **Evaluation**: the difference between **`open-loop`** and **`close-loop`**.
  - `4-1` `open-loop`: like in `supervised` learning:
    - _one question = one answer_.
    - Typically, a dataset is split into **training** and **testing data**.
    - Decisions are compared with the **recorded actions** of the demonstrator, assumed to be the **ground-truth**.
  - `4-2` `close-loop`: like in **decision _processes_**:
    - The problem consists in a **multi-step** **interaction** with some environment.
    - It directly measures the model's ability to **drive on its own**.
  - Interesting facts: Good **`open-loop` performance** does not necessarily lead to good driving ability in **`closed-loop` settings**.
    - > "Mean squared error (`MSE`) **correlates with `closed-loop` success rate only weakly** (correlation coefficient `r = 0.39`), so `MAE`, `quantized classification error` or `thresholded relative error` should be used instead (`r > 0.6` for all three)."
    - About the **`balanced-MAE` metric** for `open-loop` evaluation, which **correlates better** with `closed-loop` performance than simple `MAE`.
      - > "`Balanced-MAE` is computed by **averaging the mean values** of **unequal-length bins** according to `steering angle`. Because most data lies in the region around steering angle `0`, equally weighting the bins grows the importance of **rarely occurring** (higher) `steering angle`s."

- `5-` **Interpretability**:
  - `5-1` Either on the `trained` model ...
    - > "**Sensitivity analysis** aims to determine the **parts of an input** that a model is most sensitive to. The most common approach involves computing the **gradients** with respect to the input and using the magnitude as the **measure of sensitivity**."
    - `VisualBackProp`: which **input pixels** influence the cars driving decision the most.
  - `5-2` ... or already during `training`.
    - > "`visual attention` is a built-in mechanism present **already when learning**. Where to attend in the next timestep (the **attention mask**), is predicted as additional output in the current step and can be made to depend on additional sources of information (e.g. textual commands)."

- About `end-to-end` neural nets and humans:
  - > "[`StarCraft`, `Dota 2`, `Go` and `Chess` solved with `NN`]. Many of these solved tasks are in many aspects **more complex than driving a car**, a task that a large proportion of **people successfully perform even when tired or distracted**. A person can later recollect nothing or very little about the route, suggesting the task needs **very little conscious attention** and might be a **simple behavior reflex task**. It is therefore reasonable to believe that in the near future an `end-to-end` approach is also capable to autonomously control a vehicle."

</details>

---

**`"Efficient Latent Representations using Multiple Tasks for Autonomous Driving"`**

- **[** `2020` **]**
**[[:memo:](https://arxiv.org/abs/2003.00695)]**
**[** :mortar_board: `Aalto University` **]**

- **[** _`latent space representation`, `multi-head decoder`, `auxiliary tasks`_ **]**

<details>
  <summary>Click to expand</summary>

| ![[Source](https://arxiv.org/abs/2003.00695).](../media/2020_kargar_1.PNG "[Source](https://arxiv.org/abs/2003.00695).")  |
|:--:|
| *The `latent representation` is enforced to predict the **trajectories of both the ego vehicle and other vehicles** in addition to the input image, using a **multi-head** network structure. [Source](https://arxiv.org/abs/2003.00695).* |

Authors: Kargar, E., & Kyrki, V.

- Motivations:
  - `1-` Reduce the **dimensionality** of the `feature representation` of the scene - used as input to some `IL` / `RL` policy.
    - This is to improve most **`mid-to-x` approaches** that encode and process a vehicle’s environment as **multi-channel** and quite **high-dimensional bird view images**.
    - `->` The idea here is to learn an `encoder-decoder`.
    - The **latent space has size `64`** _(way smaller than common `64 x 64 x N` bird-views)_.
  - `2-` Learn a `latent representation` faster / with fewer data.
    - A single head decoder would just consider `reconstruction`.
    - `->` The idea here is to use have **multiple heads** in the decoder, i.e. make prediction of **multiple auxiliary** application relevant factors.
    - > "The **multi-head model** can reach the single-head model’s performance in `20` epochs, **one-fifth of training time** of the **single-head** model, with full dataset."
    - > "In general, the **multi-heal** model, using **only `6.25%`** of the dataset, **converges faster** and perform better than single head model trained on the full dataset."
  - `3-` Learn a `policy` faster / with fewer data.
- Two components to train:
  - `1-` An **`encoder-decoder`** learns to produce a latent representation (`encoder`) coupled with a **multiple-prediction-objective (`decoder`)**.
  - `2-` A `policy` use the **latent representation** to predict **low-level controls**.
- About the **`encoder-decoder`**:
  - `inputs`: **bird-view** image containing:
    - **Environment info**, built from `HD Maps` and `perception`.
    - **Ego trajectory**: `10` past poses.
    - **Other trajectory**: `10` past poses.
    - It forms a `256 x 256` image, which is resized to `64 x 64` to feed them into the models
  - `outputs`: multiple **auxiliary tasks**:
    - `1-` **Reconstruction head**: reconstructing the **input bird-view image**.
    - `2-` **Prediction head**: `1s`-**motion-prediction** for **other** agents.
    - `3-` **Planning head**: `1s`-**motion-prediction** for the **ego** car.
- About the `policy`:
  - In their example, the authors implement **`behaviour cloning`**, i.e. _supervised learning_ to reproduce the decision of `CARLA autopilot`.
  - `1-` `steering` prediction.
  - `2-` `acceleration` classification - `3` classes.
- _How to deal with the_ **_unbalanced dataset_**_?_
  - First, the authors note that **no manual labelling is required** to collect training data.
  - But the recorded `steering` angle is **zero most of the time** - leading to a **highly imbalanced** dataset.
  - Solution (_no further detail_):
    - > "Create a new dataset and balance it using **_sub-sampling_**".

</details>

---

**`"Robust Imitative Planning : Planning from Demonstrations Under Uncertainty"`**

- **[** `2019` **]**
**[[:memo:](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf)]**
**[** :mortar_board: `University of Oxford`, `UC Berkeley`, `Carnegie Mellon University` **]**

- **[** _`epistemic uncertainty`, `risk-aware decision-making`, [`CARLA`](http://carla.org/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![Illustration of the **`state distribution shift`** in behavioural cloning (`BC`) approaches. The models (e.g. neural networks) usually fail to generalize and instead **extrapolate confidently yet incorrectly**, resulting in arbitrary outputs and **dangerous** outcomes. Not to mention the **compounding (or cascading) errors**, inherent to the **sequential** decision making. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).](../media/2019_tigas_1.PNG "Illustration of the **`state distribution shift`** in behavioural cloning (`BC`) approaches. The models (e.g. neural networks) usually fail to generalize and instead **extrapolate confidently yet incorrectly**, resulting in arbitrary outputs and **dangerous** outcomes. Not to mention the **compounding (or cascading) errors**, inherent to the **sequential** decision making. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).")  |
|:--:|
| *Illustration of the **`state distribution shift`** in behavioural cloning (`BC`) approaches. The models (e.g. neural networks) usually fail to generalize and instead **extrapolate confidently yet incorrectly**, resulting in arbitrary outputs and **dangerous** outcomes. Not to mention the **compounding (or cascading) errors**, inherent to the **sequential** decision making. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).* |

| ![Testing behaviours on scenarios such as **roundabouts** that are **not present in the training set**. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).](../media/2019_tigas_2.PNG "Testing behaviours on scenarios such as **roundabouts** that are **not present in the training set**. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).")  |
|:--:|
| *Testing behaviours on scenarios such as **roundabouts** that are **not present in the training set**. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).* |

| ![Above - in their previous work, the authors introduced **`Deep imitative models`** (`IM`). The _imitative planning_ objective is the **log posterior probability** of a state trajectory, **conditioned** on satisfying some goal `G`. The state trajectory that has the **highest likelihood** w.r.t. the **expert model** `q`(`S` given `φ`; `θ`) is selected, i.e.  maximum a posteriori probability (**`MAP`**) estimate of how an expert would drive to the goal. This captures any inherent **`aleatoric` stochasticity** of the human behaviour (e.g., multi-modalities), but only uses a **point-estimate** of `θ`, thus `q`(`s` given `φ`;`θ`) **does not quantify model** (i.e. `epistemic`) uncertainty. `φ` denotes the **contextual information** (`3` previous states and current LIDAR observation) and `s` denotes the agent’s future states (i.e. the **trajectory**). Bottom - in this works, an ensemble of models is used: `q`(`s` given `φ`; `θk`) where `θk` denotes the **parameters** of the `k`-th model (neural network). The **`Aggregation Operator`** operator is applied on the posterior p(`θ` given `D`). The previous work is one example of that, where a single `θi` is selected. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).](../media/2019_tigas_4.PNG "Above - in their previous work, the authors introduced **`Deep imitative models`** (`IM`). The _imitative planning_ objective is the **log posterior probability** of a state trajectory, **conditioned** on satisfying some goal `G`. The state trajectory that has the **highest likelihood** w.r.t. the **expert model** `q`(`S` given `φ`; `θ`) is selected, i.e.  maximum a posteriori probability (**`MAP`**) estimate of how an expert would drive to the goal. This captures any inherent **`aleatoric` stochasticity** of the human behaviour (e.g., multi-modalities), but only uses a **point-estimate** of `θ`, thus `q`(`s` given `φ`;`θ`) **does not quantify model** (i.e. `epistemic`) uncertainty. `φ` denotes the **contextual information** (`3` previous states and current LIDAR observation) and `s` denotes the agent’s future states (i.e. the **trajectory**). Bottom - in this works, an ensemble of models is used: `q`(`s` given `φ`; `θk`) where `θk` denotes the **parameters** of the `k`-th model (neural network). The **`Aggregation Operator`** operator is applied on the posterior p(`θ` given `D`). The previous work is one example of that, where a single `θi` is selected. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).")  |
|:--:|
| *Above - in their previous work, the authors introduced **`Deep imitative models`** (`IM`). The _imitative planning_ objective is the **log posterior probability** of a state trajectory, **conditioned** on satisfying some goal `G`. The state trajectory that has the **highest likelihood** w.r.t. the **expert model** `q`(`S` given `φ`; `θ`) is selected, i.e.  maximum a posteriori probability (**`MAP`**) estimate of how an expert would drive to the goal. This captures any inherent **`aleatoric` stochasticity** of the human behaviour (e.g., multi-modalities), but only uses a **point-estimate** of `θ`, thus `q`(`s` given `φ`;`θ`) **does not quantify model** (i.e. `epistemic`) uncertainty. `φ` denotes the **contextual information** (`3` previous states and current LIDAR observation) and `s` denotes the agent’s future states (i.e. the **trajectory**). Bottom - in this works, an ensemble of models is used: `q`(`s` given `φ`; `θk`) where `θk` denotes the **parameters** of the `k`-th model (neural network). The **`Aggregation Operator`** operator is applied on the posterior p(`θ` given `D`). The previous work is one example of that, where a single `θi` is selected. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).* |

| ![To **save computation** and improve runtime to real-time, the authors use a trajectory library: they perform `K-means` clustering of the expert plan’s from the training distribution and keep `128` of the centroids. I see that as a way **restrict the search in the trajectory space**, similar to injecting _expert_ knowledge about the feasibility of cars trajectories. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).](../media/2019_tigas_3.PNG "To **save computation** and improve runtime to real-time, the authors use a trajectory library: they perform `K-means` clustering of the expert plan’s from the training distribution and keep `128` of the centroids. I see that as a way **restrict the search in the trajectory space**, similar to injecting _expert_ knowledge about the feasibility of cars trajectories. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).")  |
|:--:|
| *To **save computation** and improve runtime to real-time, the authors use a trajectory library: they perform `K-means` clustering of the expert plan’s from the training distribution and keep `128` of the centroids, allegedly reducing the planning time by a factor of `400`. During optimization, the `trajectory space` is limited to **only that trajectory library**. It makes me think of `templates` sometimes used for **path-planning**. I also see that as a way **restrict the search in the trajectory space**, similar to injecting _expert_ knowledge about the feasibility of cars trajectories. [Source](https://ml4ad.github.io/files/papers/Robust%20Imitative%20Planning:%20Planning%20from%20Demonstrations%20Under%20Uncertainty.pdf).* |

| ![__Estimating__ the uncertainty is **not enough**. One should then **forward** that estimate to the `planning` module. This reminds me an idea of [(McAllister et al., 2017)](https://www.ijcai.org/proceedings/2017/0661.pdf) about the key benefit of **propagating uncertainty** throughout the AV framework. [Source](https://www.ijcai.org/proceedings/2017/0661.pdf).](../media/2017_mc_allister_1.PNG "__Estimating__ the uncertainty is **not enough**. One should then **forward** that estimate to the `planning` module. This reminds me an idea of [(McAllister et al., 2017)](https://www.ijcai.org/proceedings/2017/0661.pdf) about the key benefit of **propagating uncertainty** throughout the AV framework. [Source](https://www.ijcai.org/proceedings/2017/0661.pdf).")  |
|:--:|
| *__Estimating__ the uncertainty is **not enough**. One should then **forward** that estimate to the `planning` module. This reminds me an idea of [(McAllister et al., 2017)](https://www.ijcai.org/proceedings/2017/0661.pdf) about the key benefit of **propagating uncertainty** throughout the AV framework. [Source](https://www.ijcai.org/proceedings/2017/0661.pdf).* |

Authors: Tigas, P., Filos, A., Mcallister, R., Rhinehart, N., Levine, S., & Gal, Y.

- Previous work: [`"Deep Imitative Models for Flexible Inference, Planning, and Control"`](https://arxiv.org/abs/1810.06544v4) (see below).
  - The idea was to combine the benefits of **`imitation learning`** (**`IL`**) and **`goal-directed planning`** such as `model-based RL` (**`MBRL`**).
    - In other words, to complete **planning** based on some **imitation prior**, by combining **generative modelling** from demonstration data with **planning**.
    - One key idea of this **generative model of expert behaviour**: perform context-conditioned **density estimation** of the distribution over future expert trajectories, i.e. score the **"expertness"** of any plan of future positions.
  - Limitations:
    - It **only** uses a **point-estimate** of `θ`. Hence it fails to capture **epistemic uncertainty** in the model’s density estimate.
    - > "Plans can be **risky** in scenes that are **out-of-training-distribution** since it **confidently extrapolates** in novel situations and lead to catastrophes".
- Motivations here:
  - `1-` Develop a model that captures **epistemic uncertainty**.
  - `2-` Estimating uncertainty is not a goal at itself: one also need to provide a mechanism for **taking low-risk actions** that are likely to recover in uncertain situations.
    - I.e. both `aleatoric` and `epistemic` uncertainty should be taken into account in the **planning objective**.
    - This reminds me the figure of [(McAllister et al., 2017)](https://www.ijcai.org/proceedings/2017/0661.pdf) about the key benefit of **propagating uncertainty** throughout the AV framework.
- One quote about behavioural cloning (`BC`) that suffers from **state distribution shift** (`co-variate shift`):
  - > "Where high capacity parametric models (e.g. neural networks) usually **fail to generalize**, and instead **extrapolate confidently yet incorrectly**, resulting in arbitrary outputs and dangerous outcomes".
- One quote about **_model-free_ `RL`**:
  - > "The specification of a **reward function** is as hard as solving the original control problem in the first place."
- About `epistemic` and `aleatoric` uncertainties:
  - > "**Generative models** can provide a **measure** of their uncertainty in different situations, but **robustness** in novel environments requires estimating **`epistemic uncertainty`** (e.g., _have I been in this state before?_), where conventional density estimation models only capture **`aleatoric uncertainty`** (e.g., _what’s the frequency of times I ended up in this state?_)."

- _How to_ **_capture uncertainty_** _about previously unseen scenarios?_
  - Using an **ensemble** of **density estimators** and **aggregate operators** over the models’ outputs.
    - > "By using demonstration data to learn **density models** over human-like driving, and then estimating its uncertainty about these densities using an **ensemble** of imitative models".
  - The idea it to **take the disagreement** between the models into consideration and **inform planning**.
    - > "When a trajectory that **was never seen before** is selected, the model’s **high `epistemic` uncertainty** pushes us away from it. During planning, the **disagreement between the most probable trajectories** under the ensemble of imitative models is used to inform planning."

</details>

---

**`"End-to-end Interpretable Neural Motion Planner"`**

- **[** `2019` **]**
**[[:memo:](http://www.cs.toronto.edu/~wenjie/papers/cvpr19/nmp.pdf)]**
**[** :mortar_board: `University of Toronto` **]**
**[** :car: `Uber` **]**

- **[** _`interpretability`, `trajectory sampling`_ **]**

<details>
  <summary>Click to expand</summary>

| ![The visualization of `3D` detection, **motion forecasting** as well as learned **cost-map volume** offers interpretability. A set of **candidate trajectories** is **sampled**, first considering the geometrical **path** and then then **speed** profile. The trajectory with the **minimum learned cost** is selected. [Source](http://www.cs.toronto.edu/~wenjie/papers/cvpr19/nmp.pdf).](../media/2019_zeng_1.PNG "The visualization of `3D` detection, **motion forecasting** as well as learned **cost-map volume** offers interpretability. A set of **candidate trajectories** is **sampled**, first considering the geometrical **path** and then then **speed** profile. The trajectory with the **minimum learned cost** is selected. [Source](http://www.cs.toronto.edu/~wenjie/papers/cvpr19/nmp.pdf).")  |
|:--:|
| *The visualization of `3D` detection, **motion forecasting** as well as learned **cost-map volume** offers interpretability. A set of **candidate trajectories** is **sampled**, first considering the geometrical **path** and then then **speed** profile. The trajectory with the **minimum learned cost** is selected. [Source](http://www.cs.toronto.edu/~wenjie/papers/cvpr19/nmp.pdf).* |

| ![[Source](http://www.cs.toronto.edu/~wenjie/papers/cvpr19/nmp.pdf).](../media/2019_zeng_2.PNG "[Source](http://www.cs.toronto.edu/~wenjie/papers/cvpr19/nmp.pdf).")  |
|:--:|
| *[Source](http://www.cs.toronto.edu/~wenjie/papers/cvpr19/nmp.pdf).* |

Authors: Zeng W., Luo W., Suo S., Sadat A., Yang B., Casas S. & Urtasun R.

- Motivation is to **bridge the gap** between the `traditional engineering stack` and the `end-to-end driving` frameworks.
  - `1-` Develop a **learnable** motion planner, avoiding the costly parameter tuning.
  - `2-` Ensure **interpretability** in the motion decision. This is done by offering an **intermediate representation**.
  - `3-` Handle **uncertainty**. This is allegedly achieved by using a learnt, non-parametric **cost function**.
  - `4-` Handle **multi-modality** in possible trajectories (e.g `changing lane` vs `keeping lane`).

- One quote about `RL` and `IRL`:
  - > "It is unclear if `RL` and `IRL` can **scale** to more realistic settings. Furthermore, these methods do not produce **interpretable representations**, which are desirable in safety critical applications".

- Architecture:
  - `Input`: raw LIDAR data and a HD map.
  - `1st intermediate result`: An **_"interpretable"_** bird’s eye view representation that includes:
    - `3D` detections.
    - Predictions of **future trajectories** (planning horizon of `3` seconds).
    - Some **spatio-temporal** **cost volume** defining the **goodness of each position** that the self-driving car can take within the planning horizon.
  - `2nd intermediate result`: A set of diverse physically possible trajectories (candidates).
    - They are `Clothoid` curves being **sampled**. First building the `geometrical path`. Then the `speed profile` on it.
    - > "Note that **`Clothoid` curves** can not handle circle and **straight line trajectories** well, thus we sample them separately."
  - `Final output`: The trajectory with the **minimum learned cost**.

- Multi-objective:
  - `1-` **`Perception`** Loss - to **predict** the position of vehicles at every time frame.
    - _Classification_: Distinguish a vehicle from the **background**.
    - _Regression_: Generate precise object **bounding boxes**.
  - `2-` **`Planning`** Loss.
    - > "Learning a reasonable **cost volume** is challenging as we do not have **ground-truth**. To overcome this difficulty, we minimize the **`max-margin` loss** where we use the **ground-truth trajectory as a _positive_ example**, and **randomly sampled trajectories** as **_negative_ examples**."
    - As stated, the intuition behind is to **encourage** the demonstrated trajectory to have the **minimal cost**, and others to have higher costs.
    - The model hence **learns a cost volume** that **discriminates good trajectories from bad ones**.

</details>

---

**`"Learning from Interventions using Hierarchical Policies for Safe Learning"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1912.02241)]**
**[** :mortar_board: `University of Rochester, University of California San Diego` **]**
- **[** _`hierarchical`, `sampling efficiency`, `safe imitation learning`_ **]**

<details>
  <summary>Click to expand</summary>

| ![The main idea is to use **Learning from Interventions** (**`LfI`**) in order to ensure **safety** and improve **data efficiency**, by **intervening on sub-goals** rather than trajectories. Both `top-level` policy (that generates **sub-goals**) and `bottom-level` policy are **jointly learnt**. [Source](https://arxiv.org/abs/1912.02241).](../media/2019_bi_1.PNG "The main idea is to use **Learning from Interventions** (**`LfI`**) in order to ensure **safety** and improve **data efficiency**, by **intervening on sub-goals** rather than trajectories. Both `top-level` policy (that generates **sub-goals**) and `bottom-level` policy are **jointly learnt**. [Source](https://arxiv.org/abs/1912.02241).")  |
|:--:|
| *The main idea is to use **Learning from Interventions** (**`LfI`**) in order to ensure **safety** and improve **data efficiency**, by **intervening on sub-goals** rather than trajectories. Both `top-level` policy (that generates **sub-goals**) and `bottom-level` policy are **jointly learnt**. [Source](https://arxiv.org/abs/1912.02241).* |

Authors: Bi, J., Dhiman, V., Xiao, T., & Xu, C.

- Motivations:
  - `1-` Improve **data-efficiency**.
  - `2-` Ensure **safety**.
- One term: **_"Learning from Interventions"_** (**`LfI`**).
  - One way to classify the _"learning from expert"_ techniques is to use the frequency of **expert’s engagement**.
    - `High frequency`    -> Learning from **Demonstrations**.
    - `Medium frequency`  -> learning from **Interventions**.
    - `Low frequency`     -> Learning from **Evaluations**.
  - Ideas of `LfI`:
    - > "When an **undesired state** is detected, another policy is activated to **take over actions** from the agent when necessary."
    - Hence the expert overseer only intervenes when it suspects that an unsafe action is about to be taken.
  - Two issues:
    - `1-` `LfI` (as for `LfD`) learn **reactive behaviours**.
      - > "Learning a supervised policy is known to have **'myopic' strategies**, since it **ignores the temporal dependence** between consecutive states".
      - Maybe one option could be to **stack frames** or to include the **current speed** in the `state`. But that makes the state space larger.
    - `2-` The expert only signals after a **non-negligible amount of delay**.
- One idea to solve both issues: **Hierarchy**.
  - The idea is to split the policy into **two hierarchical levels**, one that generates **`sub-goals`** for the future and another that generates **`actions`** to reach those desired sub-goals.
  - The motivation is to **intervene on sub-goals rather than trajectories**.
  - One important parameter: `k`
    - The **top-level policy** predicts a sub-goal to be achieved **`k` steps ahead** in the future.
    - It represents a trade-off between:
      - The ability for the `top-level` policy to **predict sub-goals far** into the future.
      - The ability for the `bottom-level` policy to **follow** it correctly.
  - One question: _How to deal with the_ **_absence of ground- truth sub-goals_** _?_
    - One solution is **_"Hindsight Experience Replay"_**, i.e. **consider an achieved goal** as a **desired goal for past observations**.
    - The authors present additional **interpolation** techniques.
    - They also present a `Triplet Network` to train goal-embeddings (_I did not understand everything_).

</details>

---

**`"Urban Driving with Conditional Imitation Learning"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1912.00177)]**
**[[🎞️](https://wayve.ai/blog/learned-urban-driving)]**
**[** :car: `Wayve` **]**

- **[** _`end-to-end`, `conditional IL`, `robust BC`_ **]**

<details>
  <summary>Click to expand</summary>

| ![The encoder is trained to reconstruct **`RGB`**, **`depth`** and **`segmentation`**, i.e. to **learn scene understanding**. It is augmented with **optical flow** for temporal information. As noted, such representations could be learned simultaneously with the driving policy, for example, through **distillation**. But for efficiency, this was pre-trained (Humans typically also have `~30` hours of driver training before taking the driving exam. But they start with huge prior knowledge). [Source](https://arxiv.org/abs/1912.00177).](../media/2019_hawke_1.PNG "The encoder is trained to reconstruct **`RGB`**, **`depth`** and **`segmentation`**, i.e. to **learn scene understanding**. It is augmented with **optical flow** for temporal information. As noted, such representations could be learned simultaneously with the driving policy, for example, through **distillation**. But for efficiency, this was pre-trained (Humans typically also have `~30` hours of driver training before taking the driving exam. But they start with huge prior knowledge). [Source](https://arxiv.org/abs/1912.00177).")  |
|:--:|
| *The encoder is trained to reconstruct **`RGB`**, **`depth`** and **`segmentation`**, i.e. to **learn scene understanding**. It is augmented with **`optical flow`** for temporal information. As noted, such representations could be learned simultaneously with the driving policy, for example, through **distillation**. But for efficiency, this was pre-trained (Humans typically also have `~30` hours of driver training before taking the driving exam. But they start with huge prior knowledge). Interesting idea: the **navigation `command`** is injected as **multiple locations** of the `control` part. [Source](https://arxiv.org/abs/1912.00177).* |

| ![Driving data is **inherently heavily imbalanced**, where most of the captured data will be driving **near-straight** in the middle of a lane. Any **naive training** will **collapse** to the dominant mode present in the data. **No data augmentation** is performed. Instead, during training, the authors **sample data uniformly** across **`lateral` and `longitudinal` control** dimensions. [Source](https://arxiv.org/abs/1912.00177).](../media/2019_hawke_2.PNG "Driving data is **inherently heavily imbalanced**, where most of the captured data will be driving **near-straight** in the middle of a lane. Any **naive training** will **collapse** to the dominant mode present in the data. **No data augmentation** is performed. Instead, during training, the authors **sample data uniformly** across **`lateral` and `longitudinal` control** dimensions. [Source](https://arxiv.org/abs/1912.00177).")  |
|:--:|
| *Driving data is **inherently heavily imbalanced**, where most of the captured data will be driving **near-straight** in the middle of a lane. Any **naive training** will **collapse** to the dominant mode present in the data. **No data augmentation** is performed. Instead, during training, the authors **sample data uniformly** across **`lateral` and `longitudinal` control** dimensions. [Source](https://arxiv.org/abs/1912.00177).* |

Authors: Hawke, J., Shen, R., Gurau, C., Sharma, S., Reda, D., Nikolov, N., Mazur, P., Micklethwaite, S., Griffiths, N., Shah, A. & Kendall, A.

- Motivations:
  - `1-` Learn **both** `steering` and `speed` via **Behavioural Cloning**.
  - `2-` Use **raw sensor** (camera) inputs, rather than intermediate representations.
  - `3-` Train and test on dense **urban environments**.
- _Why "conditional"?_
  - A **route command** (e.g. `turn left`, `go straight`) resolves the **ambiguity** of multi-modal behaviours (e.g. when coming at **an intersection**).
  - > "We found that inputting the command **multiple times at different stages** of the network improves robustness of the model".
- Some ideas:
  - Provide **wider state observability** through **multiple camera views** (single camera disobeys navigation interventions).
  - Add **temporal information** via **optical flow**.
    - Another option would be to **stack frames**. But it did not work well.
  - Train the **primary shared encoders** and **auxiliary independent decoders** for a number of **computer vision** tasks.
    - > "In robotics, the **`test` data is the real-world**, not a **static dataset** as is typical in most `ML` problems. Every time our cars go out, the **world is new and unique**."
- One concept: **_"Causal confusion"_**.
  - A good [video](https://www.youtube.com/watch?v=_dh2-2b1jmU) about [Causal Confusion in Imitation Learning](https://arxiv.org/abs/1905.11979) showing that **"access to more information leads to worse generalisation under distribution shift"**.
  - > "**Spurious correlations** cannot be distinguished from **true causes in the demonstrations**. [...] For example, **inputting the current speed** to the policy causes it to learn a **trivial identity mapping**, making the car **unable to start from a static position**."
  - Two ideas during training:
    - Using **flow features** to make the model use explicit motion information **without learning the trivial solution** of an **identity mapping for speed and steering**.
    - Add **random noise** and use dropout on it.
  - One alternative is to explicitly **maintain a causal model**.
  - Another alternative is to learn to **predict the speed**, as detailed in ["Exploring the Limitations of Behavior Cloning for Autonomous Driving"](https://arxiv.org/abs/1904.08980).
- Output:
  - The model decides of a **"motion plan"**, _i.e. not directly the low-level control?_
  - Concretely, the network gives one prediction and one slope, for both `speed` and `steering`, leading to two **parameterised lines**.
- Two types of tests:
  - `1-` **Closed-loop** (i.e. go outside and drive).
    - The number and type of safety-driver **interventions**.
  - `2-` **Open-loop** (i.e., evaluating on an _offline_ dataset).
    - The weighted mean absolute error for **`speed` and `steering`**.
      - As noted, this can serve as a proxy for real world performance.
  - > "As discussed by [34] and [35], the **correlation** between **`offline` `open-loop`** metrics and **`online` `closed-loop`** performance is **weak**."
- About the training data:
  - As stated, they are **two levers** to increase the performance:
    - `1-` **Algorithmic innovation**.
    - `2-` **Data**.
  - For this `IL` approach, `30` hours of demonstrations.
  - > "Re-moving a **quarter** of the data notably degrades performance, and models trained with less data are almost undriveable."
- Next steps:
  - I find the results already impressive. But as noted:
    - > "The learned driving policies presented here need significant further work to be comparable to human driving".
  - Ideas for improvements include:
    - Add some **predictive long-term planning model**. At the moment, it does not have access to **long-term** dependencies and cannot _reason_ about the road scene.
    - Learn not only from demonstration, but also **from mistakes**.
      - This reminds me the concept of `ChauffeurNet` about **_"simulate the bad rather than just imitate the good"_**.
    - **Continuous learning**: Learning from **corrective interventions** would also be beneficial.
  - The last point goes in the direction of adding **learning signals**, which was already done here.
    - **Imitation** of human expert drivers (`supervised` learning).
    - Safety driver **intervention** data (`negative reinforcement` learning) and **corrective action** (`supervised` learning).
    - Geometry, dynamics, motion and **future prediction** (`self-supervised` learning).
    - Labelled **semantic** computer vision data (`supervised` learning).
    - **Simulation** (`supervised` learning).

</details>

---

**`"Application of Imitation Learning to Modeling Driver Behavior in Generalized Environments"`**

- **[** `2019` **]**
**[[:memo:](https://www.bernardlange.com/s/Application-of-Imitation-Learning-to-Modeling-Driver-Behavior-in-Generalized-Environments.pdf)]**
**[** :mortar_board: `Stanford` **]**

- **[** _`GAIL`, `RAIL`, `domain adaption`, [`NGSIM`](https://github.com/sisl/ngsim_env)_ **]**

<details>
  <summary>Click to expand</summary>

| ![The `IL` models were trained on a **straight road** and tested on roads with **high curvature**. **`PS-GAIL`** is effective only while **surrounded by other vehicles**, while the **`RAIL`** policy remained stably within the bounds of the road thanks to the **additional rewards terms** included into the learning process.. [Source](https://www.bernardlange.com/s/Application-of-Imitation-Learning-to-Modeling-Driver-Behavior-in-Generalized-Environments.pdf).](../media/2019_lange_1.PNG "The `IL` models were trained on a **straight road** and tested on roads with **high curvature**. **`PS-GAIL`** is effective only while **surrounded by other vehicles**, while the **`RAIL`** policy remained stably within the bounds of the road thanks to the **additional rewards terms** included into the learning process.. [Source](https://www.bernardlange.com/s/Application-of-Imitation-Learning-to-Modeling-Driver-Behavior-in-Generalized-Environments.pdf).")  |
|:--:|
| *The `IL` models were trained on a **straight road** and tested on roads with **high curvature**. **`PS-GAIL`** is effective only while **surrounded by other vehicles**, while the **`RAIL`** policy remained stably within the bounds of the road thanks to the **additional rewards terms** included into the learning process.. [Source](https://www.bernardlange.com/s/Application-of-Imitation-Learning-to-Modeling-Driver-Behavior-in-Generalized-Environments.pdf).* |

Authors: Lange, B. A., & Brannon, W. D.

- One motivation: Compare the **robustness** (domain adaptation) of three **`IL`** techniques:
  - `1-` Generative Adversarial Imitation Learning ([**`GAIL`**](https://arxiv.org/abs/1606.03476)).
  - `2-` Parameter Sharing GAIL ([**`PS-GAIL`**](https://arxiv.org/abs/1803.01044)).
  - `3-` Reward Augmented Imitation Learning ([**`RAIL`**](https://arxiv.org/abs/1903.05766)).
- One take-away: This **student project** builds a **good overview** of the different `IL` algorithms and why these algorithms came out.
  - **Imitation Learning** (**`IL`**) aims at building an (efficient) **policy** using some **expert demonstrations**.
  - **Behavioural Cloning** (**`BC`**) is a sub-class of `IL`. It treats `IL` as a **supervised learning** problem: a **regression model** is fit to the `state`/`action` space given by the expert.
    - > Issue of **distribution shift**: "Because data is not infinite nor likely to **contain information** about all possible `state`/`action` pairs in a continuous `state`/`action` space, `BC` can display **undesirable effects** when **placed in these unknown or not well-known states**."
    - > "A **cascading effect** is observed as the time horizon grows and **errors expand** upon each other."
  - Several solutions (not exhaustive):
    - `1-` **`DAgger`**: **Ask the expert** to say what should be done in some encountered situations. Thus **iteratively enriching** the demonstration dataset.
    - `2-` **`IRL`**: Human driving behaviour is not modelled inside a **policy**, but rather capture into a **reward/cost function**.
      - Based on this reward function, an (optimal) **policy** can be derived with classic `RL` techniques.
      - One issue: It can be **computationally expensive**.
    - `3-` **`GAIL`** _(I still need to read more about it)_:
      - > "It fits distributions of states and actions given by an expert dataset, and a **cost function** is learned via **Maximum Causal Entropy `IRL`**."
      - > "When the `GAIL`-policy driven vehicle was placed in a **multi-agent** setting, in which multiple agents take over the learned policy, this algorithm produced undesirable results among the agents."
  - `PS-GAIL` is therefore introduced for **multi-agent** driving models (agents share a single policy learnt with `PS-TRPO`).
    - > "Though `PS-GAIL` yielded better results in **multi-agent** simulations than `GAIL`, its results still led to **undesirable driving characteristics**, including unwanted **trajectory deviation** and **off-road duration**."
  - `RAIL` offers a fix for that: the policy-learning process is **augmented with two types of reward terms**:
    - **Binary** penalties: e.g. _collision_ and _hard braking_.
    - **Smoothed** penalties: "applied in advance of undesirable actions with the theory that this would prevent these actions from occurring".
    - I see that technique as a way to **incorporate knowledge**.
- About the experiment:
  - The three policies were originally trained on the **straight roadway**: cars only consider the **lateral distance** to the edge.
  - In the "new" environment, a **road curvature** is introduced.
  - Findings:
    - > "None of them were able to fully accommodate the turn in the road."
    - `PS-GAIL` is effective **only while surrounded by other vehicles**.
    - The **smoothed** reward augmentation helped `RAIL`, but it was too late to avoid off-road (the car is already driving too fast and does not dare a `hard brake` which is strongly penalized).
    - The **reward function should therefore be updated** (back to **reward engineering** :sweat_smile:), for instance adding a **harder reward** term to prevent the car from leaving the road.

</details>

---

**`"Learning by Cheating"`**

- **[** `2019` **]**
**[[:memo:](http://vladlen.info/papers/learning-by-cheating.pdf)]**
**[[:octocat:](https://github.com/dianchen96/LearningByCheating)]**
**[** :mortar_board: `UT Austin` **]**
**[** :car: `Intel Labs` **]**

- **[** _`on-policy supervision`, `DAgger`, `conditional IL`, `mid-to-mid`, [`CARLA`](http://carla.org/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![The main idea is to **decompose** the **imitation learning** (`IL`) process into **two stages**: `1-` Learn to **act**. `2-` Learn to **see**. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).](../media/2019_chen_3.PNG "The main idea is to **decompose** the **imitation learning** (`IL`) process into **two stages**: `1-` Learn to **act**. `2-` Learn to **see**. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).")  |
|:--:|
| *The main idea is to **decompose** the **imitation learning** (`IL`) process into **two stages**: `1-` Learn to **act**. `2-` Learn to **see**. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).* |

| ![**`mid-to-mid`** learning: Based on a processed **`bird’s-eye view map`**, the **privileged agent** predicts a sequence of **waypoints** to follow. This _desired trajectory_ is eventually **converted into low-level commands** by two `PID` controllers. It is also worth noting how this `privileged` agent serves as an **oracle** that provides **adaptive on-demand supervision** to train the `sensorimotor` agent **across all possible commands**. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).](../media/2019_chen_4.PNG "**`mid-to-mid`** learning: Based on a processed **`bird’s-eye view map`**, the **privileged agent** predicts a sequence of **waypoints** to follow. This _desired trajectory_ is eventually **converted into low-level commands** by two `PID` controllers. It is also worth noting how this `privileged` agent serves as an **oracle** that provides **adaptive on-demand supervision** to train the `sensorimotor` agent **across all possible commands**. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).")  |
|:--:|
| ***`mid-to-mid`** learning: Based on a processed **`bird’s-eye view map`**, the **privileged agent** predicts a sequence of **waypoints** to follow. This _desired trajectory_ is eventually **converted into low-level commands** by two `PID` controllers. It is also worth noting how this `privileged` agent serves as an **oracle** that provides **adaptive on-demand supervision** to train the `sensorimotor` agent **across all possible commands**. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).* |

| ![Example of **_privileged map_** supplied to the first agent. And details about the **lateral `PID`** controller that **produces `steering` commands** based on a list of target waypoints. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).](../media/2019_chen_5.PNG "Example of **_privileged map_** supplied to the first agent. And details about the **lateral `PID`** controller that **produces `steering` commands** based on a list of target waypoints. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).")  |
|:--:|
| *Example of **_privileged map_** supplied to the first agent. And details about the **lateral `PID`** controller that **produces `steering` commands** based on a list of target waypoints. [Source](http://vladlen.info/papers/learning-by-cheating.pdf).* |

Authors: Chen, D., Zhou, B., Koltun, V. & Krähenbühl, P

- One motivation: **decomposing** the **imitation learning** (`IL`) process into **two stages**:
  - **`Direct IL`** (from expert trajectories to vision-based driving) conflates **two difficult tasks**:
    - `1-` Learning to **see**.
    - `2-` Learning to **act**.
- One term: **"_Cheating_"**.
  - `1-` First, train an agent that has access to **privileged information**:
    - > "This **privileged agent** _cheats_ by observing the **ground-truth** layout of the environment and the positions of all traffic participants."
    - Goal: The agent can **focus on learning to act** (it does not need to _learn to see_ because it gets direct access to the environment’s state).
  - `2-` Then, this **privileged agent** serves as a **teacher** to train a purely **vision-based** system (`abundant supervision`).
    - Goal: **Learning to see**.
- `1-` First agent (`privileged` agent):
  - Input: A processed **`bird’s-eye view map`** (with ground-truth information about _lanes_, _traffic lights_, _vehicles_ and _pedestrians_) together with high-level `navigation command` and `current speed`.
  - Output: A list of **waypoints** the vehicle should travel to.
  - Hence **`mid-to-mid`** learning approach.
  - Goal: imitate the **expert trajectories**.
  - Training: Behaviour cloning (`BC`) from a set of recorded **expert driving trajectories**.
    - **Augmentation** can be done _offline_, to facilitate **generalization**.
    - The agent is thus placed in a variety of **perturbed configurations** to learn how to **recover**
    - E.g. _facing the sidewalk or placed on the opposite lane_, it should find its way back onto the road.
- `2-` Second agent (`sensorimotor` agent):
  - Input: Monocular `RGB` image, current `speed`, and a high-level `navigation command`.
  - Output: A list of **waypoints**.
  - Goal: Imitate the **privileged agent**.
- One idea: **"White-box"** agent:
  - The **internal state** of the `privileged` agent can be examined at will.
    - Based on that, one could **test different high-level commands**: _"What would you do now if the command was [`follow-lane`] [`go left`] [`go right`] [`go straight`]"_.
  - This relates to `conditional IL`: all **conditional** branches are **supervised** during training.
- Another idea: **_"online learning"_** and **"_on-policy supervision"_**:
  - > "**“On-policy”** refers to the sensorimotor agent **rolling out its _own_ policy** during training."
    - Here, the decision of the second agents are directly implemented (`close-loop`).
    - And an **oracle** is still available for the **newly encountered situation** (hence `on-policy`), which also accelerates the training.
    - This is an advantage of using a **simulator**: it would be difficult/impossible in the physical world.
  - Here, the second agent is first trained `off-policy` (on expert demonstration) to speed up the learning (`offline BC`), and only then go `on-policy`:
    - > "Finally, we train the sensorimotor agent `on-policy`, **using the privileged agent as an oracle** that provides **adaptive on-demand supervision** in **any state reached** by the sensorimotor student."
    - The `sensorimotor` agent can thus be supervised on **all its waypoints** and **across all commands** at once.
  - It resembles the **Dataset aggregation** technique of [**`DAgger`**](https://www.cs.cmu.edu/~sross1/publications/Ross-AIStats11-NoRegret.pdf):
    - > "This enables **automatic `DAgger`-like training** in which supervision from the privileged agent is **gathered adaptively** via **online rollouts** of the sensorimotor agent."
- About the two benchmarks:
  - `1-` Original `CARLA` [benchmark](https://arxiv.org/abs/1711.03938) (`2017`).
  - `2-` **`NoCrash`** [benchmark](https://arxiv.org/abs/1904.08980) (`2019`).
  - Interesting idea for **timeout**:
    - > "The **time limit corresponds** to the amount of time needed to drive the route at a cruising speed of `10 km/h`".
- Another idea: Do not directly output **low-level** commands.
  - Instead, predict **waypoints** and **speed targets**.
  - And rely on two `PID` controllers to implement them.
    - > `1-` "We fit a **parametrized circular arc to all waypoints** using least-squares fitting and then steer towards a point on the arc."
    - > `2-` "A **longitudinal** `PID` controller tries to match a **target velocity** as closely as possible [...] We ignore negative throttle commands, and **only brake** if the predicted velocity is **below some threshold** (`2 km/h`)."

</details>

---

**`"Deep Imitative Models for Flexible Inference, Planning, and Control"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1810.06544v4)]**
**[[🎞️](https://sites.google.com/view/imitative-models)]**
**[[🎞️](https://www.youtube.com/watch?v=p-ltQdNFlVg)]**
**[[:octocat:](https://github.com/nrhine1/deep_imitative_models)]**
**[** :mortar_board: `Carnegie Mellon University`, `UC Berkeley` **]**

- **[** _`conditional IL`, `model-based RL`, [`CARLA`](http://carla.org/)_ **]**

<details>
  <summary>Click to expand</summary>

| ![The main motivation is to combine the benefits of **`IL`** (imitate expert demonstration) and **`model-based RL`** (i.e. **planning**). [Source](https://arxiv.org/abs/1810.06544v4).](../media/2019_rhinehart_1.PNG "The main motivation is to combine the benefits of **`IL`** (imitate expert demonstration) and **`goal-directed planning`** (e.g. `model-based RL`). [Source](https://arxiv.org/abs/1810.06544v4).")  |
|:--:|
| *The main motivation is to **combine the benefits** of **`IL`** (to imitate some expert demonstrations) and **`goal-directed planning`** (e.g. `model-based RL`). [Source](https://arxiv.org/abs/1810.06544v4).* |

| ![`φ` represents the scene consisted of the current `lidar scan`, `previous states` in the trajectory as well as the current `traffic light state`. [Source](https://arxiv.org/abs/1810.06544v4).](../media/2019_rhinehart_2.PNG "`φ` represents the scene consisted of the current `lidar scan`, `previous states` in the trajectory as well as the current `traffic light state`. [Source](https://arxiv.org/abs/1810.06544v4).")  |
|:--:|
| *__`φ`__ represents the **scene**. It consists of the current `lidar scan`, `previous states` in the trajectory as well as the current `traffic light state`. [Source](https://arxiv.org/abs/1810.06544v4).* |

| ![From left to right: `Point`, `Line-Segment` and `Region` (small and wide) **Final State Indicators** used for **planning**. [Source](https://arxiv.org/abs/1810.06544v4).](../media/2019_rhinehart_4.PNG "From left to right: `Point`, `Line-Segment` and `Region` (small and wide) **Final State Indicators** used for **planning**. [Source](https://arxiv.org/abs/1810.06544v4).")  |
|:--:|
| *From left to right: `Point`, `Line-Segment` and `Region` (small and wide) **Final State Indicators** used for **planning**. [Source](https://arxiv.org/abs/1810.06544v4).* |

| ![Comparison of features and implementations. [Source](https://arxiv.org/abs/1810.06544v4).](../media/2019_rhinehart_3.PNG "Comparison of features and implementations. [Source](https://arxiv.org/abs/1810.06544v4).")  |
|:--:|
| *Comparison of features and implementations. [Source](https://arxiv.org/abs/1810.06544v4).* |

Authors: Rhinehart, N., McAllister, R., & Levine, S.

- Main motivation: combine the benefits of **`imitation learning`** (**`IL`**) and **`goal-directed planning`** such as `model-based RL` (**`MBRL`**).
  - Especially to generate **interpretable**, **expert-like plans** with **offline learning** and **no reward engineering**.
  - Neither `IL` nor `MBRL` can do so.
  - In other words, it completes **planning** based on some **imitation prior**.
- One concept: **"_Imitative Models"_**
  - They are "probabilistic predictive models able to **_plan interpretable expert-like trajectories to achieve new goals_**".
  - As for `IL` -> use **expert demonstration**:
    - It generates **expert-like** behaviors **without reward function crafting**.
    - The model is learnt _"offline"_ also means it avoids costly online data collection (contrary to `MBRL`).
    - It learns **dynamics desirable behaviour models** (as opposed to learning the _dynamics of **_possible_** behaviour_ done by `MBRL`).
  - As for `MBRL` -> use **planning**:
    - It achieves **new goals** (goals that were not seen during training). Therefore, it avoids the theoretical **drift shortcomings** (_distribution shift_) of vanilla behavioural cloning (`BC`).
    - It outputs (_interpretable_) **`plan`** to them  at **test-time**, which `IL` cannot.
    - It does not need goal labels for training.
  - Binding `IL` and `planning`:
    - The learnt `imitative model` `q(S|φ)` can **generate trajectories** that resemble those that the expert might generate.
      - These manoeuvres do not have a **specific goal**. _How to direct our agent to goals?_
    - _General tasks_ are defined by a set of **goal variables `G`**.
      - At test time, a route planner provides **waypoints** to the **imitative planner**, which computes **expert-like paths for each candidate waypoint**.
    - The best plan is chosen according to the **planning objective** (e.g. _prefer routes avoiding potholes_) and provided to a low-level `PID`-controller in order to produce `steering` and `throttle` actions.
    - In other words, the derived **plan** (list of set-points) should be:
      - Maximizing the **similarity to the expert demonstrations** (term with `q`)
      - Maximizing the **probability of reaching some general goals** (term with `P(G)`).
    - _How to represent goals?_
      - `dim=0` - with points: `Final-State Indicator`.
      - `dim=1` - with lines: `Line-Segment Final-State Indicator`.
      - `dim=2` - with areas (regions): `Final-State Region Indicator`.

- _How to deal with traffic lights?_
  - The concept of **`smart waypointer`** is introduced.
  - > "It **removes far waypoints** beyond `5` meters from the vehicle when a **red light is observed** in the measurements provided by CARLA".
  - > "The planner prefers **closer goals when obstructed**, when the vehicle was already stopped, and when a **red light was detected** [...] The planner prefers **farther goals** when unobstructed and when **green lights** or no lights were observed."

- About **interpretability** and safety:
  - > "In contrast to **black-box one-step `IL`** that predicts _controls_, our method produces **interpretable multi-step plans** accompanied by two scores. One estimates the plan’s **`expertness`**, the second estimates its **probability to achieve the goal**."
    - The `imitative model` can produce some **expert probability distribution function (`PDF`)**, hence offering superior interpretability to one-step `IL` models.
    - It is able to score **how likely** a trajectory is to **come from the expert**.
    - The probability to achieve a goal is based on some "Goal Indicator methods" (using "Goal Likelihoods"). _I must say I did not fully understand that part_
  - The safety aspect relies on the fact that **experts were driving safely** and is formalized as a **_"plan reliability estimation"_**:
    - > "Besides using our model to make a best-effort attempt to reach a **user-specified goal**, the fact that our model **produces explicit likelihoods** can also be leveraged to test the **reliability** of a plan by evaluating **whether reaching particular waypoints will result in human-like behavior or not**."
    - Based on this idea, a **classification** is performed to **recognize _safe_ and _unsafe_ plans**, based on the planning criterion.

- About the baselines:
  - Obviously, the proposed approach is compared to the two methods it aims at **combining**.
  - About `MBRL`:
    - `1-` First, a **`forward dynamics model`** is learnt using given observed **expert data**.
      - It does **not imitate** the expert preferred actions, but **only models what is physically possible**.
    - `2-` The model then is used to **plan a reachability tree** through the free-space up to the waypoint while **avoiding obstacles**:
      - Playing with the `throttle` action, the search **expands each state node** and retains the `50` closest nodes to the **target waypoint**.
      - The planner finally opts for the **lowest-cost path** that **ends near the goal**.
    - > "The task of **evoking expert-like behavior** is offloaded to the **reward function**, which can be difficult and time-consuming to **craft properly**."
  - About `IL`: It used **Conditional** terms on **States**, leading to **`CILS`**.
    - `S` for `state`: Instead of emitting **low-level control** commands (`throttle`, `steering`), it outputs **set-points for some `PID`-controller**.
    - `C` for `conditional`: To **navigate at intersections**, waypoints are classified into one of several **directives**: {`Turn left`, `Turn right`, `Follow Lane`, `Go Straight`}.
      - This is inspired by ["End-to-end driving via conditional imitation learning"](https://arxiv.org/abs/1710.02410v2) - (Codevilla et al. 2018) - detailed below.

</details>

---

**`"Conditional Vehicle Trajectories Prediction in CARLA Urban Environment"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1909.00792)]**
**[[🎞️](https://www.youtube.com/watch?v=J_2HtNMV6qA)]**
**[** :car: `Valeo` **]**

- **[** _`conditional IL`, [`CARLA`](http://carla.org/), `distributional shift problem`_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![`End-to-`**`Mid`** approach: `3` inputs with **different levels of abstraction** are used to predict the future positions on a fixed `2s`-horizon of the ego vehicle and the neighbours. The ego trajectory is be **implemented by an external** `PID` controller - Therefore, **not** `end-to-`**`end`**. [Source](https://arxiv.org/abs/1909.00792).](../media/2019_buhet_1.PNG "`End-to-`**`Mid`** approach: `3` inputs with **different levels of abstraction** are used to predict the future positions on a fixed `2s`-horizon of the ego vehicle and the neighbours. The ego trajectory is be **implemented by an external** `PID` controller - Therefore, **not** `end-to-`**`end`**. [Source](https://arxiv.org/abs/1909.00792).")  |
|:--:|
| *`End-to-`**`Mid`** approach: `3` inputs with **different levels of abstraction** are used to predict the future positions on a fixed `2s`-horizon of the ego vehicle and the neighbours. The ego trajectory is be **implemented by an external** `PID` controller - Therefore, **not** `end-to-`**`end`**. [Source](https://arxiv.org/abs/1909.00792).* |

| ![The past **3D-bounding boxes** of the road users in the current reference are **projected back in the current camera space**. The **past positions** of ego and other vehicles are projected into some grid-map called **`proximity map`**. The image and the **proximity map** are concatenated to form context feature vector `C`. This **context encoding** is concatenated with the **ego encoding**, then fed into **branches** corresponding to the different high-level goals - `conditional navigation goal`. [Source](https://arxiv.org/abs/1909.00792).](../media/2019_buhet_2.PNG "The past **3D-bounding boxes** of the road users in the current reference are **projected back in the current camera space**. The **past positions** of ego and other vehicles are projected into some grid-map called **`proximity map`**. The image and the **proximity map** are concatenated to form context feature vector `C`. This **context encoding** is concatenated with the **ego encoding**, then fed into **branches** corresponding to the different high-level goals - `conditional navigation goal`. [Source](https://arxiv.org/abs/1909.00792).")  |
|:--:|
| *The past **3D-bounding boxes** of the road users in the current reference are **projected back in the current camera space**. The **past positions** of ego and other vehicles are projected into some grid-map called **`proximity map`**. The image and the **proximity map** are concatenated to form context feature vector `C`. This **context encoding** is concatenated with the **ego encoding**, then fed into **branches** corresponding to the different high-level goals - `conditional navigation goal`. [Source](https://arxiv.org/abs/1909.00792).* |

| ![Illustration of the **distribution shift** in **imitation learning**. [Source](https://arxiv.org/abs/1909.00792).](../media/2019_buhet_3.PNG "Illustration of the **distribution shift** in **imitation learning**. [Source](https://arxiv.org/abs/1909.00792).")  |
|:--:|
| *Illustration of the **distribution shift** in **imitation learning**. [Source](https://arxiv.org/abs/1909.00792).* |

| ![[`VisualBackProp`](https://arxiv.org/abs/1611.05418) highlights the **image pixels which contributed the most** to the final results - **Traffic lights** and their colours are important, together with highlights lane markings and curbs when there is a significant lateral deviation. [Source](https://arxiv.org/abs/1909.00792).](../media/2019_buhet_4.PNG "[`VisualBackProp`](https://arxiv.org/abs/1611.05418) highlights the **image pixels which contributed the most** to the final results - **Traffic lights** and their colours are important, together with highlights lane markings and curbs when there is a significant lateral deviation. [Source](https://arxiv.org/abs/1909.00792).")  |
|:--:|
| *[`VisualBackProp`](https://arxiv.org/abs/1611.05418) highlights the **image pixels which contributed the most** to the final results - **Traffic lights** and their colours are important, together with highlights lane markings and curbs when there is a significant lateral deviation. [Source](https://arxiv.org/abs/1909.00792).* |

Authors: Buhet, T., Wirbel, E., & Perrotton, X.

- Previous works:
  - ["Imitation Learning for End to End Vehicle Longitudinal Control with Forward Camera"](https://arxiv.org/abs/1812.05841) - (George, Buhet, Wirbel, Le-Gall, & Perrotton, 2018).
  - ["End to End Vehicle Lateral Control Using a Single Fisheye Camera"](https://arxiv.org/abs/1808.06940) (Toromanoff, M., Wirbel, E., Wilhelm, F., Vejarano, C., Perrotton, X., & Moutarde, F. 2018).
- One term: **_"End-To-Middle"_**.
  - It is opposed to **_"End-To-End"_**, i.e. it **does not output "end" control signals** such as `throttle` or `steering` but rather some **desired trajectory**, i.e. a mid-level representation.
    - Each trajectory is described by **two polynomial functions** (one for `x`, the other for `y`), therefore the network has to **predict a vector** (`x0`, ..., `x4`, `y0`, ..., `y4`) for each vehicle.
    - The desired ego-trajectory is then implemented by an **external controller** (`PID`). Therefore, **not `end-to-end`**.
  - Advantages of `end-to-mid`: **interpretability** for the control part + less to be learnt by the net.
  - This approach is also an instance of **"Direct perception"**:
    - > "Instead of commands, the network predicts hand-picked parameters relevant to the driving (distance to the lines, to other vehicles), which are then fed to an independent controller".
  - Small digression: if the raw perception measurements were first processed to form a **mid-level** input representation, the approach would be said `mid-to-mid`. An example is [ChauffeurNet](https://arxiv.org/abs/1812.03079), detailed on this page as well.
- About Ground truth:
  - The expert demonstrations do not come from human recordings but rather from **`CARLA` autopilot**.
  - `15` hours of driving in `Town01` were collected.
  - As for human demonstrations, **no annotation is needed**.
- One term: **_"Conditional navigation goal"_**.
  - Together with the RGB images and the **past positions**, the network takes as input a **navigation command** to describe the **desired behaviour** of the ego vehicle at intersections.
  - Hence, the future trajectory of the ego vehicle is **conditioned** by a **navigation command**.
    - If the ego-car is approaching an intersection, the **goal** can be `left`, `right` or `cross`, else the goal is to `keep lane`.
    - That means `lane-change` is not an option.
  - > "The last layers of the network are **split into branches** which are **masked with the current navigation command**, thus allowing the network to learn specific behaviours for each goal".
- Three ingredients to improve vanilla end-to-end imitation learning (`IL`):
  - `1`- **Mix of `high` and `low`-level input** (i.e. _hybrid_ input):
    - Both **raw signal** (_images_) and **partial environment abstraction** (_navigation commands_) are used.
  - `2`- **Auxiliary tasks**:
    - One head of the network predicts the future trajectories of the **surrounding vehicles**.
      - **It differs from the primary task** which should decide the `2s`-ahead trajectory for the ego car.
      - Nevertheless, this **secondary task helps**: _"Adding the neighbours prediction makes the ego prediction more compliant to traffic rules."_
    - This refers to the concept of **_"Privileged learning"_**:
      - > "The network is **partly trained with an auxiliary task** on a ground truth which is **useful to driving**, and on the rest is only trained for IL".
  - `3`- **Label augmentation**:
    - The main challenge of `IL` is the difference between **train** and **online test** distributions. This is due to the difference between
      - **`Open-loop`** control: decisions are not implemented.
      - **`Close-loop`** control: decisions are implemented, and the vehicle can end in a state absent from the _train distribution_, potentially causing _"error accumulation"_.
    - **Data augmentation** is used to reduce the gap between _train_ and _test_ distributions.
      - Classical randomization is combined with **`label augmentation`**: data similar to **failure cases** is generated a posteriori.
    - Three findings:
    - > "There is a **significant gap** in performance when introducing the **augmentation**."
    - > "The effect is much more noticeable on **complex navigation tasks**." _(Errors accumulate quicker)_.
    - > "**Online test** is the real significant indicator for `IL` when it is used for active control." _(The common offline evaluation metrics may not be correlated to the online performance)_.
- Baselines:
  - **Conditional Imitation** learning (`CIL`): ["End-to-end driving via conditional imitation learning"](http://arxiv.org/abs/1710.02410) [video](https://www.youtube.com/watch?v=cFtnflNe5fM) - (Codevilla, F., Müller, M., López, A., Koltun, V., & Dosovitskiy, A. 2017).
    - `CIL` produces **instantaneous commands**.
  - **Conditional Affordance** Learning (`CAL`): ["Conditional affordance learning for driving in urban environments"](https://arxiv.org/abs/1806.06498) [video](https://www.youtube.com/watch?v=UtUbpigMgr0) - (Sauer, A., Savinov, N. & Geiger, A. 2018).
    - `CAL` produces **_"affordances"_** which are then given to a controller.
- One word about the choice of the simulator.
  - A possible alternative to [CARLA](http://carla.org/) could be [DeepDrive](https://deepdrive.io/) or the [**`LGSVL`** simulator](https://www.lgsvlsimulator.com) developed by the Advanced Platform Lab at the **LG Electronics** America R&D Centre. _This looks promising_.

</details>

---

**`"Uncertainty Quantification with Statistical Guarantees in End-to-End Autonomous Driving Control"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1909.09884)]**
**[** :mortar_board: `Oxford University` **]**

- **[** _`uncertainty-aware decision`, `Bayesian inference`, [`CARLA`](http://carla.org/)_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![The __trust__ or __uncertainty__ in one decision can be measured based on the `probability mass function` around its mode. [Source](https://arxiv.org/abs/1904.08980).](../media/2019_michelmore_1.PNG "The __trust__ or __uncertainty__ in one decision can be measured based on the `probability mass function` around its mode. [Source](https://arxiv.org/abs/1909.09884).")  |
|:--:|
| *The __trust__ or __uncertainty__ in one decision can be measured based on the `probability mass function` around its mode. [Source](https://arxiv.org/abs/1904.08980).* |

| ![The measures of uncertainty based on __mutual information__ can be used to issue warnings to the driver and perform safety / emergency manoeuvres. [Source](https://arxiv.org/abs/1904.08980).](../media/2019_michelmore_2.PNG "The measures of uncertainty based on __mutual information__ can be used to issue warnings to the driver and perform safety / emergency manoeuvres. [Source](https://arxiv.org/abs/1909.09884).")  |
|:--:|
| *The measures of uncertainty based on __mutual information__ can be used to issue warnings to the driver and perform safety / emergency manoeuvres. [Source](https://arxiv.org/abs/1904.08980).* |

| ![As noted by the authors: while the `variance` can be useful in __collision avoidance__, the wide variance of `HMC` causes a larger proportion of trajectories to fall __outside of the safety boundary__ when a _new weather_ is applied. [Source](https://arxiv.org/abs/1904.08980).](../media/2019_michelmore_3.PNG "As noted by the authors: while the `variance` can be useful in __collision avoidance__, the wide variance of `HMC` causes a larger proportion of trajectories to fall __outside of the safety boundary__ when a _new weather_ is applied. [Source](https://arxiv.org/abs/1909.09884).")  |
|:--:|
| *As noted by the authors: while the `variance` can be useful in __collision avoidance__, the wide variance of `HMC` causes a larger proportion of trajectories to fall __outside of the safety boundary__ when a _new weather_ is applied. [Source](https://arxiv.org/abs/1904.08980).* |

Authors: Michelmore, R., Wicker, M., Laurenti, L., Cardelli, L., Gal, Y., & Kwiatkowska, M

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
    - I did not know about **"mutual information"** and liked the explanation of [Wikipedia](https://en.wikipedia.org/wiki/Mutual_information) about the link of `MI` to `entropy` and `KL-div`.
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

</details>

---

**`"Exploring the Limitations of Behavior Cloning for Autonomous Driving"`**

- **[** `2019` **]**
**[[:memo:](https://arxiv.org/abs/1904.08980)]**
**[[🎞️](https://www.youtube.com/watch?v=sXIuU_wECQc)]**
**[[:octocat:](https://github.com/felipecode/coiltraine)]**
**[** :mortar_board: `CVC, UAB, Barcelona` **]**
**[** :car: `Toyota` **]**

- **[** _`distributional shift problem`, `off-policy data collection`, [`CARLA`](http://carla.org/), `conditional imitation learning`, `residual architecture`, `reproducibility issue`, `variance caused by initialization and sampling`_ **]**

<details>
  <summary>Click to expand</summary>

One figure:

| ![Conditional Imitation Learning is extended with a ResNet architecture and Speed prediction (`CILRS`). [Source](https://arxiv.org/abs/1904.08980).](../media/2019_codevilla.PNG "Conditional Imitation Learning is extended with a ResNet architecture and Speed prediction (`CILRS`). [Source](https://arxiv.org/abs/1904.08980).")  |
|:--:|
| *Conditional Imitation Learning is extended with a ResNet architecture and Speed prediction (`CILRS`). [Source](https://arxiv.org/abs/1904.08980).* |

Authors: Codevilla, F., Santana, E., Antonio, M. L., & Gaidon, A.

- One term: **“CILRS”** = **Conditional Imitation Learning** extended with a **ResNet** architecture and **Speed prediction**.
- One Q&A: _How to include in E2E learning information about the destination, i.e. to disambiguate imitation around multiple types of intersections?_
  - Add a high-level `navigational command` (e.g. _take the next right_, _left_, or _stay in lane_) to the tuple <`observation`, `expert action`> when building the dataset.
- One idea: learn to predict the ego speed ([`mediated perception`](http://deepdriving.cs.princeton.edu/paper.pdf)) to address the _inertia problem_ stemming from [**causal confusion**](https://arxiv.org/pdf/1905.11979.pdf) (**biased correlation** between _low speed_ and _no acceleration_ - when the ego vehicle is stopped, e.g. at a red traffic light, the probability it stays static is indeed overwhelming in the training data).
  - A good [video](https://www.youtube.com/watch?v=_dh2-2b1jmU) about [Causal Confusion in Imitation Learning](https://arxiv.org/abs/1905.11979) showing that **"access to more information leads to worse generalisation under distribution shift"**.
- Another idea: The off-policy (expert) driving demonstration is not produced by a human, but rather generated from an **omniscient "AI" agent**.
- One quote:

> "The more common the vehicle model and color, the better the trained agent reacts to it. This raises ethical challenges in automated driving".

</details>

---

**`"Conditional Affordance Learning for Driving in Urban Environments"`**

- **[** `2018` **]**
**[[:memo:](http://www.cvlibs.net/publications/Sauer2018CORL.pdf)]**
**[[🎞️](https://www.youtube.com/watch?v=UtUbpigMgr0)]**
**[[🎞️](https://www.youtube.com/watch?v=SceH3Al9w_M)]**
**[[:octocat:](https://github.com/xl-sr/CAL)]**
**[** :mortar_board: `CVC, UAB, Barcelona` **]**
**[** :car: `Toyota` **]**

- **[** _[`CARLA`](http://carla.org/), `end-to-mid`, `direct perception`_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![Examples of **affordances**, i.e. **attributes of the environment** which limit the space of **allowed actions**. `A1`, `A2` and `A3` are predefined **observation areas**. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).](../media/2018_sauer_4.PNG "Examples of **affordances**, i.e. **attributes of the environment** which limit the space of **allowed actions**. `A1`, `A2` and `A3` are predefined **observation areas**. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).")  |
|:--:|
| *Examples of **affordances**, i.e. **attributes of the environment** which limit the space of **allowed actions**. `A1`, `A2` and `A3` are predefined **observation areas**. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).* |

| ![The presented __direct perception__ `DP` method predicts a __low-dimensional intermediate__ representation of the environment - __affordance__ - which is then used in a conventional control algorithm. The _affordance_ is conditioned for goal-directed navigation, i.e. before each intersection, it receives an instruction such as `go straight`, `turn left` or `turn right`. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).](../media/2018_sauer_1.PNG "The presented __direct perception__ `DP` method predicts a __low-dimensional intermediate__ representation of the environment - __affordance__ - which is then used in a conventional control algorithm. The _affordance_ is conditioned for goal-directed navigation, i.e. before each intersection, it receives an instruction such as `go straight`, `turn left` or `turn right`. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).")  |
|:--:|
| *The presented `direct perception` method predicts a __low-dimensional intermediate__ representation of the environment - __affordance__ - which is then used in a conventional control algorithm. The _affordance_ is __conditioned__ for goal-directed navigation, i.e. before each intersection, it receives an instruction such as `go straight`, `turn left` or `turn right`. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).* |

| ![The **feature maps** produced by a `CNN` **feature extractor** are stored in a **memory** and consumed by task-specific layers (one _affordance_ has one _task block_). Every task block has its **own specific temporal receptive field** - decides how much of the memory it needs. This figure also illustrates how the _navigation command_ is used as **switch between trained submodules**. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).](../media/2018_sauer_2.PNG "The **feature maps** produced by a `CNN` **feature extractor** are stored in a **memory** and consumed by task-specific layers (one _affordance_ has one _task block_). Every task block has its **own specific temporal receptive field** - decides how much of the memory it needs. This figure also illustrates how the _navigation command_ is used as **switch between trained submodules**. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).")  |
|:--:|
| *The **feature maps** produced by a `CNN` **feature extractor** are stored in a **memory** and consumed by task-specific layers (one _affordance_ has one _task block_). Every task block has its **own specific temporal receptive field** - it decides how much of the memory it needs. This figure also illustrates how the _navigation command_ is used as **switch between trained submodules**. [Source](http://www.cvlibs.net/publications/Sauer2018CORL.pdf).* |

Authors: Sauer, A., Savinov, N., & Geiger, A.

- One term: **_"Direct perception"_** (`DP`):
  - The goal of `DP` methods is to predict a **low-dimensional intermediate representation** of the environment which is then used in a conventional **control algorithm** to manoeuvre the vehicle.
  - With this regard, `DP` could also be said `end-to-`**`mid`**. The mapping to learn is less complex than `end-to-`**`end`** (from **raw input** to **controls**).
  - `DP` is meant to combine the advantages of two other commonly-used approaches: **modular pipelines** `MP` and `end-to-end` methods such as **imitation learning** `IL` or **model-free `RL`**.
  - **Ground truth affordances** are collected using `CARLA`. Several augmentations are performed.
- Related work on _affordance learning_ and _direct perception_.
  - [**`Deepdriving`**: Learning affordance for direct perception in autonomous driving](https://arxiv.org/abs/1505.00256) by (Chen, Seff, Kornhauser, & Xiao, 2015).
  - `Deepdriving` works on _highway_.
  - Here, the idea is extended to **_urban scenarios_** (with _traffic signs_, _traffic lights_, _junctions_) considering a **sequence of images** (not just one camera frame) for **temporal information**.
- One term: **_"Conditional Affordance Learning"_** (`CAL`):
  - **_"Conditional"_**: The actions of the agent are **conditioned** on a **high-level command** given by the navigation system (the planner) prior to intersections. It describes the **manoeuvre** to be performed, e.g., `go straight`, `turn left`, `turn right`.
  - **_"Affordance"_**: **Affordances** are one example of `DP` **representation**. They are **attributes of the environment** which limit the space of **allowed actions**. Only `6` affordances are used for `CARLA` urban driving:
    - `Distance to vehicle` (continuous).
    - `Relative angle` (continuous and conditional).
    - `Distance to centre-line` (continuous and conditional).
    - `Speed Sign` (discrete).
    - `Red Traffic Light` (discrete - binary).
    - `Hazard` (discrete - binary).
      - The `Class Weighted Cross Entropy` is the **loss** used for _discrete affordances_ to put **more weights on rare but important occurrences** (`hazard` occurs rarely compared to `traffic light red`).
  - **_"Learning"_**: A single **neural network** trained with multi-task learning (`MTL`) **predicts all affordances** in a single forward pass (`~50ms`). It only takes a **single front-facing camera view** as input.
- About the controllers: The **_path-velocity decomposition_** is applied. Hence two controllers are used in parallel:
  - 1- `throttle` and `brake`
    - Based on the predicted **affordances**, a state is _"rule-based"_ assigned among: `cruising`, `following`, `over limit`, `red light`, and `hazard stop` (all are mutually exclusive).
    - Based on this state, the **longitudinal control** signals are derived, using `PID` or _threshold-predefined_ values.
    - It can handle _traffic lights_, _speed signs_ and _smooth car-following_.
    - Note: The _Supplementary Material_ provides details insights on controller tuning (especially `PID`) for `CARLA`.
  - 2- `steering` is controlled by a Stanley Controller, based on two conditional affordances: `distance to centreline` and `relative angle`.
- One idea: I am often wondering what **timeout** I should set when **testing a scenario** with `CARLA`. The author computes this time based on the **length of the pre-defined path** (which is actually easily **accessible**):
  - > "The time limit equals the time needed to reach the goal when driving along the **optimal path** at `10 km/h`"
- Another idea: **Attention Analysis**.
  - For better **understanding** on how affordances are constructed, the **attention** of the `CNN` using _gradient-weighted class activation maps_ ([`Grad-CAMs`](https://arxiv.org/abs/1610.02391)).
  - This _"visual explanation"_ reminds me another technique used in `end-to-end` approaches, [`VisualBackProp`](https://arxiv.org/abs/1611.05418), that highlights the **image pixels which contributed the most** to the final results.
- Baselines and results:
  - Compared to `CARLA`-based [_Modular Pipeline_](https://arxiv.org/abs/1711.03938) (`MP`), [_Conditional Imitation Learning_](https://arxiv.org/abs/1710.02410) (`CIL`) and [_Reinforcement Learning_](https://arxiv.org/abs/1711.03938) (`RL`), `CAL` particularly excels in **generalizing to the new town**.
- _Where to provide the high-level navigation conditions?_
  - The authors find that "**conditioning in the network** has several advantages over **conditioning in the controller**".
  - In addition, in the net, it is preferable to **use the navigation command as switch** between submodules rather than an input:
    - > "We observed that **training specialized submodules** for each directional command leads to better performance compared to using the directional command as an **additional input to the task networks**".

</details>

---

**`"Variational Autoencoder for End-to-End Control of Autonomous Driving with Novelty Detection and Training De-biasing"`**

- **[** `2018` **]**
**[[:memo:](https://dspace.mit.edu/handle/1721.1/118139)]**
**[[🎞️](https://www.youtube.com/watch?v=ZwSdzcV-jr4)]**
**[[🎞️](https://www.youtube.com/watch?v=aXI4a_Nvcew)]**
**[** :mortar_board: `MIT` **]**
**[** :car: `Toyota` **]**

- **[** _`VAE`, `uncertainty estimation`, `sampling efficiency`, `augmentation`_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![One particular latent variable `^y` is **explicitly supervised** to **predict steering control**. Anther interesting idea: augmentation is based on domain knowledge - if a method __used to the middle-view__ is given some __left-view__ image, it should predict some __correction to the right__. [Source](https://dspace.mit.edu/handle/1721.1/118139).](../media/2018_amini_1.PNG "One particular latent variable `^y` is **explicitly supervised** to **predict steering control**. Another interesting idea: augmentation is based on domain knowledge - if a method __used to the middle-view__ is given some __left-view__ image, it should predict some __correction to the right__. [Source](https://dspace.mit.edu/handle/1721.1/118139).")  |
|:--:|
| *One particular latent variable `^y` is **explicitly supervised** to **predict steering control**. Another interesting idea: __augmentation__ is based on __domain knowledge__ - if a method __used to the middle-view__ is given some __left-view__ image, it should predict some __correction to the right__. [Source](https://dspace.mit.edu/handle/1721.1/118139).* |

| ![For each new image, empirical uncertainty estimates are computed by sampling from the variables of the latent space. These estimates lead to the `D` statistic that indicates __whether an observed image is well captured by our trained model__, i.e. `novelty detection`. [Source](https://dspace.mit.edu/handle/1721.1/118139).](../media/2018_amini_2.PNG "For each new image, empirical uncertainty estimates are computed by sampling from the variables of the latent space. These estimates lead to the `D` statistic that indicates __whether an observed image is well captured by our trained model__, i.e. `novelty detection`. [Source](https://dspace.mit.edu/handle/1721.1/118139).")  |
|:--:|
| *For each new image, __empirical uncertainty estimates__ are computed by sampling from the variables of the __latent space__. These estimates lead to the __`D`__ statistic that indicates __whether an observed image is well captured by our trained model__, i.e. __`novelty detection`__. [Source](https://dspace.mit.edu/handle/1721.1/118139).* |

| ![In a subsequent work, the `VAE` is __conditioned onto the road topology__. It serves multiple purposes such as localization and __`end-to-end` navigation__. The _routed_ or _unrouted map_ given as additional input goes toward the __`mid-to-end`__ approach where processing is performed and/or __external knowledge__ is embedded. [Source](https://arxiv.org/abs/1811.10119).](../media/2019_amini_1.PNG "In a subsequent work, the `VAE` is __conditioned onto the road topology__. It serves multiple purposes such as localization and __`end-to-end` navigation__. The _routed_ or _unrouted map_ given as additional input goes toward the __`mid-to-end`__ approach where processing is performed and/or __external knowledge__ is embedded. [Source](https://arxiv.org/abs/1811.10119).")  |
|:--:|
| *In a subsequent work, the `VAE` is __conditioned__ onto the __road topology__. It serves multiple purposes such as localization and __`end-to-end` navigation__. The _routed_ or _unrouted map_ given as additional input goes toward the __`mid-to-end`__ approach where processing is performed and/or __external knowledge__ is embedded. [Source](https://arxiv.org/abs/1811.10119). See this [video](https://www.youtube.com/watch?v=aXI4a_Nvcew) temporal for evolution of the predictions.* |

Authors: Amini, A., Schwarting, W., Rosman, G., Araki, B., Karaman, S., & Rus, D.

- One issue raised about _vanilla_ `E2E`:
  - The lack a **measure** of associated **confidence** in the prediction.
  - The lack of **interpretation** of the learned features.
  - Having said that, the authors present an approach to both **understand** and **estimate** the confidence of the output.
  - The idea is to use a **Variational Autoencoder** (`VAE`), taking benefit of its **intermediate latent representation** which is learnt in an **unsupervised** way and provides **uncertainty estimates** for every variable in the latent space via their parameters.
- One idea for the `VAE`: one particular latent variable is **explicitly supervised** to **predict steering control**.
  - The loss function of the `VAE` has therefore `3` parts:
    - A **`reconstruction`** loss: `L1`-norm between the input image and the output image.
    - A **`latent`** loss: `KL`-divergence between the _latent variables_ and a _unit Gaussian_, providing **regularization for the latent space**.
    - A **`supervised latent`** loss: `MSE` between the _predicted_ and _actual_ **curvature** of the vehicle’s path.
- One contribution: "**Detection of novel events**" (which have not been sufficiently trained for).
  - To check if an observed image is well captured by the trained model, the idea is to propagate the **`VAE`’s latent uncertainty** through the **decoder** and compare the result with the original input. This is done by **sampling** (empirical uncertainty estimates).
  - The resulting **pixel-wise expectation** and **variance** are used to compute a sort of **_loss_ metric** `D`(`x`, `ˆx`) whose **distribution** for the training-set is known (approximated with a _histogram_).
  - The image `x` is classified as **`novel`** if this **statistic is outside of the `95th` percentile** of the _training_ distribution and the prediction can finally be _"untrusted to produce reliable outputs"_.
  - > "Our work presents an indicator to __detect novel images__ that were not contained in the training distribution by __weighting the reconstructed image__ by the __latent uncertainty__ propagated through the network. High loss indicates that the model has __not been trained on that type of image__ and thus reflects __lower confidence__ in the network’s __ability to generalize__ to that scenario."
- A second contribution: **"Automated debiasing against learned biases"**.
  - As for the novelty detection, it takes advantage of the **latent space distribution** and the possibility of **sampling from the most representative regions** of this space.
  - Briefly said, the idea it to **increase the proportion of rarer datapoints** by **dropping over-represented regions** of the latent space to accelerate the training (**_sampling efficiency_**).
  - This debiasing is **not manually specified** beforehand but based on learned latent variables.
- One reason to use **single frame** prediction (as opposed to `RNN`):
  - > ""Note that only a **single image** is used as input at every time instant. This follows from original observations where models that were trained end-to-end with a **temporal information** (`CNN`+`LSTM`) are **unable to decouple the underlying spatial information from the temporal control aspect**. While these models perform well on **test** datasets, they face **control feedback issues** when placed on a physical vehicle and consistently drift off the road.""
- One idea about **augmentation** (also met in the _Behavioral Cloning Project_ of the [Udacity Self-Driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)):
  - > "To **inject domain knowledge** into our network we augmented the dataset with images collected from cameras placed approximately `2` feet to the **left and right** of the main centre camera. We correspondingly **changed the supervised control** value to teach the model how to **recover from off-centre positions**."
- One note about the output:
  - > "We refer to **steering command** interchangeably as the **road curvature**: the actual _steering angle_ requires reasoning about _road slip_ and _control plant parameters_ that **change between vehicles**."
- Previous and further works:
  - ["Spatial Uncertainty Sampling for End-to-End control"](https://arxiv.org/pdf/1805.04829.pdf) - (Amini, Soleimany, Karaman, & Rus, 2018)
  - ["Variational End-to-End Navigation and Localization"](https://arxiv.org/pdf/1811.10119.pdf) - (Amini, Rosman, Karaman, & Rus, 2019)
    - One idea: incorporate some **coarse-grained roadmaps** with raw perceptual data.
      - Either **unrouted** (just containing the drivable roads). `Output` = continuous **probability distribution** over steering control.
      - Or **routed** (target road highlighted). `Output` = **deterministic** steering control to navigate.
    - _How to evaluate the continuous probability distribution over steering control given the human "scalar" demonstration?_
      - > "For a range of __`z`-scores__ over the steering control distribution we __compute the number of samples__ within the test set where the __true (human) control output__ was within the predicted range."
    - About the training dataset: **`25 km`** of urban driving data.

</details>

---

**`"ChauffeurNet: Learning to Drive by Imitating the Best and Synthesizing the Worst"`**

- **[** `2018` **]**
**[[:memo:](https://arxiv.org/abs/1812.03079)]**
**[[🎞️](https://sites.google.com/view/waymo-learn-to-drive)]**
**[[🎞️](https://www.youtube.com/watch?v=mxqdVO462HU)]**
**[** :car: `Waymo` **]**

- **[** _`imitation learning`, `distributional shift problem`_ **]**

<details>
  <summary>Click to expand</summary>

Two figures:

| ![Different layers composing the `mid-level representation`. [Source](https://arxiv.org/abs/1812.03079).](../media/2018_bansal_1.PNG "Different layers composing the `mid-level representation`. [Source](https://arxiv.org/abs/1812.03079).")  |
|:--:|
| *Different layers composing the `mid-level representation`. [Source](https://arxiv.org/abs/1812.03079).* |

| ![Training architecture around `ChauffeurNet` with the different loss terms, that can be grouped into `environment` and `imitation` losses. [Source](https://arxiv.org/abs/1812.03079).](../media/2018_bansal_2.PNG "Training architecture around `ChauffeurNet` with the different loss terms, that can be grouped into `environment` and `imitation` losses. [Source](https://arxiv.org/abs/1812.03079).")  |
|:--:|
| *Training architecture around `ChauffeurNet` with the different loss terms, that can be grouped into `environment` and `imitation` losses. [Source](https://arxiv.org/abs/1812.03079).* |

Authors: Bansal, M., Krizhevsky, A., & Ogale, A.

- One term: **_"mid-level representation"_**
  - The decision-making task (between `perception` and `control`) is packed into one single "learnable" module.
    - Input: the representation divided into **several image-like layers**:
      - `Map features` such as _lanes_, _stop signs_, _cross-walks_...; `Traffic lights`; `Speed Limit`; `Intended route`; `Current agent box`; `Dynamic objects`; `Past agent poses`.
      - Such a representation is **generic**, i.e. independent of the **number of dynamic objects** and independent of the **road geometry/topology**.
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
      - **_Past dropout_**: to prevent **using the history to cheat** by **just extrapolating** from the past rather than finding the **underlying causes** of the behaviour.
      - Hence the concept of tweaking the training data in order to **_“simulate the bad rather than just imitate the good”._**
    - Going **beyond the vanilla imitation loss**.
      - Extend imitation losses.
      - Add **_environment losses_** to discourage undesirable behaviour, e.g. measuring the overlap of predicted agent positions with the _non-road_ regions.
      - Use **_imitation dropout_**, i.e. sometimes favour the _environment loss_ over the _imitation loss_.

</details>

---

**`"Imitating Driver Behavior with Generative Adversarial Networks"`**

- **[** `2017` **]**
**[[:memo:](https://arxiv.org/abs/1701.06699)]**
**[[:octocat:](https://github.com/sisl/gail-driver)]**
**[** :mortar_board: `Stanford` **]**

- **[** _`adversarial learning`, `distributional shift problem`, `cascading errors`, `IDM`, [`NGSIM`](https://ops.fhwa.dot.gov/trafficanalysistools/ngsim.htm), [`rllab`](https://github.com/rll/rllab)_ **]**

<details>
  <summary>Click to expand</summary>

Some figures:

| ![The state consists in **`51` features** divided into `3` groups: The __core features__ include hand-picked features such as `Speed`, `Curvature` and `Lane Offset`. The __LIDAR-like beams__ capture the surrounding objects in a fixed-size representation **independent of the number of vehicles**. Finally, `3` **binary indicator features** identify when the ego vehicle **encounters undesirable states** - `collision`, `drives off road`, and `travels in reverse`. [Source](https://arxiv.org/abs/1701.06699).](../media/2017_kuefler_1.PNG "The state consists in **`51` features** divided into `3` groups: The __core features__ include hand-picked features such as `Speed`, `Curvature` and `Lane Offset`. The __LIDAR-like beams__ capture the surrounding objects in a fixed-size representation **independent of the number of vehicles**. Finally, `3` **binary indicator features** identify when the ego vehicle **encounters undesirable states** - `collision`, `drives off road`, and `travels in reverse`. [Source](https://arxiv.org/abs/1812.03079).")  |
|:--:|
| *The `state` consists in **`51` features** divided into `3` groups: The __core features__ include hand-picked features such as `Speed`, `Curvature` and `Lane Offset`. The __LIDAR-like beams__ capture the surrounding objects in a fixed-size representation **independent of the number of vehicles**. Finally, `3` **binary indicator features** identify when the ego vehicle **encounters undesirable states** - `collision`, `drives off road`, and `travels in reverse`. [Source](https://arxiv.org/abs/1701.06699).* |

| ![As for common **adversarial approaches**, the objective function in `GAIL` includes some **sigmoid cross entropy** terms. The objective is to **fit `ψ`** for the **discriminator**. But this objective function is **non-differentiable with respect to `θ`**. One solution is to **optimize `πθ` separately using `RL`**. But what for `reward function`? In order to drive `πθ` into regions of the state-action space similar to those explored by the **expert `πE`**, a **surrogate reward `˜r`** is generated from `D`_`ψ` based on samples and `TRPO` is used to perform a policy update of `πθ`. [Source](https://arxiv.org/abs/1701.06699).](../media/2017_kuefler_2.PNG "As for common **adversarial approaches**, the objective function in `GAIL` includes some **sigmoid cross entropy** terms. The objective is to **fit `ψ`** for the **discriminator**. But this objective function is **non-differentiable with respect to `θ`**. One solution is to **optimize `πθ` separately using `RL`**. But what for `reward function`? In order to drive `πθ` into regions of the state-action space similar to those explored by the **expert `πE`**, a **surrogate reward `˜r`** is generated from `D`_`ψ` based on samples and `TRPO` is used to perform a policy update of `πθ`. [Source](https://arxiv.org/abs/1812.03079).")  |
|:--:|
| *As for common **adversarial approaches**, the objective function in `GAIL` includes some **sigmoid cross entropy** terms. The objective is to **fit `ψ`** for the **discriminator**. But this objective function is **non-differentiable with respect to `θ`**. One solution is to **optimize `πθ` separately using `RL`**. But what for `reward function`? In order to drive `πθ` into regions of the state-action space similar to those explored by the **expert `πE`**, a **surrogate reward `˜r`** is generated from `D`_`ψ` based on samples and `TRPO` is used to perform a policy update of `πθ`. [Source](https://arxiv.org/abs/1701.06699).* |

Authors: Kuefler, A., Morton, J., Wheeler, T., & Kochenderfer, M.

- One term: the problem of **_"cascading errors"_** in **behavioural cloning** (**`BC`**).
  - `BC`, which treats `IL` as a **supervised learning** problem, tries to fit a model to a **fixed dataset** of expert state-action pairs. In other words, `BC` solves a **regression problem** in which the policy parameterization is obtained by **maximizing the likelihood** of the **actions taken in the training data**.
  - But inaccuracies can lead the **stochastic policy** to states that are **underrepresented in the training data** (e.g., _an ego-vehicle edging towards the side of the road_). And datasets rarely contain information about **how human drivers behave** in such situations.
  - The policy network is then **forced to generalize**, and this can lead to yet **poorer predictions**, and ultimately to **invalid or unseen situations** (e.g., _off-road driving_).
  - **_"Cascading Errors"_** refers to this problem where **small inaccuracies compound** during simulation and the agent **cannot recover** from them.
    - This issue is inherent to **sequential** decision making.
  - As found by the results:
    - > "The root-weighted square error results show that the feedforward `BC` model has the **best short-horizon performance**, but then begins to **accumulate error for longer time horizons**."
    - > "Only `GAIL` (and of course `IDM`+`MOBIL`) are able to **stay on the road for extended stretches**."
- One idea: **`RL`** provides **robustness** against **_"cascading errors"_**.
  - `RL` maximizes the _global_, **expected return on a trajectory**, rather than _local_ instructions for each observation. Hence more appropriate for _sequential_ decision making.
  - Also, the reward function `r`(`s_t`, `a_t`) is defined **for all state-action pairs**, allowing an agent to receive a learning signal **even from unusual states**. And these signals can establish preferences between **mildly undesirable** behaviour (e.g., _hard braking_) and **extremely undesirable behaviour** (e.g., _collisions_).
    - In contrast, `BC` **only receives a learning signal** for those states **represented in a labelled, finite dataset**.
    - Because **handcrafting an accurate `RL` reward function is often difficult**, `IRL` seems promising. In addition, the **imitation** (via the _recovered_ reward function) **extends to unseen states**: e.g. a vehicle that is _perturbed_ toward the lane boundaries _should know to return toward the lane centre_.
- Another idea: use **`GAIL`** instead of **`IRL`**:
  - > "**IRL** approaches are typically **computationally expensive** in their **recovery of an expert cost function**. Instead, recent work has attempted to **imitate expert behaviour** through **direct policy optimization**, without **first learning a cost function**."
  - **_"Generative Adversarial Imitation Learning"_** (**`GAIL`**) implements this idea:
    - > "Expert behaviour can be **imitated by training a policy** to produce actions that a **binary classifier mistakes** for those of an expert."
    - > "`GAIL` trains a policy to **perform expert-like behaviour** by rewarding it for **“deceiving” a classifier** trained to discriminate between **policy** and **expert state-action pairs**."
  - One contribution is to extend `GAIL` to the **optimization of recurrent neural** networks (`GRU` in this case).
- One concept: **_"Trust Region Policy Optimization"_**.
  - Policy-gradient `RL` optimization with **_"Trust Region"_** is used to optimize the agent's policy `πθ`, addressing the issue of **training instability** of vanilla policy-gradient methods.
    - > "**`TRPO`** updates policy parameters through a **constrained optimization** procedure that enforces that a **policy cannot change too much in a single update**, and hence limits the damage that can be caused by noisy gradient estimates."
  - _But what reward function to apply?_ Again, we do not want to do `IRL`.
  - Some **_"surrogate"_ reward function** is empirically derived from the discriminator. Although it may be quite different from the **true reward function** optimized by **expert**, it can be used to drive `πθ` into regions of the state-action space similar to those explored by `πE`.
- One finding: _Should previous actions be included in the state `s`?_
  - > "The **previous action** taken by the ego vehicle is not included in the **set of features** provided to the policies. We found that policies can develop an over-reliance on **previous actions** at the expense of relying on the other features contained in their input."
  - But on the other hand, the authors find:
  - > "The `GAIL` `GRU` policy takes similar actions to humans, but **oscillates between actions more than humans**. For instance, rather than outputting a turn-rate of zero on straight road stretches, it **will alternate** between outputting small **positive and negative turn-rates**".
  - > "An **engineered reward function** could also be used to **penalize the oscillations** in acceleration and turn-rate produced by the `GAIL` `GRU`".
- Some interesting interpretations about the **`IDM`** and **`MOBIL`** driver models (resp. _longitudinal_ and _lateral_ control).
  - These commonly-used **rule-based parametric models** serve here as **baselines**:
  - > "The __Intelligent Driver Model__ (`IDM`) extended this work by __capturing asymmetries between acceleration and deceleration__, __preferred free road__ and bumper-to-bumper headways, and __realistic braking behaviour__."
  - > "`MOBIL` __maintains a utility function__ and __'politeness parameter'__ to capture intelligent driver behaviour in both _acceleration_ and _turning_."

</details>
