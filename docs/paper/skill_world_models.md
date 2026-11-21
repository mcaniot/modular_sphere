# Small expert world models per skill for collective step climbing of linked spheres

*Working paper, draft of 2026-11-21. Code: branch `feat/swarm-step-climb` of this repository. Every number below comes from a run listed in Section 8; results marked
**preliminary** use too few episodes for a conclusion and are being extended.*

## Abstract

We study how a swarm of rolling spheres that can link to one another and push internally can learn
to climb a step that no sphere can climb alone (step height 0.15 m, 1.5 sphere radii). A scripted
expert solves the task (80/80 seeds for 2 to 6 spheres) by a sequential strategy: one sphere serves
as a base, the others climb over it one by one, and the last one pulls the base up. Following the
project rule "every scripted approach is then learned with a world model", we first learn the whole
task from expert data: a graph world model (interaction network) and a DAgger policy reach 47/50
successes on held-out seeds. Improvement in the learned model fails in five variants (0/50 to 0/20
successes) through model exploitation and critic overestimation; only PPO with a KL penalty to the
imitation policy keeps the success rate (47/50, McNemar p = 1) and makes episodes slightly faster
(42 of 43 untied paired episodes faster, sign test p ≈ 1e-11), which mostly reproduces a sped-up
demonstration. Model-free multi-agent PPO in MuJoCo, either fine-tuned from DAgger or trained from
scratch with a step-height curriculum, does not discover a parallel strategy: the curriculum stalls
at 0.06 m, the highest step a lone sphere can climb. We then decompose the task into three skills
(roll, climb, pull), each with its own small world model trained only on its two-sphere data. These
skill models are 1.6 to 18 times more accurate at 30 steps than the whole-task model on the same
skill data. Planning with these models through model-predictive path integral control (MPPI)
solves roll (10/10) but at best 6/10 on climb and 9/10 on pull (**preliminary**), even with a
cloned proposal and a learned steps-to-go cost. A small world-action model per skill, after
GR00T 2, that jointly predicts the next 10 skill frames and a 10-step action chunk from a 4-frame
history solves climb on 50/50 paired starts against 27/50 for the cloned policy (McNemar
p = 2.4e-7) and matches it on pull and roll. Ablations show that this gain comes from capacity, not
from the world-action ingredients: without history, chunking and frame prediction the same
transformer still solves 50/50 (three training seeds), and a cloned MLP widened from 128 to 512
units solves 49–50/50, whereas the 128-unit MLP stays at 24–26/50. Group-relative self-imitation in MuJoCo shortens pull
by 9 % but collapses climb when rollouts are executed with temporal ensembling, whose lag the
model learns back. A scheduler that composes the skills in parallel lanes over sphere subsets,
choosing roles and lanes by the makespan predicted with the steps-to-go effect models, solves the
four-sphere task on 50/50 test seeds in 150 steps against 248 for the expert (all 50 seeds faster,
sign test p = 1.8e-15) when the skills are executed by their scripted controllers. With learned
executors it is as fast but fails more (cloned MLP 30/50, WAM 38/50): the learned climb fails on
about a quarter of the hand-offs from a roll, a state its training starts never covered.

## 1. Introduction

Modular and swarm robots can solve together what each one cannot, but the coordination is hard to
learn: the reward is sparse and delayed, the useful actions (link, lift) only matter in rare
configurations, and the joint action space grows with the number of agents. Self-assembling agents
that learn to link through reinforcement learning were shown by Pathak et al. [12], in simulation
and with a dense locomotion reward. Here the question is narrower and practical: given a scripted
solution, can a learned model of the world **choose the actions** (instead of only imitating
them) and find faster, more parallel strategies?

Contributions of this draft:

1. A deterministic MuJoCo benchmark of linked spheres climbing a step, with a PettingZoo interface
   and a scripted expert (Section 3).
2. A full "expert → world model → DAgger → improvement in the model" pipeline with paired
   statistical evaluation, and a set of **negative results** on improvement in learned models and
   on model-free MAPPO for this task (Sections 4–6).
3. A skill decomposition with one small world model per skill, measured against the whole-task
   model, and MPPI planning in these models (Section 5.4, partly preliminary).
4. A per-skill world-action model with paired evaluation, and a negative result on self-imitation
   from temporally ensembled rollouts (Sections 5.5–5.6).

## 2. Related work

All references were checked against arXiv metadata on 2026-11-21.

- **Learned dynamics over objects and relations.** Interaction networks [1] model a system as a
  graph of objects and relations; our world model is a small interaction network over spheres with
  link flags as edge features.
- **Imitation with corrective data.** DAgger [2] reduces covariate shift by labelling the learner's
  own states with the expert; DART [3] injects noise into the executed expert actions while keeping
  the clean labels. We use both.
- **Policy improvement inside a learned model.** ME-TRPO [4] trains a policy on an ensemble of
  learned models to limit exploitation; we observed the same exploitation (Section 6). PPO [5] with
  GAE [6] and its multi-agent variant with a centralised critic, MAPPO [7], are our RL baselines.
- **Planning with learned models.** MPPI [8] samples action sequences and averages them with
  exponential weights on the predicted cost. TD-MPC [9] and TD-MPC2 [10] plan with MPPI in a learned
  latent model and add samples from a learned policy; MoDem [11] starts this loop from
  demonstrations. Our skill planner follows this design (Section 4.7).
- **Skills and their models.** DADS [13] learns skills together with skill-conditioned dynamics
  for planning; SkiMo [14] learns a skill-level dynamics model and plans over skills; Liang et al.
  [15] learn per-skill *effect models* (outcome and cost of executing a skill from a state) and
  compose skills by graph search; Silver et al. [16] learn skills with symbolic preconditions and
  effects for bilevel planning. Our plan (Section 7) combines per-skill dynamics models for low-level
  action choice with effect models for scheduling, over **variable subsets of agents** whose link
  topology changes; we did not find this combination in the literature, based on abstracts only (a
  full-text check is still to do).

## 3. Environment

`SphereSwarmStepEnv` (`src/modular_sphere/swarm.py`), MuJoCo [17] physics (4.0.1), PettingZoo [18]
parallel API.

| Item | Value |
|---|---|
| Spheres | N = 4 by default (2 to 6 tested), radius 0.10 m, mass 0.30 kg |
| Physics / control | timestep 2 ms, frame skip 25 (20 Hz), implicit-fast integrator |
| Step | face at x = 0.5 m, height 0.15 m (1.5 radii); platform beyond |
| Action per sphere | `[fx, fy, lift, attach]` in [-1, 1]; force in N; lift up to 6 N (about twice the weight), internal: the reaction is split on linked partners |
| Links | tendon equality between centres; formed only if both spheres request it and the gap is ≤ 2 cm; broken if either releases |
| Success | every sphere on the platform within `0.2·sqrt(N)` m of the goal |
| Episode | 150·N steps; reward = progress of the mean distance − 0.01 per step + 10 on success |
| Determinism | same seed → same episode |

The scripted expert `SwarmClimbPolicy` places one base sphere against the step face; the others
climb over it one at a time (staging area, align, link, lift, release on top); the last one on top
pulls the base up from the edge. It succeeds on 80/80 seeds for N = 2, 3, 4, 6 (at most 396 steps at
N = 6). It is sequential: on seeds 200000–200004 at N = 4 it takes 227–270 steps and only 41–45 %
of sphere-steps have a horizontal speed above 5 cm/s.

## 4. Methods

### 4.1 Data
Expert episodes with DART noise on `fx, fy, lift` (σ ∈ {0.1, 0.2, 0.3}; executed noisy, labelled
clean). Training set: 400 episodes (seeds 0–399), 134k transitions; an exploration set of 300
episodes with random actions mixed in (probability 0.15/0.3/0.5, 174k transitions) was added later.
Validation: 50 episodes from seed 100000. Noisy expert success at σ = 0/0.2/0.3/0.5:
100/85/80/55 % (20 episodes each).

### 4.2 Whole-task world model
`SwarmWorldModel` (`swarm_torch.py`): interaction network over spheres; node input = sphere state
(position, velocity, angular velocity) plus boundary features (distances to walls, step face and
step top); edge input = relative state and link flag; it predicts the state change of each sphere
and the next link logits. Trained with a multi-step unrolled loss (window 8). Reward and success are
computed analytically from the predicted state (no learned reward head).

### 4.3 Imitation
`SwarmPolicy`: one network shared by all spheres, on local observations, other spheres as a set of
tokens (mean and max pooling). Behaviour cloning, then DAgger [2] (3 rounds of 100 episodes,
β = 0.5, 0, 0).

### 4.4 Improvement in the learned model
Variants tried (Section 6): actor-critic with λ-returns in imagination with a BC regulariser;
ensemble disagreement penalty; Dyna/MBPO-style loop with model refits on learner data; critic
warm-up; pure model return. Finally PPO in the model ensemble with a MAPPO critic, a time penalty
(0.05 per step) and a KL penalty (weight 1) to the DAgger policy, run in chunks with the best chunk
chosen on separate seeds (`tools/train_swarm.py`).

### 4.5 Model-free MAPPO in MuJoCo
`swarm_rl.py`: subprocess vector environment (6 workers × 4 environments), shared decentralised
actor, centralised critic on the global state and the time fraction, per-agent PPO ratio with team
GAE, time-outs bootstrapped. Two settings: fine-tuning from DAgger, and training from scratch with
a step-height curriculum (start at 0.02 m, raise the height after a streak of successful
iterations).

### 4.6 Skills and skill world models
`swarm_skills.py`. Three skills over subsets of spheres:

- `roll(i, target)`: sphere *i* rolls to an xy target on its current level;
- `climb(c, b)`: base *b* holds the face, climber *c* aligns behind it, they link, *c* lifts and
  goes over *b* onto the platform;
- `pull(h, b)`: helper *h* on the edge links with base *b* at the face, and *b* pivots up.

Each skill has a scripted controller (pieces of the expert), a sampler of start states in its
initiation set and an event-based termination (target reached at low speed; climber on top; base
above the helper), following [16]. Data are collected in a 2-sphere environment with the state
placed directly in the initiation set and DART noise σ ∈ {0.1, 0.2, 0.3} (150 episodes each;
validation 60 episodes at σ = 0.2). One small `SwarmWorldModel` (hidden 64, 30 epochs) is trained
per skill on that skill's data only.

### 4.7 MPPI in the skill world model
`SkillPlanner`: MPPI [8] over the actions of the spheres involved in the skill (others get zero),
horizon 15, 256 samples, 2 iterations, σ = 0.5, temperature 0.05, plan shifted between control
steps (warm start as in TD-MPC [9]); the attach channel is sampled continuously and executed as its
sign. The cost is a hand-written distance to the skill's end state (e.g. for climb: climber to the
point above the step plus base to the face). Optional proposal: `SkillPrior`, a behaviour-cloned
policy of the skill controller on target-relative features; a fraction (25 %) of the samples are
noisy prior rollouts inside the model, the rest Gaussian around the current plan, as in
TD-MPC2 [10] and MoDem [11]. The world model's predicted cost still selects among all samples, so
the model, not the prior, decides the executed action.

### 4.8 Statistics
Success rates with exact Clopper–Pearson 95 % intervals; paired comparisons of success on the same
seeds with the exact McNemar test [19]; paired comparisons of episode length with a two-sided sign
test on untied pairs. Test seeds 200000–200049 are never used for model selection (validation seeds
300000–300019).

## 5. Results

### 5.1 Whole-task world model (validation, open loop)

| Horizon (steps) | 1 | 10 | 30 |
|---|---|---|---|
| Position RMSE, world model | 0.9 mm | 5.5 mm | 19 mm |
| Position RMSE, constant velocity | 2.1 mm | 61 mm | 237 mm |

Link prediction accuracy 99.9 %. Adding the exploration set reduced the 30-step error on
exploratory trajectories from 132 to 34 mm and on validation from 18.7 to 16.5 mm.

### 5.2 Imitation (test seeds 200000–200049 unless noted)

| Policy | Success | 95 % CI |
|---|---|---|
| BC, seeds 200000–200029 | 24/30 | [0.61, 0.92] |
| BC | 34/50 | [0.53, 0.81] |
| DAgger round 0 (β = 0.5) | 50/50 | [0.93, 1.00] |
| DAgger round 1 | 49/50 | [0.89, 1.00] |
| DAgger round 2 (kept) | 47/50 | [0.84, 0.99] |

### 5.3 Improvement in the model and model-free RL

PPO with KL to DAgger in the model ensemble, test seeds 200000–200049:

| Policy | Success | Speed vs expert | Mean steps (both succeed) |
|---|---|---|---|
| DAgger | 47/50 | 0.919 | 253.5 |
| PPO + KL (best of chunks on validation) | 47/50 | 0.942 | 246.5 |

McNemar on success p = 1. Of the 45 episodes both solve, PPO is faster on 42, slower on 1, tied on
2: sign test p ≈ 1.0e-11. The gain is small (about 3 %) and keeps the expert's sequential strategy,
which a faster scripted demonstration could reproduce at no cost; we therefore do not treat it as
the answer to the speed question.

MAPPO fine-tuned from DAgger (validation seeds 300000–300019; DAgger 18/20, speed 0.878, 45 % of
sphere-steps moving): with learning rate 1e-4 the policy collapses (0/20); with 1e-5 the best
iteration (100) reaches 20/20 (CI [0.83, 1.00]), speed 1.012 and 62 % moving, then oscillates
between speed 0.74 and 0.96, and episodes never get shorter than about 250 steps. This run had a
critic input normalisation bug, fixed afterwards (Section 6). From scratch with the curriculum: the
height rose 0.02 → 0.03 (it. 34) → 0.04 (it. 119) → 0.05 (it. 162) → 0.06 m (it. 271), then training
success stayed at 0 up to iteration 350, with 0/20 at the real height 0.15 m and links in 5 % of
steps; the run was stopped. A lone sphere climbs steps up to 0.06 m, and a greedy go-to-goal policy
without links solves 10/10 at 0.02 m, 2/10 at 0.04 m and 0/10 at 0.06 m: the curriculum stalls
exactly where cooperation becomes necessary.

### 5.4 Skills

**Controllers** (30 episodes per row, `skills_s1.py`): every skill succeeds 30/30
(CI [0.88, 1.00]) at noise 0 and 0.2; mean duration roll 49–60 steps, climb about 52, pull about 61.

**Skill world models vs the whole-task model** on each skill's validation data (position RMSE,
open loop, `skills_s2.py`):

| Skill | Skill model h1 / h10 / h30 | Whole-task model h1 / h10 / h30 | Ratio at h30 |
|---|---|---|---|
| roll | 1.2 / 14.2 / 53.7 mm | 2.0 / 32.3 / 147.4 mm | 2.7× |
| climb | 0.5 / 2.6 / 13.1 mm | 1.2 / 26.6 / 72.2 mm | 5.5× |
| pull | 0.4 / 2.9 / 6.2 mm | 0.8 / 16.2 / 113.2 mm | 18× |

The whole-task model was trained on 4-sphere expert and exploration data; part of its error on the
skill data is distribution shift (2 spheres, placed starts), which is the point of the comparison
but also a limit (Section 7).

**MPPI in the skill model chooses the actions** (10 episodes per skill, MuJoCo, `skills_s3.py`,
**preliminary**):

| Skill | Controller | MPPI, skill model | 95 % CI (MPPI) | Mean steps on success |
|---|---|---|---|---|
| roll | 30/30 | 10/10 | [0.69, 1.00] | 58 |
| climb | 30/30 | 3/10 | [0.07, 0.65] | 38 |
| pull | 30/30 | 0/10 | [0.00, 0.31] | — |

Rolling is solved by short-horizon planning. Climb and pull need a sequence (align, link, lift)
whose payoff lies beyond a 15-step horizon under a distance cost. Follow-up runs (same 10 starts,
**preliminary**):

| Variant | climb | pull |
|---|---|---|
| MPPI, horizon 30, 512 samples, 3 iterations, temperature 0.05 | 0/10 | 2/10 (118 steps) |
| same, temperature 0.01 | 6/10 (75 steps) | 0/10 |
| Behaviour-cloned skill policy alone (no model) | 6/10 (48 steps) | 10/10 (74 steps) |
| MPPI with the cloned policy as proposal (25 % of samples) | 0/10 | 0/10 |
| MPPI, learned steps-to-go cost, temperature 0.05 / 0.5 | 0/10, 0/10 | 0/10, 0/10 |
| MPPI, learned steps-to-go cost + cloned proposal, temperature 0.05 | 0/10 | 6/10 (66 steps) |
| same, temperature 0.5 | 0/10 | 9/10 (79 steps), CI [0.55, 1.00] |

The cloned policy alone solves pull, but MPPI rejects its samples when they are ranked by the
hand-written distance cost: **the cost, not the model, is the main failure** of this stage. This
motivates replacing the hand-written cost by a learned *steps-to-go* value of the skill
(`SkillValue`, Monte Carlo targets from the skill data, failures counted as 200 steps), which is
both the planner's terminal value as in TD-MPC [9] and the duration part of a skill effect model as
in [15]. Validation error of the steps-to-go prediction (MAE / RMSE): roll 14.8 / 43.7, climb
2.3 / 3.0, pull 7.0 / 8.5 steps. With this value and the cloned proposal, the skill model now
chooses actions that solve pull in 9/10 episodes, against 0/10 with the hand-written cost. The
value alone (Gaussian samples only) fails on both skills: the proposal is needed to reach the
states where the value is informative. Climb still fails in every model-based variant although the
value is accurate on the data (MAE 2.3 steps); the next check is an oracle ablation (the simulator
instead of the skill model) to tell a model error in the link-and-lift phase from a planning
error.

### 5.5 Small world-action model per skill

Method (`swarm_wam.py`). Following the world-action models of GR00T 2 [20] and Cosmos Policy [21],
one network per skill predicts **jointly** the future of the scene and the chunk of actions that
produces it, and the action tokens attend to the predicted future frames. At our scale the "scene"
is the target-relative state of the involved spheres (`skill_features`), not video. A transformer
(width 128, 3 layers, 4 heads) reads the last 4 frames (0.2 s) and outputs the next 10 frames
(as deltas from the last frame, normalised by their standard deviation) and 10 actions (0.5 s),
with ACT-style action chunking [22]. Loss: action MSE on fx, fy, lift + BCE on attach + frame MSE
(weight 1), trained 60 epochs on each skill's expert data. Execution re-predicts a chunk at every
step and runs its first action, or averages the chunks' predictions for the current step with
ACT's temporal ensembling (weight `exp(-0.1 i)`, `i = 0` for the oldest chunk). Deviations from
GR00T 2: deterministic regression instead of flow matching, state tokens instead of video latents,
one small model per skill.

Results (MuJoCo, same starts as Section 5.4; **preliminary** at 10 episodes):

| Skill | Cloned policy (BC) | Best MPPI variant | WAM | WAM + temporal ensembling |
|---|---|---|---|---|
| climb, 10 episodes | 6/10 (48 steps) | 6/10 (75 steps) | **10/10** (52 steps) | **10/10** (50 steps) |
| pull, 10 episodes | 10/10 (74 steps) | 9/10 (79 steps) | **10/10** (66 steps) | 8/10 (87 steps) |

Paired comparison on 50 starts per skill (same env seeds 0–49 and start sampler seed 123 for every
controller; 95 % Clopper–Pearson intervals; exact McNemar on success against BC; sign test on
steps over the starts both controllers solve):

| Skill | BC | WAM | WAM + temporal ensembling |
|---|---|---|---|
| climb | 27/50 [0.39, 0.68], 48.3 steps | **50/50** [0.93, 1.00], 51.0 steps | **50/50** [0.93, 1.00], 47.9 steps |
| climb vs BC | | 23–0, p = 2.4e-7; slower on 24/27, p = 5e-5 (+3.6 steps) | 23–0, p = 2.4e-7; 12 faster, 12 slower |
| pull | 50/50 [0.93, 1.00], 74.0 steps | 50/50 [0.93, 1.00], **66.9 steps** | 39/50 [0.64, 0.88], 71.1 steps |
| pull vs BC | | 0–0; faster on 31/50, p = 0.12 (−7.1 steps) | 0–11, p = 0.001 |
| roll | 50/50 [0.93, 1.00], 57.8 steps | 48/50 [0.86, 1.00], 57.9 steps | 48/50 [0.86, 1.00], 56.4 steps |
| roll vs BC | | 0–2, p = 0.5 | 0–2, p = 0.5 |

The WAM solves climb reliably where the cloned MLP does not (50/50 against 27/50, p = 2.4e-7), at a
cost of 3.6 steps without ensembling. On pull and roll it does not differ significantly from BC; on
pull, ensembling loses 11 starts (p = 0.001 against both BC and the WAM without ensembling). The
roll WAM predicts its future frames well (6 mm one step ahead, 21 mm ten steps ahead), since a
single rolling sphere is easy to predict.

**Ablations (`skills_wam_ablate.py`).** We expected the history window (the controller's lift
phase latches only once both spheres are aligned *and* slow) or chunking to explain the climb gain.
Each variant is retrained from scratch and executed without ensembling on the same 50 starts;
McNemar is against the full WAM (50/50) and the BC of the table above (27/50):

| Climb variant | Seeds | Success | Steps | vs WAM | vs BC |
|---|---|---|---|---|---|
| full WAM, training seed 1 | 1 | 50/50 | 50.2 | 0–0 | 23–0, p = 2.4e-7 |
| no history (1 frame) | 0 | 50/50 | 48.3 | 0–0 | 23–0, p = 2.4e-7 |
| no chunk (1 action, 1 frame) | 0 | 50/50 | 50.8 | 0–0 | 23–0, p = 2.4e-7 |
| no frame loss | 0 | 50/50 | 49.9 | 0–0 | 23–0, p = 2.4e-7 |
| none of the three (transformer BC on one frame) | 0, 1, 2 | 50, 50, 50/50 | 50.2–50.9 | 0–0 | 23–0, p = 2.4e-7 |
| MLP 128, 30 epochs, lr 1e-3, batch 1024 | 0, 1, 2 | 24, 26, 24/50 | 46.6–48.4 | 0–24 to 0–26, p ≤ 1.2e-7 | n.s. |
| MLP 128, WAM recipe (60 epochs, lr 3e-4, batch 512) | 0 | 25/50 | 48.4 | 0–25, p = 6e-8 | 0–2, p = 0.5 |
| **MLP 512, WAM recipe** | 0, 1, 2 | 49, 50, 49/50 | 48.8–49.4 | 0–1 or 0–0 | 23–1 or 23–0, p ≤ 3e-6 |

No world-action ingredient matters for climb: a transformer that sees one frame, predicts one
action and no frame solves it as well as the full WAM. The training recipe does not matter either
(MLP 128 with the WAM's epochs and learning rate: 25/50). Width does: the 128-unit MLP underfits
(training loss 0.10 against 0.08 for 512 units and 0.03 for the transformer), and the 512-unit MLP
recovers 49–50/50 on three seeds. Pull confirms it: the 128-unit MLP trained 30 epochs solves 17/50,
whereas the 40-epoch one of the table above solves 50/50; the 512-unit MLP and the bare transformer
both solve 50/50 (61.0 and 61.7 steps), so the WAM's 7-step gain on pull is also matched without its
ingredients. The cloned policy of Sections 5.4–5.5 was therefore an underfitted baseline, and the
climb and pull results credited to the WAM above are a capacity effect. The predicted frames are
coarse (validation position error 30 mm one step ahead and 87 mm ten steps ahead for climb; 19 and
59 mm for pull) compared with the dedicated skill world model (0.5 mm one step ahead), so in this
model the future frames act as an auxiliary target that shapes the representation rather than as an
accurate simulator. A first version that normalised the frame deltas by the state's standard
deviation instead of the deltas' gave 5/10 (8/10 with ensembling) on climb.

### 5.6 Group-relative self-imitation in MuJoCo

Method (`rollout_skill`, `select_elite`). GR00T 2 [20] post-trains its WAM with GRPO [23] and
"GRPO-induced self-imitation", fine-tuning on its own successful rollouts. We use the second, which
needs no likelihood of the chunked policy: per round, 8 rollouts with Gaussian noise (σ = 0.2 on
fx, fy, lift) from each of 40 sampled starts; per start, keep the successful rollouts at least as
fast as the group's mean successful duration (non-negative group-relative advantage under the
reward success − time); fine-tune 15 epochs (learning rate 1e-4, normalisation frozen) on these
rollouts mixed with the expert data; evaluate deterministically on 50 fixed starts. RL runs in
MuJoCo, not in the world model: the simulator costs nothing here, unlike the real robot of [20].

| Skill, execution | Round 0 | Round 1 | Round 2 | Round 3 |
|---|---|---|---|---|
| pull, no ensembling (labels clipped, see below) | 50/50, 66.9 steps | 50/50, 64.5 | 50/50, 60.6 | 50/50, 60.6 |
| climb, ensembling, σ = 0.2 | 50/50, 47.9 | 1/50 | 29/50 | 36/50 |
| climb, ensembling, σ = 0.1 | 50/50, 47.9 | 0/50 | 50/50 (no elite: expert data only) | 5/50 |
| climb, ensembling, σ = 0, all 150 rollouts | 50/50, 47.9 | 23/50 | | |
| climb, no ensembling, σ = 0, all 150 rollouts | 50/50, 51.0 | 50/50, 50.3 | | |
| **climb, no ensembling, σ = 0.2** | 50/50, 51.0 | 50/50, 49.9 | 49/50, 50.2 | 50/50, 50.1 |
| **climb, no ensembling, σ = 0.1** | 50/50, 51.0 | 50/50, 50.3 | 50/50, 50.0 | 49/50, 50.3 |
| **pull, no ensembling, σ = 0.2** | 50/50, 66.9 | 50/50, 64.0 | 50/50, 61.9 | 50/50, 60.7 |

Without ensembling (bold rows, labels before clipping), self-imitation is stable on both skills.
On pull it keeps every success and shortens the skill by 9 % (66.9 → 60.7 steps; round 3 faster
than round 0 on 37 starts, slower on 9, sign test p = 4e-5, −6.2 steps on average). On climb it
keeps 49–50/50 and gains less than one step (σ = 0.1: 28 faster, 2 slower, p = 9e-7, −0.7 steps),
which leaves the WAM at the speed of the cloned policy (Section 5.5). With ensembling, climb
collapses, even without noise: the
executed action is a weighted average of older chunks, so the fine-tuning labels are a lagged and
smoothed copy of the model's own predictions; ensembling them again at the next round adds lag,
and the lift phase of climb is timing-critical. Without ensembling, noise-free self-imitation is
stable (50/50).

### 5.7 Parallel skill scheduling with effect models (S4)

Method (`swarm_plan.py`). A plan splits the N spheres into lanes, each with a line `y`, a base and
its climbers. In a lane, the base and the climbers first roll to fixed spots inside the climb
initiation set (base 0.11 m before its climbing position, climbers queued 0.45 m and more behind),
the climbers climb over the base one at a time, the last one pulls the base up, and every sphere on
the platform rolls to a gathering slot. Lanes run in parallel. The plan is the minimum of a
predicted makespan over every partition of the spheres into lanes of two or more, every base
choice and two or three lane spacings (±0.3 or ±0.45 m around the goal line for two lanes); the
lane's makespan sums the durations predicted by the steps-to-go models of Section 5.4 at each
skill's nominal start (Liang et al. [15]: skill effect models composed by a search). The climb
order is replanned from the true state at each climb start (smallest predicted climb first).
Skills end on their termination events and restart after 200 steps. The executor of each skill
call is the scripted controller, the 512-unit cloned MLP (Section 5.5) or the WAM.

Results (four spheres, test seeds 200000–200049, the same for every controller; McNemar on
success, sign test on steps over the seeds both solve; "moving" is the mean fraction of spheres
faster than 5 cm/s):

| Controller | Success | Steps | Moving | vs expert | vs DAgger |
|---|---|---|---|---|---|
| Expert (`SwarmClimbPolicy`) | 50/50 [0.93, 1.00] | 248.2 | 0.44 | | 3–0, p = 0.25; faster on 40/47 |
| DAgger (Section 5.2) | 47/50 [0.83, 0.99] | 253.9 | 0.45 | 0–3, p = 0.25 | |
| **Scheduler, scripted skills** | **50/50** [0.93, 1.00] | **149.9** | 0.79 | 0–0; faster on 50/50, p = 1.8e-15, ×0.61 | 3–0, p = 0.25; faster on 47/47, ×0.59 |
| Scheduler forced to one lane, scripted | 47/50 [0.83, 0.99] | 250.2 | 0.51 | 0–3, p = 0.25; 24 faster, 23 slower | 3–3; ×0.99 |
| Scheduler, cloned MLP 512 | 30/50 [0.45, 0.74] | 131.0 | 0.67 | 0–20, p = 1.9e-6; faster on 30/30, ×0.53 | 2–19, p = 2.2e-4 |
| Scheduler, WAM | 38/50 [0.62, 0.87] | 125.4 | 0.73 | 0–12, p = 4.9e-4; faster on 38/38, ×0.51 | 3–12, p = 0.035 |

The effect models choose two lanes on every seed, and that choice is the whole gain: forced to one
lane, the same scheduler and skills take as long as the expert. With scripted skills the predicted
makespan is 4 steps shorter than the real one on average (absolute error 5 steps; 9.8 steps for one
lane), so the steps-to-go models are accurate enough to rank plans. The scheduler roughly doubles
the fraction of spheres moving (0.79 against 0.44). With learned executors the successful
episodes are even faster than with the scripts (125–131 steps), but the success rate drops. Every
WAM failure is a climb that stalls (24 restarts, all climb; the cloned MLP also stalls on rolls).
The cause is the hand-off: from the nominal start at rest, all three executors climb 30/30 in both
the two- and four-sphere environments, but after a learned roll brings the two spheres to the same
spots (residual speed below 5 cm/s), the WAM climbs 30/40 and the cloned MLP 31/40, also after 20
idle steps (30/40 and 36/40). The climb data start at rest at sampled positions, so the end
states of the roll skill are outside the learned climb's training distribution: the skill-chaining
problem [24, 25] that the scripted controllers,
being closed-loop on the true state, do not have.

## 6. Negative results and fixed defects

| Attempt | Outcome | Diagnosis |
|---|---|---|
| Imagination actor-critic + BC (weights 1, 10, 100) | real success 0/50 (DAgger 47/50, McNemar p < 0.001) | the model lets spheres drive through the step (predicted x = 2.9 m, real 0.4 m) |
| Ensemble disagreement penalty (3 models) | 0/50 | ensemble members share the blind spot |
| Boundary features in the model | 60-step error on DAgger trajectories 1.1 m → 0.02 m | kept; far out-of-distribution pushes still hallucinated |
| Dyna/MBPO loop with refits | 0/20 for 3 rounds | policy stays collapsed after the model is corrected |
| Critic warm-up | imagined return 9.4, real 0/20 | imagined successes only 1–4 %: the critic overestimates, not the model |
| Pure 15-step model return | 0/20 | a greedy progress reward breaks cooperation |
| MAPPO critic input normalisation | statistics taken at reset, velocity std ≈ 1e-3 | replaced by raw state |
| Curriculum streak | reset by iterations without finished episodes | fixed |
| CUDA "illegal instruction" in long backward passes | crash at full scale only | RL runs on CPU |
| MPPI with the hand-written skill cost | climb ≤ 6/10, pull ≤ 2/10; the cloned proposal is rejected | cost misaligned with the skill's end; replaced by a learned steps-to-go value |
| WAM frame deltas normalised by the state std | climb 5/10 | near-constant state dimensions dominate the loss; normalise by the deltas' std |
| Self-imitation labels clipped to [-1, 1] | biased saturated labels (lift is 0 or ±1) | label with the sample before clipping (climb still collapsed: next row) |
| Self-imitation on temporally ensembled rollouts | climb 50/50 → 0–36/50 | lagged labels; collect without ensembling |

## 7. Threats to validity and limits

- Simulation only; nothing here says anything about hardware.
- One environment configuration (N = 4, one step height) for the learning results.
- Several MPPI and RL results use 10–20 episodes; their intervals are wide (Section 5).
- The model-comparison table measures the whole-task model out of its training distribution.
- The skill cost functions are hand-written; their shaping may decide climb/pull success more than
  the model accuracy. An ablation with the true simulator as the model (oracle MPPI) is needed to
  separate planning failures from model errors.
- The cloned proposal of every MPPI variant (Section 5.4) is the underfitted 128-unit MLP of
  Section 5.5; the MPPI climb results must be rerun with a wider prior before any conclusion on
  planning.
- Novelty claims rest on abstracts of related work.

## 8. Reproducibility

Software: Python 3.12.13, MuJoCo 4.0.1,
PyTorch 2.15.0 (CUDA 13.2), NumPy 2.5.5, Gymnasium 1.4.0, PettingZoo 1.27.0; one RTX 4070 Laptop
GPU, 6 CPU cores. Environment: `uv sync --extra dev --extra torch`.

Software tests: `uv run ruff check src tests` and `MUJOCO_GL=egl uv run pytest -q --cov` pass
with 128 tests and 100 % line and branch coverage, including `swarm_rl.py`, `swarm_skills.py`,
`swarm_wam.py` and the `step_height` option (`tests/test_swarm_rl.py`,
`tests/test_swarm_skills.py`). Tests check interfaces, invariants and that losses decrease; they do
not reproduce the success rates above.

Experiment scripts are copied in `docs/paper/experiments/` (paths inside are absolute to the
author's machine and still need to be parametrised):

| Result | Script | Seeds |
|---|---|---|
| 5.3 test comparison | `stage10.py` | 200000–200049 |
| 5.3 MAPPO from DAgger | `stage11.py` | train 1000000·(w+1); val 300000–300019 |
| 5.3 MAPPO from scratch | `stage12.py` | same |
| 3 throughput, expert parallelism | `simspeed.py` | 200000–200004 |
| 5.3 lone sphere, greedy | `solo_climb.py`, `greedy.py` | 0–9 |
| 5.4 controllers | `skills_s1.py` | 0–29 |
| 5.4 skill models | `skills_s2.py` | train 0, 1000, 2000 (+episode); val 90000 |
| 5.4 MPPI | `skills_s3.py`, `skills_prior.py` | episodes 0–9, start sampler seed 123 |
| 5.5 WAM | `skills_wam.py`, `skills_wam_stats.py` | same starts (10 or 50 episodes) |
| 5.5 ablations | `skills_wam_ablate.py`, `skills_wam_ablate_stats.py` | same starts (50); training seeds 0–2 |
| 5.7 scheduler | `skills_s4.py`, `skills_s4_stats.py` | test seeds 200000–200049 |
| 5.6 self-imitation | `skills_selfimit.py` | rollouts 100000·round (+start); eval as 5.5 |

Whole-task pipeline: `uv run python -m modular_sphere.tools.train_swarm` (see `--help`).

## 9. Next steps

1. Replace the 128-unit cloned policy by the 512-unit one as the BC baseline and MPPI proposal;
   rerun the MPPI and self-imitation comparisons against it; self-imitation for roll.
2. MPPI baseline: oracle-model ablation on climb; 50 episodes per skill.
3. S4 with learned skills: collect climb and pull data from the end states of the preceding skill
   (roll then climb, climb then pull) with DART noise, retrain the cloned MLP and the WAM, and rerun
   the scheduler on the test seeds; then N = 6 (three lanes) and a success term in the effect
   models.

## References

1. P. Battaglia et al. *Interaction Networks for Learning about Objects, Relations and Physics*.
   NeurIPS 2016. arXiv:1612.00222.
2. S. Ross, G. Gordon, J. A. Bagnell. *A Reduction of Imitation Learning and Structured Prediction
   to No-Regret Online Learning*. AISTATS 2011. arXiv:1011.0686.
3. M. Laskey, J. Lee, R. Fox, A. Dragan, K. Goldberg. *DART: Noise Injection for Robust Imitation
   Learning*. CoRL 2017. arXiv:1703.09327.
4. T. Kurutach, I. Clavera, Y. Duan, A. Tamar, P. Abbeel. *Model-Ensemble Trust-Region Policy
   Optimization*. ICLR 2018. arXiv:1802.10592.
5. J. Schulman, F. Wolski, P. Dhariwal, A. Radford, O. Klimov. *Proximal Policy Optimization
   Algorithms*. 2017. arXiv:1707.06347.
6. J. Schulman, P. Moritz, S. Levine, M. Jordan, P. Abbeel. *High-Dimensional Continuous Control
   Using Generalized Advantage Estimation*. ICLR 2016. arXiv:1506.02438.
7. C. Yu, A. Velu, E. Vinitsky, J. Gao, Y. Wang, A. Bayen, Y. Wu. *The Surprising Effectiveness of
   PPO in Cooperative Multi-Agent Games*. NeurIPS 2022 Datasets and Benchmarks. arXiv:2103.01955.
8. G. Williams, P. Drews, B. Goldfain, J. M. Rehg, E. A. Theodorou. *Information Theoretic Model
   Predictive Control: Theory and Applications to Autonomous Driving*. 2017. arXiv:1707.02342.
9. N. Hansen, X. Wang, H. Su. *Temporal Difference Learning for Model Predictive Control*. ICML
   2022. arXiv:2203.04955.
10. N. Hansen, H. Su, X. Wang. *TD-MPC2: Scalable, Robust World Models for Continuous Control*.
    ICLR 2024. arXiv:2310.16828.
11. N. Hansen, Y. Lin, H. Su, X. Wang, V. Kumar, A. Rajeswaran. *MoDem: Accelerating Visual
    Model-Based Reinforcement Learning with Demonstrations*. ICLR 2023. arXiv:2212.05698.
12. D. Pathak, C. Lu, T. Darrell, P. Isola, A. Efros. *Learning to Control Self-Assembling
    Morphologies: A Study of Generalization via Modularity*. NeurIPS 2019. arXiv:1902.05546.
13. A. Sharma, S. Gu, S. Levine, V. Kumar, K. Hausman. *Dynamics-Aware Unsupervised Discovery of
    Skills*. ICLR 2020. arXiv:1907.01657.
14. L. X. Shi, J. J. Lim, Y. Lee. *Skill-based Model-based Reinforcement Learning*. CoRL 2022.
    arXiv:2207.07560.
15. J. Liang, M. Sharma, A. LaGrassa, S. Vats, S. Saxena, O. Kroemer. *Search-Based Task Planning
    with Learned Skill Effect Models for Lifelong Robotic Manipulation*. ICRA 2022.
    arXiv:2109.08771.
16. T. Silver, A. Athalye, J. B. Tenenbaum, T. Lozano-Pérez, L. P. Kaelbling. *Learning
    Neuro-Symbolic Skills for Bilevel Planning*. CoRL 2022. arXiv:2206.10680.
17. E. Todorov, T. Erez, Y. Tassa. *MuJoCo: A physics engine for model-based control*. IROS 2012.
    doi:10.1109/IROS.2012.6386109.
18. J. K. Terry et al. *PettingZoo: Gym for Multi-Agent Reinforcement Learning*. NeurIPS 2021.
    arXiv:2009.14471.
19. Q. McNemar. *Note on the sampling error of the difference between correlated proportions or
    percentages*. Psychometrika 12(2):153–157, 1947. doi:10.1007/BF02295996.
20. NVIDIA. *GR00T 2: Scaling Robot Foundation Models for Generalization, Dexterity, and
    Adaptation* (whitepaper). November 2026. https://research.nvidia.com/labs/gear/gr00t-2/assets/gr00t2_whitepaper.pdf.
21. M. J. Kim et al. *Cosmos Policy: Fine-Tuning Video Models for Visuomotor Control and Planning*.
    ICLR 2026. arXiv:2601.16163.
22. T. Z. Zhao, V. Kumar, S. Levine, C. Finn. *Learning Fine-Grained Bimanual Manipulation with
    Low-Cost Hardware*. RSS 2023. arXiv:2304.13705.
23. Z. Shao et al. *DeepSeekMath: Pushing the Limits of Mathematical Reasoning in Open Language
    Models*. 2024. arXiv:2402.03300.
24. G. Konidaris, A. Barto. *Skill Discovery in Continuous Reinforcement Learning Domains using
    Skill Chaining*. NeurIPS 2009.
25. A. Bagaria, G. Konidaris. *Option Discovery using Deep Skill Chaining*. ICLR 2020.
