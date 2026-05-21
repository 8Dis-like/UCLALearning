# 🎓 UCLA Master of Engineering — ML & AI Coursework Portfolio

> **A curated collection of machine learning, deep learning, reinforcement learning, and data analytics projects completed during my M.Eng. at UCLA.**

[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](https://python.org)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.0+-EE4C2C?logo=pytorch&logoColor=white)](https://pytorch.org)
[![Jupyter](https://img.shields.io/badge/Jupyter-Notebook-F37626?logo=jupyter&logoColor=white)](https://jupyter.org)
[![MATLAB](https://img.shields.io/badge/MATLAB-Simulink-0076A8?logo=mathworks&logoColor=white)](https://www.mathworks.com)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## 📌 Highlights

- 🧠 **Built a GPT-style transformer from scratch** — multi-head causal self-attention, custom LayerNorm, weight tying, and autoregressive text generation on TinyStories
- 🤖 **Trained RL agents for autonomous driving** — implemented DQN, REINFORCE, PPO, and TD3 in MetaDrive and OpenAI Gym environments
- 📊 **End-to-end ML pipelines** — from classical models (decision trees, SVMs, neural nets) to modern deep learning, with full training, evaluation, and visualization

---

## 🗂️ Repository Structure

| Directory | Course | Topic | Key Skills & Tools |
|-----------|--------|-------|--------------------|
| [`C147A_C247B/`](C147A_C247B/) | ECE C147/247 — Deep Learning | **MiniGPT: Language Modelling & Transformers** | PyTorch, Transformer architecture, Attention mechanisms, Tokenization (tiktoken), W&B experiment tracking |
| [`CM146/`](CM146/) | CM 146 — Machine Learning | **Classical ML & Neural Networks** | Scikit-learn, NumPy, Classification (Mushroom UCI dataset), SVMs, Decision Trees, PyTorch fundamentals |
| [`CS260R/`](CS260R/) | CS 260R — Reinforcement Learning | **Deep RL for Control & Autonomous Driving** | OpenAI Gym, MetaDrive, DQN, Policy Gradient, PPO, TD3, Q-Learning |
| [`C263ABC/`](C263ABC/) | MECH&AE C263 — Robot Kinematics | **Trajectory Planning & Optimization** | MATLAB/Simulink, MuJoCo, Jacobian computation, Trajectory generation, Design optimization |
| [`ENG213/`](ENG213/) | ENG 213 — Data & Business Analytics | **Operations Research & Time Series** | PuLP (LP optimization), SQL, Alpha Vantage API, Pandas, Forecasting & deseasonalization |

---

## 🔥 Featured Project: MiniGPT — Transformer Language Model

**[`C147A_C247B/`](C147A_C247B/)** · Built from the ground up with no high-level library abstractions

| Component | Implementation |
|-----------|---------------|
| **Bigram LM** | Baseline language model with embedding → dropout → linear pipeline |
| **Single-Head Attention** | Scaled dot-product attention with causal masking |
| **Multi-Head Attention** | Parallel attention heads with concatenation and output projection |
| **Transformer Block** | Pre-norm architecture (LayerNorm → Attention → Residual → FFN) |
| **MiniGPT** | Full GPT-2 style model with learnable positional embeddings, weight tying, and autoregressive generation |
| **Training Pipeline** | AdamW optimizer, cosine annealing scheduler, cross-entropy loss, W&B logging, checkpointing |

Trained on the [TinyStories](https://huggingface.co/datasets/roneneldan/TinyStories) dataset · Benchmarked against a pretrained 20M-parameter model.

---

## 🤖 Reinforcement Learning Demos

**[`CS260R/`](CS260R/)** · Agents trained across multiple environments with increasing complexity

| Algorithm | Environment | Demo |
|-----------|-------------|------|
| Q-Learning | FrozenLake | [`QL FrozenLake.mp4`](CS260R/Animations/QL%20FrozenLake.mp4) |
| DQN | CartPole | [`DQN CartPole.mp4`](CS260R/Animations/DNQ%20CartPole.mp4) |
| REINFORCE | MetaDrive (Easy) | [`REINFORCE Metadrive.mp4`](CS260R/Animations/REINFORCE%20Metadrive%20easy.mp4) |
| PPO | CartPole / MetaDrive (Hard) | [`Results/`](CS260R/Results/) |
| TD3 | Pendulum / MetaDrive (Hard) | [`Results/`](CS260R/Results/) |

---

## 🛠️ Technical Skills Demonstrated

```
Languages        Python · MATLAB · SQL
ML / DL          PyTorch · Scikit-learn · NumPy · Pandas · Hugging Face Datasets
RL               OpenAI Gym · MetaDrive · DQN · PPO · TD3 · Policy Gradient
NLP / LLMs       Transformer architecture · Tokenization · Language Modelling · Text Generation
Optimization     PuLP (Linear Programming) · Trajectory Planning · Design Optimization
Data & Viz       Matplotlib · Seaborn · Weights & Biases · Jupyter Notebooks
Robotics / Sim   MuJoCo · Simulink · Forward/Inverse Kinematics · Jacobians
Dev Tools        Git · Google Colab · PyCharm · Kaggle
```

---

## 📫 Let's Connect

If you'd like to discuss any of these projects or explore collaboration opportunities, feel free to reach out!

---

<sub>📍 University of California, Los Angeles · Master of Engineering · 2025–2026</sub>
