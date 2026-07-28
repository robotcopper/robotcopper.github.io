---
title: Machine Learning
time: 2024-09-23
---

# Machine Learning

Machine Learning is used to give robotic systems the ability to extract patterns from data — for perception, control, prediction, and decision support — without hand-coding every rule.

Its value comes from choosing the right learning setup for the problem: labeled data when the target is known, structure discovery when it is not, or trial-and-error policies when the system must act in an environment. On a robot, that choice is constrained by compute, latency, data availability, and how the model connects to the rest of the stack.

It is also useful to keep the vocabulary straight. Machine Learning sits inside the broader field of Artificial Intelligence; deep learning, generative models, and transformers occupy different regions of that map. The diagram below is a compact way to see those relationships:

<div style="text-align: center; margin: 1.25rem 0;">
  <img src="/config/assets/images/Machine_Learning/ai_ml_euler_diagram.jpg" alt="Euler diagram of Artificial Intelligence, Machine Learning, and related subfields" style="width: 85%; max-width: 720px;">
  <p style="color: gray; font-size: 0.85rem; margin-top: 0.5rem;">AI / ML landscape</p>
</div>

In robotics, classical pipelines (feature extractors, classifiers, decision trees) still matter alongside what we call neural approaches: they can be lighter, easier to validate on limited datasets, and more practical when computation is not justified. Modern stacks and deep models extend the same idea when the task needs more capacity.

This section gathers technical notes, pipelines, and lessons learned from applying machine learning on real projects — choosing methods that fit the system, not only the trend.

<div class="info-cards">

  <a class="info-card mint-1" href="{{ '/Machine_Learning/HoG_and_SVM/' | relative_url }}">
    <span class="info-card-title">HoG &amp; SVM</span>
    <p>
      A lightweight object-detection pipeline with Histogram of Oriented Gradients and Support Vector Machines — useful when a CNN would be overkill.
    </p>
  </a>

</div>
