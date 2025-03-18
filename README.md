# Guide-LLM Repository

This repository contains the replication materials for our Guide-LLM. Here you will find code, instructions, and datasets necessary to reproduce the experiments and results presented in the paper.

## Overview

This research introduces Guide-LLM, an embodied Large Language Model (LLM)-based agent designed to assist persons with visual impairments (PVI) in navigating large indoor environments. Our approach integrates a novel text-based topological map, enabling the LLM to perform global path planning efficiently, hazard detection, localization error detection and recovery, and personalized navigation based on user preferences.

> **Paper Title:** [Guide-LLM: An Embodied LLM Agent and Text-Based Topological Map for Robotic Guidance of People with Visual Impairments](https://arxiv.org/abs/2410.20666)
>
> **Authors:** Sangmim Song, Sarath Kodagoda, Amal Gunatilake, Marc G. Carmichael, Karthick Thiyagarajan, Jodi Martin
>
> **Abstract:** Navigation presents significant challenges for persons with visual impairments (PVI). Traditional navigation aids, although invaluable, often lack detailed spatial guidance. Guide-LLM leverages LLMs and a text-based topological map for efficient path planning, adaptive hazard detection, and personalized navigation, significantly advancing assistive robotic navigation.

## Contents

- `main/`: Contains code for running LLMs from different services.
- `ros/`: ROS nodes for running Guide-LLM; run them in numerical order to start necessary modules.
- `embeddings/`: For creating vector embeddings from images.

## Requirements

- ROS Noetic
- iGibson Simulator
- Python 3.8 or higher
- Additional Python libraries as specified in `requirements.txt`

## Replication Instructions

Follow these steps to reproduce the experimental results:

### Setup Environment

Install iGibson simulator following the official instructions:

[Installation Instructions](https://stanfordvl.github.io/iGibson/installation.html)

Install ROS Noetic following the official instructions:

[ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)

### Download Indoor Environment

We used the Stanford Scene area_4 for experiments. Download the dataset here:

[Stanford Scene Dataset](https://stanfordvl.github.io/iGibson/dataset.html)



## Citation

Please cite our paper if you use this repository:

```bibtex
@inproceedings{song2025guidellm,
  author = {Song, Sangmim and Kodagoda, Sarath and Gunatilake, Amal and Carmichael, Marc G. and Thiyagarajan, Karthick and Martin, Jodi},
  title = {Guide-LLM: An Embodied LLM Agent and Text-Based Topological Map for Robotic Guidance of People with Visual Impairments},
  booktitle = {Proceedings of IROS},
  year = {2025}
}
```

## License

This project is released under the MIT License.

---

**Contact:** For support or questions, please contact Sangmim Song (Sangmim.Song@student.uts.edu.au).

