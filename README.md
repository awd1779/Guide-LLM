# Guide-LLM  Repository

This repository contains the replication materials for our Guide-LLM. Here you will find code, instructions, and datasets necessary to reproduce the experiments and results presented in the paper.

## Overview

This research introduces Guide-LLM, an embodied Large Language Model (LLM)-based agent designed to assist persons with visual impairments (PVI) in navigating large indoor environments. Our approach integrates a novel text-based topological map, enabling the LLM to perform global path planning efficiently, hazard detection, localization error detection and recovery, and personalized navigation based on user preferences.

> **Paper Title:** Guide-LLM: An Embodied LLM Agent and Text-Based Topological Map for Robotic Guidance of People with Visual Impairments
>
> **Authors:** Sangmim Song, Sarath Kodagoda, Amal Gunatilake, Marc G. Carmichael, Karthick Thiyagarajan, Jodi Martin
>
> **Abstract:** Navigation presents significant challenges for persons with visual impairments (PVI). Traditional navigation aids, although invaluable, often lack detailed spatial guidance. Guide-LLM leverages LLMs and a text-based topological map for efficient path planning, adaptive hazard detection, and personalized navigation, significantly advancing assistive robotic navigation.

## Contents

- 1_send_prompt.py`: Source code for experiments and implementation.
- `data/`: Dataset or instructions to download the required datasets.
- `models/`: Pre-trained models used in experiments.
- `scripts/`: Utility scripts for running experiments and evaluations.

## Requirements

Software dependencies:

```bash
Python >= 3.8
PyTorch >= 1.10
NumPy
Pandas
```

Install dependencies using:

```bash
pip install -r requirements.txt
```

## Replication Instructions

Follow these steps to reproduce the experimental results:

1. **Setup Environment**
```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

2. **Download Dataset (if applicable)**
```bash
bash scripts/download_data.sh
```

3. **Run Experiments**
```bash
python src/main.py --config config/guide-llm.yaml
```

4. **Evaluate Results**
```bash
python src/evaluate.py --model models/guide_llm.pth
```

## Expected Results

These outcomes correspond to the findings reported in our paper:

| Experiment | Success Rate | Localization Recovery | Hazard Detection |
|------------|--------------|-----------------------|------------------|
| Office     | 86%          | 72%                   | 83%              |
| House      | 84%          | N/A                   | N/A              |

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

**Contact:** For support or questions, please contact Sangmim Son
****
