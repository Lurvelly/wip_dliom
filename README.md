# [WIP] DLIOM - Direct Lidar-Intertial Odometry: Perceptive and Connective SLAM

This repo implements [DLIOM](https://arxiv.org/abs/2305.01843), based on DLIO [[repo](https://arxiv.org/abs/2203.03749)] | [[paper](https://arxiv.org/abs/2203.03749)]

## Current Status

### Frontend

* [ ] 2-step deskew
* [x] Jaccard-based keyframing
* [ ] Slip-resistant keyframing

### Backend

* Factors
    * [x] Sequential Factor
    * [x] Connective Factor
        * [ ] min/max threshold
    * [ ] Gravity factor
    * [ ] Loop factor
    * [x] GPS factor (unaligned, GTSAM)
        * [x] matched with keyframes in case of new
        * [ ] matched with any keyframe at any time
* Architecture
    * [x] separate thread
    * [ ] tightly coupled with Frontend

## Acknowledgments 

Thank you to the original Authors of DLIO; Kenny Chen, Ryan Nemiroff and Brett T. Lopez

## Citations

```bibtex
@misc{chen2023direct,
      title={Direct LiDAR-Inertial Odometry and Mapping: Perceptive and Connective SLAM}, 
      author={Kenny Chen and Ryan Nemiroff and Brett T. Lopez},
      year={2023},
      eprint={2305.01843},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

```bibtex
@article{chen2022dlio,
  title={Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction},
  author={Chen, Kenny and Nemiroff, Ryan and Lopez, Brett T},
  journal={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023},
  pages={3983-3989},
  doi={10.1109/ICRA48891.2023.10160508}
}
```

Please see the [original instructions](DLIO.md) as well.

