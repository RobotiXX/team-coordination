# Team-Coordination
This is implementation of team-coordination between multiple agents in presence of adversaries using graphs as mentioned in our paper:

Sara Oughourli, Manshi Limbu, Zechen Hu, Xuan Wang, Xuesu Xiao, Daigo Shishika, [Team Coordination on Graphs with State-Dependent Edge Cost](https://arxiv.org/abs/2303.11457)

[Youtube Link is here!](https://www.youtube.com/watch?v=UnMjOX3ffw8&ab_channel=DaigoShishika)

## Requirements
* networkx(>=3.0)
* numpy(>=1.24.2)
* matplotlib(>=3.7.0)


# Installation

```bash
pip3 install -r requirements.txt
```


## Run the demo (small nodes graph)

```bash
cd team-coordination
python3 quickCompare.py
```

## Run the demo (large nodes graph)
For `--nodes` param, add number of nodes and for `riskyedge` param, add risk edge ratio (more info in the paper). 

```bash
cd team-coordination
python3 graphCompare.py --nodes 10 --riskedge 0.2
```

## Algorithms 
You can choose between following algorithms:
* `jsg`: Converts multi-agent problem as single agent path planning algorithm. 

* `cjsg`: Heirarchial path planning algorithm that alleviates the curse of dimesionality casued by `jsg`. 


## Cite

Please cite our paper if you use this code in your own work:
```
@misc{oughourli2023team,
      title={Team Coordination on Graphs with State-Dependent Edge Cost}, 
      author={Sara Oughourli and Manshi Limbu and Zechen Hu and Xuan Wang and Xuesu Xiao and Daigo Shishika},
      year={2023},
      eprint={2303.11457},
      archivePrefix={arXiv},
      primaryClass={cs.MA}
}
```
