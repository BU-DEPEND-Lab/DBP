## README
 * * *

![](flocking.gif)

This repository contains the source code to reproduce the experiments from our paper, "Byzantine Resilience at Swarm Scale: A Decentralized Blocklist from Inter-robot Accusations."

### Usage

Install the argos simulator following the [instructions](https://www.argos-sim.info/user_manual.php) for your system.

Additionally, the experiments depend on the [C++ boost graph library](https://www.boost.org/doc/libs/1_81_0/more/getting_started/index.html).

Build and install the networked footbot plugin used in our experiments. The headers for the plugin need to be available in the argos directory in order to compile the experiment controllers.

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cd plugins
make
sudo make install
```

Now we build the rest of the project

```bash
cd ..
make
```

The experiments can now be run using

```bash
argos3 -c <path_to_experiment>
```

e.g.

```bash
argos3 -c experiments/DBP_flocking/flocking_positive-obs.argos
```

To generate figures and inspect the experiment data (after running the experiments), we use the notebook `analysis/target-tracking.ipynb`. The notebook depends on pandas, numpy, and matplotlib, which can be installed using pip.
