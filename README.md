sasy
=======

SASY (Scalable and Adjustable SYmbolic) Planner is a flexible symbolic planner which
searches for a satisfying plan to a partially observable Markov decision process, or
a POMDP, while benefiting from advantages of classical symbolic planning such as
compact belief state expression, domain-independent heuristics, and structural simplicity.

Belief space symbolic formalism, an extension of classical symbolic formalism, can be used
to transform probabilistic problems into a discretized and deterministic representation such
that domain-independent heuristics originally created for classical symbolic planning
systems can be applied to them. SASY is optimized to solve POMDPs encoded in belief space
symbolic formalism, but can also be used to find a solution to general symbolic planning
problems.

Usage:

```bash
make clean release
./main --problem=[gripper,rocksample,kitchen]
```

The gripper domain is a simple example of the conversion from a PDDL description
to a belief space symbolic problem. See sasy/third_party/planner/gripper/gripper.pddl for
the PDDL description.

The rocksample domain is the belief space symbolic version of the widely-used
[RockSample benchmark domain][].

The kitchen domain is a new POMDP benchmark domain.

[RockSample benchmark domain]: http://longhorizon.org/trey/papers/smith04_hsvi.pdf
