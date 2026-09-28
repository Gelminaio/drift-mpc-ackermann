import os
import sys

import yaml
from acados_template import AcadosOcpSolver

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ackermann_nmpc.ocp import drift_ocp, ocp  # noqa: E402

# C code of a solver for the nodes, run by CMake: generate.py <vehicle_params.yaml> <output dir> <nmpc|drift>
params_file, out, name = sys.argv[1], sys.argv[2], sys.argv[3]
p = yaml.safe_load(open(params_file))['/**']['ros__parameters']
o = ocp(p, out) if name == 'nmpc' else drift_ocp(p, out)
o.name = name    # the C functions: nmpc_acados_*, drift_acados_*
AcadosOcpSolver.generate(o)
AcadosOcpSolver.build(out, with_cython=False)
