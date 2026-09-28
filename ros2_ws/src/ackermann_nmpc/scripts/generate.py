import os
import sys

import yaml
from acados_template import AcadosOcpSolver

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ackermann_nmpc.ocp import ocp  # noqa: E402

# C code of the NMPC for the node, run by CMake: generate.py <vehicle_params.yaml> <output dir>
params_file, out = sys.argv[1], sys.argv[2]
p = yaml.safe_load(open(params_file))['/**']['ros__parameters']
o = ocp(p, out)
o.name = 'nmpc'    # the C functions of the node: nmpc_acados_*
AcadosOcpSolver.generate(o)
AcadosOcpSolver.build(out, with_cython=False)
