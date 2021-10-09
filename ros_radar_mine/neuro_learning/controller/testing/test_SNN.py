#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 16:14:58 2021

@author: marina
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 20:41:01 2021

@author: marina
"""
# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import os
import extra.aux_funcs as af # :)
import numpy as np
from evol_funcs.evol_mut_eval import initializeIndividual, evaluate, evaluateNSGA2, evaluate_PID
#from evol_funcs.evol_funcs_ANN import evaluate, evaluate_PID

config_file = "config.yaml"

# Load configuration "cf" dir
cf = af.set_config("../config/" + config_file)

network = initializeIndividual(cf)

network(5)