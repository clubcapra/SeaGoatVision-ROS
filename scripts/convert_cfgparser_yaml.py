#!/usr/bin/env python

import yaml
import sys
import ConfigParser

def isnumeric(string):
    try:
        float(string)
        return True
    except ValueError:
        return False

cfg = ConfigParser.ConfigParser()
cfg.read(sys.argv[1])

out = []

for section in cfg.sections():
    obj = {'_type': section}

    for name in cfg.options(section):
        value = cfg.get(section, name)
        if value == 'True' or value == 'False':
            obj[name] = cfg.getboolean(section, name)
        elif isnumeric(value):
            obj[name] = cfg.getfloat(section, name)
        else:
            if isinstance(value, str):
                obj[name] = str(value)

        out.append(obj)
            
print yaml.dump(out, default_flow_style=False)
