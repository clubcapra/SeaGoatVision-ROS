__author__ = 'yohan'

import ConfigParser
import rospy

def replace_filter(src_chain, dst_chain, filter):
    
    src = yaml.load(open(src_chain, 'r'))
    dst = yaml.load(open(dst_chain, 'r'))

    rospy.loginfo("Copying {} from {} to {}".format(filter, src_chain, dst_chain))
    
    if not any(obj['_type'] == filter for obj in src):
        rospy.logwarn("Unable to find filter {} in source filterchain {}".format(filter, src_chain))
        return
    if not any(obj['_type'] == filter for obj in dst):
        rospy.logwarn("Unable to find filter {} in dest filterchain {}".format(filter, dst_chain))
        return

    out = []

    for obj in dst:
        if obj['_type'] == filter:
            for obj_src in src:
                if obj_src['_type'] == filter:
                    out.append(obj_src)
                    break
        else:
            out.append(obj)

    with open(dst_chain, 'w') as f:
        yaml.dump(out, f, default_flow_style=True)
