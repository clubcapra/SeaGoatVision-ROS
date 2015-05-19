__author__ = 'yohan'

import ConfigParser
import rospy

def replace_filter(src_chain, dst_chain, filter):

    src_cfg = ConfigParser.ConfigParser()
    dst_cfg = ConfigParser.ConfigParser()

    src_cfg.read(src_chain)
    dst_cfg.read(dst_chain)

    rospy.loginfo("Copying {} from {} to {}".format(filter, src_chain, dst_chain))
    
    if not any(s.startswith(filter) for s in src_cfg.sections()):
        rospy.logwarn("Unable to find filter {} in source filterchain {}".format(filter, src_chain))
        return

    if not any(s.startswith(filter) for s in dst_cfg.sections()):
        rospy.logwarn("Unable to find filter {} in dest filterchain {}".format(filter, dst_chain))
        return

    out_cfg = ConfigParser.ConfigParser()
    for section_raw in dst_cfg.sections():
        section = section_raw
        if '-' in section:
            section = section_raw[0:section_raw.index('-')]
        if section == filter:
            out_cfg.add_section(section_raw)
            src_section_name = None

            for src_section in src_cfg.sections():
                if src_section.startswith(filter):
                    src_section_name = src_section
                    break

            for option in src_cfg.options(src_section_name):
                out_cfg.set(section_raw, option, src_cfg.get(src_section_name, option))
        else:
            out_cfg.add_section(section_raw)
            for option in dst_cfg.options(section_raw):
                out_cfg.set(section_raw, option, dst_cfg.get(section_raw, option))

    out_cfg.write(open(dst_chain, 'w'))
