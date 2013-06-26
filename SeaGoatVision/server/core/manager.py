#! /usr/bin/env python

#    Copyright (C) 2012  Octets - octets.etsmtl.ca
#
#    This file is part of SeaGoatVision.
#
#    SeaGoatVision is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
Description :
"""

from SeaGoatVision.server.tcp_server import Server
from configuration import Configuration
from resource import Resource
from SeaGoatVision.commons.keys import *
import logging

logger =  logging.getLogger("seagoat")

KEY_MEDIA = "media"
KEY_FILTERCHAIN = "filterchain"

class Manager:
    def __init__(self):
        """
            Structure of dct_execution
            {"execution_name" : {KEY_FILTERCHAIN : ref, KEY_MEDIA : ref}}
        """
        self.dct_exec = {}
        self.config = Configuration()
        self.resource = Resource()

        # tcp server for output observer
        self.server_observer = Server()
        self.server_observer.start("", 5030)

    def close(self):
        logger.info("Close manager")
        for execution in self.dct_exec.values():
            execution[KEY_MEDIA].close()
        self.server_observer.stop()

    ##########################################################################
    ################################ CLIENT ##################################
    ##########################################################################
    def is_connected(self):
        return True

    ##########################################################################
    ############################# SERVER STATE ###############################
    ##########################################################################

    ##########################################################################
    ######################## EXECUTION FILTER ################################
    ##########################################################################
    def start_filterchain_execution(self, execution_name, media_name, filterchain_name, file_name=None):
        # if file_name != None, it's a media video!
        execution = self.dct_exec.get(execution_name, None)

        if execution:
            logger.error("The execution %s is already created.", execution_name)
            return None

        filterchain = self.resource.get_filterchain(filterchain_name, force_new_filterchain=True)
        if not filterchain:
            logger.error("Filterchain %s. Maybe it not exist.", filterchain_name)
            return None

        # Exception, if not media_name, we take the default media_name from the filterchain
        if not media_name:
            media_name = filterchain.get_default_media_name()

        media = self.resource.get_media(media_name)
        if not media:
            logger.error("Media %s. Maybe it not exist.", media_name)
            return None

        if media.is_media_video() and file_name:
            media.set_file(file_name)

        media.add_observer(filterchain.execute)

        self.dct_exec[execution_name] = {KEY_FILTERCHAIN: filterchain, KEY_MEDIA : media}

        return True

    def stop_filterchain_execution(self, execution_name):
        execution = self.dct_exec.get(execution_name, None)

        if not execution:
            # change this, return the actual execution
            logger.warning("The execution %s is already stopped." % execution_name)
            return False

        # Remove execution image observer from media
        execution[KEY_MEDIA].remove_observer(execution[KEY_FILTERCHAIN].execute)
        # Destroy all memory
        execution[KEY_FILTERCHAIN].destroy()
        del self.dct_exec[execution_name]

        return True

    def get_execution_list(self):
        return self.dct_exec.keys()

    def get_execution_info(self, execution_name):
        exec_info = self.dct_exec.get(execution_name, None)
        if not exec_info:
            return None
        class Exec_info: pass
        o_exec_info = Exec_info()
        setattr(o_exec_info, KEY_MEDIA, exec_info[KEY_MEDIA].get_name())
        setattr(o_exec_info, KEY_FILTERCHAIN, exec_info[KEY_FILTERCHAIN].get_name())
        return o_exec_info

    ##########################################################################
    ################################ MEDIA ###################################
    ##########################################################################
    def get_media_list(self):
        return {name: self.resource.get_media(name).get_type_media()
                for name in self.resource.get_media_name_list()}

    def cmd_to_media(self, media_name, cmd, value=None):
        media = self.resource.get_media(media_name)
        if not media:
            return False
        if media.is_media_streaming():
            return False
        return media.do_cmd(cmd, value)

    def get_info_media(self, media_name):
        media = self.resource.get_media(media_name)
        if not media:
            return False
        return media.get_info()

    def start_record(self, media_name, path=None):
        media = self.resource.get_media(media_name)
        if not media:
            return False
        if media.is_media_video():
            return False
        return media.start_record(path=path)

    def stop_record(self, media_name):
        media = self.resource.get_media(media_name)
        if not media:
            return False
        if media.is_media_video():
            return False
        return media.stop_record()

    def get_params_media(self, media_name):
        media = self.resource.get_media(media_name)
        if not media:
            return []
        return media.get_properties_param()

    def update_param_media(self, media_name, param_name, value):
        media = self.resource.get_media(media_name)
        if not media:
            return False
        return media.update_property_param(param_name, value)

    def save_params_media(self, media_name):
        media = self.resource.get_media(media_name)
        if not media:
            return False
        return self.config.write_media(media)

    ##########################################################################
    #############################  OBSERVER  #################################
    ##########################################################################
    def add_image_observer(self, observer, execution_name, filter_name):
        """
            Inform the server what filter we want to observe
            Param :
                - ref, observer is a reference on method for callback
                - string, execution_name to select an execution
                - string, filter_name to select the filter
        """
        ret_value = False
        filterchain = self._get_filterchain(execution_name)
        if not filterchain:
            return False
        return filterchain.add_image_observer(observer, filter_name)

    def set_image_observer(self, observer, execution_name, filter_name_old, filter_name_new):
        """
            Inform the server what filter we want to change observer
            Param :
                - ref, observer is a reference on method for callback
                - string, execution_name to select an execution
                - string, filter_name_old , filter to replace
                - string, filter_name_new , filter to use
        """
        ret_value = False
        if filter_name_old != filter_name_new:
            filterchain = self._get_filterchain(execution_name)
            if not filterchain:
                return False
            filterchain.remove_image_observer(observer, filter_name_old)
            ret_value = filterchain.add_image_observer(observer, filter_name_new)
        return ret_value

    def remove_image_observer(self, observer, execution_name, filter_name):
        """
            Inform the server what filter we want to remove observer
            Param :
                - ref, observer is a reference on method for callback
                - string, execution_name to select an execution
                - string, filter_name , filter to remove
        """
        filterchain = self._get_filterchain(execution_name)
        if not filterchain:
            return False
        return filterchain.remove_image_observer(observer, filter_name)

    def add_output_observer(self, execution_name):
        """
            attach the output information of execution to tcp_server
            supported only one observer. Add observer to tcp_server
        """
        filterchain = self._get_filterchain(execution_name)
        if not filterchain:
            return False
        if self.server_observer.send in filterchain.get_filter_output_observers():
            logger.warning("Manager: observer already exist - add_output_observer")
            return True
        return filterchain.add_filter_output_observer(self.server_observer.send)

    def remove_output_observer(self, execution_name):
        """
            remove the output information of execution to tcp_server
            supported only one observer. remove observer to tcp_server
        """
        filterchain = self._get_filterchain(execution_name)
        if not filterchain:
            return False
        if not filterchain.get_filter_output_observers():
            return True
        return filterchain.remove_filter_output_observer(self.server_observer.send)

    ##########################################################################
    ########################## CONFIGURATION  ################################
    ##########################################################################
    def get_filterchain_list(self):
        return self.resource.get_filterchain_list()

    def upload_filterchain(self, filterchain_name, s_file_contain):
        return self.resource.upload_filterchain(filterchain_name, s_file_contain)

    def delete_filterchain(self, filterchain_name):
        return self.resource.delete_filterchain(filterchain_name)

    def get_filter_list_from_filterchain(self, filterchain_name):
        return self.resource.get_filters_from_filterchain(filterchain_name)

    def modify_filterchain(self, old_filterchain_name, new_filterchain_name, lst_str_filters):
        return self.resource.modify_filterchain(old_filterchain_name, new_filterchain_name, lst_str_filters)

    ##########################################################################
    ############################ FILTERCHAIN  ################################
    ##########################################################################
    def reload_filter(self, filtre=None):
        for execution in self.dct_exec.values():
            filterchain = execution.get(KEY_FILTERCHAIN, None)
            if filterchain:
                filterchain.reload_filter(filtre)

    def get_params_filterchain(self, execution_name, filter_name):
        # get actual filter from execution
        # TODO search information from configuration if execution not exist
        if not execution_name:
            return False
        filterchain = self._get_filterchain(execution_name)
        if not filterchain:
            return False
        return filterchain.get_params(filter_name=filter_name)

    def update_param(self, execution_name, filter_name, param_name, value):
        filterchain = self._get_filterchain(execution_name)
        if not filterchain:
            return False
        filter = filterchain.get_filter(name=filter_name)
        param = filter.get_params(param_name=param_name)
        if param:
            param.set(value)
            filter.configure()
            return True
        return False

    def save_params(self, execution_name):
        "Force serialization and overwrite config"
        filterchain = self._get_filterchain(execution_name)
        if filterchain is None:
            return False
        return self.config.write_filterchain(filterchain)

    ##########################################################################
    ############################### FILTER  ##################################
    ##########################################################################
    def get_filter_list(self):
        return self.resource.get_filter_info_list()

    ##########################################################################
    ############################## PRIVATE  ##################################
    ##########################################################################
    def _get_filterchain(self, execution_name):
        dct_execution = self.dct_exec.get(execution_name, {})
        if not dct_execution:
            logger.warning("Manager: don't find execution %s. List execution name: %s",
                           execution_name, self.dct_exec.keys())
            return None
        filterchain = dct_execution.get(KEY_FILTERCHAIN, None)
        if not filterchain:
            logger.critical("Manager: execution %s hasn't filterchain. Internal error.",
                           execution_name)
            return None
        return filterchain