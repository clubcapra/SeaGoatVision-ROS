#! /usr/bin/env python

#    Copyright (C) 2012  Club Capra - capra.etsmtl.ca
#
#    This file is part of CapraVision.
#
#    CapraVision is free software: you can redistribute it and/or modify
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
Description : Server Service implementation using vServer_pb2, generate by .proto file
Authors: Mathieu Benoit (mathben963@gmail.com)
Date : October 2012
"""

# Import required RPC modules
from CapraVision.proto import server_pb2
from CapraVision.server.core.manager import Manager
import time

class ProtobufServerImpl(server_pb2.CommandService):
    def __init__(self, * args, ** kwargs):
        server_pb2.CommandService.__init__(self, * args, ** kwargs)
        self.manager = Manager()
        self.observer = {}

    ##########################################################################
    ################################ CLIENT ##################################
    ##########################################################################
    def is_connected(self, controller, request, done):
        print("is_connected request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        response.status = 0
        # We're done, call the run method of the done callback
        done.run(response)

    ##########################################################################
    ######################## EXECUTION FILTER ################################
    ##########################################################################
    def start_filterchain_execution(self, controller, request, done):
        print("start_filterchain_execution request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            # dont start a filterchain execution if it already exist
            if not self.observer.get(request.execution_name, None):
                self.observer[request.execution_name] = self.manager.start_filterchain_execution(request.execution_name, request.source_name, request.filterchain_name)
                response.status = 0
            else:
                response.status = 1
                response.message = "This filterchain_execution already exist." 
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)
    
        
    def stop_filterchain_execution(self, controller, request, done):
        print("stop_filterchain_execution request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            if self.observer.get(request.execution_name, None):
                response.status = int(not self.manager.stop_filterchain_execution(request.execution_name))
                del self.observer[request.execution_name]
                response.status = 0
            else:
                response.status = 1
                response.message = "This filterchain_execution is already close."
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)

    def get_image(self, controller, request, done):
        #print("get_image request %s" % str(request).replace("\n", " "))
        
        # Create a reply
        response = server_pb2.GetImageResponse()
        try:
            if self.observer.get(request.execution_name, None):
                aTime = time.time()
                image = self.observer[request.execution_name].next()
                bTime = time.time()
                response.image = image.dumps()
                cTime = time.time()
                print("Acquisition time : %s, dumps time %s" % (bTime - aTime, cTime - bTime))
            else:
                print("get_image request on execution_name doesn't exist." % request.execution_name)
        except Exception, e:
            print "Exception: ", e

        # We're done, call the run method of the done callback
        done.run(response)

    ##########################################################################
    ############################## OBSERVATOR ################################
    ##########################################################################
    def set_image_filter_observer(self, controller, request, done):
        print("set_image_filter_observer request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            # dont start a filterchain execution if it already exist
            observer = self.observer.get(request.execution_name, None) 
            if observer:
                observer.set_filter_name(request.filter_name)
                response.status = 0
            else:
                response.status = 1
                response.message = "This filterchain_execution doesn't exist." 
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)
    
    ##########################################################################
    ################################ SOURCE ##################################
    ##########################################################################
    def get_source_list(self, controller, request, done):
        print("get_source_list request %s" % str(request).replace("\n", " "))

        response = server_pb2.GetSourceListResponse()
        for item in self.manager.get_source_list():
            response.source.append(item)
            
        done.run(response)
    
    ##########################################################################
    ############################### THREAD  ##################################
    ##########################################################################
   
    ##########################################################################
    ##########################  CONFIGURATION  ###############################
    ##########################################################################
    def get_filterchain_list(self, controller, request, done):
        print("get_filterchain_list request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.GetFilterChainListResponse()
        for item in self.manager.get_filterchain_list():
            if item.doc:
                response.filterchains.add(name=item.name, doc=item.doc)
            else:
                response.filterchains.add(name=item.name)

        # We're done, call the run method of the done callback
        done.run(response)
    
    def delete_filterchain(self, controller, request, done):
        print("delete_filterchain request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            response.status = int(not self.manager.load_chain(request.filterchain_name))
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)
    
    ##########################################################################
    ############################ FILTERCHAIN  ################################
    ##########################################################################
    def get_filter_list_from_filterchain(self, controller, request, done):
        print("get_filter_list_from_filterchain request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.GetFilterListFromFilterChainResponse()
        for item in self.manager.get_filter_list_from_filterchain(request.filterchain_name):
            response.filters.add(name=item.name, doc=item.doc)

        # We're done, call the run method of the done callback
        done.run(response)


    def load_chain(self, controller, request, done):
        print("load_chain request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            response.status = int(not self.manager.load_chain(request.filterName))
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)


    def delete_filterchain(self, controller, request, done):
        print("delete_filterchain request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            response.status = int(not self.manager.delete_filterchain(request.filterchain_name))
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)


    def upload_filterchain(self, controller, request, done):
        print("upload_filterchain request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            response.status = int(not self.manager.upload_filterchain(request.filterchain_name, request.s_file_contain))
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)


    def modify_filterchain(self, controller, request, done):
        print("modify_filterchain request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            lstStrFilter = [filter.name for filter in request.lst_str_filters]
            response.status = int(not self.manager.modify_filterchain(request.old_filterchain_name, request.new_filterchain_name, lstStrFilter))
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)


    def reload_filter(self, controller, request, done):
        print("reload_filter request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.StatusResponse()
        try:
            self.manager.reload_filter(request.filterName)
            response.status = 0
        except Exception, e:
            print "Exception: ", e
            response.status = -1

        # We're done, call the run method of the done callback
        done.run(response)

  
    ##########################################################################
    ############################### FILTER  ##################################
    ##########################################################################
    def get_filter_list(self, controller, request, done):
        print("get_filter_list request %s" % str(request).replace("\n", " "))

        # Create a reply
        response = server_pb2.GetFilterListResponse()
        for filter in self.manager.get_filter_list():
            response.filters.append(filter)

        # We're done, call the run method of the done callback
        done.run(response)
