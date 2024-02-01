#! /usr/bin/env python3

"""
.. module:: load_environment
   :platform: Unix
   :synopsis: "load_environment" typically refers to a process of loading or importing ontological data(environment), often used in 
              knowledge management systems or semantic web applications. Also, This node helps to load the environmental map to the robot

.. moduleauthor:: Ankur Kohli

 Client:
    /armor_client: A client used to interact with the aRMOR server for the purpose of ontology creation.
"""

# Import the armor client class
import time
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
client = ArmorClient("armor_client", "my_ontology") 

path = dirname(realpath(__file__))
# path of .owl file
path = path + "/../topological_map/"

# Starting with buffered manipulation and reasoning enabled or initialized.
client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)

client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

def LoadKnowledgeGraph():
		client.manipulation.add_ind_to_class('E', 'LOCATION')
		client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'E')
		client.manipulation.add_ind_to_class('C1', 'LOCATION')
		client.manipulation.add_ind_to_class('C2', 'LOCATION')
		client.manipulation.add_ind_to_class('R1', 'LOCATION')
		client.manipulation.add_ind_to_class('R2', 'LOCATION')
		client.manipulation.add_ind_to_class('R3', 'LOCATION')
		client.manipulation.add_ind_to_class('R4', 'LOCATION')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'E', 'D5')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'E', 'D6')

		client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D1')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D2')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D5')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D7')

		client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D3')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D4')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D5')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D6')

		client.manipulation.add_objectprop_to_ind('hasDoor', 'R1', 'D1')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'R2', 'D2')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'R3', 'D3')
		client.manipulation.add_objectprop_to_ind('hasDoor', 'R4', 'D4')
		client.manipulation.add_dataprop_to_ind('visitedAt', 'R1', 'Long', str(int(time.time())))
		client.manipulation.add_dataprop_to_ind('visitedAt', 'R2', 'Long', str(int(time.time())))
		client.manipulation.add_dataprop_to_ind('visitedAt', 'R3', 'Long', str(int(time.time())))
		client.manipulation.add_dataprop_to_ind('visitedAt', 'R4', 'Long', str(int(time.time())))
		client.call('DISJOINT', 'IND', '', ['E','C1','C2','R1','R2','R3','R4','D1','D2','D3','D4','D5','D6','D7'])
		
		
    		# Sync with reasoner
		client.utils.apply_buffered_changes()
		client.utils.sync_buffered_reasoner()
