#!/usr/bin/env python
from __future__ import print_function
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from util.common import find, file_age_in_seconds
import xml.etree.ElementTree as ET
import pprint
import requests
import random
import os
import ast
import re

# Define what will be encoded and sent to the web ui for a bool, True or 1 and False or 0
bool_true = 1 # True
bool_false = 0 # False

vehicles = ['APMrover2', 'ArduCopter', 'ArduPlane', 'ArduSub', 'AntennaTracker', 'PX4']

def get_ardupilot_url(vehicle):
    '''Return a formatted url to target a .xml on the ardupilot auto test server'''
    return 'http://autotest.ardupilot.org/Parameters/{0}/apm.pdef.xml'.format(vehicle)

def test_file_age(file_path, max_age):
    '''Check the age of a file in seconds and test if it is under the maximum limit'''
    sec = file_age_in_seconds(file_path)
    if (sec is not None and sec <= max_age):
        # The file age is less than the allowable limit
        return True
    else:
        # The file did not exist or exceeded the maximum age limit
        return False
                    
def get_param_meta(vehicle, remote = True, force_download = False, max_age = 60*60):
    if not vehicle in vehicles:
        # TODO: inform failure
        return {}
    else:
        if 'PX4' in vehicle:
            remote = False # PX4 does not support remote download of param meta
    if remote:
        # check to see if we have a recent file from the server
        dir_path = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(dir_path, '{0}.xml'.format(vehicle))
        if (not test_file_age(file_path, max_age) or force_download):
            url = get_ardupilot_url(vehicle)
            tree = download_param_meta(url)
            
            if tree is not None:
                save_param_meta(tree, vehicle)
            else:
                # request for meta failed, try to fall back to saved file
                # TODO: inform user
                tree = load_param_meta(vehicle)
        else:
            tree = load_param_meta(vehicle)
        return extract_param_meta_from_tree(tree, vehicle)
    else:
        # TODO: generate param meta from ardupilot / PX4 code base
        # FIXME: for now just load the PX4 / ardu params from file
        if 'PX4' in vehicle:
            # generate PX4 param meta
            pass
        else:
            # generate Ardu* param meta
            pass
        
        # load the newly generated param meta file
        tree = load_param_meta(vehicle)
        return extract_param_meta_from_tree(tree, vehicle)
        
def save_param_meta(tree, file_name, dir_path = None):
    '''Write a tree structure to a local .xml file'''
    if tree is None:
        return False
    if not dir_path:
        dir_path = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(dir_path, '{0}.xml'.format(file_name))
    param_meta_data = ET.tostring(tree)
    print('Saving meta to {0}'.format(file_path))
    with open(file_path, 'w') as fid:
        fid.write(param_meta_data)
    
def download_param_meta(url, timeout = (3.0, 5.0)): # 3 second connect and 5 second read timeout
    '''Download a .xml file from a remote server and return the tree structure''' 
    print('Downloading meta from {0}'.format(url))
    tree = None
    try:
        response = requests.get(url, timeout=timeout) 
        tree = ET.fromstring(response.content)
    except requests.exceptions.ConnectionError as e:
        print('Could not retreive param meta data from remote server: {0} {1}'.format(url, e))
    finally:
        return tree
        
def load_param_meta(file_name, dir_path = None):
    '''Load a param meta .xml file and return the tree structure'''
    tree = None
    if not dir_path:
        dir_path = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(dir_path, '{0}.xml'.format(file_name))
    print('Loading meta from {0}'.format(file_path))
    try:
        with open(file_path, 'rt') as f:
            tree = ET.parse(f)
        return tree
    except IOError as e:
        print('An error occurred while loading meta from {0} {1}'.format(file_path, e))
        return None

def extract_text(node):
    '''Sanitize string white space'''
    return re.sub('\s+',' ',node.text) # replace newlines, tabs and white space with single space
        
def run_bool_test(curr_param):
    '''Ardu* specific test to see if a set of values should be a boolean type'''
    if (curr_param['values'] is not None and curr_param['type'] is not 'BITMASK'): # Don't try to turn bitmasks into bools
        if len(curr_param['values']) == 2 and curr_param['values'].keys() == [0,1]: # Can only contain two keys and they must be 0 and 1
            if (('DISA' in curr_param['values'][0].upper() and 'ENAB' in curr_param['values'][1].upper())
            or ('NORM' in curr_param['values'][0].upper() and 'REVE' in curr_param['values'][1].upper())):
                curr_param['values'] = {bool_true:curr_param['values'][1], bool_false:curr_param['values'][0]}
                curr_param['type'] = 'BOOLEAN'
    return curr_param
    
def extract_param_meta_from_tree(tree, vehicle):
    '''Single entry point for parsing param meta files'''
    if 'PX4' in vehicle:
        # It's PX4
        meta = extract_param_meta_from_tree_px4(tree, vehicle)
    else:
        # It's Ardu*
        meta = extract_param_meta_from_tree_ardupilot(tree, vehicle)
    print('Obtained meta for {0} params'.format(len(meta)))
    return meta
    
def extract_param_meta_from_tree_px4(tree, vehicle):
    '''Parse a PX4 param meta file'''
    if tree is None:
        print('Error: No valid tree')
        return {}
    root = {}
    curr_param_dict = {}
    curr_group_name = ''
    curr_param_name = ''
    curr_name = ''
    
    groups = tree.findall('./group')
    for group in groups:
        for sub in group.iter():
            # the first element is the group tag itself
            if sub.tag == 'group':
                curr_group_name = sub.attrib['name'] # we enforce that there is a group name
            elif sub.tag == 'parameter':
                if curr_param_dict: # we have already populated curr_param_dict, write it to the root and reset
                    # this is not run on the first loop
                    root[curr_param_name] = curr_param_dict
                    curr_param_dict = {} # reset the param dict
                    curr_param_name = ''
                curr_param_name = sub.attrib.get('name').upper()
                    
                curr_param_dict = {'humanName':None, 'humanGroup':curr_group_name,
                'rebootRequired':False, 'increment':None,
                'unitText':None, 'units':None, 'min':None, 'max':None,
                'values':None, 'documentation':None,
                'bitmask':None, 'decimal':None, 'type':None
                }
                # get rid of lover case names! e.g. broken uavcan stuff

                curr_param_dict['group'] = curr_param_name.split('_')[0].strip().rstrip('_').upper()
                curr_param_type = sub.attrib.get('type').upper()
                if 'INT' in curr_param_type:
                    curr_param_dict['type'] = 'INTERGER'
                else:
                    curr_param_dict['type'] = curr_param_type
                
                
            elif sub.tag in ['long_desc', 'unit', 'min', 'max', 'reboot_required',
                            'increment', 'decimal', 'short_desc']:
                tag_string = extract_text(sub)
                if sub.tag == 'long_desc':
                    curr_param_dict['documentation'] = tag_string
                elif sub.tag == 'short_desc':
                    curr_param_dict['humanName'] = tag_string
                elif sub.tag == 'unit':
                    curr_param_dict['units'] = tag_string
                elif sub.tag == 'reboot_required':
                    if 'TRUE' in tag_string.upper():
                        curr_param_dict['rebootRequired'] = True
                    else:
                        curr_param_dict['rebootRequired'] = False
                elif sub.tag == 'decimal':
                    curr_param_dict['decimal'] = ast.literal_eval(tag_string)
                elif sub.tag == 'increment':
                    curr_param_dict['increment'] = ast.literal_eval(tag_string)
                elif sub.tag in ['min', 'max']:
                    curr_param_dict[sub.tag] = ast.literal_eval(tag_string)
                else:
                    pass
            elif sub.tag == 'value':
                if curr_param_dict['values'] is None:
                    curr_param_dict['values'] = {}
                curr_param_dict['values'][ast.literal_eval(sub.attrib['code'].encode('utf-8').strip())]=sub.text.encode('utf-8')
            elif sub.tag == 'bit':
                if curr_param_dict['bitmask'] is None:
                    curr_param_dict['bitmask'] = {}
                tag_string = extract_text(sub)
                curr_param_dict['bitmask'][2**int(sub.attrib.get('index'))] = tag_string
                curr_param_dict['type'] = 'BITMASK'
            
            elif sub.tag == 'boolean':
                curr_param_dict['values'] = {bool_false: 'Disabled', bool_true:'Enabled'}
                curr_param_dict['type'] = 'BOOLEAN'
            else:
                pass
            
    root[curr_param_name] = curr_param_dict
    
    # pprint.pprint(root)
    return root
    
def extract_param_meta_from_tree_ardupilot(tree, vehicle):
    '''Parse an Ardu param meta file'''
    if tree is None:
        print('Error: No valid tree')
        return {}
    root = {}
    curr_param_dict = {}
    curr_param_name = ''
    curr_name = ''
    
    # find both the vehicles and libraries parameters
    nodes = tree.findall('.//parameters')
    for node in nodes:
        for sub in node.iter():
            if sub.tag == 'parameters':
                if curr_param_dict: # we have already populated curr_param_dict, write it to the root and reset
                    # this is not run on the first loop
                    curr_param_dict = run_bool_test(curr_param_dict)
                    root[curr_name][curr_param_name] = curr_param_dict
                    curr_param_dict = {}
                    
                curr_name = str(sub.attrib['name'])
                root[curr_name] = {}
            elif sub.tag == 'param':
                if curr_param_dict: # we have already populated curr_param_dict, write it to the root and reset
                    # this is not run on the first loop
                    curr_param_dict = run_bool_test(curr_param_dict)
                    root[curr_name][curr_param_name] = curr_param_dict
                    curr_param_dict = {}
                    
                curr_param_name = sub.attrib['name']
                if vehicle in curr_param_name:
                    # remove un needed vehicle name if present
                    curr_param_name = curr_param_name.lstrip(vehicle).lstrip(':')
                curr_param_dict = {'humanName':sub.attrib.get('humanName', None), 'humanGroup':None,
                'rebootRequired':False, 'increment':None,
                'unitText':None, 'units':None, 'min':None, 'max':None,
                'values':None, 'documentation':sub.attrib.get('documentation', None),
                'bitmask':None, 'decimal':None, 'type':None
                }
            
            # process all 'fields', range, increment, units, unitText, etc...
            elif sub.tag == 'field':
                name = sub.attrib['name'].strip().upper()
                if name == 'RANGE':
                    # split the range values
                    param_range = sub.text.strip().split(' ')
                    if len(param_range) != 2:
                        # TODO: log the error
                        pass
                    else:
                        curr_param_dict['min'] = ast.literal_eval(param_range[0].strip())
                        curr_param_dict['max'] = ast.literal_eval(param_range[1].strip().lstrip('+'))
                elif name == 'UNITS':
                    curr_param_dict['units'] = sub.text.strip()
                elif name == 'UNITTEXT':
                    curr_param_dict['unitText'] = sub.text.strip()
                elif name == 'INCREMENT':
                    curr_param_dict['increment'] = ast.literal_eval(sub.text.strip())
                elif name == 'REBOOTREQUIRED':
                    if 'TRUE' in sub.text.strip().upper():
                        curr_param_dict['rebootRequired'] = True
                    else:
                        curr_param_dict['rebootRequired'] = False
                elif name == 'BITMASK':
                    if curr_param_dict['bitmask'] is None:
                        curr_param_dict['bitmask'] = {}
                    bitmask_text = sub.text.strip().rstrip(',')
                    bitmask_ents = bitmask_text.split(',')
                    for ent in bitmask_ents:
                        try:
                            [idx, val] = ent.split(':')
                            curr_param_dict['bitmask'][2**int(idx.strip())] = val.strip()
                        except:
                            pass
                    curr_param_dict['type'] = 'BITMASK'
                else:
                    pass
            elif sub.tag == 'value':
                if curr_param_dict['values'] is None:
                    curr_param_dict['values'] = {}
                code = ast.literal_eval(sub.attrib['code'].strip())
                text = sub.text.strip()
                curr_param_dict['values'][code]=text
                
            else:
                pass
                # print sub.tag, sub.attrib, sub.text
                
    if curr_param_dict: # we have unwritten param data, write it to the root
        curr_param_dict = run_bool_test(curr_param_dict)
        root[curr_name][curr_param_name] = curr_param_dict
        
    # pprint.pprint(root)
    
    meta = {}
    for param_group in root:
        for param in root[param_group].keys():
            param_meta = root[param_group][param]
            # dont use param_group here in order to avoid vehicle name as group
            param_meta['group'] = param.split('_')[0].strip().rstrip('_').upper()
            meta[param] = param_meta
    # pprint.pprint(meta)
    return meta

def download_and_save_all_param_meta(timeout = 60):
    '''Try to download all param meta from ardupilot auto test server'''
    for vehicle in vehicles:
        if 'PX4' in vehicle:
            # PX4 does not support parm meta downloading
            continue
        url = get_ardupilot_url(vehicle)
        tree = download_param_meta(url, timeout = timeout)
        save_param_meta(tree, file_name = vehicle)
    
if __name__ == '__main__':
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
    # randomly select a vehicle and obtain the meta for its params
    vehicle = random.choice(vehicles)
    print(vehicle)
    meta = get_param_meta(vehicle, remote = True)
    # pprint.pprint(meta)
    # download_and_save_all_param_meta()