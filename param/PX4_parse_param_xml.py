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
import os, re
import ast

# define what will be encoded and sent to the web ui for a bool, True or 1 and False or 0
bool_true = 1 # True
bool_false = 0 # False

def test_file_age(file_path, max_age):
    sec = file_age_in_seconds(file_path)
    if (sec is not None and sec <= max_age):
        return True
    else:
        return False
                    
def get_param_meta(vehicle, remote = True, force_download = False, max_age = 60*10):
    if not vehicle in vehicles:
        # TODO: inform failure
        return {}
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
        # TODO: generate param meta from ardupilot code base
        return extract_param_meta_from_tree(None, vehicle)
        
def save_param_meta(tree, file_name, dir_path = None):
    if tree is None:
        return False
    if not dir_path:
        dir_path = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(dir_path, '{0}.xml'.format(file_name))
    param_meta_data = ET.tostring(tree)
    print('Saving meta to {0}'.format(file_path))
    with open(file_path, 'w') as fid:
        fid.write(param_meta_data)
        
def load_param_meta(file_name, dir_path = None):
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
    return re.sub('\s+',' ',node.text) # replace newlines, tabs and white space with single space

def extract_param_meta_from_tree(tree, vehicle):
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
    
    pprint.pprint(root)
    return root

if __name__ == '__main__':
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
    
    tree = load_param_meta('parameters')
    meta = extract_param_meta_from_tree(tree, 'test')
    # print(meta)