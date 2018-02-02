# Script to backup the SQLite DB

import sys
import os
import datetime

def get_db_filename():
    return os.path.join(os.getcwd(), 'data', 'maverick-api-database.db')

db_filename = get_db_filename()
backup_path = os.path.join(os.getcwd(), 'data', 'backups')

if not os.path.exists(backup_path):
    os.mkdir('backups')

# TODO: do we need a unique Maverick ID to associate a database with a system?
# for now just use date time 
backup_file = os.path.join(backup_path, 'backup_db_'+ datetime.datetime.now().strftime('%Y_%m_%d-%H_%M'))

os.system('sqlite3 '+db_filename+' ".backup '+backup_file+'.sqlite"')
os.system('sqlite3 '+db_filename+' "SELECT * from Logs" >'+backup_file+'.sql')

num_lines = sum(1 for line in open(backup_file+'.sql'))
print('Backed up {} records to {}'.format(num_lines, backup_file+'.sqlite')) # need to log this!