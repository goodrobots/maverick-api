import os, errno

# https://stackoverflow.com/questions/273192/how-can-i-safely-create-a-nested-directory-in-python

def makePath(directory):
    # TODO: check to ensure there is no '.' within the final file path (e.g. "foo.txt")
    try:
        os.makedirs(directory)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise