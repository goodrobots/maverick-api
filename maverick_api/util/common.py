import time, os, stat

def file_age_in_seconds(pathname):
    try:
        return time.time() - os.stat(pathname)[stat.ST_MTIME]
    except OSError as e:
        print('Failed to obtain file age {0}'.format(e))
        return None

def merge_two_dicts(x, y):
    z = x.copy()   # start with x's keys and values
    z.update(y)    # modifies z with y's keys and values & returns None
    return z
    
def find(key, dictionary):
    '''Find all occurences of a key in nested python dictionaries and lists'''
    # https://stackoverflow.com/questions/9807634/find-all-occurrences-of-a-key-in-nested-python-dictionaries-and-lists
    for k, v in dictionary.iteritems():
        if k == key:
            yield v
        elif isinstance(v, dict):
            for result in find(key, v):
                yield result
        elif isinstance(v, list):
            for d in v:
                for result in find(key, d):
                    yield result