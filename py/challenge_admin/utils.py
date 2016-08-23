import os

class utils:
    #clear console so that printing looks clean
    clear_console = lambda: os.system('clear')

    # setup printing of colored items
    print_active = lambda s: print('    \033[1;44m{}\033[1;m'.format(s))

    # status printing
    status_begin = lambda s: print(s, '... ', sep='', end='', flush=True)
    status_done = lambda: print('DONE')
    status_fail = lambda: print('FAILED')
