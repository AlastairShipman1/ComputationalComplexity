import timeit
import config
import main
import cProfile
import_module = "import main, config"

testcode = ''' 
def test(): 
    config.DISPLAY_ON=True
    main.main()
test()
'''

if config.PROFILING:
    cProfile.run('main.main()')
else:
    num_runs = 1
    x = timeit.repeat(stmt=testcode, setup=import_module, number=num_runs)
    for i in range(len(x)):
        x[i] /= num_runs
    print(x)


