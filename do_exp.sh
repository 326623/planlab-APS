# !/bin/bash
TIME_LIMIT=1

./firstApp --factory_world=../input_example.txt --verify_solution false --lambda=0.01 --time_limit=$TIME_LIMIT --output_file=output
clp problem.lp solve solu sol001
./firstApp --factory_world=../input_example.txt --verify_solution false --lambda=0.1 --time_limit=$TIME_LIMIT --output_file=output
clp problem.lp solve solu sol01
./firstApp --factory_world=../input_example.txt --verify_solution false --lambda=0 --time_limit=$TIME_LIMIT --output_file=output
clp problem.lp solve solu sol0
./firstApp --factory_world=../input_example.txt --verify_solution false --lambda=1 --time_limit=$TIME_LIMIT --output_file=output
clp problem.lp solve solu sol1
./firstApp --factory_world=../input_example.txt --verify_solution false --lambda=10 --time_limit=$TIME_LIMIT --output_file=output
clp problem.lp solve solu sol10
./firstApp --factory_world=../input_example.txt --verify_solution false --lambda=100 --time_limit=$TIME_LIMIT --output_file=output
clp problem.lp solve solu sol100
